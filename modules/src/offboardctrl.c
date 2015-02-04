#define DEBUG_MODULE "OFFBOARDCTRL"

#include "math.h"
#include "string.h"

#include "FreeRTOS.h"
#include "task.h"

#include "offboardctrl.h"
#include "crtp.h"
#include "motors.h"
#include "system.h"
#include "sensfusion6.h"
#include "imu.h"
#include "pm.h"
#include "debug.h"

#define A_OFFSET -1.208905335853438f
#define V_MAX 4.0f

struct InputCrtpValues
{
  float input1;
  float input2;
  float input3;
  float input4;
  int type;
} __attribute__((packed));

#define OFFBOARDCTRL_UPDATE_FREQ 100

static Axis3f gyro; // Gyro axis data in deg/s
static Axis3f acc;  // Accelerometer axis data in mG
static float eulerRollActual;
static float eulerPitchActual;
static float eulerYawActual;

static bool isInit;
static bool isInactive;
static uint32_t lastUpdate;
static struct InputCrtpValues inputCmd;

static float Va = V_MAX;
static float omega1 = 0.0;
static float omega2 = 0.0;
static float omega3 = 0.0;
static float omega4 = 0.0;
static float thrust1 = 0.0;
static float thrust2 = 0.0;
static float thrust3 = 0.0;
static float thrust4 = 0.0;

static CRTPPacket pk;

static void offboardCtrlCrtpCB(CRTPPacket* pk);
static void offboardCtrlWatchdogReset(void);
static void updateSensors(void);
static void updateThrusts(void);
void offboardCtrlTask(void* param);

#undef max
#define max(a,b) ((a) > (b) ? (a) : (b))
#undef min
#define min(a,b) ((a) < (b) ? (a) : (b))

void offboardCtrlInit(void)
{
  if(isInit)
    return;

  crtpInit();
  motorsInit();
  imu6Init();
  sensfusion6Init();
  crtpRegisterPortCB(CRTP_PORT_OFFBOARDCTRL, offboardCtrlCrtpCB);

  eulerRollActual = 0.0;
  eulerPitchActual = 0.0;
  eulerYawActual = 0.0;
  gyro.x = 0.0;
  gyro.y = 0.0;
  gyro.z = 0.0;

  lastUpdate = xTaskGetTickCount();
  isInactive = TRUE;
  isInit = TRUE;

  pk.header = CRTP_HEADER(CRTP_PORT_SENSORS, 0);
  pk.size = 6*4;

  xTaskCreate(offboardCtrlTask, (const signed char * const)"OFFBOARDCTRL",
              2*configMINIMAL_STACK_SIZE, NULL, /*Piority*/2, NULL);
}

bool offboardCtrlTest(void)
{
  bool pass = true;
  pass &= crtpTest();
  pass &= motorsTest();
  pass &= imu6Test();
  pass &= sensfusion6Test();
  return pass;
}

static void offboardCtrlCrtpCB(CRTPPacket* inputPk)
{
  inputCmd = *((struct InputCrtpValues*)inputPk->data);
  offboardCtrlWatchdogReset();
}

void offboardCtrlWatchdog(void)
{
  uint32_t ticktimeSinceUpdate = xTaskGetTickCount() - lastUpdate;
  if (ticktimeSinceUpdate > OFFBOARDCTRL_WDT_TIMEOUT_SHUTDOWN)
  {
    inputCmd.input1 = 0.0;
    inputCmd.input2 = 0.0;
    inputCmd.input3 = 0.0;
    inputCmd.input4 = 0.0;
    inputCmd.type = 1;
    isInactive = TRUE;
  }
  else
  {
    isInactive = FALSE;
  }
}

static void offboardCtrlWatchdogReset(void)
{
  lastUpdate = xTaskGetTickCount();
}

static void updateThrusts(void)
{
  offboardCtrlWatchdog();

  if (inputCmd.type==1)
  {
    thrust1 = inputCmd.input1;
    thrust2 = inputCmd.input2;
    thrust3 = inputCmd.input3;
    thrust4 = inputCmd.input4; 
  } 
  else if (inputCmd.type==2)
  {
    Va = pmGetBatteryVoltage();
    omega1 = sqrt(max(0.0,inputCmd.input1));
    omega2 = sqrt(max(0.0,inputCmd.input2));
    omega3 = sqrt(max(0.0,inputCmd.input3));
    omega4 = sqrt(max(0.0,inputCmd.input4));
    thrust1 = ((V_MAX/Va)*omega1+A_OFFSET)*10000.0;
    thrust2 = ((V_MAX/Va)*omega2+A_OFFSET)*10000.0;
    thrust3 = ((V_MAX/Va)*omega3+A_OFFSET)*10000.0;
    thrust4 = ((V_MAX/Va)*omega4+A_OFFSET)*10000.0;
  }

  thrust1 = min(65000.0,max(0.0,thrust1));
  thrust2 = min(65000.0,max(0.0,thrust2));
  thrust3 = min(65000.0,max(0.0,thrust3));
  thrust4 = min(65000.0,max(0.0,thrust4));
  motorsSetRatio(MOTOR_M1,(uint16_t) thrust1);
  motorsSetRatio(MOTOR_M2,(uint16_t) thrust2);
  motorsSetRatio(MOTOR_M3,(uint16_t) thrust3);
  motorsSetRatio(MOTOR_M4,(uint16_t) thrust4);
}

static void updateSensors(void)
{
  imu6Read(&gyro, &acc);
  if (imu6IsCalibrated())
  {
    sensfusion6UpdateQ(gyro.x, gyro.y, gyro.z, acc.x, acc.y, acc.z, 1.0/OFFBOARDCTRL_UPDATE_FREQ);
    sensfusion6GetEulerRPY(&eulerRollActual, &eulerPitchActual, &eulerYawActual);
    memcpy(pk.data,&eulerRollActual,4);
    memcpy(pk.data+4,&eulerPitchActual,4);
    memcpy(pk.data+8,&eulerYawActual,4);
    memcpy(pk.data+12,&(gyro.x),4);
    memcpy(pk.data+16,&(gyro.y),4);
    memcpy(pk.data+20,&(gyro.z),4);
    crtpSendPacketNoWait(&pk);
  }
}

void offboardCtrlTask(void* param)
{
  vTaskSetApplicationTaskTag(0, (void*)TASK_OFFBOARDCTRL_ID_NBR);
  systemWaitStart();

  uint32_t xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
  for(;;)
  {
    vTaskDelayUntil(&xLastWakeTime, F2T(OFFBOARDCTRL_UPDATE_FREQ));
    
    updateThrusts();
    updateSensors();
  }
}