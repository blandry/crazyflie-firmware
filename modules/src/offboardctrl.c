#define DEBUG_MODULE "OFFBOARDCTRL"

#include <math.h>

#include "FreeRTOS.h"
#include "task.h"

#include "offboardctrl.h"
#include "crtp.h"
#include "motors.h"
#include "system.h"
#include "debug.h"
#include "pm.h"
#include "stabilizer.h"

struct InputCrtpValues
{
  float input1;
  float input2;
  float input3;
  float input4;
} __attribute__((packed));

#define THRUSTS_UPDATE_FREQ 500
#define A_OFFSET -1.208905335853438f

static bool isInit;
static bool isInactive;
static uint32_t lastUpdate;
static struct InputCrtpValues inputCmd;

static float Va;
static float Vm;
static float omega1;
static float omega2;
static float omega3;
static float omega4;
static float thrust1;
static float thrust2;
static float thrust3;
static float thrust4;

#define ATTITUDE_UPDATE_RATE_DIVIDER  2

static void offboardCtrlCrtpCB(CRTPPacket* pk);
void offboardCtrlTask(void* param);
static void offboardCtrlWatchdogReset(void);
static void updateThrusts(void);
float limit_thrust(float);
float limit_omegasqu(float);

void offboardCtrlInit(void)
{
  if(isInit)
    return;

  crtpInit();
  motorsInit();
  crtpRegisterPortCB(CRTP_PORT_OFFBOARDCTRL, offboardCtrlCrtpCB);

  lastUpdate = xTaskGetTickCount();
  isInactive = TRUE;
  isInit = TRUE;
}

bool offboardCtrlTest(void)
{
  bool pass = true;
  pass &= crtpTest();
  pass &= motorsTest();
  return pass;
}

static void offboardCtrlCrtpCB(CRTPPacket* pk)
{
  inputCmd = *((struct InputCrtpValues*)pk->data);
  offboardCtrlWatchdogReset();
}

void offboardCtrlWatchdog(void)
{
  uint32_t ticktimeSinceUpdate = xTaskGetTickCount() - lastUpdate;
  if (ticktimeSinceUpdate > OFFBOARDCTRL_WDT_TIMEOUT_SHUTDOWN)
  {
    inputCmd.input1 = 0;
    inputCmd.input2 = 0;
    inputCmd.input3 = 0;
    inputCmd.input4 = 0;
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

  Va = pmGetBatteryVoltage();
  Vm = 4;

  omega1 = sqrt(limit_omegasqu(inputCmd.input1));
  omega2 = sqrt(limit_omegasqu(inputCmd.input2));
  omega3 = sqrt(limit_omegasqu(inputCmd.input3));
  omega4 = sqrt(limit_omegasqu(inputCmd.input4));
  
  thrust1 = ((Vm/Va)*omega1+A_OFFSET)*10000;
  thrust2 = ((Vm/Va)*omega2+A_OFFSET)*10000;
  thrust3 = ((Vm/Va)*omega3+A_OFFSET)*10000;
  thrust4 = ((Vm/Va)*omega4+A_OFFSET)*10000;

  motorsSetRatio(MOTOR_M1,(uint32_t) limit_thrust(thrust1));
  motorsSetRatio(MOTOR_M2,(uint32_t) limit_thrust(thrust2));
  motorsSetRatio(MOTOR_M3,(uint32_t) limit_thrust(thrust3));
  motorsSetRatio(MOTOR_M4,(uint32_t) limit_thrust(thrust4));
}

float limit_omegasqu(float raw_omegasqu)
{
  float omegasqu_output = raw_omegasqu;
  if (omegasqu_output>59.4){
    omegasqu_output = 59.4;
  } else if (omegasqu_output<A_OFFSET*A_OFFSET) {
    omegasqu_output = A_OFFSET*A_OFFSET;
  }
  return omegasqu_output;
}

float limit_thrust(float raw_thrust)
{
  float thrust_output = raw_thrust;
  if (thrust_output>65000){
    thrust_output = 65000;
  } else if (thrust_output<0) {
    thrust_output = 0;
  }
  return thrust_output;
}

void offboardCtrlTask(void* param)
{
  vTaskSetApplicationTaskTag(0, (void*)TASK_OFFBOARDCTRL_ID_NBR);
  systemWaitStart();
  uint32_t lastWakeTime = xTaskGetTickCount();
  uint32_t attitudeCounter = 0;
  while(1)
  {
    vTaskDelayUntil(&lastWakeTime, F2T(THRUSTS_UPDATE_FREQ));
    if (++attitudeCounter >= ATTITUDE_UPDATE_RATE_DIVIDER)
    {
      stabilizerUpdateEuler();
      attitudeCounter = 0;
    }
    updateThrusts();
  }
}