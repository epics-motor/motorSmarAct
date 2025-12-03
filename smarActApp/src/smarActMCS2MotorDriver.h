/*
FILENAME...   SmarActMCS2MotorDriver.h
USAGE...      Motor driver support for the SmarAct MCS2 controller.

David Vine
Adapted from Mark Rivers' ACR Driver
Jan 19, 2019

Note:
The MCS2 controller uses 64-bit int for the encoder and target positions. The motor record is limited
to 32 bit int for RMP (https://github.com/epics-modules/motor/issues/8,
https://epics.anl.gov/tech-talk/2018/msg00087.php) which effectively limits the travel
range to +/- 2.1mm.
Since it doesn't seem the motor record will update to using 64bit int the choices I can see are:
* 1 - using a non-standard motor support
* 2 - rescaling the minimum resolution to 1nm to effectively increase the range to 2.1m

I chose option 2.
1 step = 1nm

Someone with more experience may have a better solution.

Note on controller capability:
The controller supports many more sophisticated features than are supported in this driver.
The two that may be of significant interest are:
  * TTL triggering at specified positions
  * "scan" mode where the piezo stick slip can flex up to 1.6micron to give
     very precise and fast motion

*/

#include "asynMotorController.h"
#include "asynMotorAxis.h"

/* This is the same for lin and rot positioners
 * lin: controller pm --> driver nm. Because of this the user can use the positioner for mm ranges
 * rot: controller ndeg --> driver udeg. Because of this the user can use the positioner for deg ranges
 * If this scaling was not implemented the maximum range would be ~2.147 mm/deg, now it's ~2147 mm/deg */
#define PULSES_PER_STEP 1000

/** MCS2 Axis status flags **/
#define CH_STATE_ACTIVELY_MOVING         0x0001
#define CH_STATE_CLOSED_LOOP_ACTIVE      0x0002
#define CH_STATE_CALIBRATING             0x0004
#define CH_STATE_REFERENCING             0x0008
#define CH_STATE_MOVE_DELAYED            0x0010
#define CH_STATE_SENSOR_PRESENT          0x0020
#define CH_STATE_IS_CALIBRATED           0x0040
#define CH_STATE_IS_REFERENCED           0x0080
#define CH_STATE_END_STOP_REACHED        0x0100
#define CH_STATE_RANGE_LIMIT_REACHED     0x0200
#define CH_STATE_FOLLOWING_LIMIT_REACHED 0x0400
#define CH_STATE_MOVEMENT_FAILED         0x0800
#define CH_STATE_STREAMING               0x1000
#define CH_STATE_POSITIONER_OVERLOAD     0x2000
#define CH_STATE_OVERTEMP                0x4000
#define CH_STATE_REFERENCE_MARK          0x8000
#define CH_STATE_IS_PHASED           0x00010000
#define CH_STATE_POSITIONER_FAULT    0x00020000
#define CH_STATE_AMPLIFIER_ENABLED   0x00040000
#define CH_STATE_IN_POSITION         0x00080000
#define CH_STATE_BRAKE_ENABLED       0x00100000

/** MCS2 Axis reference options **/
const unsigned short   START_DIRECTION         = 0x0001;
const unsigned short   REVERSE_DIRECTION       = 0x0002;
const unsigned short   AUTO_ZERO               = 0x0004;
const unsigned short   ABORT_ON_END_STOP       = 0x0008;
const unsigned short   CONTINUE_ON_REF_FOUND   = 0x0010;
const unsigned short   STOP_ON_REF_FOUND       = 0x0020;

/** MCS2 Axis constants **/
#define HOLD_FOREVER 0xffffffff
#define MAX_FREQUENCY 20000

/** drvInfo strings for extra parameters that the MCS2 controller supports */
#define MCS2MclfString "MCLF"
#define MCS2PtypString "PTYP"
#define MCS2PtypRbString "PTYP_RB"
#define MCS2PstatString "PSTAT"
#define MCS2RefString "REF"
#define MCS2CalString "CAL"
#define MCS2FReadbackString "FREADBACK"
#define MCS2ErrTxtString "ErrTxt"
#define MCS2OpenloopString "OPENLOOP"
#define MCS2STEPFREQString "STEPFREQ"
#define MCS2STEPCNTString "STEPCNT"
#define MCS2STEPSIZEFString "STEPSIZEF"
#define MCS2STEPSIZERString "STEPSIZER"
#define MCS2SensorPowerModeString "SensorPowerMode"
#define MCS2SensorDelayString "SensorDelay"
#define MCS2HoldString "HOLD"

class epicsShareClass MCS2Axis : public asynMotorAxis
{
public:
  /* These are the methods we override from the base class */
  MCS2Axis(class MCS2Controller *pC, int axis);
  void report(FILE *fp, int level);
  asynStatus poll(bool *moving);
  asynStatus move(double position, int relative, double min_velocity, double max_velocity, double acceleration);
  asynStatus home(double min_velocity, double max_velocity, double acceleration, int forwards);
  asynStatus stop(double acceleration);
  asynStatus setPosition(double position);
  asynStatus setClosedLoop(bool closedLoop);
  asynStatus setIntegerParam(int function, epicsInt32 value);
  asynStatus setDoubleParam(int function, double value);


private:
  MCS2Controller *pC_;      /**< Pointer to the asynMotorController to which this axis belongs.
                                *   Abbreviated because it is used very frequently */
  int sensorPresent_;
  double stepTargetPos_nm_;
  double stepTargetSteps_;
  //asynStatus comStatus_;
  int initialPollDone_;
  int openLoop_;
  int sensorIsDisabled_;
  double stepsizef_;
  double stepsizer_;
  asynStatus initialPoll(void);
  asynStatus reportHelperCheckError(const char *scpi_leaf, char *input, size_t maxChars);
#define REPORTHELPERCHECKERROR(a,b) reportHelperCheckError(a,b,sizeof(b))
  asynStatus reportHelperInteger(const char *scpi_leaf, int *pResult);
  asynStatus reportHelperDouble(const char *scpi_leaf, double *pResult);

friend class MCS2Controller;
};

class epicsShareClass MCS2Controller : public asynMotorController {
public:
  MCS2Controller(const char *portName, const char *MCS2PortName, int numAxes, double movingPollPeriod, double idlePollPeriod, int unusedMask = 0);
  void handleStatusChange(asynStatus status);
  asynStatus writeReadHandleDisconnect(void);
  virtual asynStatus clearErrors(void);
  virtual void printOneError(int errorCode);

  /* These are the methods that we override from asynMotorDriver */
  void report(FILE *fp, int level);
  MCS2Axis* getAxis(asynUser *pasynUser);
  MCS2Axis* getAxis(int axisNo);

protected:
  asynStatus oldStatus_;
  int mclf_; /**< MCL frequency */
#define FIRST_MCS2_PARAM mclf_
  int ptyp_; /**< positioner type */
  int ptyprb_; /**< positioner type readback */
  int pstatrb_; /**< positoner status word readback */
  int ref_;  /**< reference command */
  int cal_;  /**< calibration command */
  int freadback_; /** readback in picometer as floating point*/
  int errTxt_;
  int openLoop_;
  int sensorIsDisabled_ ;
  int stepfreq_; /** step frequency */ /* 1 .. 20000 */
  int stepcnt_;  /** step count (to move) */ /* -100000 .. + 100000 */
  int stepsizef_; /** size of an open loop step, forward, in pm */
  int stepsizer_; /** size of an open loop step, reverse==backward, in pm */
  int sensorPowerMode_; /** Sensor power mode */
  int sensorDelay_; /** Sensor power save delay */
  int hold_; /** hold time */

#define LAST_MCS2_PARAM hold_
#define NUM_MCS2_PARAMS (&LAST_MCS2_PARAM - &FIRST_MCS2_PARAM + 1)

friend class MCS2Axis;
};

