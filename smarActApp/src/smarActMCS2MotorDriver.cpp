/*
FILENAME... SmarActMCS2MotorDriver.cpp
USAGE...    Motor driver support for the SmarAct MCS2 controller.

David Vine
Based on the ACR driver of Mark Rivers.
Jan 19, 2019

*/

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include <iocsh.h>
#include <epicsThread.h>

#include <asynOctetSyncIO.h>

#include "asynMotorController.h"
#include "asynMotorAxis.h"

#include <epicsExport.h>
#include "smarActMCS2MotorDriver.h"

static const char *driverName = "SmarActMCS2MotorDriver";

/** Creates a new MCS2Controller object.
 * \param[in] portName             The name of the asyn port that will be created for this driver
 * \param[in] MCS2PortName         The name of the drvAsynIPPPort that was created previously to connect to the MCS2 controller
 * \param[in] numAxes              The number of axes that this controller supports
 * \param[in] movingPollPeriod     The time between polls when any axis is moving
 * \param[in] idlePollPeriod       The time between polls when no axis is moving
 */
MCS2Controller::MCS2Controller(const char *portName, const char *MCS2PortName, int numAxes,
                               double movingPollPeriod, double idlePollPeriod, int unusedMask)
    : asynMotorController(portName, numAxes, NUM_MCS2_PARAMS,
                          0, 0,
                          ASYN_CANBLOCK | ASYN_MULTIDEVICE,
                          1,    // autoconnect
                          0, 0) // Default priority and stack size
{
  int axis, axisMask = 0;
  asynStatus status;
  static const char *functionName = "MCS2Controller";
  asynPrint(this->pasynUserSelf, ASYN_TRACEIO_DRIVER, "MCS2Controller::MCS2Controller: Creating controller\n");

  // Create controller-specific parameters
  createParam(MCS2MclfString, asynParamInt32, &this->mclf_);
  createParam(MCS2PtypString, asynParamInt32, &this->ptyp_);
  createParam(MCS2PtypRbString, asynParamInt32, &this->ptyprb_);
  createParam(MCS2PstatString, asynParamInt32, &this->pstatrb_); // whole positioner status word
  createParam(MCS2RefString, asynParamInt32, &this->ref_);
  createParam(MCS2CalString, asynParamInt32, &this->cal_);

  // Connect to MCS2 controller
  status = pasynOctetSyncIO->connect(MCS2PortName, 0, &pasynUserController_, NULL);
  pasynOctetSyncIO->setInputEos(pasynUserController_, "\r\n", 2);
  pasynOctetSyncIO->setOutputEos(pasynUserController_, "\r\n", 2);

  asynPrint(this->pasynUserSelf, ASYN_TRACEIO_DRIVER, "MCS2Controller::MCS2Controller: Connecting to controller\n");
  if (status) {
    asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
              "%s:%s: cannot connect to MCS2 controller\n",
              driverName, functionName);
  }
  asynPrint(this->pasynUserSelf, ASYN_TRACEIO_DRIVER, "MCS2Controller::MCS2Controller: Clearing error messages\n");
  this->clearErrors();

  sprintf(this->outString_, ":DEV:SNUM?");
  status = this->writeReadController();
  if (status) {
    asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
              "%s:%s: cannot connect to MCS2 controller\n",
              driverName, functionName);
  }
  asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, "MCS2Controller::MCS2Controller: Device Name: %s\n", this->inString_);
  this->clearErrors();

  // Create the axis objects
  asynPrint(this->pasynUserSelf, ASYN_TRACEIO_DRIVER, "MCS2Controller::MCS2Controller: Creating axes\n");

  for(axis=0; axis<numAxes; axis++){ 
    axisMask = (unusedMask & (1 << axis)) >> axis;
    if (!axisMask)
      new MCS2Axis(this, axis);
  }

  startPoller(movingPollPeriod, idlePollPeriod, 2);
}

/** Creates a new MCS2Controller object.
 * Configuration command, called directly or from iocsh
 * \param[in] portName          The name of the asyn port that will be created for this driver
 * \param[in] MCS2PortName      The name of the drvAsynIPPPort that was created previously to connect to the MCS2 controller
 * \param[in] numAxes           The number of axes that this controller supports
 * \param[in] movingPollPeriod  The time in ms between polls when any axis is moving
 * \param[in] idlePollPeriod    The time in ms between polls when no axis is moving
 */
extern "C" int MCS2CreateController(const char *portName, const char *MCS2PortName, int numAxes,
                                    int movingPollPeriod, int idlePollPeriod, int unusedMask)
{
  new MCS2Controller(portName, MCS2PortName, numAxes, movingPollPeriod / 1000., idlePollPeriod / 1000., unusedMask);
  return (asynSuccess);
}

asynStatus MCS2Controller::clearErrors()
{

  asynStatus comStatus;
  int numErrorMsgs;
  char errorMsg[50];
  int errorCode;

  // Read out error messages
  sprintf(this->outString_, ":SYST:ERR:COUN?");
  comStatus = this->writeReadController();
  if (comStatus) goto skip;
  numErrorMsgs = atoi(this->inString_);
  for (int i=0; i<numErrorMsgs; i++){
    sprintf(this->outString_, ":SYST:ERR?");
    comStatus = this->writeReadController();
	  if (comStatus) goto skip;
    printf("%s", this->inString_);
    errorCode = atoi(this->inString_);
	  switch (errorCode){
    case 259:    sprintf(errorMsg, "No sensor present");
      break;
    case 34:     sprintf(errorMsg, "Invalid channel index");
      break;
    case 0:      sprintf(errorMsg, "No error");
      break;
    case -101:   sprintf(errorMsg, "Invalid character");
      break;
    case -103:   sprintf(errorMsg, "Invalid seperator");
      break;
    case -104:   sprintf(errorMsg, "Data type error");
      break;
    case -108:   sprintf(errorMsg, "Parameter not allowed");
      break;
    case -109:   sprintf(errorMsg, "Missing parameter");
      break;
    case -113:   sprintf(errorMsg, "Command not exist");
      break;
    case -151:   sprintf(errorMsg, "Invalid string");
      break;
    case -350:   sprintf(errorMsg, "Queue overflow");
      break;
    case -363:   sprintf(errorMsg, "Buffer overrun");
      break;
    default:     sprintf(errorMsg, "Unable to decode %d", errorCode);
      break;
    }
    asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
              "MCS2Controller::clearErrors: %s\n", errorMsg);
  }

skip:
  setIntegerParam(this->motorStatusProblem_, comStatus ? 1 : 0);
  callParamCallbacks();
  return comStatus ? asynError : asynSuccess;
}

/** Reports on status of the driver
 * \param[in] fp The file pointer on which report information will be written
 * \param[in] level The level of report detail desired
 *
 * If details > 0 then information is printed about each axis.
 * After printing controller-specific information calls asynMotorController::report()
 */
void MCS2Controller::report(FILE *fp, int level)
{
  fprintf(fp, "MCS2 motor driver %s, numAxes=%d, moving poll period=%f, idle poll period=%f\n",
          this->portName, numAxes_, movingPollPeriod_, idlePollPeriod_);

  // Call the base class method
  asynMotorController::report(fp, level);
}

/** Returns a pointer to an MCS2MotorAxis object.
 * Returns NULL if the axis number encoded in pasynUser is invalid.
 * \param[in] pasynUser asynUser structure that encodes the axis index number. */
MCS2Axis *MCS2Controller::getAxis(asynUser *pasynUser)
{
  return static_cast<MCS2Axis *>(asynMotorController::getAxis(pasynUser));
}

/** Returns a pointer to an MCS2MotorAxis object.
 * Returns NULL if the axis number encoded in pasynUser is invalid.
 * \param[in] axisNo Axis index number. */
MCS2Axis *MCS2Controller::getAxis(int axisNo)
{
  return static_cast<MCS2Axis *>(asynMotorController::getAxis(axisNo));
}

/** Called when asyn clients call pasynInt32->write().
 * Extracts the function and axis number from pasynUser.
 * Sets the value in the parameter library.
 * For all other functions it calls asynMotorController::writeInt32.
 * Calls any registered callbacks for this pasynUser->reason and address.
 * \param[in] pasynUser asynUser structure that encodes the reason and address.
 * \param[in] value     Value to write. */
asynStatus MCS2Controller::writeInt32(asynUser *pasynUser, epicsInt32 value)
{
  int function = pasynUser->reason;
  asynStatus status = asynSuccess;
  MCS2Axis *pAxis = getAxis(pasynUser);
  static const char *functionName = "writeInt32";

  // Check if axis exists
  if(!pAxis) return asynError;

  /* Set the parameter and readback in the parameter library.  This may be overwritten when we read back the
   * status at the end, but that's OK */
  status = setIntegerParam(pAxis->axisNo_, function, value);

  if (function == mclf_) {
    // set MCLF
    sprintf(pAxis->pC_->outString_, ":CHAN%d:MCLF:CURR %d", pAxis->axisNo_, value);
    status = pAxis->pC_->writeController();
  }
  else if (function == ptyp_) {
    // set positioner type
    sprintf(pAxis->pC_->outString_, ":CHAN%d:PTYP %d", pAxis->axisNo_, value);
    status = pAxis->pC_->writeController();
  }
  else if (function == cal_) {
    // send calibration command
    sprintf(pAxis->pC_->outString_, ":CAL%d", pAxis->axisNo_);
    status = pAxis->pC_->writeController();
  }
  else {
    // Call base class method
    status = asynMotorController::writeInt32(pasynUser, value);
  }

  // Do callbacks so higher layers see any changes
  callParamCallbacks(pAxis->axisNo_);
  if (status)
    asynPrint(pasynUser, ASYN_TRACE_ERROR,
              "%s:%s: error, status=%d function=%d, value=%d\n",
              driverName, functionName, status, function, value);
  else
    asynPrint(pasynUser, ASYN_TRACEIO_DRIVER,
              "%s:%s: function=%d, value=%d\n",
              driverName, functionName, function, value);
  return status;
}

// These are the MCS2Axis methods

/** Creates a new MCS2Axis object.
 * \param[in] pC Pointer to the ACRController to which this axis belongs.
 * \param[in] axisNo Index number of this axis, range 0 to pC->numAxes_-1.
 *
 * Initializes register numbers, etc.
 */
MCS2Axis::MCS2Axis(MCS2Controller *pC, int axisNo)
    : asynMotorAxis(pC, axisNo),
      pC_(pC)
{
  // asynStatus status;

  asynPrint(pC->pasynUserSelf, ASYN_TRACEIO_DRIVER, "MCS2Axis::MCS2Axis: Creating axis %u\n", axisNo);
  channel_ = axisNo;

  // Set hold time
  sprintf(pC_->outString_, ":CHAN%d:HOLD %d", channel_, HOLD_FOREVER);
  /*status = */ pC_->writeController();
  pC_->clearErrors();
  callParamCallbacks();
}

/** Reports on status of the driver
 * \param[in] fp The file pointer on which report information will be written
 * \param[in] level The level of report detail desired
 *
 * If details > 0 then information is printed about each axis.
 * After printing controller-specific information calls asynMotorController::report()
 */
void MCS2Axis::report(FILE *fp, int level)
{
  if (level > 0) {
    int pcode;
    char pname[256];
    int channelState;
    int vel;
    int acc;
    int mclf;
    int followError;
    int error;
    int temp;

    asynStatus status;

    sprintf(pC_->outString_, ":CHAN%d:PTYP?", channel_);
    status = pC_->writeReadController();
    pcode = atoi(pC_->inString_);
    sprintf(pC_->outString_, ":CHAN%d:PTYP:NAME?", channel_);
    status = pC_->writeReadController();
    strcpy(pC_->inString_, pname);
    sprintf(pC_->outString_, ":CHAN%d:STAT?", channel_);
    status = pC_->writeReadController();
    channelState = atoi(pC_->inString_);
    sprintf(pC_->outString_, ":CHAN%d:VEL?", channel_);
    status = pC_->writeReadController();
    vel = atoi(pC_->inString_);
    sprintf(pC_->outString_, ":CHAN%d:ACC?", channel_);
    status = pC_->writeReadController();
    acc = atoi(pC_->inString_);
    sprintf(pC_->outString_, ":CHAN%d:MCLF?", channel_);
    status = pC_->writeReadController();
    mclf = atoi(pC_->inString_);
    sprintf(pC_->outString_, ":CHAN%d:FERR?", channel_);
    status = pC_->writeReadController();
    followError = atoi(pC_->inString_);
    sprintf(pC_->outString_, ":CHAN%d:ERR?", channel_);
    status = pC_->writeReadController();
    error = atoi(pC_->inString_);
    sprintf(pC_->outString_, ":CHAN%d:TEMP?", channel_);
    status = pC_->writeReadController();
    temp = atoi(pC_->inString_);

    if (status)
      fprintf(fp, "  axis %d -> asyn status error %d\n", axisNo_, status);
    fprintf(fp, "  axis %d\n"
                "    positioner type %d\n"
                "    positioner name %s\n"
                "    state %d\n"
                "    velocity %d\n"
                "    acceleration %d\n"
                "    max closed loop frequency %d\n"
                "    following error %d\n"
                "    error %d\n"
                "    temp %d\n",
            axisNo_, pcode, pname, channelState, vel,
            acc, mclf, followError, error, temp);
    pC_->clearErrors();
  }

  // Call the base class method
  asynMotorAxis::report(fp, level);
}

asynStatus MCS2Axis::move(double position, int relative, double minVelocity, double maxVelocity, double acceleration)
{
  asynStatus status = asynSuccess;

  /* MCS2 move mode is:
   *	- absolute=0
   *	- relative=1
   *	- step=4
   */
  if(sensorPresent_) {
    // closed loop move
    // Set hold time
    sprintf(pC_->outString_, ":CHAN%d:MMOD %d", channel_, relative > 0 ? 1 : 0);
    status = pC_->writeController();
    // Set acceleration
    sprintf(pC_->outString_, ":CHAN%d:ACC %f", channel_, acceleration * PULSES_PER_STEP);
    status = pC_->writeController();
    // Set velocity
    sprintf(pC_->outString_, ":CHAN%d:VEL %f", channel_, maxVelocity * PULSES_PER_STEP);
    status = pC_->writeController();
    // Do move
    sprintf(pC_->outString_, ":MOVE%d %f", channel_, position * PULSES_PER_STEP);
    status = pC_->writeController();
  }
  else {
    // open loop move
    PositionType dtg = position - stepTarget_; // distance to go
    stepTarget_ = (PositionType)position;      // store position in global scope
    // Set mode; 4 == STEP
    sprintf(pC_->outString_, ":CHAN%d:MMOD 4", channel_);
    status = pC_->writeController();
    // Set frequency; range 1..20000 Hz
    unsigned short frequency = (unsigned short)maxVelocity;
    if(frequency >= MAX_FREQUENCY) {
      frequency = MAX_FREQUENCY;
    }
    sprintf(pC_->outString_, ":CHAN%d:STEP:FREQ %u", channel_, frequency);
    status = pC_->writeController();
    // Do move
    sprintf(pC_->outString_, ":MOVE%d %lld", channel_, dtg);
    status = pC_->writeController();
    setDoubleParam(pC_->motorPosition_, (double)stepTarget_);
  }

  return status;
}

asynStatus MCS2Axis::home(double minVelocity, double maxVelocity, double acceleration, int forwards)
{
  asynStatus status = asynSuccess;
  printf("Home command received %d\n", forwards);
  unsigned short refOpt = 0;

  if (forwards==0){
    refOpt |= START_DIRECTION;
  }
  refOpt |= AUTO_ZERO;

  // Set default reference options - direction and autozero
  printf("ref opt: %d\n", refOpt);
  sprintf(pC_->outString_, ":CHAN%d:REF:OPT %d", channel_, refOpt);
  status = pC_->writeController();
  pC_->clearErrors();

  // Set hold time
  sprintf(pC_->outString_, ":CHAN%d:HOLD %d", channel_, HOLD_FOREVER);
  status = pC_->writeController();
  pC_->clearErrors();
  // Set acceleration
  sprintf(pC_->outString_, ":CHAN%d:ACC %f", channel_, acceleration * PULSES_PER_STEP);
  status = pC_->writeController();
  pC_->clearErrors();
  // Set velocity
  sprintf(pC_->outString_, ":CHAN%d:VEL %f", channel_, maxVelocity * PULSES_PER_STEP);
  status = pC_->writeController();
  pC_->clearErrors();
  // Begin move
  sprintf(pC_->outString_, ":REF%d", channel_);
  status = pC_->writeController();
  pC_->clearErrors();

  return status;
}

asynStatus MCS2Axis::stop(double acceleration)
{
  asynStatus status;

  sprintf(pC_->outString_, ":STOP%d", channel_);
  status = pC_->writeController();

  return status;
}

asynStatus MCS2Axis::setPosition(double position)
{
  asynStatus status = asynSuccess;

  printf("Set position receieved\n");
  sprintf(pC_->outString_, ":CHAN%d:POS %f", channel_, position * PULSES_PER_STEP);
  status = pC_->writeController();
  return status;
}

/** Polls the axis.
 * This function reads the controller position, encoder position, the limit status, the moving status,
 * the drive power-on status and positioner type. It does not current detect following error, etc.
 * but this could be added.
 * It calls setIntegerParam() and setDoubleParam() for each item that it polls,
 * and then calls callParamCallbacks() at the end.
 * \param[out] moving A flag that is set indicating that the axis is moving (1) or done (0). */
asynStatus MCS2Axis::poll(bool *moving)
{
  int done;
  int chanState;
  int closedLoop;
  int sensorPresent;
  int isCalibrated;
  int isReferenced;
  int endStopReached;
  int followLimitReached;
  int movementFailed;
  int refMark;
  int positionerType;
  double encoderPosition;
  double theoryPosition;
  int driveOn;
  int mclf;
  asynStatus comStatus = asynSuccess;

  // Read the channel state
  sprintf(pC_->outString_, ":CHAN%d:STAT?", channel_);
  comStatus = pC_->writeReadController();
  if (comStatus) goto skip;
  chanState = atoi(pC_->inString_);
  setIntegerParam(pC_->pstatrb_, chanState);
  done = (chanState & ACTIVELY_MOVING) ? 0 : 1;
  closedLoop = (chanState & CLOSED_LOOP_ACTIVE) ? 1 : 0;
  sensorPresent = (chanState & SENSOR_PRESENT) ? 1 : 0;
  isCalibrated = (chanState & IS_CALIBRATED) ? 1 : 0;
  isReferenced = (chanState & IS_REFERENCED) ? 1 : 0;
  endStopReached = (chanState & END_STOP_REACHED) ? 1 : 0;
  followLimitReached = (chanState & FOLLOWING_LIMIT_REACHED) ? 1 : 0;
  movementFailed = (chanState & MOVEMENT_FAILED) ? 1 : 0;
  refMark = (chanState & REFERENCE_MARK) ? 1 : 0;

  *moving = done ? false : true;
  setIntegerParam(pC_->motorStatusDone_, done);
  setIntegerParam(pC_->motorClosedLoop_, closedLoop);
  setIntegerParam(pC_->motorStatusHasEncoder_, sensorPresent);
  setIntegerParam(pC_->motorStatusGainSupport_, sensorPresent);
  setIntegerParam(pC_->motorStatusHomed_, isReferenced);
  setIntegerParam(pC_->motorStatusHighLimit_, endStopReached);
  setIntegerParam(pC_->motorStatusLowLimit_, endStopReached);
  setIntegerParam(pC_->motorStatusFollowingError_, followLimitReached);
  setIntegerParam(pC_->motorStatusProblem_, movementFailed);
  setIntegerParam(pC_->motorStatusAtHome_, refMark);

  // Read the current encoder position, if the positioner has a sensor
  sensorPresent_ = sensorPresent;
  if(sensorPresent){
    sprintf(pC_->outString_, ":CHAN%d:POS?", channel_);
    comStatus = pC_->writeReadController();
    if (comStatus) goto skip;
    encoderPosition = (double)strtod(pC_->inString_, NULL);
    encoderPosition /= PULSES_PER_STEP;
    setDoubleParam(pC_->motorEncoderPosition_, encoderPosition);

    // Read the current theoretical position
    sprintf(pC_->outString_, ":CHAN%d:POS:TARG?", channel_);
    comStatus = pC_->writeReadController();
    if (comStatus) goto skip;
    theoryPosition = (double)strtod(pC_->inString_, NULL);
    theoryPosition /= PULSES_PER_STEP;
    setDoubleParam(pC_->motorPosition_, theoryPosition);
  }

  // Read the drive power on status
  sprintf(pC_->outString_, ":CHAN%d:AMPL?", channel_);
  comStatus = pC_->writeReadController();
  if (comStatus) goto skip;
  driveOn = atoi(pC_->inString_) ? 1 : 0;
  setIntegerParam(pC_->motorStatusPowerOn_, driveOn);
  setIntegerParam(pC_->motorStatusProblem_, 0);

  // Read the currently selected positioner type
  sprintf(pC_->outString_, ":CHAN%d:PTYP?", channel_);
  comStatus = pC_->writeReadController();
  if (comStatus) goto skip;
  positionerType = atoi(pC_->inString_);
  setIntegerParam(pC_->ptyprb_, positionerType);

  // Read CAL/REF status and MCLF when idle
  if (done)
  {
    setIntegerParam(pC_->cal_, isCalibrated);
    setIntegerParam(pC_->ref_, isReferenced);
    sprintf(pC_->outString_, ":CHAN%d:MCLF?", channel_);
    comStatus = pC_->writeReadController();
        if (comStatus) goto skip;
    mclf = atoi(pC_->inString_);
    setIntegerParam(pC_->mclf_, mclf);
  }

skip:
  setIntegerParam(pC_->motorStatusProblem_, comStatus ? 1 : 0);
  callParamCallbacks();
  return comStatus ? asynError : asynSuccess;
}

/** Code for iocsh registration */
static const iocshArg MCS2CreateControllerArg0 = {"Port name", iocshArgString};
static const iocshArg MCS2CreateControllerArg1 = {"MCS2 port name", iocshArgString};
static const iocshArg MCS2CreateControllerArg2 = {"Number of axes", iocshArgInt};
static const iocshArg MCS2CreateControllerArg3 = {"Moving poll period (ms)", iocshArgInt};
static const iocshArg MCS2CreateControllerArg4 = {"Idle poll period (ms)", iocshArgInt};
static const iocshArg MCS2CreateControllerArg5 = {"Unused bit mask", iocshArgInt};
static const iocshArg *const MCS2CreateControllerArgs[] = {&MCS2CreateControllerArg0,
                                                           &MCS2CreateControllerArg1,
                                                           &MCS2CreateControllerArg2,
                                                           &MCS2CreateControllerArg3,
                                                           &MCS2CreateControllerArg4,
                                                           &MCS2CreateControllerArg5};
static const iocshFuncDef MCS2CreateControllerDef = {"MCS2CreateController", 6, MCS2CreateControllerArgs};
static void MCS2CreateContollerCallFunc(const iocshArgBuf *args)
{
  MCS2CreateController(args[0].sval, args[1].sval, args[2].ival, args[3].ival, args[4].ival, args[5].ival);
}

static void MCS2MotorRegister(void)
{
  iocshRegister(&MCS2CreateControllerDef, MCS2CreateContollerCallFunc);
}

extern "C" {
  epicsExportRegistrar(MCS2MotorRegister);
}
