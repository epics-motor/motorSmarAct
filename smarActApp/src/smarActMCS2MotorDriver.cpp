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

/* ESS defined one bit for INFO. Tried to get that upstream, but failed */
#ifndef ASYN_TRACE_INFO
#define ASYN_TRACE_INFO 0x0040
#endif

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
  :  asynMotorController(portName, numAxes, NUM_MCS2_PARAMS,
#ifdef SMARACT_ASYN_ASYNPARAMINT64
                         asynInt64Mask |
#endif
                         0,
#ifdef SMARACT_ASYN_ASYNPARAMINT64
                         asynInt64Mask |
#endif
                         0,
                         ASYN_CANBLOCK | ASYN_MULTIDEVICE,
                         1, // autoconnect
                         0, 0)  // Default priority and stack size
{
  int axis, axisMask = 0;
  asynStatus status;
  static const char *functionName = "MCS2Controller";
  asynPrint(this->pasynUserSelf, ASYN_TRACEIO_DRIVER, "MCS2Controller::MCS2Controller: Creating controller\n");

  // Create controller-specific parameters
  createParam(MCS2MclfString, asynParamInt32, &this->mclf_);
  createParam(MCS2PtypString, asynParamInt32, &this->ptyp_);
  createParam(MCS2PtypRbString, asynParamInt32, &this->ptyprb_);
  createParam(MCS2PstatString, asynParamInt32, &this->pstatrb_);   // whole positioner status word
  createParam(MCS2RefString, asynParamInt32, &this->ref_);
  createParam(MCS2CalString, asynParamInt32, &this->cal_);
  createParam(MCS2FReadbackString, asynParamFloat64, &this->freadback_);
#ifdef SMARACT_ASYN_ASYNPARAMINT64
  createParam(MCS2IReadbackString, asynParamInt64, &this->ireadback_);
#else
  this->ireadback_ = -1;
#endif
  createParam(MCS2ErrTxtString, asynParamOctet, &this->errTxt_);

  createParam(MCS2HoldString, asynParamInt32, &this->hold_);
  createParam(MCS2OpenloopString, asynParamInt32, &this->openLoop_);
  createParam(MCS2STEPFREQString, asynParamInt32, &this->stepfreq_);
  createParam(MCS2STEPCNTString,  asynParamInt32, &this->stepcnt_);
  createParam(MCS2STEPSIZEFString, asynParamFloat64, &this->stepsizef_);
  createParam(MCS2STEPSIZERString, asynParamFloat64, &this->stepsizer_);

  /* Connect to MCS2 controller */
  status = pasynOctetSyncIO->connect(MCS2PortName, 0, &pasynUserController_, NULL);
  pasynOctetSyncIO->setInputEos (pasynUserController_, "\r\n", 2);
  pasynOctetSyncIO->setOutputEos(pasynUserController_, "\r\n", 2);

  asynPrint(this->pasynUserSelf, ASYN_TRACEIO_DRIVER, "MCS2Controller::MCS2Controller: Connecting to controller\n");
  if (status) {
    asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
      "%s:%s: cannot connect to MCS2 controller\n",
      driverName, functionName);
  }
  asynPrint(this->pasynUserSelf, ASYN_TRACEIO_DRIVER, "MCS2Controller::MCS2Controller: Clearing error messages\n");
  this->clearErrors();

  oldStatus_ = asynError;
  snprintf(this->outString_, sizeof(this->outString_)-1,":DEV:SNUM?");
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
    if(!axisMask)
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
  new MCS2Controller(portName, MCS2PortName, numAxes, movingPollPeriod/1000., idlePollPeriod/1000., unusedMask);
  return(asynSuccess);
}

extern "C" const char *mcs2AsynStatusToString(asynStatus status) {
  switch ((int)status) {
    case asynSuccess:
      return "asynSuccess";
    case asynTimeout:
      return "asynTimeout";
    case asynOverflow:
      return "asynOverflow";
    case asynError:
      return "asynError";
    case asynDisconnected:
      return "asynDisconnected";
    case asynDisabled:
      return "asynDisabled";
    case asynParamAlreadyExists:
      return "asynParamAlreadyExists";
    case asynParamNotFound:
      return "asynParamNotFound";
    case asynParamWrongType:
      return "asynParamWrongType";
    case asynParamBadIndex:
      return "asynParamBadIndex";
    case asynParamUndefined:
      return "asynParamUndefined";
    default:
      return "??";
  }
}

asynStatus MCS2Controller::writeReadHandleDisconnect(void)
{
  asynStatus status = asynError;
  inString_[0] = '\0';
  status = writeReadController();
  handleStatusChange(status);
  if (status) {
    return asynError;
  }
  return status;
}

void MCS2Controller::handleStatusChange(asynStatus status) {
  static const char *functionName = "handleStatusChange";
  if (status != oldStatus_) {
    asynPrint(
        pasynUserController_, ASYN_TRACE_INFO,
        "%s oldStatus=%s(%d) newStatus=%s(%d)\n",
        functionName , mcs2AsynStatusToString(oldStatus_),
        (int)oldStatus_, mcs2AsynStatusToString(status), (int)status);
    if (status) {
      /* Connected -> Disconnected */
      int axisNo;
      // setAlarmStatusSeverityAllReadbacks(asynDisconnected);
      for (axisNo = 0; axisNo < numAxes_; axisNo++) {
        asynMotorAxis *pAxis = getAxis(axisNo);
        if (!pAxis) continue;
        pAxis->asynMotorAxis::setIntegerParam(motorStatusCommsError_, 1);
        pAxis->callParamCallbacks();
      }
    } else {
      /* Disconnected -> Connected */
    }
    oldStatus_ = status;
  }
  callParamCallbacks();
}

asynStatus MCS2Controller::clearErrors()
{

  asynStatus comStatus;
  int numErrorMsgs;
  char errorMsg[50];
  int errorCode;

  // Read out error messages
  snprintf(this->outString_, sizeof(this->outString_)-1,":SYST:ERR:COUN?");
  comStatus = this->writeReadController();
  if (comStatus) goto skip;
  numErrorMsgs = atoi(this->inString_);
  for (int i=0; i<numErrorMsgs; i++){
    snprintf(this->outString_, sizeof(this->outString_)-1,":SYST:ERR?");
    comStatus = this->writeReadController();
    if (comStatus) goto skip;
    printf("%s", this->inString_);
    errorCode = atoi(this->inString_);
    switch (errorCode){
      case 259:   snprintf(errorMsg,sizeof(errorMsg)-1, "No sensor present");
            break;
      case 34:    snprintf(errorMsg,sizeof(errorMsg)-1, "Invalid channel index");
            break;
      case 0:   snprintf(errorMsg,sizeof(errorMsg)-1, "No error");
            break;
      case -101:  snprintf(errorMsg,sizeof(errorMsg)-1, "Invalid character");
            break;
      case -103:  snprintf(errorMsg,sizeof(errorMsg)-1, "Invalid seperator");
            break;
      case -104:  snprintf(errorMsg,sizeof(errorMsg)-1, "Data type error");
            break;
      case -108:  snprintf(errorMsg,sizeof(errorMsg)-1, "Parameter not allowed");
            break;
      case -109:    snprintf(errorMsg,sizeof(errorMsg)-1, "Missing parameter");
            break;
      case -113:  snprintf(errorMsg,sizeof(errorMsg)-1, "Command not exist");
            break;
      case -151:  snprintf(errorMsg,sizeof(errorMsg)-1, "Invalid string");
            break;
      case -350:  snprintf(errorMsg,sizeof(errorMsg)-1, "Queue overflow");
            break;
      case -363:  snprintf(errorMsg,sizeof(errorMsg)-1, "Buffer overrun");
            break;
      default:      snprintf(errorMsg,sizeof(errorMsg)-1, "Unable to decode %d", errorCode);
            break;
    }
    asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
        "MCS2Controller::clearErrors: %s\n", errorMsg);
  }

  skip:
  {
    int axisNo;
    for (axisNo = 0; axisNo < numAxes_; axisNo++) {
      asynMotorAxis *pAxis = getAxis(axisNo);
      if (!pAxis) continue;
      pAxis->asynMotorAxis::setIntegerParam(motorStatusCommsError_, 1);
    }
  }
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
MCS2Axis* MCS2Controller::getAxis(asynUser *pasynUser)
{
  return static_cast<MCS2Axis*>(asynMotorController::getAxis(pasynUser));
}

/** Returns a pointer to an MCS2MotorAxis object.
  * Returns NULL if the axis number encoded in pasynUser is invalid.
  * \param[in] axisNo Axis index number. */
MCS2Axis* MCS2Controller::getAxis(int axisNo)
{
  return static_cast<MCS2Axis*>(asynMotorController::getAxis(axisNo));
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
  asynPrint(pC->pasynUserSelf, ASYN_TRACEIO_DRIVER, "MCS2Axis::MCS2Axis: Creating axis %u\n", axisNo);
  stepTargetSteps_ = 0;
  stepTargetPos_nm_ = 0.0;
  initialPollDone_ = 0;
  openLoop_ = 0;
  stepsizef_ = 0.0;
  stepsizer_ = 0.0;

  // Set hold time in the parameter database
  asynMotorAxis::setIntegerParam(pC_->hold_, HOLD_FOREVER);
  // Tell motorRecord that CNEN (and PCOV, ICOV, DCOV, which we dont use) work
  asynMotorAxis::setIntegerParam(pC_->motorStatusGainSupport_, 1);
  callParamCallbacks();
}


/** Helper to report()
 */
asynStatus MCS2Axis::reportHelperCheckError(const char *scpi_leaf, char *input, size_t maxChars)
{
  static const char *functionName = "reportHelperCheckError";

  char outString[128];
  size_t nread = 0;
  asynStatus status;
  snprintf(outString, sizeof(outString), ":CHAN%d%s", axisNo_, scpi_leaf);
  memset(input, 0, maxChars);
  status = pC_->writeReadController(outString, input, maxChars, &nread, DEFAULT_CONTROLLER_TIMEOUT);
  if (status == asynTimeout)  {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_ERROR, "%s(%d) outString='%s' input='%s' status=%d\n",
              functionName, axisNo_, outString, input, (int)status);
    pC_->clearErrors();
  } else {
    asynPrint(pC_->pasynUserController_, ASYN_TRACEIO_DRIVER, "%s(%d) outString='%s' input='%s' status=%d\n",
              functionName, axisNo_, outString, input, (int)status);
  }
  return status;
}

asynStatus MCS2Axis::reportHelperInteger(const char *scpi_leaf, int *pResult)
{
  char inString[128];
  asynStatus status = reportHelperCheckError(scpi_leaf, inString, sizeof(inString));
  if (status == asynSuccess) {
    *pResult = atoi(inString);
  }
  return status;
}

asynStatus MCS2Axis::reportHelperDouble(const char *scpi_leaf, double *pResult)
{
  char inString[128];
  asynStatus status = reportHelperCheckError(scpi_leaf, inString, sizeof(inString));
  if (status == asynSuccess) {
    *pResult = atof(inString);
  }
  return status;
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
    int pcode = -1;
    struct {
      char pname[256];
      char rlimit_current_min[32]; /* 27 coluld be enough for a 64 bit int */
      char rlimit_current_max[32]; /* Use 32 to keep buffers aligned */
      char in_position_threshold[32];
      char in_position_delay[32];
      char target_reached_threshold[32];
      char hold_time[32];
      char step_freq[32];
      char step_ampl[32];
      char diag_clf_max[32];
      char diag_clf_aver[32];
    } buf;
    int channelState = -1;
    int vel = -1;
    int acc = -1;
    int mclf = -1;
    int followError = -1;
    int error = -1;
    int temp = -1;

    memset(&buf, 0, sizeof(buf)); /* clear all ASCII buffer in one go */

    reportHelperInteger(":PTYP?", &pcode);
    reportHelperCheckError(":PTYP:NAME?", buf.pname, sizeof(buf.pname));
    reportHelperInteger(":STAT?", &channelState);
    reportHelperInteger(":VEL?", &vel);
    reportHelperInteger(":ACC?", &acc);
    reportHelperInteger(":MCLF?", &mclf);
    reportHelperInteger(":FERR?", &followError);
    reportHelperInteger(":ERR?", &error);
    reportHelperInteger(":TEMP?", &temp);
    REPORTHELPERCHECKERROR(":RLIM:MIN?", buf.rlimit_current_min);
    REPORTHELPERCHECKERROR(":RLIM:MAX?", buf.rlimit_current_max);
    REPORTHELPERCHECKERROR(":INP:THR?", buf.in_position_threshold);
    REPORTHELPERCHECKERROR(":INP:DEL?", buf.in_position_delay);
    REPORTHELPERCHECKERROR(":TUN:THR:TRE?", buf.target_reached_threshold);
    REPORTHELPERCHECKERROR(":HOLD?", buf.hold_time);
    REPORTHELPERCHECKERROR(":STEP:FREQ?", buf.step_freq);
    REPORTHELPERCHECKERROR(":STEP:AMPL?", buf.step_ampl);
    REPORTHELPERCHECKERROR(":DIAG:CLF:MAX?", buf.diag_clf_max);
    REPORTHELPERCHECKERROR(":DIAG:CLF:AVER?", buf.diag_clf_aver);

    fprintf(fp, "  axis %d\n"
                " positioner type %d\n"
                " positioner name %s\n"
                " state %d 0x%X\n"
                " rlimit_current_min %s\n"
                " rlimit_current_max %s\n"
                " in_position_threshold %s\n"
                " in_position_delay %s\n"
                " target_reached_threshold %s\n"
                " hold_time %s\n"
                " step_freq %s\n"
                " step_ampl %s\n"
                " diag_clf_max %s\n"
                " diag_clf_aver %s\n"
                " velocity %d\n"
                " acceleration %d\n"
                " max closed loop frequency %d\n"
                " following error %d\n"
                " error %d\n"
                " temp %d\n",
            axisNo_, pcode, buf.pname, channelState, channelState,
            buf.rlimit_current_min, buf.rlimit_current_max,
            buf.in_position_threshold,
            buf.in_position_delay,
            buf.target_reached_threshold,
            buf.hold_time,
            buf.step_freq,
            buf.step_ampl,
            buf.diag_clf_max,
            buf.diag_clf_aver,
            vel, acc, mclf, followError, error, temp);
    pC_->clearErrors();
  }

  // Call the base class method
  asynMotorAxis::report(fp, level);
}


asynStatus MCS2Axis::move(double position, int relative, double minVelocity, double maxVelocity, double acceleration)
{
  asynStatus status = asynSuccess;
  //static const char *functionName = "move";

  /* MCS2 move mode is:
   *  - absolute=0
   *  - relative=1
   *  - step=4
   */
  unsigned traceMask = ASYN_TRACE_INFO;
  double steps_to_go_f = 0;
  if (relative) {
    steps_to_go_f = position;
    stepTargetPos_nm_ += position;  // store position in global scope
  } else {
    steps_to_go_f = position - stepTargetPos_nm_;
    stepTargetPos_nm_ = position;       // store position in global scope
  }
  asynPrint(pC_->pasynUserController_, traceMask,
            "%smove(%d) position=%f relative=%d sensorPresent=%d openLoop=%d minVelocity=%f maxVelocity=%f"
            " acceleration=%f\n",
            "MCS2Axis::", axisNo_, position, relative, sensorPresent_, openLoop_,
            minVelocity, maxVelocity, acceleration);

  if(sensorPresent_ && !openLoop_) {
    // closed loop move
    snprintf(pC_->outString_,sizeof(pC_->outString_)-1, ":CHAN%d:MMOD %d", axisNo_, relative > 0 ? 1 : 0);
    status = pC_->writeController();
    // Set acceleration
    snprintf(pC_->outString_,sizeof(pC_->outString_)-1, ":CHAN%d:ACC %f", axisNo_, acceleration * PULSES_PER_STEP);
    status = pC_->writeController();
    // Set velocity
    snprintf(pC_->outString_,sizeof(pC_->outString_)-1, ":CHAN%d:VEL %f", axisNo_, maxVelocity * PULSES_PER_STEP);
    status = pC_->writeController();
    // Do move
    snprintf(pC_->outString_,sizeof(pC_->outString_)-1, ":MOVE%d %f", axisNo_, position * PULSES_PER_STEP);
    status = pC_->writeController();
  } else {
    // open loop move
    double frequency = maxVelocity;
    PositionType steps_to_go_i = 0;
    if (stepsizef_ && stepsizer_) {
      /*
       * This is the optional new handling of the driver, allowing a
       * snooth swithing between closed- and open loop.
       * Note that we configure position and velocity (that come to this driver)
       * in nanometer.
       * Closed loop is always picometer towards the controller.
       * Open loop is always steps towards the controller.
       * The effective travelling distance by one step is dependent
       * on the direction, thus 2 variables. step size forward/reverse
       */
      asynMotorAxis::setDoubleParam(pC_->motorPosition_, stepTargetPos_nm_);
      steps_to_go_f *= PULSES_PER_STEP; // now we are in pm
      if (steps_to_go_f > 0) {
        steps_to_go_f /= stepsizef_; // step size in pm
        frequency = maxVelocity * PULSES_PER_STEP / stepsizef_;
      } else if (steps_to_go_f < 0) {
        steps_to_go_f /= stepsizer_; // step size in pm
        frequency = maxVelocity * PULSES_PER_STEP / stepsizer_;
      }
      steps_to_go_i = (long long)steps_to_go_f;
    } else {
      /*
       * The "old" handling of the driver: the position is in steps
       * Keep track of the absolute steps in stepTargetSteps_
       * to handle both absolute and relative movements
       */
      if (relative) {
        steps_to_go_i = (PositionType)(position);
        stepTargetSteps_ += steps_to_go_i;  // store position in global scope
      } else {
        steps_to_go_i = (PositionType)(position - stepTargetSteps_);
        stepTargetSteps_ = (PositionType)position;       // store position in global scope
      }
      asynMotorAxis::setDoubleParam(pC_->motorPosition_, (double)stepTargetSteps_);
    }
    // Set frequency; range 1..20000 Hz
    if(frequency >= MAX_FREQUENCY) {
      frequency = MAX_FREQUENCY;
    }
    asynMotorAxis::setIntegerParam(pC_->stepfreq_, (int)frequency);
    asynPrint(pC_->pasynUserController_, traceMask,
              "%smove(%d) frequency=%f steps_to_go_i=%lld\n",
              "MCS2Axis::", axisNo_, frequency, steps_to_go_i);
    if (!steps_to_go_i)
      return status;
    // Set mode; 4 == STEP
    snprintf(pC_->outString_,sizeof(pC_->outString_)-1, ":CHAN%d:MMOD 4", axisNo_);
    status = pC_->writeController();

    snprintf(pC_->outString_,sizeof(pC_->outString_)-1, ":CHAN%d:STEP:FREQ %u", axisNo_, (unsigned short)frequency);
    status = pC_->writeController();
    // Do move
    snprintf(pC_->outString_,sizeof(pC_->outString_)-1, ":MOVE%d %lld", axisNo_, steps_to_go_i);
    status = pC_->writeController();
  }

  return status;
}

asynStatus MCS2Axis::home(double minVelocity, double maxVelocity, double acceleration, int forwards)
{
  asynStatus status=asynSuccess;
  //static const char *functionName = "homeAxis";
  printf("Home command received %d\n", forwards);
  unsigned short refOpt = 0;

  if (forwards==0){
    refOpt |= START_DIRECTION;
  }
  refOpt |= AUTO_ZERO;

  // Set default reference options - direction and autozero
  printf("ref opt: %d\n", refOpt);
  snprintf(pC_->outString_,sizeof(pC_->outString_)-1, ":CHAN%d:REF:OPT %d", axisNo_, refOpt);
  status = pC_->writeController();
  pC_->clearErrors();

  // Set acceleration
  snprintf(pC_->outString_,sizeof(pC_->outString_)-1, ":CHAN%d:ACC %f", axisNo_, acceleration*PULSES_PER_STEP);
  status = pC_->writeController();
  pC_->clearErrors();
  // Set velocity
  snprintf(pC_->outString_,sizeof(pC_->outString_)-1, ":CHAN%d:VEL %f", axisNo_, maxVelocity*PULSES_PER_STEP);
  status = pC_->writeController();
  pC_->clearErrors();
  // Begin move
  snprintf(pC_->outString_,sizeof(pC_->outString_)-1, ":REF%d", axisNo_);
  status = pC_->writeController();
  pC_->clearErrors();

  return status;
}

asynStatus MCS2Axis::stop(double acceleration )
{
  asynStatus status;
  //static const char *functionName = "stopAxis";

  snprintf(pC_->outString_,sizeof(pC_->outString_)-1, ":STOP%d", axisNo_);
  status = pC_->writeController();

  return status;
}

asynStatus MCS2Axis::setPosition(double position)
{
  asynStatus status=asynSuccess;

  printf("Set position receieved\n");
  snprintf(pC_->outString_,sizeof(pC_->outString_)-1, ":CHAN%d:POS %f", axisNo_, position*PULSES_PER_STEP);
  status = pC_->writeController();
  return status;
}

/** Initial poll (and update) of the axis.
  * \param[out] moving A flag that is set indicating that the axis is moving (1) or done (0). */
asynStatus MCS2Axis::initialPoll(void)
{
  asynStatus status=asynSuccess;
  // Set hold time
  {
    int hold = HOLD_FOREVER;
    (void)pC_->getIntegerParam(axisNo_, pC_->hold_,
                               &hold);
    snprintf(pC_->outString_,sizeof(pC_->outString_)-1, ":CHAN%d:HOLD %d", axisNo_, hold);
    status = pC_->writeController();
    pC_->clearErrors();
  }
  return status;
}

/** Polls the axis.
  * This function reads the controller position, encoder position, the limit status, the moving status,
  * the drive power-on status and positioner type. It does not current detect following error, etc.
  * but this could be added.
  * It calls asynMotorAxis::setIntegerParam() and setDoubleParam() for each item that it polls,
  * and then calls callParamCallbacks() at the end.
  * \param[out] moving A flag that is set indicating that the axis is moving (1) or done (0). */
asynStatus MCS2Axis::poll(bool *moving)
{
  int done;
  int chanState;
  int closedLoop;
  int isCalibrated;
  int isReferenced;
  int endStopReached;
  int followLimitReached;
  int movementFailed = 0;
  int positionerType;
  double encoderPosition;
  double theoryPosition;
  int mclf;
  asynStatus comStatus = asynSuccess;

  if (!initialPollDone_) {
    comStatus = initialPoll();
    if (comStatus) goto skip;
    initialPollDone_ = 1;
  }
  // Read the channel state
  snprintf(pC_->outString_,sizeof(pC_->outString_)-1, ":CHAN%d:STAT?", axisNo_);
  comStatus = pC_->writeReadHandleDisconnect();
  if (comStatus) goto skip;
  chanState = atoi(pC_->inString_);
  asynMotorAxis::setIntegerParam(pC_->pstatrb_, chanState);
  done               = (chanState & CH_STATE_ACTIVELY_MOVING)?0:1;
  closedLoop         = (chanState & CH_STATE_CLOSED_LOOP_ACTIVE)?1:0;
  sensorPresent_     = (chanState & CH_STATE_SENSOR_PRESENT)?1:0;
  isCalibrated       = (chanState & CH_STATE_IS_CALIBRATED)?1:0;
  isReferenced       = (chanState & CH_STATE_IS_REFERENCED)?1:0;
  endStopReached     = (chanState & CH_STATE_END_STOP_REACHED)?1:0;
  followLimitReached = (chanState & CH_STATE_FOLLOWING_LIMIT_REACHED)?1:0;
  movementFailed     = (chanState & CH_STATE_MOVEMENT_FAILED)?1:0;

  *moving = done ? false:true;
  asynMotorAxis::setIntegerParam(pC_->motorStatusDone_, done);
  asynMotorAxis::setIntegerParam(pC_->motorClosedLoop_, closedLoop);
  asynMotorAxis::setIntegerParam(pC_->motorStatusHasEncoder_, sensorPresent_);
  asynMotorAxis::setIntegerParam(pC_->motorStatusHomed_, isReferenced);
  asynMotorAxis::setIntegerParam(pC_->motorStatusHighLimit_, endStopReached);
  asynMotorAxis::setIntegerParam(pC_->motorStatusLowLimit_, endStopReached);
  asynMotorAxis::setIntegerParam(pC_->motorStatusFollowingError_, followLimitReached || movementFailed);
  asynMotorAxis::setIntegerParam(pC_->motorStatusAtHome_, (chanState & CH_STATE_REFERENCE_MARK)?1:0);
  asynMotorAxis::setIntegerParam(pC_->motorStatusPowerOn_, (chanState & CH_STATE_AMPLIFIER_ENABLED)?1:0);

  // Read the current encoder position, if the positioner has a sensor
  if(sensorPresent_) {
    snprintf(pC_->outString_,sizeof(pC_->outString_)-1, ":CHAN%d:POS?", axisNo_);
    comStatus = pC_->writeReadHandleDisconnect();
    if (comStatus) goto skip;
    encoderPosition = (double)strtod(pC_->inString_, NULL);
    asynMotorAxis::setDoubleParam(pC_->freadback_, encoderPosition);
    asynMotorAxis::setDoubleParam(pC_->motorEncoderPosition_, encoderPosition / PULSES_PER_STEP);
#ifdef SMARACT_ASYN_ASYNPARAMINT64
    pC_->setInteger64Param(axisNo_, pC_->ireadback_, atoll(pC_->inString_));
#endif
    if (!openLoop_) {
      // Read the current theoretical position
      snprintf(pC_->outString_,sizeof(pC_->outString_)-1, ":CHAN%d:POS:TARG?", axisNo_);
      comStatus = pC_->writeReadHandleDisconnect();
      if (comStatus) goto skip;
      theoryPosition = (double)strtod(pC_->inString_, NULL);
      theoryPosition /= PULSES_PER_STEP;
      asynMotorAxis::setDoubleParam(pC_->motorPosition_, theoryPosition);
    }
  }


  // Read the currently selected positioner type
  snprintf(pC_->outString_,sizeof(pC_->outString_)-1, ":CHAN%d:PTYP?", axisNo_);
  comStatus = pC_->writeReadHandleDisconnect();
  if (comStatus) goto skip;
  positionerType = atoi(pC_->inString_);
  asynMotorAxis::setIntegerParam(pC_->ptyprb_, positionerType);

  // Read CAL/REF status and MCLF when idle
  if(done)
  {
        asynMotorAxis::setIntegerParam(pC_->cal_, isCalibrated);
        asynMotorAxis::setIntegerParam(pC_->ref_, isReferenced);
        snprintf(pC_->outString_,sizeof(pC_->outString_)-1, ":CHAN%d:MCLF?", axisNo_);
        comStatus = pC_->writeReadHandleDisconnect();
        if (comStatus) goto skip;
        mclf = atoi(pC_->inString_);
        asynMotorAxis::setIntegerParam(pC_->mclf_, mclf);
  }

  skip:
  if (comStatus) initialPollDone_ = 0;
  asynMotorAxis::setIntegerParam(pC_->motorStatusCommsError_, comStatus ? 1:0);
  {
    const char *strErrorMessage = "";
    if (comStatus)
      strErrorMessage = "E: Communication";
    else if (!isReferenced && sensorPresent_ && !openLoop_)
      strErrorMessage = "E: Axis not homed";
    else if (!isCalibrated && sensorPresent_ && !openLoop_)
      strErrorMessage = "E: Not calibrated";
    else if (movementFailed)
      strErrorMessage = "E: movement failed";
    else if (followLimitReached)
      strErrorMessage = "E: follow limit";
    else if (chanState & CH_STATE_POSITIONER_FAULT)
      strErrorMessage = "positioner fault";
    else if (chanState & CH_STATE_POSITIONER_OVERLOAD)
      strErrorMessage = "positioner overload";
    else if (chanState & CH_STATE_OVERTEMP)
      strErrorMessage = "overtemperature";

    setStringParam(pC_->errTxt_, strErrorMessage);
    /* ESS motor has a MsgTxt variable */
#ifdef motorMessageTextString
    updateMsgTxtFromDriver(strErrorMessage);
#endif

  }
  callParamCallbacks();
  return comStatus ? asynError : asynSuccess;
}

asynStatus MCS2Axis::setClosedLoop(bool closedLoop) {
  static const char *functionName = "setClosedLoop";
  int value = closedLoop ? 1 : 0;
  snprintf(pC_->outString_,sizeof(pC_->outString_)-1, "CHAN%d:AMPL %d", axisNo_, value);
  asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
            "%ssetClosedLoop(%d)=%d '%s'\n", functionName, axisNo_, value,
            pC_->outString_);
  return pC_->writeController();
}


asynStatus MCS2Axis::setIntegerParam(int function, epicsInt32  value) {
  asynStatus status = asynError;
  static const char *functionName = "setIntegerParam";
  if (function == pC_->mclf_) {
    /* set MCLF */
    snprintf(pC_->outString_,sizeof(pC_->outString_)-1, ":CHAN%d:MCLF:CURR %d", axisNo_, value);
    status = pC_->writeController();
  }
  else if (function == pC_->ptyp_) {
    /* set positioner type */
    snprintf(pC_->outString_,sizeof(pC_->outString_)-1, ":CHAN%d:PTYP %d", axisNo_, value);
    status = pC_->writeController();
  }
  else if (function == pC_->cal_) {
    asynMotorAxis::setIntegerParam(pC_->cal_, 0);
    /* send calibration command */
    snprintf(pC_->outString_,sizeof(pC_->outString_)-1, ":CAL%d", axisNo_);
    return pC_->writeController();
  }
  else if (function == pC_->hold_) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO, "%s(%d) hold=%d\n",
              functionName, axisNo_, value);
    snprintf(pC_->outString_,sizeof(pC_->outString_)-1, ":CHAN%d:HOLD %d", axisNo_, value);
    status = pC_->writeController();
  }
  else if (function == pC_->openLoop_) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO, "%s(%d) openLoop=%d\n",
              functionName, axisNo_, value);
    openLoop_ = value;
    status = asynSuccess;
  }
  else if (function == pC_->stepcnt_) {
    int frequency;
    (void)pC_->getIntegerParam(axisNo_, pC_->stepfreq_,  &frequency);
    if(frequency >= MAX_FREQUENCY) {
      frequency = MAX_FREQUENCY;
    }
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO, "%s(%d) move stepcnt=%d frequency=%d\n",
              functionName, axisNo_, value, frequency);
    // Set mode; 4 == STEP
    snprintf(pC_->outString_,sizeof(pC_->outString_)-1, ":CHAN%d:MMOD 4", axisNo_);
    status = pC_->writeController();
    snprintf(pC_->outString_,sizeof(pC_->outString_)-1, ":CHAN%d:STEP:FREQ %u", axisNo_, (unsigned short)frequency);
    status = pC_->writeController();
    // Do move
    snprintf(pC_->outString_,sizeof(pC_->outString_)-1, ":MOVE%d %d", axisNo_, value);
    return pC_->writeController();
  }
  /* Call base class method */
  status = asynMotorAxis::setIntegerParam(function, value);

  return status;
}

asynStatus MCS2Axis::setDoubleParam(int function, double value) {
  asynStatus status;
  if (function == pC_->stepsizef_) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
            "%ssetDoubleParam(%d) function=stepsizef value=%f\n",
              "MCS2Axis::", axisNo_, value);
    this->stepsizef_ = value;
  }
  else if (function == pC_->stepsizer_) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
            "%ssetDoubleParam(%d) function=stepsizer value=%f\n",
              "MCS2Axis::", axisNo_, value);
    this->stepsizer_ = value;
  }
  // Call the base class method
  status = asynMotorAxis::setDoubleParam(function, value);
  return status;
}

/** Code for iocsh registration */
static const iocshArg MCS2CreateControllerArg0 = {"Port name", iocshArgString};
static const iocshArg MCS2CreateControllerArg1 = {"MCS2 port name", iocshArgString};
static const iocshArg MCS2CreateControllerArg2 = {"Number of axes", iocshArgInt};
static const iocshArg MCS2CreateControllerArg3 = {"Moving poll period (ms)", iocshArgInt};
static const iocshArg MCS2CreateControllerArg4 = {"Idle poll period (ms)", iocshArgInt};
static const iocshArg MCS2CreateControllerArg5 = {"Unused bit mask", iocshArgInt};
static const iocshArg * const MCS2CreateControllerArgs[] = {&MCS2CreateControllerArg0,
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

