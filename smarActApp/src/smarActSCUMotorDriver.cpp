
/* Motor driver support for smarAct SCU Controllers */

/* Derived from smarActSCUDriver.cpp by Till Straumann <strauman@slac.stanford.edu> */

/* Author:  Mark Rivers, University of Chicago, March 29, 2020 */

#include <iocsh.h>

#include <asynOctetSyncIO.h>
#include <asynMotorController.h>
#include <asynMotorAxis.h>
#include <smarActSCUMotorDriver.h>
#include <errlog.h>

#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include <exception>

#include <math.h>

#include <epicsString.h>
#include <epicsExport.h>

/* Static configuration parameters (compile-time constants) */
#undef  DEBUG

#define CMD_LEN 50
#define REP_LEN 50
#define DEFAULT_TIMEOUT 2.0

#define HOLD_FOREVER 60000
#define HOLD_NEVER       0
#define FAR_AWAY     1000000000 /*nm*/
#define UDEG_PER_REV 360000000
#define STEPS_PER_EGU 1000.

/* The asyn motor driver apparently can't cope with exceptions */
#undef  ASYN_CANDO_EXCEPTIONS
/* Define this if exceptions should be thrown and it is OK to abort the application */
#undef  DO_THROW_EXCEPTIONS

#if defined(ASYN_CANDO_EXCEPTIONS) || defined(DO_THROW_EXCEPTIONS)
#define THROW_(e) throw e
#else
#define THROW_(e) epicsPrintf("%s\n",e.what());
#endif

enum SmarActSCUStatus {
	Stopped     = 0,
	AmplSetting = 1,
	Moving      = 2,
	Targeting   = 3,
	Holding     = 4,
	Calibrating = 5,
	Referencing = 6,
	Unknown     = 7
};

static SmarActSCUStatus parseMovingStatus(char statusChar)
{
    SmarActSCUStatus status = Unknown;
    
    if      ('S' == statusChar) status = Stopped;
    else if ('A' == statusChar) status = AmplSetting;
    else if ('M' == statusChar) status = Moving;
    else if ('T' == statusChar) status = Targeting;
    else if ('H' == statusChar) status = Holding;
    else if ('C' == statusChar) status = Calibrating;
    else if ('R' == statusChar) status = Referencing;
    return status;
}

SmarActSCUException::SmarActSCUException(SmarActSCUExceptionType t, const char *fmt, ...)
  : t_(t)
{
va_list ap;
  if ( fmt ) {
    va_start(ap, fmt);
    epicsVsnprintf(str_, sizeof(str_), fmt, ap);
    va_end(ap);
  } else {
    str_[0] = 0;
  }
};

SmarActSCUException::SmarActSCUException(SmarActSCUExceptionType t, const char *fmt, va_list ap)
    : t_(t)
{
  epicsVsnprintf(str_, sizeof(str_), fmt, ap);
}

SmarActSCUController::SmarActSCUController(const char *portName, const char *IOPortName, int numAxes, double movingPollPeriod, double idlePollPeriod)
  : asynMotorController(portName, numAxes,
                        0, // parameters
                        0, // interface mask
                        0, // interrupt mask
                        ASYN_CANBLOCK | ASYN_MULTIDEVICE,
                        1, // autoconnect
                        0,0) // default priority and stack size
{
asynStatus       status;
pAxes_ = (SmarActSCUAxis **)(asynMotorController::pAxes_);

  status = pasynOctetSyncIO->connect(IOPortName, 0, &pasynUserController_, NULL);
  if ( status ) {
    asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
              "SmarActSCUController:SmarActSCUController: cannot connect to SCU controller\n");
    THROW_(SmarActSCUException(SCUConnectionError, "SmarActSCUController: unable to connect serial channel"));
  }

  startPoller( movingPollPeriod, idlePollPeriod, 0 );

}

/* Obtain value of the 'motorClosedLoop_' parameter (which
 * maps to the record's CNEN field)
 */
int
SmarActSCUAxis::getClosedLoop()
{
int val;
  pC_->getIntegerParam(axisNo_, pC_->motorClosedLoop_, &val);
  return val;
}

SmarActSCUAxis::SmarActSCUAxis(class SmarActSCUController *cnt_p, int axis, int channel)
  : asynMotorAxis(cnt_p, axis), pC_(cnt_p)
{
  char moveStatus;
  double currentPosition;
  int rev;
  channel_ = channel;

  asynPrint(pC_->pasynUserSelf, ASYN_TRACEIO_DRIVER, "SmarActSCUAxis::SmarActSCUAxis -- creating axis %u\n", axis);

  comStatus_ = getIntegerVal("GCLF", &maxFreq_);
#ifdef DEBUG
  printf("GCLF%u returned %i\n", axis, comStatus_);
#endif
  if ( comStatus_ )
    goto bail;
  if ( (comStatus_ = getCharVal("M", &moveStatus)) )
    goto bail;

  if ( 'H' == moveStatus ) {
    // still holding? This means that - in a previous life - the
    // axis was configured for 'infinite holding'. Inherit this
    // (until the next 'move' command that is).
    ///
    holdTime_ = HOLD_FOREVER;
  } else {
    // initial value from 'closed-loop' property
    holdTime_ = getClosedLoop() ? HOLD_FOREVER : 0;
  }

  // Query the sensor type
  if ( (comStatus_ = getIntegerVal("GST", &positionerType_)) )
    goto bail;

        // Determine if stage is a rotation stage
  if (positionerType_ == 2 ||
      positionerType_ == 8 || 
      positionerType_ == 14 || 
      positionerType_ == 20 || 
      positionerType_ == 22 || 
      positionerType_ == 23 || 
      (positionerType_ >= 25 && positionerType_ <= 29)) {
    isRot_ = 1;   
    if ( asynSuccess == getAngle(&currentPosition, &rev) ) {
      setIntegerParam(pC_->motorStatusHasEncoder_, 1);
      setIntegerParam(pC_->motorStatusGainSupport_, 1);
    }
  }
  else {
    isRot_ = 0;
    if (asynSuccess == getDoubleVal("GP", &currentPosition)) {
      setIntegerParam(pC_->motorStatusHasEncoder_, 1);
      setIntegerParam(pC_->motorStatusGainSupport_, 1);
    }
  }


bail:
  setIntegerParam(pC_->motorStatusProblem_, comStatus_ ? 1 : 0);
  setIntegerParam(pC_->motorStatusCommsError_, comStatus_ ? 1 : 0);

  callParamCallbacks();

  if ( comStatus_ ) {
    THROW_(SmarActSCUException(SCUCommunicationError, "SmarActSCUAxis::SmarActSCUAxis -- channel %u ASYN error %i", axis, comStatus_));
  }

}

/* Send a command to the controller and read the response.
 * Uses the toController_ and fromController_ strings in asynMotorController.
 * 
 * RETURNS:  asynError if an error occurred, asynSuccess otherwise.
 */
asynStatus SmarActSCUAxis::sendCmd()
{
  size_t replyLen;
  asynStatus status;
  
  status = pC_->writeReadController(toController_, fromController_, sizeof(fromController_), &replyLen, DEFAULT_TIMEOUT);
  if (status)
    asynPrint(pasynUser_, ASYN_TRACE_ERROR, "ERROR: sendCmd: status=%d, sent: %s, received: %s\n", status, toController_, fromController_);
  else
    asynPrint(pasynUser_, ASYN_TRACEIO_DRIVER, "sendCmd: status=%d, sent: %s, received: %s\n", status, toController_, fromController_);  
  return status;
}

/* Read an integer parameter from the SCU (nothing to do with asyn's parameter library).
 *
 * parm_cmd: SCU command (w/o ':' char) to read parameter
 * val_p:    where to store the value returned by the SCU
 * 
 * RETURNS:  asynError if an error occurred, asynSuccess otherwise.
 */
asynStatus
SmarActSCUAxis::getIntegerVal(const char *parm_cmd, int *val_p)
{
  char       cmd[REP_LEN];
  char       param[REP_LEN];
  int        axis;
  asynStatus status;

  epicsSnprintf(toController_, sizeof(toController_), ":%s%u", parm_cmd, this->channel_);
  status = sendCmd();
  if (status)
    return status;
  if (4 != sscanf(fromController_, ":%10[A-Z]%i%10[A-Z]%i", cmd, &axis, param, val_p)) {
    asynPrint(pasynUser_, ASYN_TRACE_ERROR, "getIntegerVal:ERROR parsing response %s\n", fromController_);
    return asynError;
  }

  if ('E' == cmd[0])
    return asynError;
  return asynSuccess;
}

/* Read a double parameter from the SCU (nothing to do with asyn's parameter library).
 *
 * parm_cmd: SCU command (w/o ':' char) to read parameter
 * val_p:    where to store the value returned by the SCU
 * 
 * RETURNS:  asynError if an error occurred, asynSuccess otherwise.
 */
asynStatus
SmarActSCUAxis::getDoubleVal(const char *parm_cmd, double *val_p)
{
  char       cmd[REP_LEN];
  char       param[REP_LEN];
  int        axis;
  asynStatus status;

  epicsSnprintf(toController_, sizeof(toController_), ":%s%u", parm_cmd, this->channel_);
  status = sendCmd();
  if (status)
    return status;
  if (4 != sscanf(fromController_, ":%10[A-Z]%i%10[A-Z]%lf", cmd, &axis, param, val_p)) {
    asynPrint(pasynUser_, ASYN_TRACE_ERROR, "getDoubleVal:ERROR parsing response %s\n", fromController_);
    return asynError;
  }
  if ('E' == cmd[0])
    return asynError;
  return asynSuccess;
}

/* Read a char parameter from the SCU (nothing to do with asyn's parameter library).
 *
 * parm_cmd: SCU command (w/o ':' char) to read parameter
 * val_p:    where to store the value returned by the SCU
 * 
 * RETURNS:  asynError if an error occurred, asynSuccess otherwise.
 */
asynStatus
SmarActSCUAxis::getCharVal(const char *parm_cmd, char *val_p)
{
  char       cmd[REP_LEN];
  int        axis;
  asynStatus status;

  epicsSnprintf(toController_, sizeof(toController_), ":%s%u", parm_cmd, this->channel_);
  status = sendCmd();
  if (status)
    return status;
  if (3 != sscanf(fromController_, ":%10[A-Z]%i%c", cmd, &axis, val_p)) {
    asynPrint(pasynUser_, ASYN_TRACE_ERROR, "getCharVal:ERROR parsing response %s\n", fromController_);
    return asynError;
  }
  if ('E' == cmd[0])
    return asynError;
  return asynSuccess;
}

/* Read the position of rotation stage
 *
 * parm_cmd: SCU command (w/o ':' char) to read parameter
 * val_p:    where to store the value returned by the SCU
 * 
 * RETURNS:  asynError if an error occurred, asynSuccess otherwise.
 */
asynStatus
SmarActSCUAxis::getAngle(double *val_p, int *rev_p)
{
  asynStatus status;
  int        axis;

  epicsSnprintf(toController_, sizeof(toController_), ":GA%u", this->channel_);
  status = sendCmd();
  if (status)
    return status;

  if (4 != sscanf(fromController_, ":A%iA%lfR%i", &axis, val_p, rev_p)) {
    asynPrint(pasynUser_, ASYN_TRACE_ERROR, "getAngle:ERROR parsing response %s\n", fromController_);
    return asynError;
  }

  return asynSuccess;
}

asynStatus
SmarActSCUAxis::poll(bool *moving_p)
{
double                 doubleVal;
int                    integerVal;
char                   charVal;
double                 angle;
int                    rev;
enum SmarActSCUStatus  movingStatus;

  if (isRot_) {
    if ((comStatus_ = getAngle(&angle, &rev)))
      goto bail;
    // Convert angle and revs to total angle
    doubleVal = rev * UDEG_PER_REV + angle;
  }
  else {
    if ((comStatus_ = getDoubleVal("GP", &doubleVal)))
      goto bail;
  }

  setDoubleParam(pC_->motorEncoderPosition_, (doubleVal+positionOffset_)*STEPS_PER_EGU);
  setDoubleParam(pC_->motorPosition_, (doubleVal+positionOffset_)*STEPS_PER_EGU);
#ifdef DEBUG
  printf("POLL (position %f)", doubleVal);
#endif

  if ((comStatus_ = getCharVal("M", &charVal)))
    goto bail;

  movingStatus = parseMovingStatus(charVal);

  switch (movingStatus) {
    default:
      *moving_p = false;
    break;

    /* If we use 'infinite' holding (until the next 'move' command)
     * then the 'Holding' state must be considered 'not moving'. However,
     * if we use a 'finite' holding time then we probably should consider
     * the 'move' command incomplete until the holding time expires.
     */
    case Holding:
      *moving_p = HOLD_FOREVER == holdTime_ ? false : true;
    break;

    case Targeting:
    case Moving:
    case Calibrating:
    case Referencing:
      *moving_p = true;
    break;
  }

  setIntegerParam(pC_->motorStatusDone_, ! *moving_p );


  /* Check if the sensor 'knows' absolute position and
   * update the MSTA 'HOMED' bit.
   */
  if ((comStatus_ = getIntegerVal("GPPK", &integerVal))) 
    goto bail;

  setIntegerParam(pC_->motorStatusHomed_, integerVal ? 1 : 0 );

#ifdef DEBUG
  printf(" status %u", status);
#endif

bail:
  setIntegerParam(pC_->motorStatusProblem_,    comStatus_ ? 1 : 0 );
  setIntegerParam(pC_->motorStatusCommsError_, comStatus_ ? 1 : 0 );
#ifdef DEBUG
  printf("\n");
#endif

  callParamCallbacks();

  return comStatus_;
}

asynStatus  
SmarActSCUAxis::move(double position, int relative, double min_vel, double max_vel, double accel)
{
  double rpos;
  double angle;
  int rev;

#ifdef DEBUG
  printf("Move to %f (speed %f - %f)\n", position, min_vel, max_vel);
#endif

  /* cache 'closed-loop' setting until next move */
  holdTime_  = getClosedLoop() ? HOLD_FOREVER : 0;

  rpos = (position / STEPS_PER_EGU) - positionOffset_;

  if ( isRot_ ) {
    angle = (long)rpos % UDEG_PER_REV;
    rev = (int)(rpos / UDEG_PER_REV);
    if (angle < 0) {
      angle += UDEG_PER_REV;
      rev -= 1;
    }
    if (relative) {
      epicsSnprintf(toController_, sizeof(toController_), ":MAR%uA%.3fR%dH%d:GP%u", this->channel_, angle, rev, holdTime_, this->channel_);
      comStatus_ = sendCmd();
    }
    else {
      epicsSnprintf(toController_, sizeof(toController_), ":MAA%uA%.3fR%dH%d:GP%u", this->channel_, angle, rev, holdTime_, this->channel_);
      comStatus_ = sendCmd();
    }
  } else {
    if (relative) {
      epicsSnprintf(toController_, sizeof(toController_), ":MPR%uP%.3fH%d:GP%u", this->channel_, rpos, holdTime_, this->channel_);
      comStatus_ = sendCmd();
    }
    else {
      epicsSnprintf(toController_, sizeof(toController_), ":MPA%uP%.3fH%d:GP%u", this->channel_, rpos, holdTime_, this->channel_);
      comStatus_ = sendCmd();
    }
  }

  if (comStatus_) {
    setIntegerParam(pC_->motorStatusProblem_, 1);
    setIntegerParam(pC_->motorStatusCommsError_, 1);
    callParamCallbacks();
  }
  return comStatus_;
}

asynStatus
SmarActSCUAxis::home(double min_vel, double max_vel, double accel, int forwards)
{
#ifdef DEBUG
  printf("Home %u\n", forwards);
#endif

  /* cache 'closed-loop' setting until next move */
  holdTime_  = getClosedLoop() ? HOLD_FOREVER : 0;

  epicsSnprintf(toController_, sizeof(toController_), ":MTR%uH%dZ0:GP%u", this->channel_, holdTime_, this->channel_);
  comStatus_ = sendCmd();

  if (comStatus_) {
    setIntegerParam(pC_->motorStatusProblem_, 1);
    setIntegerParam(pC_->motorStatusCommsError_, 1);
    callParamCallbacks();
  }

  return comStatus_;
}

asynStatus
SmarActSCUAxis::stop(double acceleration)
{
#ifdef DEBUG
  printf("Stop\n");
#endif
  epicsSnprintf(toController_, sizeof(toController_), ":S%u:GP%u", this->channel_, this->channel_);
  comStatus_ = sendCmd();

  if (comStatus_) {
    setIntegerParam(pC_->motorStatusProblem_, 1);
    setIntegerParam(pC_->motorStatusCommsError_, 1);
    callParamCallbacks();
  }
  return comStatus_;
}

asynStatus
SmarActSCUAxis::setPosition(double position)
{
  positionOffset_ = position / STEPS_PER_EGU;

  return asynSuccess;
}

asynStatus
SmarActSCUAxis::moveVelocity(double min_vel, double max_vel, double accel)
{
double       tgt_pos = FAR_AWAY;

  /* No SCU command we an use directly. Just use a 'relative move' to
   * very far target.
   */

#ifdef DEBUG
  printf("moveVelocity (%f - %f)\n", min_vel, max_vel);
#endif

  if ( 0 == max_vel ) {
    return this->stop(0.);
  }

  if ( max_vel < 0 ) {
    tgt_pos = -tgt_pos; 
  }

  return this->move(tgt_pos, 1, 0, max_vel, accel);
}

/* iocsh wrapping and registration business (stolen from ACRMotorDriver.cpp) */
static const iocshArg cc_a0 = {"Port name [string]",               iocshArgString};
static const iocshArg cc_a1 = {"I/O port name [string]",           iocshArgString};
static const iocshArg cc_a2 = {"Number of axes [int]",             iocshArgInt};
static const iocshArg cc_a3 = {"Moving poll period (s) [double]",  iocshArgDouble};
static const iocshArg cc_a4 = {"Idle poll period (s) [double]",    iocshArgDouble};

static const iocshArg * const cc_as[] = {&cc_a0, &cc_a1, &cc_a2, &cc_a3, &cc_a4};

static const iocshFuncDef cc_def = {"smarActSCUCreateController", sizeof(cc_as)/sizeof(cc_as[0]), cc_as};

extern "C" void *
smarActSCUCreateController(
  const char *motorPortName,
  const char *ioPortName,
  int         numAxes,
  double      movingPollPeriod,
  double      idlePollPeriod)
{
void *rval = 0;
  // the asyn stuff doesn't seem to be prepared for exceptions. I get segfaults
  // if constructing a controller (or axis) incurs an exception even if its
  // caught (IMHO asyn should behave as if the controller/axis never existed...)
#ifdef ASYN_CANDO_EXCEPTIONS
  try {
#endif
    rval = new SmarActSCUController(motorPortName, ioPortName, numAxes, movingPollPeriod, idlePollPeriod);
#ifdef ASYN_CANDO_EXCEPTIONS
  } catch (SmarActSCUException &e) {
    epicsPrintf("smarActSCUCreateController failed (exception caught):\n%s\n", e.what());
    rval = 0;
  }
#endif

  return rval;
}

static void cc_fn(const iocshArgBuf *args)
{
  smarActSCUCreateController(
    args[0].sval,
    args[1].sval,
    args[2].ival,
    args[3].dval,
    args[4].dval);
}


static const iocshArg ca_a0 = {"Controller Port name [string]",    iocshArgString};
static const iocshArg ca_a1 = {"Axis number [int]",                iocshArgInt};
static const iocshArg ca_a2 = {"Channel [int]",                    iocshArgInt};

static const iocshArg * const ca_as[] = {&ca_a0, &ca_a1, &ca_a2};

/* iocsh wrapping and registration business (stolen from ACRMotorDriver.cpp) */
/* smarActSCUCreateAxis called to create each axis of the smarActSCU controller*/
static const iocshFuncDef ca_def = {"smarActSCUCreateAxis", 3, ca_as};

extern "C" void *
smarActSCUCreateAxis(
  const char *controllerPortName,
  int        axisNumber,
  int        channel)
{
void *rval = 0;

SmarActSCUController *pC;
asynMotorAxis *pAsynAxis;

  // the asyn stuff doesn't seem to be prepared for exceptions. I get segfaults
  // if constructing a controller (or axis) incurs an exception even if its
  // caught (IMHO asyn should behave as if the controller/axis never existed...)
#ifdef ASYN_CANDO_EXCEPTIONS
  try {
#endif
//    rval = new SmarActSCUAxis(, axisNumber, channel);
    pC = (SmarActSCUController*) findAsynPortDriver(controllerPortName);
    if (!pC) {
      printf("smarActSCUCreateAxis: Error port %s not found\n", controllerPortName);
      rval = 0;
      return rval;
    }
    // check if axis number already exists
    pAsynAxis = pC->getAxis(axisNumber);
    if (pAsynAxis != NULL) { // axis already exists
      epicsPrintf("SmarActSCUCreateAxis failed:axis %u already exists\n", axisNumber);
#ifdef ASYN_CANDO_EXCEPTIONS
      THROW_(SmarActSCUException(SCUCommunicationError, "axis %u already exists", axisNumber));
#endif
      rval = 0;
      return rval;
    }
    pC->lock();
    new SmarActSCUAxis(pC, axisNumber, channel);
    pC->unlock();

#ifdef ASYN_CANDO_EXCEPTIONS
  } catch (SmarActSCUException &e) {
    epicsPrintf("SmarActSCUAxis failed (exception caught):\n%s\n", e.what());
    rval = 0;
  }
#endif

  return rval;
}

static void ca_fn(const iocshArgBuf *args)
{
  smarActSCUCreateAxis(
    args[0].sval,
    args[1].ival,
    args[2].ival);
}

static void smarActSCUMotorRegister(void)
{
  iocshRegister(&cc_def, cc_fn);  // smarActSCUCreateController
  iocshRegister(&ca_def, ca_fn);  // smarActSCUCreateAxis
}

extern "C" {
epicsExportRegistrar(smarActSCUMotorRegister);
}
