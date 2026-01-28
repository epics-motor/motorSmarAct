#ifndef SMARACT_SCU_MOTOR_DRIVER_H
#define SMARACT_SCU_MOTOR_DRIVER_H

/* Motor driver support for smarAct SCU Controllers */

/* Derived from smarActSCUDriver.cpp by Till Straumann <strauman@slac.stanford.edu> */

/* Author:  Mark Rivers, University of Chicago, March 29, 2020 */

#ifdef __cplusplus

#include <asynMotorController.h>
#include <asynMotorAxis.h>
#include <stdarg.h>
#include <exception>

enum SmarActSCUExceptionType {
  SCUUnknownError,
  SCUConnectionError,
  SCUCommunicationError,
};

class SmarActSCUException : public std::exception {
public:
  SmarActSCUException(SmarActSCUExceptionType t, const char *fmt, ...);
  SmarActSCUException(SmarActSCUExceptionType t)
      : t_(t)
  { str_[0] = 0; }
  SmarActSCUException()
      : t_(SCUUnknownError)
  { str_[0] = 0; }
  SmarActSCUException(SmarActSCUExceptionType t, const char *fmt, va_list ap);
  SmarActSCUExceptionType getType()
      const { return t_; }
  virtual const char *what()
      const throw() { return str_; }

protected:
  char str_[100];
  SmarActSCUExceptionType t_;
};

class SmarActSCUAxis : public asynMotorAxis
{
public:
  SmarActSCUAxis(class SmarActSCUController *cnt_p, int axis, int channel);
  asynStatus poll(bool *moving_p);
  asynStatus move(double position, int relative, double min_vel, double max_vel, double accel);
  asynStatus home(double min_vel, double max_vel, double accel, int forwards);
  asynStatus stop(double acceleration);
  asynStatus setPosition(double position);
  asynStatus moveVelocity(double min_vel, double max_vel, double accel);

  virtual asynStatus getCharVal(const char *parm, char *val_p);
  virtual asynStatus getIntegerVal(const char *parm, int *val_p);
  virtual asynStatus getDoubleVal(const char *parm, double *val_p);
  virtual asynStatus getAngle(double *val_p, int *rev_p);
  virtual int getClosedLoop();

  int getMaxFreq() const { return maxFreq_; }

protected:
  asynStatus setSpeed(double velocity);

private:
  SmarActSCUController *pC_; // pointer to asynMotorController for this axis
  asynStatus comStatus_;
  int maxFreq_;
  unsigned holdTime_;
  int channel_;
  int positionerType_;
  int isRot_;
  double positionOffset_;
  asynStatus sendCmd();
  char toController_[MAX_CONTROLLER_STRING_SIZE];
  char fromController_[MAX_CONTROLLER_STRING_SIZE];

  friend class SmarActSCUController;
};

class SmarActSCUController : public asynMotorController
{
public:
  SmarActSCUController(const char *portName, const char *IOPortName, int numAxes, double movingPollPeriod, double idlePollPeriod);

  static int parseIntegerReply(const char *reply, int *ax_p, int *val_p);
  static int parseAngle(const char *reply, int *ax_p, int *val_p, int *rot_p);

protected:
  SmarActSCUAxis **pAxes_;

private:
  friend class SmarActSCUAxis;
};

#endif // _cplusplus
#endif // SMARACT_SCU_MOTOR_DRIVER_H
