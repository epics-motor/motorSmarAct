# Makefile
TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

#==================================================
# Build an IOC support library
LIBRARY_IOC  = smarActMotor

# motorRecord.h will be created from motorRecord.dbd
# install devMotorSoft.dbd into <top>/dbd
DBD += devSmarActMCSMotor.dbd
DBD += devSmarActMCS2Motor.dbd

INC += smarActMCSMotorDriver.h
INC += smarActMCS2MotorDriver.h

# The following are compiled and added to the Support library
smarActMotor_SRCS += smarActMCSMotorDriver.cpp
smarActMotor_SRCS += smarActMCS2MotorDriver.cpp

smarActMotor_LIBS += motor
smarActMotor_LIBS += asyn
smarActMotor_LIBS += $(EPICS_BASE_IOC_LIBS)

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

