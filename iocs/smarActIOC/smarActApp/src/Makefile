TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

# The following are used for debugging messages.
#!USR_CXXFLAGS += -DDEBUG

#=============================
# Build the IOC application

PROD_IOC = smarAct
# smarAct.dbd will be created and installed
DBD += smarAct.dbd

# smarAct.dbd will be made up from these files:
smarAct_DBD += base.dbd

# Include dbd files from all support applications:
ifdef ASYN
smarAct_DBD += asyn.dbd
smarAct_DBD += drvAsynSerialPort.dbd
smarAct_DBD += drvAsynIPPort.dbd
endif
smarAct_DBD += motorSupport.dbd
smarAct_DBD += devSmarActMCSMotor.dbd
smarAct_DBD += devSmarActMCS2Motor.dbd
smarAct_DBD += devSmarActSCUMotor.dbd

ifdef PVXS_MAJOR_VERSION
smarAct_LIBS += pvxsIoc
smarAct_LIBS += pvxs
smarAct_DBD  += pvxsIoc.dbd
else
  ifdef BASE_7_0
    smarAct_LIBS += qsrv
    smarAct_LIBS += pvDatabase
    smarAct_LIBS += $(EPICS_BASE_PVA_CORE_LIBS)
    smarAct_DBD += PVAServerRegister.dbd
    smarAct_DBD += PVACommonRegister.dbd
    smarAct_DBD += softIocPVA.dbd
    smarAct_DBD += registerChannelProviderLocal.dbd
  endif
endif



# Add all the support libraries needed by this IOC
smarAct_LIBS += smarActMotor
smarAct_LIBS += motor
ifdef ASYN
smarAct_LIBS += asyn
endif
ifdef SNCSEQ
smarAct_LIBS += seq pv
endif
ifdef DEVIOCSTATS
smarAct_DBD += devIocStats.dbd iocAdmin.dbd
smarAct_LIBS += devIocStats
endif
ifdef AUTOSAVE
smarAct_DBD += asSupport.dbd
smarAct_LIBS += autosave
endif

# smarAct_registerRecordDeviceDriver.cpp derives from smarAct.dbd
smarAct_SRCS += smarAct_registerRecordDeviceDriver.cpp

# Build the main IOC entry point on workstation OSs.
smarAct_SRCS_DEFAULT += smarActMain.cpp
smarAct_SRCS_vxWorks += -nil-

# Add support from base/src/vxWorks if needed
#smarAct_OBJS_vxWorks += $(EPICS_BASE_BIN)/vxComLibrary

# Finally link to the EPICS Base libraries
smarAct_LIBS += $(EPICS_BASE_IOC_LIBS)

#===========================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

