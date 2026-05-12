#!../../bin/darwin-x86/smarAct

epicsEnvSet("MOTOR","../../../../../..")

## Register all support components
dbLoadDatabase  "../../../../iocs/smarActIOC/dbd/smarAct.dbd"
smarAct_registerRecordDeviceDriver pdbbase

#
drvAsynIPPortConfigure("MCS2_ETH", "ymir-mcs2-piezo-uf601.cslab.esss.lu.se:55551", 0, 0, 0)
##asynSetOption "MCS2_ETH" -1 disconnectOnReadTimeout Y


# PORT, MCS_PORT, number of axes, active poll period (ms), idle poll period (ms), unusedMask
# we use only 1 of the possible 3 axes
MCS2CreateController("MCS2", "MCS2_ETH", 1, 100, 1000, 0)
asynSetTraceIOMask("MCS2", -1, 0xFF)


######################
#/* traceMask definitions*/
#define ASYN_TRACE_ERROR     0x0001
#define ASYN_TRACEIO_DEVICE  0x0002
#define ASYN_TRACEIO_FILTER  0x0004
#define ASYN_TRACEIO_DRIVER  0x0008
#define ASYN_TRACE_FLOW      0x0010
#define ASYN_TRACE_WARNING   0x0020
#define ASYN_TRACE_INFO      0x0040
asynSetTraceMask("MCS2_ETH", -1, 0x41)
#asynSetTraceMask("MCS2_ETH", -1, 0xFF)

#/* traceIO mask definitions*/
#define ASYN_TRACEIO_NODATA 0x0000
#define ASYN_TRACEIO_ASCII  0x0001
#define ASYN_TRACEIO_ESCAPE 0x0002
#define ASYN_TRACEIO_HEX    0x0004
#asynSetTraceIOMask("MCS2_ETH", -1, 2)
asynSetTraceIOMask("MCS2_ETH", -1, 6)


#/* traceInfo mask definitions*/
#define ASYN_TRACEINFO_TIME 0x0001
#define ASYN_TRACEINFO_PORT 0x0002
#define ASYN_TRACEINFO_SOURCE 0x0004
#define ASYN_TRACEINFO_THREAD 0x0008
asynSetTraceInfoMask("MCS2_ETH", -1, 5)


###############################################
# Loading records, the new way
epicsEnvSet("P",       "BIFROST-SpSl1:")
epicsEnvSet("TIMEOUT", "2")
epicsEnvSet("PORT",    "MCS2")

#
# Axis 1, counting from 0
#
epicsEnvSet("M",    "MC-SlXm:Mtr")
epicsEnvSet("ADDR", "0")
epicsEnvSet("DESC", "Xm")
epicsEnvSet("DHLM", "265")
epicsEnvSet("DLLM", "-265")
epicsEnvSet("STEPSIZEF", "0.0007")
epicsEnvSet("STEPSIZER", "0.0007")
dbLoadRecords("$(MOTOR)/Db/ess-mcs2.template", "P=$(P), M=$(M), PORT=$(PORT), ADDR=$(ADDR), DESC=$(DESC), TIMEOUT=$(TIMEOUT), URIP=0, DHLM=$(DHLM), DLLM=$(DLLM), STEPSIZEF=$(STEPSIZEF), STEPSIZER=$(STEPSIZER),")


###############################################
iocInit
