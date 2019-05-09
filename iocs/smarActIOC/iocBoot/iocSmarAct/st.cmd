#!../../bin/linux-x86_64/smarAct

< envPaths

cd "${TOP}"

## Register all support components
dbLoadDatabase "dbd/smarAct.dbd"
smarAct_registerRecordDeviceDriver pdbbase

cd "${TOP}/iocBoot/${IOC}"

## motorUtil (allstop & alldone)
dbLoadRecords("$(MOTOR)/db/motorUtil.db", "P=smarAct:")

# Uncomment one of the following lines for MCS or MCS2 controller
#< smaractmcs.iocsh
#< smaractmcs2.iocsh

## 

iocInit

## motorUtil (allstop & alldone)
motorUtilInit("smarAct:")

# Boot complete
