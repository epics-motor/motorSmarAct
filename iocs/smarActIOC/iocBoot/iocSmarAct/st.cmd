#!../../bin/linux-x86_64/smarAct

< envPaths

cd "${TOP}"

## Register all support components
dbLoadDatabase "dbd/smarAct.dbd"
smarAct_registerRecordDeviceDriver pdbbase

cd "${TOP}/iocBoot/${IOC}"

## motorUtil (allstop & alldone)
dbLoadRecords("$(MOTOR)/db/motorUtil.db", "P=smarAct:")

## 

iocInit

## motorUtil (allstop & alldone)
motorUtilInit("smarAct:")

# Boot complete
