#!../../bin/linux-x86_64/smarAct

< envPaths
epicsEnvSet("P", "smarAct:")
epicsEnvSet("MC_CT", "unit1")

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
#< smaractscu.iocsh
## 
# Optional: load devIocStats records (requires DEVIOCSTATS module)
#dbLoadRecords("$(DEVIOCSTATS)/db/iocAdminSoft.db", "IOC=$(P)$(MC_CT)")

# autosave/restore machinery
<AutoSave.cmd


iocInit

## motorUtil (allstop & alldone)
motorUtilInit("smarAct:")

# save motor positions every five seconds
create_monitor_set("auto_positions.req",5,"P=$(P)")
# save settings every thirty seconds
create_monitor_set("auto_settings.req",30,"P=$(P)")

# Boot complete
