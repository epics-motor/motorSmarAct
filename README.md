# motorSmarAct
EPICS motor drivers for the following [SmarAct](http://www.smaract.com/) controllers: MCS, MCS2, SCU

[![Build Status](https://github.com/epics-motor/motorSmarAct/actions/workflows/ci-scripts-build.yml/badge.svg)](https://github.com/epics-motor/motorSmarAct/actions/workflows/ci-scripts-build.yml)
<!--[![Build Status](https://travis-ci.org/epics-motor/motorSmarAct.png)](https://travis-ci.org/epics-motor/motorSmarAct)-->

motorSmarAct is a submodule of [motor](https://github.com/epics-modules/motor).  When motorSmarAct is built in the ``motor/modules`` directory, no manual configuration is needed.

motorSmarAct can also be built outside of motor by copying it's ``EXAMPLE_RELEASE.local`` file to ``RELEASE.local`` and defining the paths to ``MOTOR`` and itself.

motorSmarAct contains an example IOC that is built if ``CONFIG_SITE.local`` sets ``BUILD_IOCS = YES``.  The example IOC can be built outside of driver module.
