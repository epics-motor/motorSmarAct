# motorSmarAct Releases

## __R1-2-1 (2020-05-14)__
R1-2-1 is a release based on the master branch.  

### Changes since R1-2

#### New features
* None

#### Modifications to existing features
* None

#### Bug fixes
* Pull request [#6](https://github.com/epics-motor/motorSmarAct/pull/6): Include a local definition of rint on fewer platforms

## __R1-2 (2020-05-12)__
R1-2 is a release based on the master branch.  

### Changes since R1-1

#### New features
* Added support for the SCU controllers from [Mark Rivers](https://github.com/MarkRivers)
* Pull request [#2](https://github.com/epics-motor/motorSmarAct/pull/2): [Christoph Schröder](https://github.com/chrschroeder) added parameters for MCLF and CAL commands to the SmarActMCS2 support

#### Modifications to existing features
* Pull request [#4](https://github.com/epics-motor/motorSmarAct/pull/4): [Keenan Lang](https://github.com/keenanlang) eliminated hard-coded sensor types which allows the SmarActMCS driver to work with newer stages

#### Bug fixes
* Commit [22a1fbe](https://github.com/epics-motor/motorSmarAct/commit/22a1fbe9043879330568ae8695d1f10a695fe336): Include ``$(MOTOR)/modules/RELEASE.$(EPICS_HOST_ARCH).local`` instead of ``$(MOTOR)/configure/RELEASE``

## __R1-1 (2019-08-08)__
R1-1 is a release based on the master branch.  

### Changes since R1-0

#### New features
* Commit [bfbb28d](https://github.com/epics-motor/motorSmarAct/commit/bfbb28dc871cc978dbbc20cba09760ac08651ba0): Added support for the MCS2 controller from [David Vine](https://github.com/djvine)

## __R1-0 (2019-04-18)__
R1-0 is a release based on the master branch.  

### Changes since motor-6-11

motorSmarAct is now a standalone module, as well as a submodule of [motor](https://github.com/epics-modules/motor)

#### New features
* motorSmarAct can be built outside of the motor directory
* motorSmarAct has a dedicated example IOC that can be built outside of motorSmarAct

#### Modifications to existing features
* None

#### Bug fixes
* None
