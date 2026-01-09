#!/bin/sh

# Name of the destination file
FILE=$1
shift

# Name of the source file file
BASENAME=$1
shift

STATUSBITS='$(M)-StatusBits'
yauxleft=396
yauxright=396

MOTORRECORDFIELDS=".SPDB .RDBD .MDEL"
#ANALOGINS="-RawEncStep"
ANALOGINS="-CommandedPos -SensorPos -MotorPosWhenDone"
ANALOGOUTS="-STEPCNT -STEPFREQ -STEPSIZEF -STEPSIZER -SensorPowerMode -SensorDelay -hold"

# Names of the status bits in PSTAT for a piezo motor
# Start with bit 0 going upward
NAMEPSTATBITS=""
NAMEPSTATBITS="$NAMEPSTATBITS ACTIVELY_MOVING CLOSED_LOOP_ACTIVE CALIBRATING REFERENCING "
NAMEPSTATBITS="$NAMEPSTATBITS MOVE_DELAYED SENSOR_PRESENT IS_CALIBRATED IS_REFERENCED "
NAMEPSTATBITS="$NAMEPSTATBITS END_STOP_REACHED RANGE_LIMIT_REACHED FOLLOWING_LIMIT_REACHED MOVEMENT_FAILED "
NAMEPSTATBITS="$NAMEPSTATBITS STREAMING POSITIONER_OVERLOAD OVERTEMP REFERENCE_MARK "
NAMEPSTATBITS="$NAMEPSTATBITS IS_PHASED POSITIONER_FAULT AMPLIFIER_ENABLED IN_POSITION "
NAMEPSTATBITS="$NAMEPSTATBITS BRAKE_ENABLED"

# pick up all arguments
# currently non allowed
PARAM="$1"
while test "$PARAM" != ""; do
  case $1 in
  *)
      echo >&2 "illegal option: $1"
      exit 1
      ;;
  esac
done

im=0
x=0
y=0

echo "Creating $FILE" &&
  cat $BASENAME.start >/tmp/$$ &&
  echo $0: FILE=$FILE BASENAME=$BASENAME rest=$@
cmd=$(echo ./shiftopi.py --shiftx $x --shifty $y --shiftm $im) &&
  echo $0: $BASENAME cmd=$cmd &&
  eval $cmd <$BASENAME.mid >>/tmp/$$ &&
  cmd=$(echo ./shiftopi.py --shiftx $x --shifty $y --shiftm $im)
echo $0: cmd=$cmd
eval $cmd <e-cnen.mid >>/tmp/$$
eval $cmd <e-foff.mid >>/tmp/$$
eval $cmd <e-homf-cal-homr.mid >>/tmp/$$
eval $cmd <e-retry.mid >>/tmp/$$
eval $cmd <e-URIP.mid >>/tmp/$$
eval $cmd <e-openloop.mid >>/tmp/$$
# create extra analog output records
n=0
for MOTORRECORDFIELD in $MOTORRECORDFIELDS; do
  echo MOTORRECORDFIELD=$MOTORRECORDFIELD
  cmd=$(echo ./genExpertWithAuxBits.py --shiftn $n --shifty $yauxleft)
  echo cmd=$cmd
  eval $cmd <smaractMotorRecordField.mid | sed -e "s/fieldXXYYZZ/$MOTORRECORDFIELD/g" >>/tmp/$$
  yauxleft=$(($yauxleft + 20))
  n=$(($n + 1))
done
for ANALOGOUT in $ANALOGOUTS; do
  echo ANALOGOUT=$ANALOGOUT
  cmd=$(echo ./genExpertWithAuxBits.py --shiftn $n --shifty $yauxleft)
  echo cmd=$cmd
  eval $cmd <smaractAnalogOutput.mid | sed -e "s/AoXXYYZZ/$ANALOGOUT/g" >>/tmp/$$
  yauxleft=$(($yauxleft + 20))
  n=$(($n + 1))
done

# create extra analog input records
n=0
for ANALOGIN in $ANALOGINS; do
  echo ANALOGIN=$ANALOGIN
  cmd=$(echo ./genExpertWithAuxBits.py --shiftn $n --shiftx 320 --shifty $yauxright)
  echo cmd=$cmd
  eval $cmd <smaractAnalogInput.mid | sed -e "s/AiXXYYZZ/$ANALOGIN/g" >>/tmp/$$
  yauxright=$(($yauxright + 20))
  n=$(($n + 1))
done

# Whichever has more columns, left or rigth
if test $yauxright -gt $yauxleft; then
  yaux=$yauxright
else
  yaux=$yauxleft
fi

# Create the status bits
n=0
xaux=0
yauxBase=$yaux
for NAMEPSTATBIT in $NAMEPSTATBITS; do
  echo NAMEPSTATBIT=$NAMEPSTATBIT
  yaux=$(($yaux + 20))
  cmd=$(echo ./genExpertWithAuxBits.py --shiftn $n --shiftx $xaux --shifty $yaux)
  echo cmd=$cmd
  eval $cmd <smaractaxisPiezoPSTAT.mid | sed -e "s/StatusBits/$STATUSBITS/g" -e "s/NamPstatBitXXX/$NAMEPSTATBIT/g" >>/tmp/$$
  shift
  n=$(($n + 1))
  if test $yaux -ge $(($yauxBase + 220)); then
    # open the second column
    xaux=$(($xaux + 200))
    yaux=$yauxBase
  fi
done
touch "$FILE" &&
  chmod +w "$FILE" &&
  cat $BASENAME.end >>/tmp/$$ &&
  mv -f /tmp/$$ "$FILE"
chmod -w "$FILE"
