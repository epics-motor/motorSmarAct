#!/bin/sh

echo genMotorShutter.sh "$@"
# Number of motors in Y direction
Y=1
# Hight of one "motor widget"
MOTORHIGHT=204
# Width of one "motor widget"
WIDTH=120
# hight of a temperature wdget
TEMPSENSORHIGHT=36
# File name extension
EXT=opi
y0=16

export y0 WIDTH MOTORHIGHT TEMPSENSORHIGHT

genMatrix() {
  echo $0::genMatrix "$@"
  numparameaten=0
  cntx=0
  cnty=0
  y=$y0
  im=0 # motor number, start at 0
  it=0 # tempsensor number, start at 0
  XCNTMAX=1
  YCNTMAX=1
  haveseenY=0
  while test -n "$1"; do
    echo genMatrix "numparameaten=$numparameaten param=$1"
    case "$1" in
      [123456789])
        if test $haveseenY -eq 0; then
          XCNTMAX=$1
          echo genMatrix XCNTMAX=$XCNTMAX
        elif test $haveseenY -eq 1; then
          cntx=0
          YCNTMAX=$1
          echo genMatrix YCNTMAX=$YCNTMAX
          # loop x times y
          cntx=0
          echo genMatrix y=$y cnty=$cnty YCNTMAX=$YCNTMAX cntx=$cntx XCNTMAX=$XCNTMAX
          while test $cnty -lt $YCNTMAX; do
            while test $cntx -lt $XCNTMAX; do
              x=$(($cntx * $WIDTH))
              cmd=$(echo ./shiftopi.py --shiftx $x --shifty $y --shiftm $im)
              echo xcmd=$cmd <motorx-piezo.mid
              eval $cmd <motorx-piezo.mid >>/tmp/$$
              im=$(($im + 1))
              cntx=$(($cntx + 1))
            done
            cntx=0
            cnty=$(($cnty + 1))
            y=$(($y + $MOTORHIGHT))
          done
        else
          echo >&2 "Illegale numbers"
          echo >&2 "numparameaten=$numparameaten haveseenY=$haveseenY"
          exit 1
        fi
        ;;
      x)
        haveseenY=$(($haveseenY + 1))
        ;;
      *)
        echo >&2 "invalid: parameter $1"
        echo >&2 "allowed: m s n"
        exit 1
        ;;
    esac
    shift
    numparameaten=$(($numparameaten + 1))
  done
}

########################

FILE=$1
shift
BASENAMEF=${FILE##*/}

  sed -e "s!<name>motorx</name>!<name>$BASENAMEF</name>!" <motorx.start >/tmp/$$ &&
  echo "Creating $FILE" &&
  genMatrix "$@" &&
  cat motorx.end >>/tmp/$$ &&
  touch $FILE &&
  chmod +w $FILE &&
  mv /tmp/$$ $FILE &&
  chmod -w $FILE
