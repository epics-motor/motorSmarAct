#!/bin/sh
# script to run make and the starts the IOC with a logfile if wanted

## functions
help_and_exit() {
  echo >&2 $0 "[simulator|handheld]"
  echo >&2 $0 "[simulator|handheld] [-l]"
  echo >&2 $0 "[simulator|handheld] [-l] creates a log file"
  exit 1
}


# pick up arguments
PARAM="$1"
while test "$PARAM" != ""; do
	#echo "loop: PARAM=$PARAM"
  case "$PARAM" in
    -h | --help)
      help_and_exit help "$@"
      ;;
    -l)
      DOLOG=y
      ;;
		simulator)
			DEVICE="$PARAM"
			;;
		handheld)
			DEVICE="$PARAM"
			;;
    *)
      help_and_exit asterix "$@"
      ;;
  esac
  shift
  PARAM="$1"
done

if test "$DEVICE" = ""; then
  help_and_exit "device missing"
fi

# log files handling, save old logfiles
LOG_TXT=log-${DEVICE}.txt
if test -f $LOG_TXT; then
  timestamp=$(date "+%y-%m-%d-%H.%M.%S")
  mkdir -p ./logs/ &&
    mv $LOG_TXT ./logs/$timestamp-$LOG_TXT || exit 1
fi


# log/tee to file
(cd ../../../.. && make  )  &&
if test "$DOLOG" = y; then
  ./st-${DEVICE}.cmd  2>&1 | tee $LOG_TXT
else
  ./st-${DEVICE}.cmd
fi

