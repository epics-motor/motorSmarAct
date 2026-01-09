
GNERATEDOPIS = \
  ../motor-1-piezo.opi \
  ../motor-2-piezo.opi \
  ../smaractAxisExpert.opi \
  ../IOC.opi \


#Main target:
all: checkwhitespace $(GNERATEDOPIS)

clean:
	rm -f $(GNERATEDOPIS)

DEPENDENCIES_FOR_MOTORX_PIEZO_OPI = \
	m-motor.mid \
	m-msgtxt.mid \


# Depencies, when do we need to revuild a .opi file
DEPENDENCIES_FOR_MOTOR_OPI = Makefile
DEPENDENCIES_FOR_MOTOR_OPI += genMotor.sh
DEPENDENCIES_FOR_MOTOR_OPI += shiftopi.py
DEPENDENCIES_FOR_MOTOR_OPI += motorx.start
DEPENDENCIES_FOR_MOTOR_OPI += m-egu-rbv.mid
DEPENDENCIES_FOR_MOTOR_OPI += motorx.end


# expert screens. e- stands for expert
DEPENDENCIES_FOR_MOTOR_EXPERT_OPI = Makefile
DEPENDENCIES_FOR_MOTOR_EXPERT_OPI += genExpertPiezo.sh
DEPENDENCIES_FOR_MOTOR_EXPERT_OPI += e-cnen.mid
DEPENDENCIES_FOR_MOTOR_EXPERT_OPI += e-cnen-vis.mid
DEPENDENCIES_FOR_MOTOR_EXPERT_OPI += e-foff.mid
DEPENDENCIES_FOR_MOTOR_EXPERT_OPI += e-foff-vis.mid
DEPENDENCIES_FOR_MOTOR_EXPERT_OPI += e-homf-cal-homr.mid
DEPENDENCIES_FOR_MOTOR_EXPERT_OPI += e-retry.mid
DEPENDENCIES_FOR_MOTOR_EXPERT_OPI += e-urip.mid
DEPENDENCIES_FOR_MOTOR_EXPERT_OPI += e-openloop.mid
DEPENDENCIES_FOR_MOTOR_EXPERT_OPI += e-errtxt.mid
DEPENDENCIES_FOR_MOTOR_EXPERT_OPI += smaractAnalogInput.mid
DEPENDENCIES_FOR_MOTOR_EXPERT_OPI += smaractAnalogOutput.mid
DEPENDENCIES_FOR_MOTOR_EXPERT_OPI += smaractAxisExpert.start
DEPENDENCIES_FOR_MOTOR_EXPERT_OPI += smaractAxisExpert.mid
DEPENDENCIES_FOR_MOTOR_EXPERT_OPI += smaractMotorRecordField.mid
DEPENDENCIES_FOR_MOTOR_EXPERT_OPI += smaractAxisPiezoPSTAT.mid
DEPENDENCIES_FOR_MOTOR_EXPERT_OPI += smaractAxisExpert.end
DEPENDENCIES_FOR_MOTOR_EXPERT_OPI += genExpertWithAuxBits.py
DEPENDENCIES_FOR_MOTOR_EXPERT_OPI += genExpertPiezo.sh


./e-cnen.mid: e-cnen-vis.mid Makefile
	sed -e '/<rules>/,/<\/rules>/d'<e-cnen-vis.mid >$@

./e-foff.mid: e-foff-vis.mid Makefile
	sed -e '/<rules>/,/<\/rules>/d'<e-foff-vis.mid >$@

./e-homf-homr.mid: e-homf-homr-vis.mid Makefile
	sed -e '/<rules>/,/<\/rules>/d'<e-homf-homr-vis.mid >$@

./motorx-piezo.mid: $(DEPENDENCIES_FOR_MOTORX_PIEZO_OPI) Makefile
	echo $(DEPENDENCIES_FOR_MOTORX_PIEZO_OPI) '>$@'
	cat $(DEPENDENCIES_FOR_MOTORX_PIEZO_OPI) >$@

../motor-1-piezo.opi: $(DEPENDENCIES_FOR_MOTOR_OPI) ./motorx-piezo.mid
	./genMotor.sh $@ 1 x 1

../motor-2-piezo.opi: $(DEPENDENCIES_FOR_MOTOR_OPI) ./motorx-piezo.mid
	./genMotor.sh $@ 2 x 1

../smaractAxisExpert.opi: $(DEPENDENCIES_FOR_MOTOR_EXPERT_OPI)
	./genExpertPiezo.sh $@ smaractAxisExpert

../IOC.opi: Makefile genIOC.sh IOC*.*
	./genIOC.sh ../IOC.opi IOC


checkwhitespace:
	./checkws.sh

checkshellfmt:
	shfmt -i 2 -ci -d -w .
