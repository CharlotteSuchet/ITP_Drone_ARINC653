CC=/opt/Codesourcery/bin/arm-none-linux-gnueabi-gcc
LS=$(CC)
CFLAGS= -march=armv7-a -lm

EXE = acquisition_capteur \
	  clean

all:	$(EXE)

acquisition_capteur: acquisition_capteur.o 			    					\
			log.o					     						\
			math/pprz_trig_int.o 								\
			math/pprz_orientation_conversion.o					\
			math/pprz_geodetic_double.o							\
			math/pprz_geodetic_float.o							\
			math/pprz_geodetic_int.o							\
			state.o												\
			sys_time.o											\
			sys_time_arch.o										\
			capteur/baro_board.o 								\
			capteur/navdata.o  									\
			capteur/imu.o 										\
			capteur/ins.o										\
			capteur/ins_int.o									\
			capteur/vf_float.o									\
			stabilization/stabilization_attitude_ref_quat_int.o \
			stabilization/stabilization_attitude_quat_int.o 	\
			stabilization/guidance_v.o							\
			stabilization/guidance_v_ref.o						\
			moteur/gpio_ardrone.o								\
			moteur/actuators_ardrone2_raw.o						\
			moteur/actuators.o									\
			moteur/motor_mixing.o								\
			util/util.o                                                                                      \
			commands.o
	$(CC) $(CFLAGS) -o $@.elf $^


.PHONY: depend clean realclean all tests

depend:
	makedepend -Y -r *.c

clean:

	-$(RM) *.o
	-$(RM) */*.o
	-$(RM) */*/*.o
	-$(RM) gmon.out
	-$(RM) *.bak core Makefile.bak
	-$(RM) $(EXE) a.out

realclean: clean

# DO NOT DELETE

commands.o: commands.h conf.h
log.o: log.h
state.o: state.h math/pprz_algebra_int.h std.h math/pprz_algebra.h
state.o: math/pprz_trig_int.h math/pprz_algebra_int.h
state.o: math/pprz_algebra_float.h math/pprz_geodetic_int.h
state.o: math/pprz_geodetic.h math/pprz_geodetic_float.h
state.o: math/pprz_algebra_float.h math/pprz_orientation_conversion.h std.h
sys_time_arch.o: sys_time.h std.h sys_time_arch.h
sys_time.o: sys_time.h std.h sys_time_arch.h
acquisition_capteur.o: sys_time.h std.h sys_time_arch.h log.h state.h
acquisition_capteur.o: math/pprz_algebra_int.h std.h math/pprz_algebra.h
acquisition_capteur.o: math/pprz_trig_int.h math/pprz_algebra_int.h
acquisition_capteur.o: math/pprz_algebra_float.h math/pprz_geodetic_int.h
acquisition_capteur.o: math/pprz_geodetic.h math/pprz_geodetic_float.h
acquisition_capteur.o: math/pprz_algebra_float.h math/pprz_orientation_conversion.h
acquisition_capteur.o: commands.h conf.h capteur/baro_board.h capteur/navdata.h
acquisition_capteur.o: capteur/imu.h math/pprz_algebra_int.h
acquisition_capteur.o: math/pprz_algebra_float.h capteur/conf_capteur.h
acquisition_capteur.o: capteur/ahrs.h state.h capteur/ahrs/ahrs_int_cmpl_quat.h
acquisition_capteur.o: capteur/ahrs.h std.h math/pprz_algebra_int.h
acquisition_capteur.o: capteur/ahrs/ahrs_aligner.h capteur/ins.h
acquisition_capteur.o: math/pprz_geodetic_int.h capteur/ins_int.h capteur/ins.h
acquisition_capteur.o: stabilization/stabilization_attitude_quat_int.h
acquisition_capteur.o: stabilization/stabilization_attitude_common_int.h
acquisition_capteur.o: stabilization/conf_asser.h
acquisition_capteur.o: stabilization/stabilization_attitude_ref_quat_int.h
acquisition_capteur.o: stabilization/stabilization_attitude_ref_int.h
acquisition_capteur.o: stabilization/guidance_v.h stabilization/guidance_v_ref.h
acquisition_capteur.o: math/pprz_algebra.h stabilization/guidance_v_adpt.h
acquisition_capteur.o: moteur/actuators.h moteur/conf_moteur.h
acquisition_capteur.o: util/util.h
