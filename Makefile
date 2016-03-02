CC=/opt/Codesourcery/bin/arm-none-linux-gnueabi-gcc
LS=$(CC)
CFLAGS= -march=armv7-a -lm -lpthread

EXE = main_stabilisation.elf \
	  clean

all:	$(EXE)

main_stabilisation.elf: main_stabilisation.o 			    					\
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
			capteur/vf_float.o									\
			util/util.o                                                                                      \
			stabilisation/stabilisation.o                       \
			gpio/gpio.o                         \
			motorboard/mot.o   \
			motorboard/motorboard.o   \
			commands.o
	$(CC) $(CFLAGS) -o $@ $^


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
main_stabilisation.o: sys_time.h std.h sys_time_arch.h log.h state.h
main_stabilisation.o: math/pprz_algebra_int.h std.h math/pprz_algebra.h
main_stabilisation.o: math/pprz_trig_int.h math/pprz_algebra_int.h
main_stabilisation.o: math/pprz_algebra_float.h math/pprz_geodetic_int.h
main_stabilisation.o: math/pprz_geodetic.h math/pprz_geodetic_float.h
main_stabilisation.o: math/pprz_algebra_float.h math/pprz_orientation_conversion.h
main_stabilisation.o: commands.h conf.h capteur/baro_board.h capteur/navdata.h
main_stabilisation.o: capteur/imu.h math/pprz_algebra_int.h
main_stabilisation.o: math/pprz_algebra_float.h capteur/conf_capteur.h
main_stabilisation.o: math/pprz_geodetic_int.h capteur/ins_int.h capteur/ins.h
main_stabilisation.o: math/pprz_algebra.h
main_stabilisation.o: util/util.h
main_stabilisation.o: util/type.h
main_stabilisation.o: stabilisation/stabilisation.h
main_stabilisation.o: gpio/gpio.h
main_stabilisation.o: motorboard/mot.h
main_stabilisation.o: motorboard/motorboard.h



