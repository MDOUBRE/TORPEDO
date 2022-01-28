
# configuration
APP = main
STLINK_ROOT=stlink-1.5.1/build/Release
LOG = stm32f4.log
#GDB = gdb-multiarch
GDB = gdb
GDB_FLAGS=-command ../scripts/gdbinit

# rules
all:
	cd src; $(MAKE) all

clean:
	rm -f $(LOG)
	cd src; $(MAKE) clean

openocd:
	-rm $(LOG)
	openocd -f scripts/openocd.cfg

debug:
	gdb-multiarch src/$(APP).elf

log:
	tail -f $(LOG)


# exercises
moteurs:
	cd src; $(MAKE) moteurs.elf

debug_moteurs: moteurs
	cd src; $(GDB) $(GDB_FLAGS) moteurs.elf

suivi_de_ligne:
	cd src; $(MAKE) suivi_de_ligne.elf

debug_suivi_de_ligne: suivi_de_ligne
	cd src; $(GDB) $(GDB_FLAGS) suivi_de_ligne.elf
	
	
# build distribution
DIST_FILES = \
	COPYING.md \
	.gdbinit \
	include \
	scripts \
	Makefile
DIST_SRC = \
	src/Makefile \
	src/startup.c \
	src/tinyprintf.c \
	src/moteurs.c \
	src/suivi_de_ligne.c
	
DIST_NAME=code

dist:
	if [ -e $(DIST_NAME) ]; then \
		rm -rf $(DIST_NAME); \
	fi
	mkdir $(DIST_NAME)
	cp -R $(DIST_FILES) $(DIST_NAME)
	mkdir $(DIST_NAME)/src
	#cp scripts/Makefile.014 $(DIST_NAME)/Makefile
	for f in $(DIST_SRC); do \
		autofilter.py < $$f > $(DIST_NAME)/$$f; \
	done
	cd $(DIST_NAME); make; make clean
	tar cvfz $(DIST_NAME).tgz $(DIST_NAME)

DATE = $(shell date +"%y%m%d")
assign:
	tar cvfz assign-$(DATE).tgz src/*.c
