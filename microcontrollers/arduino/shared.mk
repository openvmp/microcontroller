OBJDIR = ../../../../build/microcontrollers/arduino
OBJDIR_CREATE = $(mkdir -p $OBJDIR)

USER_LIB_PATH := ../
ARDUINO_LIBS = arduino
ARDUINO_LIBS += servo softwareserial

CFLAGS = -I$(realpath .) -I../../include
CXXFLAGS = -I$(realpath .) -I../../include
LDFLAGS = # Ignore LDFLAGS supplied by OS or the dev environment

default: clean all

-include /usr/share/arduino/Arduino.mk # Linux
-include /usr/local/opt/arduino-mk/Arduino.mk # MacOS
