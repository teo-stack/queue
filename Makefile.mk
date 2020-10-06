CFLAGS		+= -I./sources/app
CPPFLAGS	+= -I./sources/app

VPATH += sources/app

SOURCES += sources/app/app.c
SOURCES += sources/app/system_i2c.c
SOURCES += sources/app/i2c_hal.c
SOURCES += sources/app/sht3x.c
