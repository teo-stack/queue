CFLAGS		+= -I./sources/app
CPPFLAGS	+= -I./sources/app

VPATH += sources/app

SOURCES += sources/app/app.c
SOURCES += sources/app/I2C_STM32F0.c
SOURCES += sources/app/SHTx.c
