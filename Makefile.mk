CFLAGS		+= -I./sources/app
CPPFLAGS	+= -I./sources/app

VPATH += sources/app

SOURCES += sources/app/app.c
#SOURCES += sources/app/queue.c
SOURCES += sources/app/queuell.c
