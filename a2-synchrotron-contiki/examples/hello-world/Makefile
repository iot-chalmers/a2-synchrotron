TARGET ?= sky

CONTIKI_PROJECT = hello-world
CONTIKI_WITH_CHAOS = 0
CONTIKI_WITH_RIME = 1

all: $(CONTIKI_PROJECT)

CONTIKI = ../..
include $(CONTIKI)/Makefile.include

cooja: CFLAGS+=-DCOOJA=1
cooja: $(CONTIKI_PROJECT)

