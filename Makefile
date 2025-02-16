# Author: Josh Williams

BINARY		= main
SRCFILES	= main.c miniprintf.c rtos/heap_4.c rtos/list.c rtos/port.c rtos/tasks.c rtos/opencm3.c ili9341_driver.c fonts.c #rtos/queue.c
LDSCRIPT	= stm32f103c8t6.ld

include Makefile.incl
include Makefile.rtos
