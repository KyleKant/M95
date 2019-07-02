#ifndef __CONFIG_H_
#define __CONFIG_H_

#include <24ep512gp806.h>
#device adc = 10
#fuses noprotect, nobrownout, nowdt, cksfsm, nojtag, icsp1
#use delay(clock =40000000, crystal = 20000000)

#pin_select U1TX=PIN_E2
#pin_select U1RX=PIN_E4
#pin_select U2TX = PIN_F2
#pin_select U2RX = PIN_F6
#pin_select U3TX = PIN_E0
#pin_select U3RX = PIN_F1

#use rs232(UART1, errors, brgh1ok, baud= 57600, stream = DEBUG)
#use rs232(UART3, errors, brgh1ok, baud = 115200, stream = SIMM95)
#use rs232(UART2, errors, baud = 9600, parity = n, bits = 8, stop = 1, stream = MODBUS)

#use timer(TIMER = 1, tick = 1ms, bits = 16, noisr, stream = T1)
#use timer(TIMER = 2, tick = 1ms, bits = 16, noisr, stream = T2)
#use timer(TIMER = 3, tick = 0.1ms, bits = 16, noisr, stream = T3)

#define	pkey		pin_e1
#define	linkLed	pin_e5
#define	pwrLed	pin_e6
#define	dataLed	pin_e7

#define pinControl485	pin_f3

#endif