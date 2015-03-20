/*
 * delay.c
 *
 * Created: 30.09.2014 01:27:39
 *  Author: Kjetil
 */ 

#include "delay.h"

static uint32_t freq = 4000000; // default value for RC ocsilator at startup

void delay_set_frequency(uint32_t frequency)
{
	freq = frequency;
}

// sjekk register eventuelt bruke cycles i assembly
void delay_clk(volatile uint32_t cycles) // is this safe or can cycles be optimized away?? worked so far....
{
	// R0 and R1 can be used freely inside the function, referance ARM calling convetion
	__asm("MOV R1, #3"); // 3 clock cycles per loop
	__asm("UDIV R0, R1"); // unsigned division
	__asm("loop: SUBS R0, R0, #1");
	__asm("BNE loop");
}

void delay_ms(uint32_t ms)
{
	delay_clk(ms*(freq/1000));
}

void delay_us(uint32_t us)
{
	delay_clk(us*(freq/1000000));
}