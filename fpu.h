/*
 * fpu.h
 *
 * Created: 30.03.2015 11:51:14
 *  Author: Simen
 */ 


#ifndef FPU_H_
#define FPU_H_

#include <stdbool.h>
#include <stdint.h>

/** Address for ARM CPACR */
#define ADDR_CPACR 0xE000ED88
/** CPACR Register */
#define REG_CPACR (*((volatile uint32_t *)ADDR_CPACR))
/**
 * Enable FPU
 */
static void fpu_enable(void)
{
	 REG_CPACR |= (0xFu << 20);
}
/**
 * Disable FPU
 */
static void fpu_disable(void)
{
 REG_CPACR &= ~(0xFu << 20);
}
/**
 * Check if FPU is enabled
 */
static bool fpu_is_enabled(void)
{
 return (REG_CPACR & (0xFu << 20));
}



#endif /* FPU_H_ */