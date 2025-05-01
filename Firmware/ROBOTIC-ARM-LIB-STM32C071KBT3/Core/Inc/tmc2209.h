/*
 * tmc2209.h
 *
 *  Created on: Feb 12, 2025
 *      Author: noah
 */

#ifndef INC_TMC2209_H_
#define INC_TMC2209_H_

#include "main.h"
#include "motor_defines.h"
#include <string.h>
#include <stdio.h>
#include <stdbool.h>

extern TIM_HandleTypeDef htim1;
extern UART_HandleTypeDef huart1;

typedef struct {
	GPIO_TypeDef* ENABLE_PORT;
	uint16_t ENABLE_PIN;

	GPIO_TypeDef* MS1_PORT;
	uint16_t MS1_PIN;

	GPIO_TypeDef* MS2_PORT;
	uint16_t MS2_PIN;

	GPIO_TypeDef* STEP_PORT;
	uint16_t STEP_PIN;

	GPIO_TypeDef* DIR_PORT;
	uint16_t DIR_PIN;

	GPIO_TypeDef* ENDSTOP_PORT;
	uint16_t ENDSTOP_PIN;

	uint8_t ENDSTOP_FLAG;

	uint16_t position;
	uint16_t position_max;

} stepper_driver_t;

void stepper_init(stepper_driver_t *driver,
                  GPIO_TypeDef* enable_port, uint16_t enable_pin,
                  GPIO_TypeDef* ms1_port, uint16_t ms1_pin,
                  GPIO_TypeDef* ms2_port, uint16_t ms2_pin,
                  GPIO_TypeDef* step_port, uint16_t step_pin,
                  GPIO_TypeDef* dir_port, uint16_t dir_pin,
				  GPIO_TypeDef* endstop_port, uint16_t endstop_pin);

void stepper_configure_steps(stepper_driver_t *driver, uint8_t step_mode);

void stepper_enable(stepper_driver_t *driver);

void stepper_disable(stepper_driver_t *driver);

void delay_us (uint16_t delay);

void stepper_move(stepper_driver_t *driver_one, stepper_driver_t *driver_two, uint16_t steps, bool move_x, bool move_y);

void stepper_process_command(stepper_driver_t *driver_one, stepper_driver_t *driver_two, volatile char* command);

void stepper_home(stepper_driver_t *driver_one, stepper_driver_t *driver_two, stepper_driver_t *driver_three);


#endif /* INC_TMC2209_H_ */
