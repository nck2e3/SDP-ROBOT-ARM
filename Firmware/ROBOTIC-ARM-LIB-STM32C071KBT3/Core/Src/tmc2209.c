/*
 * tmc2209.c
 *
 *  Created on: Feb 12, 2025
 *      Author: noah
 */



#include "tmc2209.h"

void stepper_init(stepper_driver_t *driver,
                  GPIO_TypeDef* enable_port, uint16_t enable_pin,
                  GPIO_TypeDef* ms1_port, uint16_t ms1_pin,
                  GPIO_TypeDef* ms2_port, uint16_t ms2_pin,
                  GPIO_TypeDef* step_port, uint16_t step_pin,
                  GPIO_TypeDef* dir_port, uint16_t dir_pin,
				  GPIO_TypeDef* endstop_port, uint16_t endstop_pin) {

    driver->ENABLE_PORT = enable_port;
    driver->ENABLE_PIN = enable_pin;

    driver->MS1_PORT = ms1_port;
    driver->MS1_PIN = ms1_pin;

    driver->MS2_PORT = ms2_port;
    driver->MS2_PIN = ms2_pin;

    driver->STEP_PORT = step_port;
    driver->STEP_PIN = step_pin;

    driver->DIR_PORT = dir_port;
    driver->DIR_PIN = dir_pin;

    driver->DIR_PORT = dir_port;
    driver->DIR_PIN = dir_pin;

    driver->ENDSTOP_PORT = endstop_port;
    driver->ENDSTOP_PIN = endstop_pin;

    driver->ENDSTOP_FLAG = 0;

    driver->position = 0; 		    // Default Position
    driver->position_max = 32*1.8;  // Max Steps 32 degrees 1/64th microstep...

    stepper_configure_steps(driver, 2); // 0'b01 1/64th Microstep Default
    stepper_enable(driver);				// Enable motor by default...
}

void stepper_configure_steps(stepper_driver_t *driver, uint8_t step_mode)
{
	switch (step_mode)
	{
		case 0:	// {MS2,MS1} = 2'b00 : 1/8th Microstep
		  HAL_GPIO_WritePin(driver->MS2_PORT, driver->MS2_PIN, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(driver->MS1_PORT, driver->MS1_PIN, GPIO_PIN_RESET);
		  break;
		case 1:	// {MS2,MS1} = 2'b01 : 1/32nd Microstep
		  HAL_GPIO_WritePin(driver->MS2_PORT, driver->MS2_PIN, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(driver->MS1_PORT, driver->MS1_PIN, GPIO_PIN_SET);
		  break;
		case 2:	// {MS2,MS1} = 2'b10 : 1/64th Microstep
		  HAL_GPIO_WritePin(driver->MS2_PORT, driver->MS2_PIN, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(driver->MS1_PORT, driver->MS1_PIN, GPIO_PIN_RESET);
		  break;
		case 3: 	// {MS2,MS1} = 2'b11 : 1/16th Microstep
		  HAL_GPIO_WritePin(driver->MS2_PORT, driver->MS2_PIN, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(driver->MS1_PORT, driver->MS1_PIN, GPIO_PIN_SET);
		  break;
		default:	// {MS2,MS1} = 2'b00 : 1/8th Microstep
		  HAL_GPIO_WritePin(driver->MS2_PORT, driver->MS2_PIN, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(driver->MS1_PORT, driver->MS1_PIN, GPIO_PIN_RESET);
		  break;
	}
}

void stepper_enable(stepper_driver_t *driver)
{
	// ENABLE MOTOR (ACTIVE LOW, DRIVE LOW TO ENABLE)
	HAL_GPIO_WritePin(driver->ENABLE_PORT, driver->ENABLE_PIN, GPIO_PIN_RESET);

}

void stepper_disable(stepper_driver_t *driver)
{
	// DISABLE MOTOR (ACTIVE LOW, DRIVE HIGH TO DISABLE)
	HAL_GPIO_WritePin(driver->ENABLE_PORT, driver->ENABLE_PIN, GPIO_PIN_SET);
}

void delay_us (uint16_t delay) //REMEMBER TO CONFIGURE TIMER CORRECTLY!!!
{
  __HAL_TIM_SET_COUNTER(&htim1, 0);
  while (__HAL_TIM_GET_COUNTER(&htim1) < delay);
}

void stepper_move(stepper_driver_t *driver_one, stepper_driver_t *driver_two, uint16_t steps, bool move_x, bool move_y)
{
    for (uint16_t i = 0; i < steps; i++)
    {
        if (move_x)
        {
            HAL_GPIO_WritePin(driver_one->STEP_PORT, driver_one->STEP_PIN, GPIO_PIN_SET);
        }
        if (move_y)
        {
            HAL_GPIO_WritePin(driver_two->STEP_PORT, driver_two->STEP_PIN, GPIO_PIN_SET);
        }
        delay_us(125);

        if (move_x)
        {
            HAL_GPIO_WritePin(driver_one->STEP_PORT, driver_one->STEP_PIN, GPIO_PIN_RESET);
        }
        if (move_y)
        {
            HAL_GPIO_WritePin(driver_two->STEP_PORT, driver_two->STEP_PIN, GPIO_PIN_RESET);
        }
        delay_us(250);
    }
}

void stepper_process_command(stepper_driver_t *driver_one, stepper_driver_t *driver_two, volatile char* command)
{



    // Temporary buffers for extracting directions from the command string
    char *dx_command = strtok(command, ",");
    char *dy_command = strtok(NULL, ",");  // Assumes command is in the format "DX, DY"

    bool move_x = false;
    bool move_y = false;
    uint16_t steps = 64;

    if (dx_command != NULL && dy_command != NULL)
    {
        // Handle the dx direction (LEFT/RIGHT or NONE)
        if(strcmp(dx_command, "LEFT") == 0)
        {
            HAL_GPIO_WritePin(driver_one->DIR_PORT, driver_one->DIR_PIN, GPIO_PIN_RESET);
            move_x = true;
        }
        else if(strcmp(dx_command, "RIGHT") == 0)
        {
            HAL_GPIO_WritePin(driver_one->DIR_PORT, driver_one->DIR_PIN, GPIO_PIN_SET);
            move_x = true;
        }

        // Handle the dy direction (UP/DOWN or NONE)
        if(strcmp(dy_command, "DOWN") == 0)
        {
            if(driver_two->position + steps <= driver_two->position_max)
            {
                HAL_GPIO_WritePin(driver_two->DIR_PORT, driver_two->DIR_PIN, GPIO_PIN_RESET);
                move_y = true;
                driver_two->position += steps;
            }
        }
        else if(strcmp(dy_command, "UP") == 0)
        {
            if(HAL_GPIO_ReadPin(driver_two->ENDSTOP_PORT, driver_two->ENDSTOP_PIN) == GPIO_PIN_SET)
            {
                if(driver_two->position >= steps)
                {
                    HAL_GPIO_WritePin(driver_two->DIR_PORT, driver_two->DIR_PIN, GPIO_PIN_SET);
                    move_y = true;
                    driver_two->position -= steps;
                    if(steps < 0)
                    	driver_two->position = 0;
                }
            }
        }

        // Move motors only in the specified directions
        if (move_x || move_y)
        {
            stepper_move(driver_one, driver_two, steps, move_x, move_y);
        }
    }
    else
    {
        // Handle invalid command format
        char debug_msg[32];
        sprintf(debug_msg, "Invalid command format: %s\r\n", command);
        HAL_UART_Transmit(&huart1, (uint8_t*)debug_msg, strlen(debug_msg), HAL_MAX_DELAY);
    }
}

void stepper_home(stepper_driver_t *driver_one, stepper_driver_t *driver_two, stepper_driver_t *driver_three)
{
	uint16_t steps = 1;


	HAL_GPIO_WritePin(driver_two->DIR_PORT, driver_two->DIR_PIN, GPIO_PIN_RESET);	// MOVE TOWARDS SWITCH
	HAL_GPIO_WritePin(driver_three->DIR_PORT, driver_three->DIR_PIN, GPIO_PIN_SET); // MOVE TOWARDS SWITCH


	while(driver_three->ENDSTOP_FLAG == 0)
	{
		if(HAL_GPIO_ReadPin(driver_three->ENDSTOP_PORT, driver_three->ENDSTOP_PIN) == GPIO_PIN_RESET)
		{
			driver_three->ENDSTOP_FLAG = 1;
			driver_three->position = 0;
			break;
		} else {
			stepper_move(driver_two, driver_three, steps, false, true);

		}
	}

	while(driver_two->ENDSTOP_FLAG == 0)
	{
		if(HAL_GPIO_ReadPin(driver_two->ENDSTOP_PORT, driver_two->ENDSTOP_PIN) == GPIO_PIN_RESET)
		{
			driver_two->ENDSTOP_FLAG = 1;
			driver_two->position = 0;
			break;
		} else {
			stepper_move(driver_two, driver_three, steps, true, false);

		}
	}



	HAL_GPIO_WritePin(driver_two->DIR_PORT, driver_two->DIR_PIN, GPIO_PIN_SET);
	uint16_t driver_two_position = 64 * 95;
	for(int i = 0; i < driver_two_position; i++)
	{
		stepper_move(driver_two, driver_three, steps, true, false);
		driver_two->position += 64;
	}

	driver_three->position_max = 90*64 + driver_two_position;



}



