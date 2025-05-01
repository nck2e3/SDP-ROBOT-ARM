/*
 * motor_defines.h
 *
 *  Created on: Feb 12, 2025
 *      Author: noah
 */

#ifndef INC_MOTOR_DEFINES_H_
#define INC_MOTOR_DEFINES_H_


// === Global Step Control (All Motors) ===
#define GLOBAL_STEP_DELAY 1000

#define GLOBAL_MS1_PIN    GPIO_PIN_5   // PB5: Microstepping pin 1
#define GLOBAL_MS1_PORT   GPIOB

#define GLOBAL_MS2_PIN    GPIO_PIN_4   // PB4: Microstepping pin 2
#define GLOBAL_MS2_PORT   GPIOB


// === Motor #1 Control Pins ===
#define M1_EN_PIN         GPIO_PIN_6   // PB6: Enable pin for TMC2209 (active-low)
#define M1_EN_PORT        GPIOB

#define M1_STEP_PIN       GPIO_PIN_3   // PB3: Step pulse pin for TMC2209
#define M1_STEP_PORT      GPIOB

#define M1_DIR_PIN        GPIO_PIN_15  // PA15: Direction pin for TMC2209
#define M1_DIR_PORT       GPIOA

#define M1_ENDSTOP_PIN    GPIO_PIN_9   // PB9: Endstop switch input
#define M1_ENDSTOP_PORT   GPIOB



// === Motor #2 Control Pins ===
#define M2_EN_PIN         GPIO_PIN_12  // PA12: Enable pin for TMC2209 (active-low)
#define M2_EN_PORT        GPIOA

#define M2_STEP_PIN       GPIO_PIN_11  // P11: Step pulse pin for TMC2209
#define M2_STEP_PORT      GPIOA

#define M2_DIR_PIN        GPIO_PIN_10  // PA10: Direction pin for TMC2209
#define M2_DIR_PORT       GPIOA

#define M2_ENDSTOP_PIN    GPIO_PIN_8   // PB8: Endstop switch input
#define M2_ENDSTOP_PORT   GPIOB

#define M2_DEBUG_PIN	  GPIO_PIN_1   // PB1: Debug LED
#define M2_DEBUG_PORT	  GPIOB

// === Motor #3 Control Pins ===
#define M3_EN_PIN         GPIO_PIN_6   // PC6: Enable pin for TMC2209 (active-low)
#define M3_EN_PORT        GPIOC

#define M3_STEP_PIN       GPIO_PIN_9   // PA9: Step pulse pin for TMC2209
#define M3_STEP_PORT      GPIOA

#define M3_DIR_PIN        GPIO_PIN_8   // PA8: Direction pin for TMC2209
#define M3_DIR_PORT       GPIOA

#define M3_ENDSTOP_PIN    GPIO_PIN_7   // PB7: Endstop switch input
#define M3_ENDSTOP_PORT   GPIOB

#define M3_DEBUG_PIN	  GPIO_PIN_1   // PB5: Debug LED
#define M3_DEBUG_PORT	  GPIOB


//// === Motor #X Control Pins === (copy paste as necessary for as many motors you have)...
//#define M1_EN_PIN         GPIO_PIN_X   // PXx: Enable pin for TMC2209 (active-low)
//#define M1_EN_PORT        GPIOX
//
//#define M1_STEP_PIN       GPIO_PIN_X   // PXx: Step pulse pin for TMC2209
//#define M1_STEP_PORT      GPIOX
//
//#define M1_DIR_PIN        GPIO_PIN_X   // PXx: Direction pin for TMC2209
//#define M1_DIR_PORT       GPIOX
//
//#define M1_ENDSTOP_PIN    GPIO_PIN_X   // PXx: Endstop switch input
//#define M1_ENDSTOP_PORT   GPIOX

// === Debug LED Pin ===

#endif /* INC_MOTOR_DEFINES_H_ */
