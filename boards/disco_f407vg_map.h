/*
  disco_f407vg_map.h - driver code for STM32F4xx (Discovery) dev board

  Part of GrblHAL

  Copyright (c) 2021 rvalotta

  GrblHAL is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  GrblHAL is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with GrblHAL. If not, see <http://www.gnu.org/licenses/>.
*/

/* Pin Assignments:
 *                                                 STM32F4-DISCOVERY
 *                            GND	|.|.|	GND	                                GND	|.|.|	GND
 *                            VDD	|.|.|	VDD	                                 5V	|.|.|	5V
 *                            GND	|.|.|	NRST (RST BUTTON)                    3V	|.|.|	3V
 *                            PC1	|.|.|	PC0	(USB-FS)                        PH0	|.|.|	PH1
 *                 (MIC PDM)  PC3	|.|.|	PC2	                               PC14 |.|.|	PC15
 *                      Feed  PA1	|.|.|	PA0	(USR BUTTON) RST       M4 Step  PE6	|.|.|	PC13
 *                            PA3	|.|.|	PA2	Cycle             M4 Direction  PE4	|.|.|	PE5  M5 Direction
 *               (MEMS SCLK)  PA5	|.|.|	PA4	(DAC)                  M5 Step  PE2	|.|.|	PE3  (MEMS CS_I2C_SPI)
 *               (MEMS MOSI)  PA7	|.|.|	PA6	(MEMS MISO)        (MEMS INT0)  PE0	|.|.|	PE1  (MEMS INT2)
 *                            PC5	|.|.|	PC4	                                PB8	|.|.|	PB9  (DAC I2C)
 *       XYZ Steppers enable  PB1	|.|.|	PB0	 M4 Enable                    BOOT0	|.|.|	VDD
 *                            GND	|.|.|	PB2	 M5 Enable                      PB6	|.|.|	PB7  Spindle PWM
 *                   X Step   PE7	|.|.|	PE8	  Z Step         (DAC-Reset-N)  PB4	|.|.|	PB5  Spindle direction
 *               X Direction  PE9	|.|.|	PE10	Z Direction                   PD7	|.|.|	PB3  Spindle enable
 *                   Y Step  PE11	|.|.|	PE12	M3 Step             (USB-FS)  PD5	|.|.|	PD6
 *              Y Direction  PE13	|.|.|	PE14	M3 Direction                  PD3	|.|.|	PD4
 *                           PE15	|.|.|	PB10	(MIC CLK)                     PD1	|.|.|	PD2
 *                  X Limit  PB11	|.|.|	PB12	Y Limit               (DAC)  PC12	|.|.|	PD0
 *                  Z Limit  PB13	|.|.|	PB14	M3 Limit              (DAC)  PC10	|.|.|	PC11
 *                 M4 Limit  PB15	|.|.|	PD8	                               PA14 |.|.|	PA15
 *                            PD9	|.|.|	PD10	                   (USB-FS)  PA10	|.|.|	PA13
 *                           PD11	|.|.|	PD12	(LED GRN)                     PA8	|.|.|	PA9  (USB-FS)
 *                (LED ORG)  PD13	|.|.|	PD14	(LED RED)                     PC8	|.|.|	PC9
 *                (LED BLU)  PD15	|.|.|	NC	                                PC6	|.|.|	PC7  (DAC)
 *                            GND	|.|.|	GND	                                GND	|.|.|	GND
 *
 *                                                   PA11 |USB| P12
 */

#if N_ABC_MOTORS > 1
#error "Axis configuration is not supported!"
#endif

#define BOARD_NAME "DiscoveryF4"

// Define step pulse output pins.
#define STEP_PORT               GPIOE
#define X_STEP_PIN              7
#define Y_STEP_PIN              11
#define Z_STEP_PIN              8
#define STEP_OUTMODE            GPIO_MAP

#define DIRECTION_PORT          GPIOE
#define X_DIRECTION_PIN         9
#define Y_DIRECTION_PIN         13
#define Z_DIRECTION_PIN         10
#define DIRECTION_OUTMODE       GPIO_MAP

// Define stepper driver enable/disable output pin.
#define STEPPERS_ENABLE_PORT    GPIOB
#define STEPPERS_ENABLE_PIN     1
#define STEPPERS_ENABLE_MASK    STEPPERS_ENABLE_BIT

// Define homing/hard limit switch input pins.
#define LIMIT_PORT              GPIOB
#define X_LIMIT_PIN             11
#define Y_LIMIT_PIN             12
#define Z_LIMIT_PIN             13
#define LIMIT_INMODE            GPIO_SHIFT11 

// Define ganged axis or A axis step pulse and step direction output pins.
#if N_ABC_MOTORS == 1
#define M3_AVAILABLE
#define M3_STEP_PORT            STEP_PORT
#define M3_STEP_PIN             12
#define M3_DIRECTION_PORT       DIRECTION_PORT
#define M3_DIRECTION_PIN        14
#if N_AUTO_SQUARED
#define M3_LIMIT_PORT           LIMIT_PORT
#define M3_LIMIT_PIN            14
#endif
#endif

// Define ganged axis or B axis step pulse and step direction output pins.
#if N_ABC_MOTORS > 1
#define M4_AVAILABLE                // E1
#define M4_STEP_PORT                STEP_PORT
#define M4_STEP_PIN                 6
#define M4_DIRECTION_PORT           DIRECTION_PORT
#define M4_DIRECTION_PIN            4
#define M4_LIMIT_PORT               LIMIT_PORT
#define M4_LIMIT_PIN                15
#define M4_ENABLE_PORT              STEPPERS_ENABLE_PORT
#define M4_ENABLE_PIN               0
#endif

// Define ganged axis or C axis step pulse and step direction output pins.
#if N_ABC_MOTORS == 3
#define M5_AVAILABLE                // E2
#define M5_STEP_PORT                STEP_PORT
#define M5_STEP_PIN                 2
#define M5_DIRECTION_PORT           DIRECTION_PORT
#define M5_DIRECTION_PIN            5
//#define M5_LIMIT_PORT             LIMIT_PORT
//#define M5_LIMIT_PIN              8
#define M5_ENABLE_PORT              STEPPERS_ENABLE_PORT
#define M5_ENABLE_PIN               2
#endif

#define AUXOUTPUT2_PORT         GPIOB // Spindle PWM
#define AUXOUTPUT2_PIN          7
#define AUXOUTPUT3_PORT         GPIOB // Spindle direction
#define AUXOUTPUT3_PIN          5
#define AUXOUTPUT4_PORT         GPIOB // Spindle enable
#define AUXOUTPUT4_PIN          3

// Define driver spindle pins
#if DRIVER_SPINDLE_ENABLE
#define SPINDLE_ENABLE_PORT     AUXOUTPUT4_PORT
#define SPINDLE_ENABLE_PIN      AUXOUTPUT4_PIN
#if DRIVER_SPINDLE_PWM_ENABLE
#define SPINDLE_PWM_PORT        AUXOUTPUT2_PORT
#define SPINDLE_PWM_PIN         AUXOUTPUT2_PIN
#endif
#if DRIVER_SPINDLE_DIR_ENABLE
#define SPINDLE_DIRECTION_PORT  AUXOUTPUT3_PORT
#define SPINDLE_DIRECTION_PIN   AUXOUTPUT3_PIN
#endif
#endif //DRIVER_SPINDLE_ENABLE

// Define flood and mist coolant enable output pins.
#define COOLANT_FLOOD_PORT      GPIOC
#define COOLANT_FLOOD_PIN       15
#define COOLANT_MIST_PORT       GPIOC
#define COOLANT_MIST_PIN        14

// Define user-control controls (cycle start, reset, feed hold) input pins.
#define CONTROL_PORT            GPIOA
#define RESET_PIN               0
#define FEED_HOLD_PIN           1
#define CYCLE_START_PIN         2
#define CONTROL_INMODE          GPIO_SHIFT0

// Spindle encoder pins.
#if SPINDLE_ENCODER_ENABLE

#define RPM_COUNTER_N           2
#define RPM_TIMER_N             3
#define SPINDLE_INDEX_PORT      GPIOB
#define SPINDLE_INDEX_PIN       3
#define SPINDLE_PULSE_PORT      GPIOA
#define SPINDLE_PULSE_PIN       15

#endif

#define AUXINPUT0_PORT          GPIOB
#define AUXINPUT0_PIN           9
#if !N_AUTO_SQUARED
#define AUXINPUT1_PORT          GPIOB // Probe input
#define AUXINPUT1_PIN           15
#endif

#if N_ABC_MOTORS == 0
#define AUXOUTPUT0_PORT         GPIOA
#define AUXOUTPUT0_PIN          7
#define AUXOUTPUT1_PORT         GPIOA
#define AUXOUTPUT1_PIN          6
#endif

#if PROBE_ENABLE && defined(AUXINPUT1_PIN)
#define PROBE_PORT              AUXINPUT1_PORT
#define PROBE_PIN               AUXINPUT1_PIN
#endif

#if SAFETY_DOOR_ENABLE
#define SAFETY_DOOR_PORT        AUXINPUT0_PORT
#define SAFETY_DOOR_PIN         AUXINPUT0_PIN
#endif

#if MOTOR_FAULT_ENABLE
#define MOTOR_FAULT_PORT        AUXINPUT0_PORT
#define MOTOR_FAULT_PIN         AUXINPUT0_PIN
#endif

#if I2C_STROBE_ENABLE
#define I2C_STROBE_PORT         AUXINPUT2_PORT
#define I2C_STROBE_PIN          AUXINPUT2_PIN
#endif

// NOT SUPPORTED
#if SDCARD_ENABLE
//#error SDcard not supported
#endif

/* EOF */