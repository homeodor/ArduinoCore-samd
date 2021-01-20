/*
  Copyright (c) 2014-2015 Arduino LLC.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  See the GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#pragma once

// The definitions here needs a SAMD core >=1.6.10
#define ARDUINO_SAMD_VARIANT_COMPLIANCE 10610

#include <WVariant.h>

// General definitions
// -------------------

// Frequency of the board main oscillator
#define VARIANT_MAINOSC (32768ul)

// Master clock frequency
#define VARIANT_MCK     (48000000ul)

// Pins
// ----

// Number of pins defined in PinDescription array
#ifdef __cplusplus
extern "C" unsigned int PINCOUNT_fn();
#endif
#define PINS_COUNT           (PINCOUNT_fn())
#define NUM_DIGITAL_PINS     (50u)
#define NUM_ANALOG_INPUTS    (16u)
#define NUM_ANALOG_OUTPUTS   (1u)

// Low-level pin register query macros
// -----------------------------------
#define digitalPinToPort(P)      (&(PORT->Group[g_APinDescription[P].ulPort]))
#define digitalPinToBitMask(P)   (1 << g_APinDescription[P].ulPin)
//#define analogInPinToBit(P)    ()
#define portOutputRegister(port) (&(port->OUT.reg))
#define portInputRegister(port)  (&(port->IN.reg))
#define portModeRegister(port)   (&(port->DIR.reg))
#define digitalPinHasPWM(P)      (g_APinDescription[P].ulPWMChannel != NOT_ON_PWM || g_APinDescription[P].ulTCChannel != NOT_ON_TIMER)

#define PA00	(0u)
#define PA01	(1u)
#define PA02	(2u)
#define PA03	(3u)
#define PA04	(4u)
#define PA05	(5u)
#define PA06	(6u)
#define PA07	(7u)
#define PA08	(8u)
#define PA09	(9u)
#define PA10	(10u)
#define PA11	(11u)
#define PA12	(12u)
#define PA13	(13u)
#define PA14	(14u)
#define PA15	(15u)
#define PA16	(16u)
#define PA17	(17u)
#define PA18	(18u)
#define PA19	(19u)
#define PA20	(20u)
#define PA21	(21u)
#define PA22	(22u)
#define PA23	(23u)
#define PA24	(24u)
#define PA25	(25u)
#define PA27	(26u)
#define PA28	(27u)
#define PB00	(28u)
#define PB01	(29u)
#define PB02	(30u)
#define PB03	(31u)
#define PB04	(32u)
#define PB05	(33u)
#define PB06	(34u)
#define PB07	(35u)
#define PB08	(36u)
#define PB09	(37u)
#define PB10	(38u)
#define PB11	(39u)
#define PB12	(40u)
#define PB13	(41u)
#define PB14	(42u)
#define PB15	(43u)
#define PB16	(44u)
#define PB17	(45u)
#define PB22	(46u)
#define PB23	(47u)
#define PB30	(48u)
#define PB31	(49u)

#define PIN_DAC0 PA02

/*
 * digitalPinToTimer(..) is AVR-specific and is not defined for SAMD
 * architecture. If you need to check if a pin supports PWM you must
 * use digitalPinHasPWM(..).
 *
 * https://github.com/arduino/Arduino/issues/1833
 */
// #define digitalPinToTimer(P)

//Battery
#define ADC_BATTERY	PA07

// LEDs
// ----
#define PIN_LED     PA18
#define LED_BUILTIN PIN_LED

// Analog pins
// -----------
#define PIN_A0  PA02
#define PIN_A1  PA03
#define PIN_A2  PB08
#define PIN_A3  PB09
#define PIN_A4  PA04
#define PIN_A5  PA05
#define PIN_A6  PA06
#define PIN_A7  PA07
#define PIN_A8  PB00
#define PIN_A9  PB01
#define PIN_A10 PB02
#define PIN_A11 PB03
#define PIN_A12 PB04
#define PIN_A13 PB05
#define PIN_A14 PB06
#define PIN_A15 PB07
static const uint8_t A0  = PIN_A0;
static const uint8_t A1  = PIN_A1;
static const uint8_t A2  = PIN_A2;
static const uint8_t A3  = PIN_A3;
static const uint8_t A4  = PIN_A4;
static const uint8_t A5  = PIN_A5;
static const uint8_t A6  = PIN_A6;
static const uint8_t A7  = PIN_A7;
static const uint8_t A8  = PIN_A8;
static const uint8_t A9  = PIN_A9;
static const uint8_t A10 = PIN_A10;
static const uint8_t A11 = PIN_A11;
static const uint8_t A12 = PIN_A12;
static const uint8_t A13 = PIN_A13;
static const uint8_t A14 = PIN_A14;
static const uint8_t A15 = PIN_A15;
#define ADC_RESOLUTION 12

// SPI Interfaces
// --------------
#define SPI_INTERFACES_COUNT 2

// SPI: Connected to SD... I mean, SPI Flash
#define PIN_SPI_MISO PA20
#define PIN_SPI_MOSI PB16
#define PIN_SPI_SCK  PB17
#define PIN_SPI_SS   PA21
#define PERIPH_SPI   sercom5
#define PAD_SPI_TX   SPI_PAD_0_SCK_1
#define PAD_SPI_RX   SERCOM_RX_PAD_2
static const uint8_t SS   = PIN_SPI_SS;
static const uint8_t MOSI = PIN_SPI_MOSI;
static const uint8_t MISO = PIN_SPI_MISO;
static const uint8_t SCK  = PIN_SPI_SCK;


// SPI
#define PIN_SPI1_MISO  PA18 //(10u)
#define PIN_SPI1_MOSI  PA19
#define PIN_SPI1_SCK   PA17
#define PIN_SPI1_SS    PA23
#define PERIPH_SPI1    sercom1
#define PAD_SPI1_TX    SPI_PAD_3_SCK_1
#define PAD_SPI1_RX    SERCOM_RX_PAD_2

static const uint8_t SS1   = PIN_SPI1_SS;   // SPI Slave SS not used. Set here only for reference.
static const uint8_t MOSI1 = PIN_SPI1_MOSI;
static const uint8_t MISO1 = PIN_SPI1_MISO;
static const uint8_t SCK1  = PIN_SPI1_SCK;

// Needed for SD library
#define SDCARD_SPI      SPI
#define SDCARD_MISO_PIN PIN_SPI_MISO
#define SDCARD_MOSI_PIN PIN_SPI_MOSI
#define SDCARD_SCK_PIN  PIN_SPI_SCK
#define SDCARD_SS_PIN   PIN_SPI_SS

// Wire Interfaces
// ---------------
#define WIRE_INTERFACES_COUNT 1

// Wire
#define PIN_WIRE_SDA        PA08
#define PIN_WIRE_SCL        PA09
#define PERIPH_WIRE         sercom0
#define WIRE_IT_HANDLER     SERCOM0_Handler /// ?...

// USB
// ---
#define PIN_USB_DM          PA24
#define PIN_USB_DP          PA25
#define PIN_USB_HOST_ENABLE PB04 // ... whatever

// I2S Interfaces
// --------------
#define I2S_INTERFACES_COUNT 1

#define I2S_DEVICE          0
#define I2S_CLOCK_GENERATOR 3
#define PIN_I2S_SD          (PIN_A6)
#define PIN_I2S_SCK         PB03
#define PIN_I2S_FS          PA03

// Serial ports
// ------------
#ifdef __cplusplus
#include "SERCOM.h"
#include "Uart.h"

// Instances of SERCOM
extern SERCOM sercom0;
extern SERCOM sercom1;
extern SERCOM sercom2;
extern SERCOM sercom3;
extern SERCOM sercom4;
extern SERCOM sercom5;

//#error "YEs"

// Serial1
extern Uart Serial1;
#define PIN_SERIAL1_RX PB12
#define PIN_SERIAL1_TX PB13
#define PAD_SERIAL1_TX (UART_TX_PAD_0)
#define PAD_SERIAL1_RX (SERCOM_RX_PAD_1)

// Serial1
extern Uart Serial2;
#define PIN_SERIAL2_RX PA12
#define PIN_SERIAL2_TX PA13
#define PAD_SERIAL2_TX (UART_TX_PAD_0)
#define PAD_SERIAL2_RX (SERCOM_RX_PAD_1)

#endif // __cplusplus

#ifdef __cplusplus
extern "C" {
#endif
unsigned int PINCOUNT_fn();
#ifdef __cplusplus
}
#endif

// These serial port names are intended to allow libraries and architecture-neutral
// sketches to automatically default to the correct port name for a particular type
// of use.  For example, a GPS module would normally connect to SERIAL_PORT_HARDWARE_OPEN,
// the first hardware serial port whose RX/TX pins are not dedicated to another use.
//
// SERIAL_PORT_MONITOR        Port which normally prints to the Arduino Serial Monitor
//
// SERIAL_PORT_USBVIRTUAL     Port which is USB virtual serial
//
// SERIAL_PORT_LINUXBRIDGE    Port which connects to a Linux system via Bridge library
//
// SERIAL_PORT_HARDWARE       Hardware serial port, physical RX & TX pins.
//
// SERIAL_PORT_HARDWARE_OPEN  Hardware serial ports which are open for use.  Their RX & TX
//                            pins are NOT connected to anything by default.
#define SERIAL_PORT_USBVIRTUAL      SerialUSB
#define SERIAL_PORT_MONITOR         SerialUSB
#define SERIAL_PORT_HARDWARE        Serial1
#define SERIAL_PORT_HARDWARE_OPEN   Serial1

// Alias Serial to SerialUSB
#define Serial                      SerialUSB

