// ===================================================================================
// Project:     Battery Capacity Tester
// Version:     v2.0
// Date:        Nov 2025
// Modified by: Henry Cheung (https://github.com/e-tinkers)
// Author:      Stefan Wagner (https://github.com/wagiminator)
// License:     http://creativecommons.org/licenses/by-sa/3.0/
// ===================================================================================
//
// Description:
// ------------
// The ATtiny412 controlled battery capacity tester measures the capacity of 
// single-cell Li-Ion, LiPo and LiFePO4 batteries using the built-in constant 
// current load. Discharge termination (cutoff) voltage and discharge current
// can be selected by the user. During the discharging process, all relevant 
// data is displayed on an OLED.
//
// References:
// -----------
// Stefan Wagner original project 
// https://github.com/wagiminator/ATtiny412-BatteryCapacityTester/
//
// The OLED font was adapted from Neven Boyanov and Stephen Denne:
// https://github.com/datacute/Tiny4kOLED
//
// No Arduino core functions or libraries are used. To compile and upload without
// Arduino IDE download AVR 8-bit toolchain at:
// https://www.microchip.com/mplab/avr-support/avr-and-arm-toolchains-c-compilers
// and extract to tools/avr-gcc. Use the makefile to compile and upload.
//
// Operating Instruction:
// ----------------------
// 1. Connect the device to a 5V power supply via the USB-C port.
// 2. Use the buttons to select the discharge termination voltage and the maximum 
//    discharge current.
// 3. Connect the previously fully charged battery to be tested to the device.
// 4. Press the START button.
// 5. Wait for the discharging process to finish. The capacity can now be read.
// 6. Remove the battery afterwards and press any key.
//
// Software Modifiication (by Henry Cheung):
// 1. Modified the code to connect each button to a GPIO pin instead of using one ADC pin
// 2. Remove some magic numbers in the code and replace with #define macros
// 3. Rewrite DAC_init() and only be called after user selected cut-off voltage
// 4. DAC_setLoad() no longer need and integrated into DAC_init()
// 5. Correct OLED initialization commands to display 8 rows instead of 4 rows
// 6. Re-arrange the display information and prompts to fully unitize the 8 rows display

// Hardware Modification (by Henry Cheung):
// 1. Use ATtiny3217 instead of ATtiny412
// 2. Use LM358 instead of LMV321
// 3. Connect 3 buttons directly to 3 GPIO pins, add a 1uF capacitor across each button
//    for debouncing
// 4. Solder everything on a protoboard instead of fabricating a PCB.
// 5. Battery Connector for 18650 capable to support non-protected and protected 18650.
// 6. Add JST connector for Li-Po/Li-Ion battery


// ===================================================================================
// Libraries, Definitions and Macros
// ===================================================================================

// Libraries
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

// Pin assignments
#define PIN_START   PIN3_bm            // START button on PORTA PIN3
#define PIN_VOLT    PIN4_bm            // Voltage setting button on PORTA PIN4
#define PIN_CURR    PIN5_bm            // Current setting button on PORTA PIN5
#define PIN_DAC     PIN6_bm            // DAC output, controls electronic load
#define PIN_SENSE   ADC_MUXPOS_AIN1_gc // ADC ch1, voltage sense pin (PORTA PIN1)

// Firmware parameters
#define VOLT_MIN    2500               // minimum selectable termination voltage
#define VOLT_MAX    3200               // maximum selectable termination voltage
#define VOLT_STEP   100                // voltage selection steps

#define CURR_MIN    200                // minimum selectable load current
#define CURR_MAX    2000               // maximum selectable load current
#define CURR_STEP   200                // current selection steps

#define ADC_SAMPLES 64                 // number of ADC samples to read
#define ADC_VREF    4340               // ADC interval voltage reference 4.34V
#define ADC_STEPS   1024               // 10-bit ADC resolution

#define R1          10000              // R1, use 1% tolerence and measure the actual value
#define R2          1000               // R2, use 1% tolerence and measure the actual value

// ===================================================================================
// I2C Master Implementation (Write only)
// ===================================================================================

#define I2C_FREQ  400000UL                        // I2C clock frequency in Hz
#define I2C_BAUD  (F_CPU / I2C_FREQ - 10) / 2;    // simplified BAUD calculation

// I2C init function
void I2C_init(void) {
  TWI0.MBAUD   = I2C_BAUD;                        // set TWI master BAUD rate
  TWI0.MCTRLA  = TWI_ENABLE_bm;                   // enable TWI master
  TWI0.MSTATUS = TWI_BUSSTATE_IDLE_gc;            // set bus state to idle
}

// I2C start transmission
void I2C_start(uint8_t addr) {
  TWI0.MADDR = addr;                              // start sending address
}

// I2C stop transmission
void I2C_stop(void) {
  while(~TWI0.MSTATUS & TWI_WIF_bm);              // wait for last transfer to complete
  TWI0.MCTRLB = TWI_MCMD_STOP_gc;                 // send stop condition
}

// I2C transmit one data byte to the slave, ignore ACK bit
void I2C_write(uint8_t data) {
  while(~TWI0.MSTATUS & TWI_WIF_bm);              // wait for last transfer to complete
  TWI0.MDATA = data;                              // start sending data byte 
}

// ===================================================================================
// OLED Implementation
// ===================================================================================

// OLED definitions
#define OLED_ADDR       0x78                      // OLED write address
#define OLED_CMD_MODE   0x00                      // set command mode
#define OLED_DAT_MODE   0x40                      // set data mode

// OLED init settings
const uint8_t OLED_INIT_CMD[] = {
  0xC8, 0xA1,                                     // flip screen
  0xA8, 0x3F,                                     // set multiplex ratio (num of rows - 1)
  0xD3, 0,                                        // set row offset
  0xDA, 0x12,                                     // set com pins hardware configuration
  0x8D, 0x14,                                     // set DC-DC enable
  0xDB, 0x40,                                     // Set vcom deselect
  0xAF                                            // display on
};

// Standard ASCII 5x8 font (adapted from Neven Boyanov and Stephen Denne)
const uint8_t OLED_FONT[] = {
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x2F, 0x00, 0x00, 0x00, 0x07, 0x00, 0x07, 0x00,
  0x14, 0x7F, 0x14, 0x7F, 0x14, 0x24, 0x2A, 0x7F, 0x2A, 0x12, 0x23, 0x13, 0x08, 0x64, 0x62,
  0x36, 0x49, 0x55, 0x22, 0x50, 0x00, 0x05, 0x03, 0x00, 0x00, 0x00, 0x1C, 0x22, 0x41, 0x00,
  0x00, 0x41, 0x22, 0x1C, 0x00, 0x14, 0x08, 0x3E, 0x08, 0x14, 0x08, 0x08, 0x3E, 0x08, 0x08,
  0x00, 0x00, 0xA0, 0x60, 0x00, 0x08, 0x08, 0x08, 0x08, 0x08, 0x00, 0x60, 0x60, 0x00, 0x00,
  0x20, 0x10, 0x08, 0x04, 0x02, 0x3E, 0x51, 0x49, 0x45, 0x3E, 0x00, 0x42, 0x7F, 0x40, 0x00,
  0x42, 0x61, 0x51, 0x49, 0x46, 0x21, 0x41, 0x45, 0x4B, 0x31, 0x18, 0x14, 0x12, 0x7F, 0x10,
  0x27, 0x45, 0x45, 0x45, 0x39, 0x3C, 0x4A, 0x49, 0x49, 0x30, 0x01, 0x71, 0x09, 0x05, 0x03,
  0x36, 0x49, 0x49, 0x49, 0x36, 0x06, 0x49, 0x49, 0x29, 0x1E, 0x00, 0x36, 0x36, 0x00, 0x00,
  0x00, 0x56, 0x36, 0x00, 0x00, 0x08, 0x14, 0x22, 0x41, 0x00, 0x14, 0x14, 0x14, 0x14, 0x14,
  0x00, 0x41, 0x22, 0x14, 0x08, 0x02, 0x01, 0x51, 0x09, 0x06, 0x32, 0x49, 0x59, 0x51, 0x3E,
  0x7C, 0x12, 0x11, 0x12, 0x7C, 0x7F, 0x49, 0x49, 0x49, 0x36, 0x3E, 0x41, 0x41, 0x41, 0x22,
  0x7F, 0x41, 0x41, 0x22, 0x1C, 0x7F, 0x49, 0x49, 0x49, 0x41, 0x7F, 0x09, 0x09, 0x09, 0x01,
  0x3E, 0x41, 0x49, 0x49, 0x7A, 0x7F, 0x08, 0x08, 0x08, 0x7F, 0x00, 0x41, 0x7F, 0x41, 0x00,
  0x20, 0x40, 0x41, 0x3F, 0x01, 0x7F, 0x08, 0x14, 0x22, 0x41, 0x7F, 0x40, 0x40, 0x40, 0x40,
  0x7F, 0x02, 0x0C, 0x02, 0x7F, 0x7F, 0x04, 0x08, 0x10, 0x7F, 0x4E, 0x71, 0x01, 0x71, 0x4E,
  0x7F, 0x09, 0x09, 0x09, 0x06, 0x3E, 0x41, 0x51, 0x21, 0x5E, 0x7F, 0x09, 0x19, 0x29, 0x46,
  0x46, 0x49, 0x49, 0x49, 0x31, 0x01, 0x01, 0x7F, 0x01, 0x01, 0x3F, 0x40, 0x40, 0x40, 0x3F,
  0x1F, 0x20, 0x40, 0x20, 0x1F, 0x3F, 0x40, 0x38, 0x40, 0x3F, 0x63, 0x14, 0x08, 0x14, 0x63,
  0x07, 0x08, 0x70, 0x08, 0x07, 0x61, 0x51, 0x49, 0x45, 0x43, 0x00, 0x7F, 0x41, 0x41, 0x00,
  0x02, 0x04, 0x08, 0x10, 0x20, 0x00, 0x41, 0x41, 0x7F, 0x00, 0x04, 0x02, 0x01, 0x02, 0x04,
  0x40, 0x40, 0x40, 0x40, 0x40, 0x00, 0x01, 0x02, 0x04, 0x00, 0x20, 0x54, 0x54, 0x54, 0x78,
  0x7F, 0x48, 0x44, 0x44, 0x38, 0x38, 0x44, 0x44, 0x44, 0x20, 0x38, 0x44, 0x44, 0x48, 0x7F,
  0x38, 0x54, 0x54, 0x54, 0x18, 0x08, 0x7E, 0x09, 0x01, 0x02, 0x18, 0xA4, 0xA4, 0xA4, 0x7C,
  0x7F, 0x08, 0x04, 0x04, 0x78, 0x00, 0x44, 0x7D, 0x40, 0x00, 0x40, 0x80, 0x84, 0x7D, 0x00,
  0x7F, 0x10, 0x28, 0x44, 0x00, 0x00, 0x41, 0x7F, 0x40, 0x00, 0x7C, 0x04, 0x18, 0x04, 0x78,
  0x7C, 0x08, 0x04, 0x04, 0x78, 0x38, 0x44, 0x44, 0x44, 0x38, 0xFC, 0x24, 0x24, 0x24, 0x18,
  0x18, 0x24, 0x24, 0x18, 0xFC, 0x7C, 0x08, 0x04, 0x04, 0x08, 0x48, 0x54, 0x54, 0x54, 0x20,
  0x04, 0x3F, 0x44, 0x40, 0x20, 0x3C, 0x40, 0x40, 0x20, 0x7C, 0x1C, 0x20, 0x40, 0x20, 0x1C,
  0x3C, 0x40, 0x30, 0x40, 0x3C, 0x44, 0x28, 0x10, 0x28, 0x44, 0x1C, 0xA0, 0xA0, 0xA0, 0x7C,
  0x44, 0x64, 0x54, 0x4C, 0x44, 0x08, 0x36, 0x41, 0x41, 0x00, 0x00, 0x00, 0x7F, 0x00, 0x00,
  0x00, 0x41, 0x41, 0x36, 0x08, 0x08, 0x04, 0x08, 0x10, 0x08, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF
};

// OLED variables
uint8_t OLED_x, OLED_y;                           // current cursor position
const uint16_t DIVIDER[] = {
  1, 10, 100, 1000, 10000
};                                                // for BCD conversion

// OLED init function
void OLED_init(void) {
  _delay_ms(30);                                  // wait for display to be ready from power up
  I2C_start(OLED_ADDR);
  I2C_write(OLED_CMD_MODE);
  for (uint8_t i = 0; i < sizeof(OLED_INIT_CMD); i++)
    I2C_write(OLED_INIT_CMD[i]);
  I2C_stop();
}

// OLED set the cursor
void OLED_setCursor(uint8_t xpos, uint8_t ypos) {
  xpos &= 0x7F; ypos &= 0x07;                     // limit cursor position values
  I2C_start(OLED_ADDR);                           // start transmission to OLED
  I2C_write(OLED_CMD_MODE);                       // set command mode
  I2C_write(xpos & 0x0F);                         // set low nibble of start column
  I2C_write(0x10 | (xpos >> 4));                  // set high nibble of start column
  I2C_write(0xB0 | ypos);                         // set start page
  I2C_stop();                                     // stop transmission
  OLED_x = xpos; OLED_y = ypos;                   // set the cursor variables
}

// OLED clear rest of the current line
void OLED_clearLine(void) {
  I2C_start(OLED_ADDR);                           // start transmission to OLED
  I2C_write(OLED_DAT_MODE);                       // set data mode
  while(OLED_x++ < 128) I2C_write(0);             // clear rest of the line
  I2C_stop();                                     // stop transmission
  OLED_setCursor(0, ++OLED_y);                    // set cursor to start of next line
}

// OLED clear screen
void OLED_clearScreen(void) {
  OLED_setCursor(0, 0);                           // set cursor to home position
  for(uint8_t i=8; i; i--) OLED_clearLine();      // clear all 8 lines
}

// OLED plot a single character
void OLED_plotChar(char c) {
  uint16_t ptr = c - ' ';                         // character pointer
  ptr += ptr << 2;                                // -> ptr = (ch - 32) * 5;
  I2C_write(0x00);                                // write space between characters
  for (uint8_t i=5 ; i; i--) 
    I2C_write(OLED_FONT[ptr++]);
  OLED_x += 6;                                    // update cursor
  if (OLED_x > 122) {                             // line end ?
    I2C_stop();                                   // stop data transmission
    OLED_setCursor(0,++OLED_y);                   // set next line start
    I2C_start(OLED_ADDR);                         // start transmission to OLED
    I2C_write(OLED_DAT_MODE);                     // set data mode
  }
}

// OLED print a string
void OLED_print(const char* str) {
  I2C_start(OLED_ADDR);                           // start transmission to OLED
  I2C_write(OLED_DAT_MODE);                       // set data mode
  while(*str) OLED_plotChar(*str++);              // plot each character
  I2C_stop();                                     // stop transmission
}

// OLED print a string with new line
void OLED_println(const char* str) {
  OLED_print(str);                                // print the string
  OLED_clearLine();                               // clear rest of the line
}

// OLED print value (BCD conversion by substraction method)
void OLED_printVal(uint16_t value) {
  uint8_t digits   = 5;                           // print 5 digits
  uint8_t leadflag = 0;                           // flag for leading spaces
  I2C_start(OLED_ADDR);                           // start transmission to OLED
  I2C_write(OLED_DAT_MODE);                       // set data mode
  while(digits--) {                               // for all digits digits
    uint8_t digitval = 0;                         // start with digit value 0
    uint16_t divider = DIVIDER[digits];           // read current divider
    while(value >= divider) {                     // if current divider fits into the value
      leadflag = 1;                               // end of leading spaces
      digitval++;                                 // increase digit value
      value -= divider;                           // decrease value by divider
    }
    if(!digits)  leadflag++;                      // least digit has to be printed
    if(leadflag) OLED_plotChar(digitval + '0');   // print the digit
    else         OLED_plotChar(' ');              // or print leading space
  }
  I2C_stop();                                     // stop transmission
}

// OLED print 8-bit value as 2-digit decimal (BCD conversion by substraction method)
void OLED_printDec(uint8_t value) {
  I2C_start(OLED_ADDR);                           // start transmission to OLED
  I2C_write(OLED_DAT_MODE);                       // set data mode
  uint8_t digitval = 0;                           // start with digit value 0
  while(value >= 10) {                            // if current divider fits into the value
    digitval++;                                   // increase digit value
    value -= 10;                                  // decrease value by divider
  }
  OLED_plotChar(digitval + '0');                  // print first digit
  OLED_plotChar(value + '0');                     // print second digit
  I2C_stop();                                     // stop transmission
}

// ===================================================================================
// GPIO Implementation for buttons
// ===================================================================================

// Setup the gpio pins for the 3 buttons as input with internal pullup
void GPIO_init(void) {
  PORTA.DIRCLR = (PIN_START | PIN_VOLT | PIN_CURR);
  PORTA.PIN3CTRL = PORT_PULLUPEN_bm | PORT_ISC_FALLING_gc;
  PORTA.PIN4CTRL = PORT_PULLUPEN_bm | PORT_ISC_FALLING_gc;
  PORTA.PIN5CTRL = PORT_PULLUPEN_bm | PORT_ISC_FALLING_gc;
}

volatile uint8_t start_pressed = 0;
volatile uint8_t curr_keypressed = 0;
volatile uint8_t volt_keypressed = 0;

// ISR for PORTA, respond to interrupts on any pins on PORTA
ISR(PORTA_PORT_vect) {
    if (PORTA.INTFLAGS & PORT_INT3_bm) {          // only if triggered by PIN_START(PB3)
        start_pressed = 1;
        PORTA.INTFLAGS = PORT_INT3_bm; 
    }
    else if (PORTA.INTFLAGS & PORT_INT4_bm) {     // only if triggered by PIN_VOLT(PB4)
        volt_keypressed = 1;
        PORTA.INTFLAGS = PORT_INT4_bm;
    }
    else if (PORTA.INTFLAGS & PORT_INT5_bm) {     // only if triggered by PIN_CURR(PB5)
        curr_keypressed = 1;
        PORTA.INTFLAGS = PORT_INT5_bm;
    }
}

// ===================================================================================
// DAC Implementation for Electronic Load
// ===================================================================================

// VREF_CTRLA values
const uint8_t VREF_CTRL[] = {
  VREF_DAC0REFSEL_0V55_gc, 
  VREF_DAC0REFSEL_1V1_gc, 
  VREF_DAC0REFSEL_1V5_gc, 
  VREF_DAC0REFSEL_2V5_gc, 
  VREF_DAC0REFSEL_4V34_gc
};
// VREF voltages in mA
const uint16_t VREF_VOLT[] = {
  550, 
  1100, 
  1500, 
  2500, 
  4340
};
uint8_t VrefIndex = 0;

// Setup the DAC
void DAC_init(uint16_t voltage) {
  PORTA.PIN6CTRL &= ~PORT_ISC_gm;                 // setup PA6 as an analog pin  
  PORTA.PIN6CTRL |= PORT_ISC_INPUT_DISABLE_gc;
  PORTA.PIN6CTRL &= ~PORT_PULLUPEN_bm;

  // configure VREF for DAC based on voltage requirement
  VrefIndex = 0;
  while (voltage >= VREF_VOLT[VrefIndex])
    VrefIndex++;
  VREF.CTRLA &= ~VREF_DAC0REFSEL_gm;
  VREF.CTRLA |= VREF_CTRL[VrefIndex];
  VREF.CTRLB |= VREF_DAC0REFEN_bm;
  _delay_us(25);

  // set DAC output
  DAC0.DATA = (uint32_t) voltage * 256 / VREF_VOLT[VrefIndex];
  DAC0.CTRLA  = DAC_ENABLE_bm | DAC_OUTEN_bm;     // enable output buffer and DAC
}

// Get DAC voltage
uint16_t DAC_readVolt(void) {
  return (uint32_t) VREF_VOLT[VrefIndex] * DAC0.DATA / 256;
}

void DAC_reset() {
  DAC0.DATA = 0;
  DAC0.CTRLA  &= ~(DAC_ENABLE_bm | DAC_OUTEN_bm); // disable output buffer and DAC
}

// ===================================================================================
// ADC Implementation for Voltage Sensing and Button Detection
// ===================================================================================

// Init analog to digital converter (ADC)
void ADC_init(void) {
  PORTA.PIN1CTRL &= ~PORT_ISC_gm;                 // disable PA1 as digital pin    
  PORTA.PIN1CTRL |= PORT_ISC_INPUT_DISABLE_gc;
  PORTA.PIN1CTRL &= ~PORT_PULLUPEN_bm;

  VREF.CTRLA &= ~VREF_ADC0REFSEL_gm;              // clear Vref for ADC0
  VREF.CTRLA |= VREF_ADC0REFSEL_4V34_gc;          // select 4.3V ADC reference
  VREF.CTRLB |= VREF_ADC0REFEN_bm;                // keep the reference running

  ADC0.CTRLB = ADC_SAMPNUM_ACC64_gc;              // accumulate 64 samples
  ADC0.CTRLC = ADC_SAMPCAP_bm                     // select sample capacitance
              | ADC_REFSEL_INTREF_gc              // set internal reference
              | ADC_PRESC_DIV8_gc;                // set prescaler -> 625kHz ADC clock
  ADC0.CTRLD  = ADC_INITDLY_DLY64_gc;             // set init delay to 64 cycles
  ADC0.MUXPOS = PIN_SENSE;                        // set the input pin
  ADC0.CTRLA  = ADC_ENABLE_bm;                    // enable ADC with 10-bit resolution
}

// ADC read battery voltage in mV
uint16_t ADC_readVolt(void) {
  ADC0.INTFLAGS = ADC_RESRDY_bm;                  // clear RESRDY flag
  ADC0.COMMAND  = ADC_STCONV_bm;                  // start sampling
  while(~ADC0.INTFLAGS & ADC_RESRDY_bm);          // wait until sampling complete
  uint32_t raw = ADC0.RES / ADC_SAMPLES;          // accumulated sampling result / 64
  return (uint16_t) (raw * ADC_VREF / ADC_STEPS); // calculate and return voltage value
}

// ===================================================================================
// Millis Counter using TCB Periodic Interrupt
// ===================================================================================

volatile uint32_t TCB_counter = 0;                // millis counter

// Init millis counter (TCB)
void TCB_init(void) {
  TCB0.CCMP    = (F_CPU / 1000) - 1;              // set TOP value (period)
  TCB0.CTRLA   = TCB_ENABLE_bm;                   // enable timer/counter
  TCB0.INTCTRL = TCB_CAPT_bm;                     // enable periodic interrupt
}

// Read millis counter value
uint32_t millis(void) {
  while(TCB0.INTFLAGS & TCB_CAPT_bm);
  return TCB_counter;
}

// Timer interrupt service routine (every millisecond)
ISR(TCB0_INT_vect) {
  TCB0.INTFLAGS = TCB_CAPT_bm;                    // clear interrupt flag
  TCB_counter++;                                  // increase millis counter
}

// ===================================================================================
// Main Function
// ===================================================================================

int main(void) {
  // Local variables
  uint16_t volt_set = VOLT_MIN;                   // discharge termination voltage
  uint16_t curr_set = CURR_MIN;                   // discharge current

  // Setup
  _PROTECTED_WRITE(CLKCTRL_MCLKCTRLB, (CLKCTRL_PEN_bm | CLKCTRL_PDIV_4X_gc)); // 5MHz
  GPIO_init();                                    // init buttons
  I2C_init();                                     // init I2C
  ADC_init();                                     // init ADC for voltage sensing
  TCB_init();                                     // init TCB for millis counter
  OLED_init();                                    // init OLED
  sei();                                          // enable interrupts
  OLED_clearScreen();

  // Loop
  while(1) {                                      // loop until forever
    // Set start screen
    OLED_setCursor(0, 5);
    OLED_println("Change settings and");
    OLED_println("connect the battery");
    OLED_println("Press START.");

    // Get user settings for termination voltage and discharge current
    while (!start_pressed) {
    // while (!start_pressed || (ADC_readVolt() <= volt_set)) {
      OLED_setCursor(0, 0);
      OLED_print("Set: ");
      OLED_printVal(volt_set); OLED_print("mV ");
      OLED_printVal(curr_set); OLED_println("mA");
      OLED_println("---------------------");
      
      if(volt_keypressed) {
        volt_set += VOLT_STEP;
        if(volt_set > VOLT_MAX) volt_set = VOLT_MIN;
        volt_keypressed = 0;
      }
      if(curr_keypressed) {
        curr_set += CURR_STEP;
        if(curr_set > CURR_MAX) curr_set = CURR_MIN;
        curr_keypressed = 0;
      }
    }

    uint16_t volt = ADC_readVolt();               // read battery voltage prior to load

    // Start discharge
    uint16_t vinp = curr_set / 10;                // OpAmp vin+ = vin- = curr_set * R4 = curr_set * 1/10 (i.e. 0.1 ohm)
    uint16_t vDAC = (uint32_t) vinp * (R1 + R2) / R2;
    DAC_init(vDAC);
  
    uint16_t vLoad = ADC_readVolt();              // read battery voltage with load
    uint16_t resistance = (uint32_t)(volt - vLoad) * 1000 / DAC_readVolt();  // internal battery resistance in milli-ohm
    uint16_t curr = (uint32_t) DAC_readVolt() * 10 * R2 / (R1 + R2);         // Actual load current on R4 (100m-ohm)

    OLED_setCursor(0, 2);
    OLED_printVal(resistance); OLED_print("mO      ");
    OLED_printVal(curr); OLED_println("mA");

    OLED_setCursor(0, 5);
    OLED_println("");
    OLED_println(" *** Dischaging ***  ");
    OLED_println("");

    uint16_t seconds  = 0;
    uint16_t minutes = 0;
    uint32_t capacity = 0;
    uint32_t energy   = 0;
    uint32_t nextmillis = millis() + 1000;        // first cycle

    // Discharge loop
    while(volt > (volt_set - 10)) {               // 10mV margin to ensure the battery truly fall below volt_set
      // Wait for next cycle (one cycle/second)
      while(millis() < nextmillis);               // wait for the next cycle
      nextmillis += 1000;                         // set end time for next cycle
      
      // Read voltage and current
      volt = ADC_readVolt();                      // read battery voltage in mV

      // Calculate timings
      seconds += 1;                               // calculate total seconds
      minutes = seconds / 60;                     // calculate minutes
      
      // Calculate power, capacity and energy
      uint16_t power = (uint32_t)volt * curr / 1000;
      capacity += curr;
      energy   += power;

      // Update OLED
      OLED_setCursor(0, 3);
      OLED_printVal(volt); OLED_print("mV     ");
      OLED_printDec(minutes / 60); OLED_print(":");
      OLED_printDec(minutes % 60); OLED_print(":");
      OLED_printDec(seconds % 60); OLED_println("");
      OLED_printVal(capacity / 3600); OLED_print("mAh    ");
      OLED_printVal(energy / 3600); OLED_println("mWh");

    }

    // Terminate discharge
    DAC_reset();
    OLED_setCursor(0, 5);
    OLED_println("");
    OLED_println("  *** Completed *** ");
    OLED_println(" remove the battery!");
    while(!start_pressed || ADC_readVolt() > 500);  // wait for button pressed or if battery has not remove yet
  }
}
