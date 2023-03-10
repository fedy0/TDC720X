/*
  Per ipsum, et cum ipso, et in ipso,
  est tibi Deo Patri omnipotenti in unitate Spiritus Sancti,
  omnis honor et gloria per omnia saecula saeculorum...

  Copyright (c) 2023 Elochukwu Ifediora. All Rights Reserved
  Contact:  <ifediora elochukwu c @ gmail dot com>
  
  Arduino Library to support the TI's TDC720X Series Time-to-Digital converter (TDC).
  TDC7200 and TDC7201 are fully supported. To use this library on TDC7201,
  you must instantiate two TDC720X objects with the same "enable" pin number.
  The library assumes that the DOUT pin of the 2 TDC cores in TDC7201 are shorted.
  Therefore, in a real-time application, the objects representing the cores must be guarded
  Note: This library was not written to deeply optimize for memory or speed but for 
  maintainability and easy-of-use with a balance of speed and memory considerations.
  Excluding the Arduino dependencies, it roughly uses 256B of .bss RAM and < 4KiB of Flash
  of the host MCU.

  For information on installing libraries, see: http://www.arduino.cc/en/Guide/Libraries

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  SOFTWARE.
*/


#ifndef _TDC720X_H_INCLUDED
#define _TDC720X_H_INCLUDED

#if (ARDUINO >= 100)
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif

#include <SPI.h>

// Comment the code line below to prevent the host MCU from reading the 8-bit registers twice for every read or write to verify the correctness of the SPI transaction
// For the 24-bit registers, you can enable parity check as in enable_parity(YES);
// Pros: Data reliability
// Cons: Relatively slow access speed
#define VERIFY_SPI_TRANSACTION

// TDC720X Interface Definition
#define TDC720X_DEFAULT_SPI                    SPI
#define TDC720X_DEFAULT_SPI_FREQUENCY          20000000U
#define TDC720X_DEFAULT_SPI_BITORDER           MSBFIRST
#define TDC720X_DEFAULT_SPI_DATAMODE           SPI_MODE0
#define TDC720X_DEFAULT_CS_PIN                 -1
#define TDC720X_DEFAULT_ENABLE_PIN             -1
#define TDC720X_DEFAULT_INTERRUPT_PIN          -1

#define TDC720X_REG_SPI_READ                   0  // 0B00000000
#define TDC720X_REG_SPI_WRITE                  1  // 0B01000000
#define TDC720X_REG_SPI_AUTOINC                1  // 0B10000000

// Timing/Frequency
#define TDC720X_ENABLE_LOW_MS                  5
#define TDC720X_ENABLE_T3_LDO_SET3_MS          2
#define TDC720X_DEFAULT_EXTERNAL_OSC_FREQ_HZ   8000000U
#define TDC720X_INTERNAL_RING_OSC_FREQ_HZ      144000000U
#define PS_PER_SEC                             1000000000000UL // (1E12) 
#define US_PER_SEC                             1000000U        // (1E6)

// TDC720X Registers' Address
#define TDC720X_REG_ADR_CONFIG1                0x00
#define TDC720X_REG_ADR_CONFIG2                0x01
#define TDC720X_REG_ADR_INT_STATUS             0x02
#define TDC720X_REG_ADR_INT_MASK               0x03
#define TDC720X_REG_ADR_COARSE_CNTR_OVF_H      0x04
#define TDC720X_REG_ADR_COARSE_CNTR_OVF_L      0x05
#define TDC720X_REG_ADR_CLOCK_CNTR_OVF_H       0x06
#define TDC720X_REG_ADR_CLOCK_CNTR_OVF_L       0x07
#define TDC720X_REG_ADR_CLOCK_CNTR_STOP_MASK_H 0x08
#define TDC720X_REG_ADR_CLOCK_CNTR_STOP_MASK_L 0x09
#define TDC720X_REG_ADR_TIME1                  0x10
#define TDC720X_REG_ADR_CLOCK_COUNT1           0x11
#define TDC720X_REG_ADR_TIME2                  0x12
#define TDC720X_REG_ADR_CLOCK_COUNT2           0x13
#define TDC720X_REG_ADR_TIME3                  0x14
#define TDC720X_REG_ADR_CLOCK_COUNT3           0x15
#define TDC720X_REG_ADR_TIME4                  0x16
#define TDC720X_REG_ADR_CLOCK_COUNT4           0x17
#define TDC720X_REG_ADR_TIME5                  0x18
#define TDC720X_REG_ADR_CLOCK_COUNT5           0x19
#define TDC720X_REG_ADR_TIME6                  0x1A
#define TDC720X_REG_ADR_CALIBRATION1           0x1B
#define TDC720X_REG_ADR_CALIBRATION2           0x1C

// TDC720X Default Power-On Register Values
#define TDC720X_REG_DEFAULTS_CONFIG2           0x40U
#define TDC720X_REG_DEFAULTS_INT_MASK          0x07U
#define TDC720X_REG_DEFAULTS_OVERFLOWS         0xFFU
#define TDC720X_REG_DEFAULTS_OTHERS            0x00U

// Masks and Bit Positions
#define TDC720X_REG_MASK_ADDR                  0x1F
#define TDC720X_BITS_MASK_CALIBRATION2_PERIODS 0xFFFFFF3FU
#define TDC720X_BITS_MASK_AVG_CYCLES           0xFFFFFFC7U
#define TDC720X_BITS_MASK_NUM_STOP             0xFFFFFFF8U
#define TDC720X_PARITY_BIT_POSITION            23  // MSB [23 22 .....  2 1 0] LSB


namespace TDC72XX {
  // The following enumeration represents the Time-to-Digital Converters available/provided by Texas Instruments
  typedef enum tdc_types {
    TDC7200 = 1,
    TDC7201
  } tdc_t;

  // Access address and data as tdc[tdc_reg_index].addr & tdc[tdc_reg_index].data respectively
  // The register indexes correspond to their names in the datasheet
  typedef enum tdc_reg_index {
    CONFIG1,
    CONFIG2,
    INT_STATUS,
    INT_MASK,
    COARSE_CNTR_OVF_H,
    COARSE_CNTR_OVF_L,
    CLOCK_CNTR_OVF_H,
    CLOCK_CNTR_OVF_L,
    CLOCK_CNTR_STOP_MASK_H,
    CLOCK_CNTR_STOP_MASK_L,
    TIME1,
    CLOCK_COUNT1,
    TIME2,
    CLOCK_COUNT2,
    TIME3,
    CLOCK_COUNT3,
    TIME4,
    CLOCK_COUNT4,
    TIME5,
    CLOCK_COUNT5,
    TIME6,
    CALIBRATION1,
    CALIBRATION2
  } tdc_reg_index_t;

  // The following enumeration represents values acceptable in the two config registers in the TDC ASIC
  typedef enum tdc_config {
    // For CONFIG1 Bit Positions
    START_MEAS,
    MEAS_MODE,
    // Empty      = 2
    START_EDGE    = 3,
    STOP_EDGE,
    TRIGG_EDGE,
    PARITY_EN,
    FORCE_CAL,

    // For CONFIG1 Possible Values
    NO            = 0,
    YES,
    RISING_EGDE   = 0,
    FALLING_EDGE,
    MODE_1        = 0,
    MODE_2
  } tdc_config_t;

  typedef enum tdc_calibration_periods {
    // Calibration Periods
    CALIBRATION2_PERIODS_2  = 0B00000000,
    CALIBRATION2_PERIODS_10 = 0B01000000,
    CALIBRATION2_PERIODS_20 = 0B10000000,
    CALIBRATION2_PERIODS_40 = 0B11000000
  } tdc_calibration_periods_t;

  typedef enum tdc_avg_cycles {
    // Average Cycles
    AVG_CYCLE_1   = 0B00000000,
    AVG_CYCLE_2   = 0B00001000,
    AVG_CYCLE_4   = 0B00010000,
    AVG_CYCLE_8   = 0B00011000,
    AVG_CYCLE_16  = 0B00100000,
    AVG_CYCLE_32  = 0B00101000,
    AVG_CYCLE_64  = 0B00110000,
    AVG_CYCLE_128 = 0B00111000
  } tdc_avg_cycles_t;

  typedef enum tdc_stops {
    // Number of Stops
    STOP_1,
    STOP_2,
    STOP_3,
    STOP_4,
    STOP_5
  } tdc_stops_t;

  // The following enumeration represents Interrupt Status & Mask Bit Field/Positions in the TDC ASIC
  typedef enum intr {
    // For INT_STATUS: Interrupt Status Register
    // Writing a 1 will clear the status
    NEW_MEAS_INT,
    COARSE_CNTR_OVF_INT,
    CLOCK_CNTR_OVF_INT,
    MEAS_STARTED_FLAG,
    MEAS_COMPLETE_FLAG,

    // For INT_MASK: Interrupt Mask Register
    // Write 1 to enable, and 0 to disable
    NEW_MEAS_MASK = 0,
    COARSE_CNTR_OVF_MASK,
    CLOCK_CNTR_OVF_MASK
  } intr_t;

  // The following type definition represents the layout of each register in the TDC ASIC
  typedef union tdc_address {
      uint8_t address;
      struct addr_bits {
        uint8_t raddr : 6;  // Register address
        uint8_t rw   : 1;   // Read(0) or Write (1)
        uint8_t incr : 1;   // Auto increment ON (1) or OFF (0)
      } bits; // __attribute__ ((packed));
  } tdc_addr_t;

  typedef struct tdc_registers {
    const uint8_t addr;
    uint32_t data;
  } tdc_reg_t;

  typedef void (*callbackFunction)();

  // The Time-to-Digital Converter Class that encapsulates all functions provided by the TDC ASIC
  class TDC720X {
    public:
      explicit TDC720X(tdc_t _tdc_type  = TDC7200, uint32_t clock_frequency = TDC720X_DEFAULT_EXTERNAL_OSC_FREQ_HZ);
      ~TDC720X();
      // Manually set the SPI protocol after the TDC object has been created
      inline void set_spi(SPIClass* _spi) { spi = _spi; };
      inline void spi_settings(SPISettings& _settings) { settings = _settings; };
      inline void spi_settings(uint32_t frequency=TDC720X_DEFAULT_SPI_FREQUENCY, uint8_t bitorder=TDC720X_DEFAULT_SPI_BITORDER, uint8_t mode=TDC720X_DEFAULT_SPI_DATAMODE) { \
        settings = SPISettings(frequency, bitorder, mode); \
      };
      bool begin(int8_t _cs=TDC720X_DEFAULT_CS_PIN, int8_t _en=TDC720X_DEFAULT_ENABLE_PIN);
      
      void attach_interrupt(int8_t in=TDC720X_DEFAULT_INTERRUPT_PIN, callbackFunction callback = NULL, uint8_t mode = RISING);
      inline void enable_interrupt(intr_t bit_position);
      inline void disable_interrupt(intr_t bit_position);
      static void tdc_interrupt_handler(void);
      static callbackFunction tdc_interrupt_callback;
      
      inline void enable (void) { \
        if (en >= 0) { \
        /* As recommended in the datasheet; a low-to-high transition */ \
          digitalWrite(en, LOW); \
          delay(TDC720X_ENABLE_LOW_MS); \
          digitalWrite(en, HIGH);
          delay(TDC720X_ENABLE_T3_LDO_SET3_MS); \
        } \
      };
      inline void disable (void) { \
        if (en >= 0) { \
          digitalWrite(en, LOW); \
        } \
      };

      inline void enable_auto_increment(void) { address.bits.incr = TDC720X_REG_SPI_AUTOINC; };
      inline void disable_auto_increment(void) { address.bits.incr = 0; };

      bool write(const tdc_reg_index_t reg_index, const uint32_t reg_data);
      bool read(const tdc_reg_index_t reg_index);
      inline uint32_t data(const tdc_reg_index_t reg_index) { read(reg_index); return tdc[reg_index].data; };

      // Reads the value of the interrupt bits in a given register
      bool read(intr_t bit_position, tdc_reg_index_t reg_index) { read(reg_index); return ((tdc[reg_index].data >> bit_position) & 1) ? true : false; };
      // Reads the value of the individual bit positions in a given register
      inline bool read(uint8_t bit_position, tdc_reg_index_t reg_index) { return ((tdc[reg_index].data >> bit_position) & 1) ? true : false; };

      inline void set (uint8_t bit_position, tdc_reg_index_t reg_index)   { tdc[reg_index].data |= (1 << bit_position);  };
      inline void clear (uint8_t bit_position, tdc_reg_index_t reg_index) { tdc[reg_index].data &= ~(1 << bit_position); };
      
      inline void force_calibration(tdc_config_t config) { if (config) set(FORCE_CAL,  CONFIG1); else clear(FORCE_CAL,  CONFIG1); write(CONFIG1, tdc[CONFIG1].data); };
      inline void enable_parity(tdc_config_t config)     { if (config) set(PARITY_EN,  CONFIG1); else clear(PARITY_EN,  CONFIG1); write(CONFIG1, tdc[CONFIG1].data); };
      inline void trigger_edge(tdc_config_t config)      { if (config) set(TRIGG_EDGE, CONFIG1); else clear(TRIGG_EDGE, CONFIG1); write(CONFIG1, tdc[CONFIG1].data); };
      inline void stop_edge(tdc_config_t config)         { if (config) set(STOP_EDGE,  CONFIG1); else clear(STOP_EDGE,  CONFIG1); write(CONFIG1, tdc[CONFIG1].data); };
      inline void start_edge(tdc_config_t config)        { if (config) set(START_EDGE, CONFIG1); else clear(START_EDGE, CONFIG1); write(CONFIG1, tdc[CONFIG1].data); };
      inline void measurement_mode(tdc_config_t config)  { clear(2, CONFIG1); if (config) set(MEAS_MODE, CONFIG1); else clear(MEAS_MODE, CONFIG1); write(CONFIG1, tdc[CONFIG1].data); };
      void start_measurement(void);

      inline void calibration_period(const tdc_calibration_periods_t config) { tdc[CONFIG2].data &= TDC720X_BITS_MASK_CALIBRATION2_PERIODS; tdc[CONFIG2].data |= config; write(CONFIG2, tdc[CONFIG2].data); };
      inline void avg_cycles(const tdc_avg_cycles_t config)                  { tdc[CONFIG2].data &= TDC720X_BITS_MASK_AVG_CYCLES;           tdc[CONFIG2].data |= config; write(CONFIG2, tdc[CONFIG2].data); };
      inline void stops(const tdc_stops_t config)                            { tdc[CONFIG2].data &= TDC720X_BITS_MASK_NUM_STOP;             tdc[CONFIG2].data |= config; write(CONFIG2, tdc[CONFIG2].data); };

      void stop_mask(const uint64_t stop_mask_in_ps);
      void overflow(const uint64_t overflow_in_ps);
      void calculate_lsb();
      bool read_measurement(const tdc_stops_t stop, float& tof);
      
      inline void end (void) { disable(); };

    private:
      SPISettings settings;
      SPIClass* spi;

      int8_t cs;                   // Chip Select Pin
      int8_t en;                   // Enable Pin
      int8_t interrupt_pin;
      float lsb;                   // The normalized LSB (normLSB)
      float clock_period_in_ps;
      
      tdc_t tdc_type;
      tdc_addr_t address;
      tdc_reg_t tdc[23];
  };
}

using namespace TDC72XX;

#endif // End _TDC720X_H_INCLUDED
