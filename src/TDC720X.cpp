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


#include <TDC720X.h>

TDC720X::TDC720X(tdc_t _tdc_type, uint32_t clock_frequency): \
                settings(TDC720X_DEFAULT_SPI_FREQUENCY, TDC720X_DEFAULT_SPI_BITORDER, TDC720X_DEFAULT_SPI_DATAMODE), \
                spi(&TDC720X_DEFAULT_SPI), \
                cs(-1), \
                en(-1), \
                interrupt_pin(-1), \
                lsb(0), \
                clock_period_in_ps(uint64_t(PS_PER_SEC) / clock_frequency), \
                tdc_type(_tdc_type), \
                address{0}, \
                tdc{//Register Address,                       Register Default Data
                    { TDC720X_REG_ADR_CONFIG1,                 TDC720X_REG_DEFAULTS_OTHERS   }, \
                    { TDC720X_REG_ADR_CONFIG2,                 TDC720X_REG_DEFAULTS_CONFIG2  }, \
                    { TDC720X_REG_ADR_INT_STATUS,              TDC720X_REG_DEFAULTS_OTHERS   }, \
                    { TDC720X_REG_ADR_INT_MASK,                TDC720X_REG_DEFAULTS_INT_MASK }, \
                    { TDC720X_REG_ADR_COARSE_CNTR_OVF_H,       TDC720X_REG_DEFAULTS_OVERFLOWS}, \
                    { TDC720X_REG_ADR_COARSE_CNTR_OVF_L,       TDC720X_REG_DEFAULTS_OVERFLOWS}, \
                    { TDC720X_REG_ADR_CLOCK_CNTR_OVF_H,        TDC720X_REG_DEFAULTS_OVERFLOWS}, \
                    { TDC720X_REG_ADR_CLOCK_CNTR_OVF_L,        TDC720X_REG_DEFAULTS_OVERFLOWS}, \
                    { TDC720X_REG_ADR_CLOCK_CNTR_STOP_MASK_H,  TDC720X_REG_DEFAULTS_OTHERS   }, \
                    { TDC720X_REG_ADR_CLOCK_CNTR_STOP_MASK_L,  TDC720X_REG_DEFAULTS_OTHERS   }, \
                    { TDC720X_REG_ADR_TIME1,                   TDC720X_REG_DEFAULTS_OTHERS   }, \
                    { TDC720X_REG_ADR_CLOCK_COUNT1,            TDC720X_REG_DEFAULTS_OTHERS   }, \
                    { TDC720X_REG_ADR_TIME2,                   TDC720X_REG_DEFAULTS_OTHERS   }, \
                    { TDC720X_REG_ADR_CLOCK_COUNT2,            TDC720X_REG_DEFAULTS_OTHERS   }, \
                    { TDC720X_REG_ADR_TIME3,                   TDC720X_REG_DEFAULTS_OTHERS   }, \
                    { TDC720X_REG_ADR_CLOCK_COUNT3,            TDC720X_REG_DEFAULTS_OTHERS   }, \
                    { TDC720X_REG_ADR_TIME4,                   TDC720X_REG_DEFAULTS_OTHERS   }, \
                    { TDC720X_REG_ADR_CLOCK_COUNT4,            TDC720X_REG_DEFAULTS_OTHERS   }, \
                    { TDC720X_REG_ADR_TIME5,                   TDC720X_REG_DEFAULTS_OTHERS   }, \
                    { TDC720X_REG_ADR_CLOCK_COUNT5,            TDC720X_REG_DEFAULTS_OTHERS   }, \
                    { TDC720X_REG_ADR_TIME6,                   TDC720X_REG_DEFAULTS_OTHERS   }, \
                    { TDC720X_REG_ADR_CALIBRATION1,            TDC720X_REG_DEFAULTS_OTHERS   }, \
                    { TDC720X_REG_ADR_CALIBRATION2,            TDC720X_REG_DEFAULTS_OTHERS   }  \
                }  \
{

}

TDC720X::~TDC720X() {
    end();
}

// Initialize necessary variables that the event measurement is dependent on
bool TDC720X::begin(int8_t _cs, int8_t _en) {

    if (_cs >= 0) {
        cs = _cs;
        pinMode(cs, OUTPUT);
        digitalWrite(cs, HIGH);
    }
    else {
        return false; // CS pin must be set; it is not optional
    }

    if (_en >= 0) {
        en = _en;
        enable();
    }
    
    // Read default registers to be sure that the TDC ASIC is enabled or not faulty
    uint32_t reg_data;
    reg_data = tdc[CONFIG2].data;
    read(CONFIG2);
    if (reg_data != tdc[CONFIG2].data)
        return false;

    reg_data = tdc[INT_MASK].data;
    read(INT_MASK);
    if (reg_data != tdc[INT_MASK].data)
        return false;
    
    disable_auto_increment();
    calibration_period(CALIBRATION2_PERIODS_10);
    stops(STOP_1);
    calculate_lsb();

    return true;
}

// Define the default callback (callee) to capture and execute the user interrupt service routine
callbackFunction TDC720X::tdc_interrupt_callback = (callbackFunction) NULL;

// Set the host interrupt input pin, its mode and the user's routine to service the interrupt
void TDC720X::attach_interrupt(int8_t in, callbackFunction callback, uint8_t mode) {
    interrupt_pin = in;
    tdc_interrupt_callback = callback;

    if (in >= 0) {
        //pinMode(interrupt_pin, INPUT_PULLUP);
        attachInterrupt(digitalPinToInterrupt(interrupt_pin), tdc_interrupt_handler, mode);
    } 
    else {
        detachInterrupt(digitalPinToInterrupt(interrupt_pin));
    }
}

// Enable the type of interrupt that would be visible in the INT pin of TDC ASIC
void TDC720X::enable_interrupt(intr_t bit_position) {
    set((uint8_t) bit_position, INT_MASK);
    write(INT_MASK, tdc[INT_MASK].data);
}

// Disable the type of interrupt that is not allowed to be visible in the INT pin of TDC ASIC
void TDC720X::disable_interrupt(intr_t bit_position) {
    clear((uint8_t) bit_position, INT_MASK);
    write(INT_MASK, tdc[INT_MASK].data);
}

// Caller: Execute the callback (callee) that implements the interrupt service routines (i.e. The default, and that of the user)
void TDC720X::tdc_interrupt_handler(void) {
    // Execute default function

    // Maybe include user-defined function
    if (tdc_interrupt_callback != NULL)
        tdc_interrupt_callback();
}

// Write the value "reg_data" into the location ( both mirriored and actual memory layout of the TDC ASIC) specified by the index
bool TDC720X::write(tdc_reg_index_t reg_index, uint32_t reg_data) {
    if (reg_index >= TIME1) {
        // The registers from TIME1 to CALIBRATION2 are read-only registers
        return false;
    }

    address.bits.raddr = tdc[reg_index].addr;
    address.bits.rw = TDC720X_REG_SPI_WRITE;
    tdc[reg_index].data = reg_data;
    
    spi->beginTransaction(settings);
    digitalWrite(cs, LOW);
    spi->transfer(address.address);
    //if (reg_index < TIME1) {
        spi->transfer((uint8_t)reg_data);
    //}
    //else {
        // spi->transfer((uint8_t)(reg_data >> 16));
        // spi->transfer((uint8_t)(reg_data >> 8));
        // spi->transfer((uint8_t)reg_data);
    //}
    digitalWrite(cs, HIGH);
    spi->endTransaction();

    #ifdef VERIFY_SPI_TRANSACTION
    spi->beginTransaction(settings);
    digitalWrite(cs, LOW);
    spi->transfer(address.address);
    uint32_t reg_data2 = spi->transfer(0);
    digitalWrite(cs, HIGH);
    spi->endTransaction();
    if (reg_data != reg_data2)
        return false;
    #endif
    
    return true;
}

// Copy the value in the actual TDC ASIC memory specified by the index to the mirriored memory layout of the TDC ASIC
bool TDC720X::read(tdc_reg_index_t reg_index) {
    uint32_t reg_data;
    address.bits.raddr = tdc[reg_index].addr;
    address.bits.rw = TDC720X_REG_SPI_READ;
    
    spi->beginTransaction(settings);
    digitalWrite(cs, LOW);
    spi->transfer(address.address);
    reg_data = spi->transfer(0);
    if (reg_index >= TIME1) {
        reg_data <<= 8;
        reg_data |= spi->transfer(0);
        reg_data <<= 8;
        reg_data |= spi->transfer(0);
    }
    digitalWrite(cs, HIGH);
    spi->endTransaction();

    // Parity check if enabled
    if (reg_index >= TIME1) {
        if (read(PARITY_EN, CONFIG1)) {
            uint8_t sum = 0;
            for (uint8_t bit = 0; bit <= TDC720X_PARITY_BIT_POSITION; bit++)
                sum += (uint8_t)read(bit, reg_index);
            if (sum % 2 == 1)
                return false;
        }
        clear(TDC720X_PARITY_BIT_POSITION, CONFIG1);
    }
    #ifdef VERIFY_SPI_TRANSACTION
    else {
        spi->beginTransaction(settings);
        digitalWrite(cs, LOW);
        spi->transfer(address.address);
        uint32_t reg_data2 = spi->transfer(0);
        digitalWrite(cs, HIGH);
        spi->endTransaction();
        if (reg_data != reg_data2)
            return false;
    }
    #endif

    tdc[reg_index].data = reg_data;
    return true;
}

// Convert duration of stopmask from picoseconds to clock count.
void TDC720X::stop_mask(const uint64_t stop_mask_in_ps) {
    uint32_t count = stop_mask_in_ps / clock_period_in_ps;
    write(CLOCK_CNTR_STOP_MASK_H, count >> 8);
    write(CLOCK_CNTR_STOP_MASK_L, count);
}

// Convert duration of overflow from picoseconds to coarse or clock count.
void TDC720X::overflow(const uint64_t overflow_in_ps) {
    // Check if the measurement is in MODE 2. If so, then set COARSE_CNTR_OVF to max value i.e. FFFFH (COARSE_CNTR_OVF_H * 2^8 + COARSE_CNTR_OVF_L)
    uint16_t coarse_overflow = 0xFFFF;   // MODE_1: 0xFFFF is 454.164us which is the maximum
    uint16_t clock_overflow  = 0xFFFF;   // MODE_2: 0xFFFF is 8.192ms which is the maximum
    calculate_lsb();
    if (read(MEAS_MODE, CONFIG1)) {
        // MODE 2
        clock_overflow = overflow_in_ps / clock_period_in_ps;
        clock_overflow = (clock_overflow < 0xFFFF) ? clock_overflow : 0xFFFF;
    }
    else {
        // MODE 1
        coarse_overflow = overflow_in_ps / (lsb * 63);
        coarse_overflow = (coarse_overflow < 0xFFFF) ? coarse_overflow : 0xFFFF;
    }

    write(COARSE_CNTR_OVF_H, coarse_overflow >> 8);
    write(COARSE_CNTR_OVF_L, coarse_overflow);
    write(CLOCK_CNTR_OVF_H, clock_overflow >> 8);
    write(CLOCK_CNTR_OVF_L, clock_overflow);
}

// Force the calculation of the normalized LSB value.
// For increased accuracy, this method should be ran/called often (say, per minute)
// to update the normLSB as variations in the environment introduces real-time errors
void TDC720X::calculate_lsb(void) {
    uint8_t default_period = ((tdc[CONFIG2].data & (~TDC720X_BITS_MASK_CALIBRATION2_PERIODS)) >> 6); 
    switch (default_period) {
        case CALIBRATION2_PERIODS_2:
            default_period = 2;
        break;
        case CALIBRATION2_PERIODS_20:
            default_period = 20;
        break;
        case CALIBRATION2_PERIODS_40:
            default_period = 40;
        break;
        case CALIBRATION2_PERIODS_10:
        default:
            default_period = 10;
            calibration_period(CALIBRATION2_PERIODS_10);
        break;
    }
    bool default_force_calibration = read(FORCE_CAL, CONFIG1);
    force_calibration(YES);
    start_measurement();

    // Delay to ensure overflow occurs to force the TDC ASIC to re-calibrate
    // The optimized delay value was obtained from the following equation:
    // delay = max. overflow for mode 2 + (max external clock period x max calibration period)
    // delay = 8.192ms (which is the biggest overflow, bigger than mode 1) + 1/1MHz (the slowest clock) x 40 calibration cycle = 8.232ms
    delay(10);
    force_calibration((tdc_config_t)default_force_calibration); // Reset this settings to avoid messing with the user settings
    write(CONFIG1, tdc[CONFIG1].data);

    read(CALIBRATION1);
    read(CALIBRATION2);

    float calibration_count = (tdc[CALIBRATION2].data - tdc[CALIBRATION1].data) / (default_period - 1);

    lsb = (float)(clock_period_in_ps / PS_PER_SEC) / calibration_count;
}

// Start the event measurement
void TDC720X::start_measurement(void) {
    // Reset necessary flag for th new round of measurement
    // NOTE: According to the datasheet, writing a 1 will clear the interrupt statuses
    // set((uint8_t) NEW_MEAS_INT, INT_STATUS);
    // set((uint8_t) COARSE_CNTR_OVF_INT, INT_STATUS);
    // set((uint8_t) CLOCK_CNTR_OVF_INT, INT_STATUS);
    // set((uint8_t) MEAS_STARTED_FLAG, INT_STATUS);
    // set((uint8_t) MEAS_COMPLETE_FLAG, INT_STATUS);
    // write(INT_STATUS, tdc[INT_STATUS].data);
    tdc[INT_STATUS].data = 0;

    // If interrupt pin was enabled, clear the interrupt statuses
    // if (interrupt_pin >= 0) {
    //     set((uint8_t) EW_MEAS_MASK, INT_STATUS);
    //     set((uint8_t) COARSE_CNTR_OVF_MASK, INT_STATUS);
    //     set((uint8_t) CLOCK_CNTR_OVF_MASK, INT_STATUS);
    //     write(INT_MASK, tdc[INT_MASK].data);
    // }
    
    // If interrupt pins was enabled, clear the interrupt statuses
    if (interrupt_pin >= 0) {
        set((uint8_t) CLOCK_CNTR_OVF_MASK, INT_MASK);
        set((uint8_t) COARSE_CNTR_OVF_MASK, INT_MASK);
        set((uint8_t) NEW_MEAS_MASK, INT_MASK);
        write(INT_MASK, tdc[INT_MASK].data);
    }
    
    // Start measurement
    set(START_MEAS, CONFIG1);
    write(CONFIG1, tdc[CONFIG1].data);
}

// Read measurement result between START and multiple STOPS (max = 5 STOPs).
// Success: The method returns "true" with the time-of-flight in "tof"
// Fail: The method returns "false" with the failure description in "tof" as -1, 0, 1, and 2 corresponding to
// "Reading STOPs beyond configuration", "No New Measurement", "Coarse Counter Overflow", and "Clock Counter Overflow" respectivly
bool TDC720X::read_measurement(const tdc_stops_t stop, float& tof) {
    // TODO: Service the type of interrupt that occured (Imeplementations below are temporary)
    if (read(CLOCK_CNTR_OVF_INT, INT_STATUS)) {
        tof = CLOCK_CNTR_OVF_INT;                        // Error: Clock Counter Overflow occurred
        set((uint8_t) CLOCK_CNTR_OVF_INT, INT_STATUS);   // Clear the interrupt: Writing a '1' clears the interrupt according to the datasheet. The same applies for other interrupt statuses below
        return false;
    }
    if (read(COARSE_CNTR_OVF_INT, INT_STATUS)) {
        tof = COARSE_CNTR_OVF_INT;                       // Error: Coarse Counter Overflow occurred
        set((uint8_t) COARSE_CNTR_OVF_INT, INT_STATUS);  // Clear the interrupt
        return false;
    }

    // Prevent "stop" reads which was not configured.
    uint8_t index = tdc[CONFIG2].data & (~TDC720X_BITS_MASK_NUM_STOP);           // Now, "index" holds "number of configured stops 
    if (index <= stop) {
        // Check measurement interrupt status before reading measurements eg. Has measurement completed?
        if (read(NEW_MEAS_INT, INT_STATUS) == false) {
            tof = NEW_MEAS_INT;                               // Error: No new measurement found
            set((uint8_t) COARSE_CNTR_OVF_INT, INT_STATUS);   // Clear the interrupt
            return false;
        }
        // Check measurement mode, then compute measurement accordingly
        if (read(MEAS_MODE, CONFIG1)) {
            // MODE 2   
            uint32_t time_n_plus_one;
            uint32_t clock_count_n;
            
            read(TIME1);                               // Read TIME1
            
            index = TIME1 + (2 * (stop + 1));
            read((tdc_reg_index_t)index);              // Read TIME(n+1)
            time_n_plus_one = tdc[index].data;

            index = CLOCK_COUNT1 + (2 * stop);
            read((tdc_reg_index_t)index);              // Read CLOCK_COUNTn
            clock_count_n = tdc[index].data;

            index = ((tdc[CONFIG2].data & (~TDC720X_BITS_MASK_AVG_CYCLES)) >> 3); // Now, "index" holds "the average cycle configured for multi-avaraging mode
            if (index > 0) {
                // Multi-Cycle Averaging Mode
                uint8_t shift = log(index)/log(2);
                tof = (lsb * (tdc[TIME1].data - time_n_plus_one)) + ((clock_count_n >> shift) * (clock_period_in_ps / PS_PER_SEC));
            }
            else {
                // No Multi-Cycle Averaging Mode
                tof = (lsb * (tdc[TIME1].data - time_n_plus_one)) + (clock_count_n * (clock_period_in_ps / PS_PER_SEC));
            }
        }
        else {
            // MODE 1
            uint8_t index = (uint8_t)TIME1 + (2 * (uint8_t)stop);
            read((tdc_reg_index_t)index);              // Read TIMEn
            tof = tdc[index].data * lsb;
        }
    }
    else {
        tof = -1;                                      // Error: You're trying to read beyond the maximum supported stops
        return false;
    }
    
    return true;
}
