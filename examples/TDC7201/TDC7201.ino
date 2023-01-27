/* This example show how to use the TDC720X Time-to-Digital Converter (TDC) Dual core TDC7201
   from Texax Instruments Inc. to measure variable time between events on its START and STOP pins.

   Created by Elochukwu Ifediora on Jan. 25, 2023
*/

#include "TDC720X.h"

#define EXAMPLE_ENABLE_PIN         25

#define EXAMPLE_SPI_CS_PIN1        13
#define EXAMPLE_INTERRUPT_PIN1     16
#define EXAMPLE_START_PIN1         26
#define EXAMPLE_STOP_PIN1          4

#define EXAMPLE_SPI_CS_PIN2        12
#define EXAMPLE_INTERRUPT_PIN2     17
#define EXAMPLE_START_PIN2         2
#define EXAMPLE_STOP_PIN2          0

#define EXTERNAL_OSC_FREQ_HZ       8000000U    // 8MHz
#define NUMBER_OF_STOPS            4           // Note: STOP_5 which is equal to '4' is the maximum number of stops that can be measured by the TDC ASIC

TDC720X TDC1(TDC7201, EXTERNAL_OSC_FREQ_HZ);   // Note: External clock are the same for both TDC cores. 
TDC720X TDC2(TDC7201, EXTERNAL_OSC_FREQ_HZ);

void generate_pulse(const uint32_t time_between_pulses_in_us, const uint8_t stops)
{
    noInterrupts();
    digitalWrite(EXAMPLE_START_PIN1, HIGH);
    digitalWrite(EXAMPLE_START_PIN1, LOW);
    digitalWrite(EXAMPLE_START_PIN2, HIGH);
    digitalWrite(EXAMPLE_START_PIN2, LOW);
    for (uint8_t i = 0; i < stops; ++i)
    {
        delayMicroseconds(time_between_pulses_in_us);
        digitalWrite(EXAMPLE_STOP_PIN1, HIGH);
        digitalWrite(EXAMPLE_STOP_PIN1, LOW);
        digitalWrite(EXAMPLE_STOP_PIN2, HIGH);
        digitalWrite(EXAMPLE_STOP_PIN2, LOW);
    }
    interrupts();
}

void setup()
{
    Serial.begin(115200);
    Serial.println("\nTDC720X Time-to-Digital Converter (TDC) Dual Core example\n");
    
    SPI.begin();
    
    // Note: Enable pins are the same for both TDC cores. Also note that DOUTs for both cores are assumed shorted electrically
    if (!TDC1.begin(EXAMPLE_SPI_CS_PIN1, EXAMPLE_ENABLE_PIN)) {
        Serial.println(F("\nError: TDC1 Initialization failed. Please set at least the CS pin\n"));
        while(1)
            delay(50);
    }
    if (!TDC2.begin(EXAMPLE_SPI_CS_PIN2, EXAMPLE_ENABLE_PIN)) {
        Serial.println(F("\nError: TDC2 Initialization failed. Please set at least the CS pin\n"));
        while(1)
            delay(50);
    }

    pinMode(EXAMPLE_INTERRUPT_PIN1, INPUT_PULLUP);
    pinMode(EXAMPLE_START_PIN1, OUTPUT);
    digitalWrite(EXAMPLE_START_PIN1, LOW);
    pinMode(EXAMPLE_STOP_PIN1, OUTPUT);
    digitalWrite(EXAMPLE_STOP_PIN1, LOW);

    pinMode(EXAMPLE_INTERRUPT_PIN2, INPUT_PULLUP);
    pinMode(EXAMPLE_START_PIN2, OUTPUT);
    digitalWrite(EXAMPLE_START_PIN2, LOW);
    pinMode(EXAMPLE_STOP_PIN2, OUTPUT);
    digitalWrite(EXAMPLE_STOP_PIN2, LOW);

    TDC1.measurement_mode(MODE_2);
    TDC1.calibration_period(CALIBRATION2_PERIODS_20);
    TDC1.calculate_lsb();
    TDC1.avg_cycles(AVG_CYCLE_16);
    TDC1.stops((tdc_stops_t) NUMBER_OF_STOPS);         // Note: The proper usage is TDC1.read_measurement(STOP_1, time), TDC.read_measurement(STOP_2, time) ...etc.
    // TDC1.stop_mask(121000000UL);                    // Setup stop mask to suppress noise between stops.
    // TDC1.overflow(130000000UL);                     // Setup overflow to timeout if no stop was captured before clock timeout.


    TDC2.measurement_mode(MODE_2);
    TDC2.calibration_period(CALIBRATION2_PERIODS_20);
    TDC2.calculate_lsb();
    TDC2.avg_cycles(AVG_CYCLE_16);
    TDC2.stops((tdc_stops_t) NUMBER_OF_STOPS);         // Note: The proper usage is TDC2.read_measurement(STOP_1, time), TDC.read_measurement(STOP_2, time) ...etc.
    // TDC2.stop_mask(121000000UL);                    // Setup stop mask to suppress noise between stops.
    // TDC2.overflow(130000000UL);                     // Setup overflow to timeout if no stop was captured before clock timeout.
}

void loop()
{
    static uint16_t time_between_pulses_in_us = 0;
    time_between_pulses_in_us += 100;
    
    // Note: 8192uS or 8.192mS is the maximum possible event measurement between START and STOP by the TDC ASIC
    if (time_between_pulses_in_us > 8000)
    {
        time_between_pulses_in_us = 0;
    }

    Serial.print(F("\nTime between pulses (given): ")); Serial.println(time_between_pulses_in_us);

    TDC1.start_measurement();
    TDC2.start_measurement();

    // Generate 5 pulses
    generate_pulse(time_between_pulses_in_us, NUMBER_OF_STOPS);

    // Crudely wait for TDC interrupt to indicate overflow or new measurment available
    while (digitalRead(EXAMPLE_INTERRUPT_PIN1) == HIGH);
    while (digitalRead(EXAMPLE_INTERRUPT_PIN2) == HIGH);

    for (uint8_t stop = 0; stop <= 4; stop++)
    {
        float time;
        
        if (TDC1.read_measurement((tdc_stops_t)stop, time))
        {
            char buffer[40];
            dtostrf(time, 8, 8, buffer);
            Serial.print(F("TDC1 tof")); Serial.print(stop); Serial.print(':'); Serial.print(buffer);
        }
        
        if (TDC2.read_measurement((tdc_stops_t)stop, time))
        {
            char buffer[40];
            dtostrf(time, 8, 8, buffer);
            Serial.print(F("\t\tTDC2 tof")); Serial.print(stop); Serial.print(':'); Serial.println(buffer);
        }
    }
    
    Serial.println();
    delay(2000);
}
