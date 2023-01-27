/* This example show how to use the TDC720X Time-to-Digital Converter (TDC) Series (TDC7200 & TDC7201)
   from Texax Instruments Inc. to measure variable time between events on its START and STOP pins.

   Created by Elochukwu Ifediora on Jan. 25, 2023
   _________________________________________________
   |                   Connections                  |
   |________________________________________________|
   |   TDC7200     | ARDUINO PRO MINI |    ESP32    |
   |   1 Enable    |        4         |             |
   |   2 Trigg     |        -         |             |
   |   3 Start     |        3         |             |
   |   4 Stop      |        5         |             |
   |   5 Clock     |        8         |             |     <- 8Mhz clock output configured in Pro Mini Fuse settings
   |   6 NC        |        -         |             |
   |   7 GND       |       GND        |             |
   |   8 Intb      |        2         |             |
   |   9 Dout      |       12         |             |
   |  10 Din       |       11         |             |
   |  11 Csb       |       10         |             |
   |  12 Sclk      |       13         |             |
   |  13 Vreg      |        -         |             |     <- Via 1uF to Gnd
   |  14 VDD       |     VCC 3.3V     |             |
   |_______________|__________________|_____________|

*/

#include "TDC720X.h"

#define EXAMPLE_SPI_CS_PIN         6
#define EXAMPLE_INTERRUPT_PIN      2
#define EXAMPLE_ENABLE_PIN         4
#define EXAMPLE_START_PIN          5
#define EXAMPLE_STOP_PIN           6

#define NUMBER_OF_STOPS            4           // Note: STOP_5 which is equal to '4' is the maximum number of stops that can be measured by the TDC ASIC

TDC720X TDC;                                   // Default: This is the same as TDC720X TDC(TDC7200, TDC720X_DEFAULT_EXTERNAL_OSC_FREQ_HZ)

volatile uint8_t interrupt_flag = 0;

// The Interrupt Service Routine (ISR)
void isr (void) {
    interrupt_flag = 1;
}

void generate_pulse(const uint32_t time_between_pulses_in_us, const uint8_t stops)
{
    noInterrupts();
    digitalWrite(EXAMPLE_START_PIN, HIGH);
    digitalWrite(EXAMPLE_START_PIN, LOW);
    for (uint8_t i = 0; i < stops; ++i)
    {
        delayMicroseconds(time_between_pulses_in_us);
        digitalWrite(EXAMPLE_STOP_PIN, HIGH);
        digitalWrite(EXAMPLE_STOP_PIN, LOW);
    }
    interrupts();
}

void setup()
{
    Serial.begin(115200);
    Serial.println("\nTDC720X Time-to-Digital Converter (TDC) example\n");
    
    SPI.begin();
    
    if (!TDC.begin(EXAMPLE_SPI_CS_PIN, EXAMPLE_ENABLE_PIN)) {
        Serial.println(F("\nError: TDC Initialization failed. Please set at least the CS pin\n"));
        while(1)
            delay(50);
    }

    pinMode(EXAMPLE_START_PIN, OUTPUT);
    digitalWrite(EXAMPLE_START_PIN, LOW);

    pinMode(EXAMPLE_STOP_PIN, OUTPUT);
    digitalWrite(EXAMPLE_STOP_PIN, LOW);

    TDC.attach_interrupt(EXAMPLE_INTERRUPT_PIN, isr, RISING);
    TDC.measurement_mode(MODE_2);
    TDC.calibration_period(CALIBRATION2_PERIODS_20);
    TDC.calculate_lsb();
    TDC.avg_cycles(AVG_CYCLE_16);
    TDC.stops((tdc_stops_t) NUMBER_OF_STOPS);         // Note: The proper usage is TDC.read_measurement(STOP_1, time), TDC.read_measurement(STOP_2, time) ...etc.
    // TDC.stop_mask(121000000UL);                    // Setup stop mask to suppress noise between stops.
    // TDC.overflow(130000000UL);                     // Setup overflow to timeout if no stop was captured before clock timeout.
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

    Serial.print(F("\nTime between pulses (given): ")); Serial.print(time_between_pulses_in_us);

    TDC.start_measurement();

    // Generate 5 pulses
    generate_pulse(time_between_pulses_in_us, NUMBER_OF_STOPS);

    // Crudely wait for TDC interrupt to indicate overflow or new measurment available
    while (interrupt_flag == 0);
    interrupt_flag = 0;

    for (uint8_t stop = 0; stop <= 4; stop++)
    {
        float time;
        if (TDC.read_measurement((tdc_stops_t)stop, time))
        {
            char buffer[40];
            dtostrf(time, 8, 8, buffer);
            Serial.print(F(", measured")); Serial.print(stop); Serial.print(':'); Serial.print(buffer);
        }
    }
    Serial.println();
    delay(2000);
}
