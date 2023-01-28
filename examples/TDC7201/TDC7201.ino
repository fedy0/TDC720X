/* This example show how to use the TDC720X Time-to-Digital Converter (TDC) Dual core TDC7201
   from Texax Instruments Inc. to measure variable time between events on its START and STOP pins.
   _________________________________________________
   |                   Connections                  |
   |________________________________________________|
   |   TDC7201     | ARDUINO PRO MINI |    ESP32    |
   |   ENABLE      |        4         |     26      |  <-- Shared by the 2 TDC cores
   |   SCLK        |       13         |     18      |  <-- Shared by the 2 TDC cores
   |   DIN         |       11         |     19      |  <-- Shared by the 2 TDC cores
   |   DOUT1       |       12         |     23      |  <-- Terminate with 22R and Short to DOUT1
   |   DOUT2       |       12         |     23      |  <-- Terminate with 22R and Short to DOUT2
   |   GND1        |       GND        |    GND      |
   |   GND2        |       GND        |    GND      |
   |   VDD1        |     VCC 3.3V     |  VCC 3.3V   |
   |   VDD2        |     VCC 3.3V     |  VCC 3.3V   |
   |   VREG1       |   1uF to GND     | 1uF to GND  |  <-- Add external decoupling Capacitor
   |   VREG2       |   1uF to GND     | 1uF to GND  |  <-- Add external decoupling Capacitor
   |   TRIGG1      |        -         |     -       |
   |   TRIGG2      |        -         |     -       |
   |   START1      |        7         |     17      |
   |   START2      |        14        |     32      |
   |   STOP1       |        5         |     16      |
   |   STOP2       |        15        |     33      |
   |   INTB1       |        2         |     4       |
   |   INTB2       |        3         |     15      |
   |   CSB1        |       10         |     5       |
   |   CSB2        |       16         |     0       |
   |   CLOCK       |        8         |     25      |  <-- External clock source; emulated in this example by the host MCUs. See comments below
   |_______________|__________________|_____________|
   
   8MHz EXTERNAL CLOCK EMULATION:
   ARDUINO PRO MINI: 8MHz clock output is configured in Pro Mini Fuse settings.
   ESP32: LEDC Peripheral is used for an 8MHz external clock source on GPIO 25.
   
   Note: The example was tested on ESP32 using arduino-esp32 core version 2.0.5
   
   Created by Elochukwu Ifediora on Jan. 25, 2023
*/

#include "TDC720X.h"

#ifdef ESP32

#include <stdio.h>
#include "driver/ledc.h"
#include "esp_err.h"

#define EXAMPLE_ENABLE_PIN         26

#define EXAMPLE_SPI_CS_PIN1        5
#define EXAMPLE_INTERRUPT_PIN1     4
#define EXAMPLE_START_PIN1         17
#define EXAMPLE_STOP_PIN1          16

#define EXAMPLE_SPI_CS_PIN2        0
#define EXAMPLE_INTERRUPT_PIN2     15
#define EXAMPLE_START_PIN2         32
#define EXAMPLE_STOP_PIN2          33

// Generate an External Clock (Emulate crystal osc.) for the TDC with LEDC Peripheral
#define LEDC_TIMER                 LEDC_TIMER_0
#define LEDC_MODE                  LEDC_HIGH_SPEED_MODE
#define LEDC_OUTPUT_IO             (25)                 // Define the output GPIO
#define LEDC_CHANNEL               LEDC_CHANNEL_0
#define LEDC_DUTY_RES              LEDC_TIMER_5_BIT     // Set duty resolution to 5 bits
#define LEDC_DUTY                  (16)                 // Set duty to 50%. ((2 ** 5) - 1) * 50% = 32
#define LEDC_FREQUENCY             (8000000U)           // Frequency in Hertz. Set frequency at 8 MHz

static void example_ledc_init(void);

#else  // ARDUINO PRO MINIs

#define EXAMPLE_ENABLE_PIN         4

#define EXAMPLE_SPI_CS_PIN1        10
#define EXAMPLE_INTERRUPT_PIN1     2
#define EXAMPLE_START_PIN1         7
#define EXAMPLE_STOP_PIN1          5

#define EXAMPLE_SPI_CS_PIN2        16
#define EXAMPLE_INTERRUPT_PIN2     3
#define EXAMPLE_START_PIN2         14
#define EXAMPLE_STOP_PIN2          15

#endif

#define NUMBER_OF_STOPS            4           // Note: STOP_5 which is equal to '4' is the maximum number of stops that can be measured by the TDC ASIC

TDC720X TDC1(TDC7201, EXTERNAL_OSC_FREQ_HZ);   // Note: External clock are the same for both TDC cores. 
TDC720X TDC2(TDC7201, EXTERNAL_OSC_FREQ_HZ);

void generate_pulse(const uint32_t time_between_pulses_in_us, const uint8_t stops);

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
    
#ifdef ESP32
    // Set the LEDC peripheral configuration
    example_ledc_init();
    // Set duty to 50%
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY));
    // Update duty to apply the new value
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));
#endif

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

// ESP32 Emulating 8MHz External Clock Source
static void example_ledc_init(void)
{
    // Prepare and then apply the LEDC PWM timer configuration
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
        .duty_resolution  = LEDC_DUTY_RES,
        .timer_num        = LEDC_TIMER,
        .freq_hz          = LEDC_FREQUENCY,  // Set output frequency at 5 kHz
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // Prepare and then apply the LEDC PWM channel configuration
    ledc_channel_config_t ledc_channel = {
        .gpio_num       = LEDC_OUTPUT_IO,
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL,
        .intr_type      = LEDC_INTR_DISABLE,
        .timer_sel      = LEDC_TIMER,
        .duty           = 50, // Set duty to 50%
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
}
