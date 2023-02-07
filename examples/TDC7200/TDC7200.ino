/* This example show how to use the TDC720X Time-to-Digital Converter (TDC) Series (TDC7200 & TDC7201)
   from Texas Instruments Inc. to measure variable time between events on its START and STOP pins.
   _________________________________________________
   |                   Connections                  |
   |________________________________________________|
   |   TDC7200     | ARDUINO PRO MINI |    ESP32    |
   |   ENABLE      |        4         |     26      |
   |   TRIGG       |        -         |     -       |
   |   START       |        3         |     17      |
   |   STOP        |        5         |     16      |
   |   CLOCK       |        8         |     25      |
   |   NC          |        -         |     -       |
   |   GND         |       GND        |    GND      |
   |   INTB        |        2         |     4       |
   |   DOUT        |       12         |     23      |
   |   CSB         |       10         |     5       |
   |   SCLK        |       13         |     18      |
   |   DIN         |       11         |     19      |
   |   VREG        |    1uF to GND    | 1uF to GND  |
   |   VDD         |     VCC 3.3V     |  VCC 3.3V   |
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

#define EXAMPLE_SPI_CS_PIN         5
#define EXAMPLE_INTERRUPT_PIN      4
#define EXAMPLE_ENABLE_PIN         26
#define EXAMPLE_START_PIN          17
#define EXAMPLE_STOP_PIN           16

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

#define EXAMPLE_SPI_CS_PIN         10
#define EXAMPLE_INTERRUPT_PIN      2
#define EXAMPLE_ENABLE_PIN         4
#define EXAMPLE_START_PIN          3
#define EXAMPLE_STOP_PIN           5

#endif

#define NUMBER_OF_STOPS            4           // Note: STOP_5 which is equal to '4' is the maximum number of stops that can be measured by the TDC ASIC

TDC720X TDC;                                   // Default: This is the same as TDC720X TDC(TDC7200, TDC720X_DEFAULT_EXTERNAL_OSC_FREQ_HZ) and TDC720X_DEFAULT_EXTERNAL_OSC_FREQ_HZ = 8000000Hz

volatile uint8_t interrupt_flag = 0;
void isr (void);
void generate_pulse(const uint32_t time_between_pulses_in_us, const uint8_t stops);

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

#ifdef ESP32
    // Set the LEDC peripheral configuration
    example_ledc_init();
    // Set duty to 50%
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY));
    // Update duty to apply the new value
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));
#endif

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

    // Crudely wait for TDC interrupt to indicate overflow or new measurement available
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

// The Interrupt Service Routine (ISR)
void isr (void) {
    interrupt_flag = 1;
}

// Generate pulse for the start and stop pins of the TDC
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

#ifdef ESP32
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
#endif
