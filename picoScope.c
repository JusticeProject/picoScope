#include <stdio.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"
#include "hardware/dma.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"
#include "pico/multicore.h"
#include "math.h"
#include "hardware/regs/addressmap.h"

#define ADC_GPIO_PIN 28
#define ADC_CHANNEL 2

#define PWM_GPIO 0

//*************************************************************************************************

#define NUM_SAMPLE_BLOCKS 2048
#define SAMPLE_BLOCK_SIZE 64
#define NUM_SAMPLES (NUM_SAMPLE_BLOCKS * SAMPLE_BLOCK_SIZE)
uint8_t capture_buf[NUM_SAMPLES];
int currentBlock = 0;

#define SIN_LOOKUP_TABLE_SIZE 10000
uint16_t sinLookupDuty[SIN_LOOKUP_TABLE_SIZE];

uint16_t clkDiv = 0;
uint16_t currentWrap = 0;
uint16_t duty = 0;
bool generateSinWave = false;
uint16_t sinFreqHz = 10;

//*************************************************************************************************

uint16_t getIntFromUser()
{
    char buf[6];
    memset(buf, 0, sizeof(buf));
    fgets(buf, sizeof(buf), stdin);
    uint16_t number = atoi(buf);
    
    return number;
}

//*************************************************************************************************

void doDMA()
{
    uint dma_chan = dma_claim_unused_channel(true);
    dma_channel_config cfg = dma_channel_get_default_config(dma_chan);
    channel_config_set_transfer_data_size(&cfg, DMA_SIZE_8); // Reading from constant address, writing to incrementing byte addresses
    channel_config_set_read_increment(&cfg, false);
    channel_config_set_write_increment(&cfg, true);
    channel_config_set_dreq(&cfg, DREQ_ADC); // Pace transfers based on availability of ADC samples

    dma_channel_configure(dma_chan, &cfg,
        capture_buf,    // dst
        &adc_hw->fifo,  // src
        NUM_SAMPLES,  // transfer count
        true            // start immediately
    );

    adc_run(true);
    dma_channel_wait_for_finish_blocking(dma_chan);
    adc_run(false);
    adc_fifo_drain();

    dma_channel_cleanup(dma_chan);
    dma_channel_unclaim(dma_chan);
}

//*************************************************************************************************

void core1_entry()
{
    const uint SAMPLE_PERIOD_US = 10; // 1 / 10us = 100kHz
    uint SAMPLES_PER_SIN_PERIOD = 0;
    uint lookupLocation = 0;
    bool on = false;

    while (true)
    {
        if (!on && generateSinWave)
        {
            on = true;
            // calculate lookup table
            // and make sure we don't go past the end of the lookup table
            SAMPLES_PER_SIN_PERIOD = (1.0 / sinFreqHz) / (SAMPLE_PERIOD_US / 1000000.0); // secs / secs per sample = samples
            if (SAMPLES_PER_SIN_PERIOD >= SIN_LOOKUP_TABLE_SIZE)
            {
                SAMPLES_PER_SIN_PERIOD = SIN_LOOKUP_TABLE_SIZE;
            }

            for (int n = 0; n < SAMPLES_PER_SIN_PERIOD; n++)
            {
                double t = n * SAMPLE_PERIOD_US / 1000000.0;
                double signal = sin(2 * 3.14159 * sinFreqHz * t);
                signal += 1; // give it an offset, the signal will now swing from 0 to 2, centered at 1
                // When the signal is 0, we want a duty cycle of 0%.
                // when the signal is 2, we want a duty cycle of 100%.
                // duty = slope * signal + yIntercept (the yIntercept will be 0)
                double slope = currentWrap / 2;
                sinLookupDuty[n] = round(slope * signal); // convert it to fit in 0-100% duty cycle
            }
            lookupLocation = 0;
        }
        else if (on && generateSinWave)
        {
            // spit out lookup table, look for end
            pwm_set_gpio_level(PWM_GPIO, sinLookupDuty[lookupLocation]);
            lookupLocation++;
            if (lookupLocation >= SAMPLES_PER_SIN_PERIOD)
            {
                lookupLocation = 0;
            }

            // TODO: to get better timing, we need to make sure the start of this calculation happens every SAMPLE_PERIOD_MS, try an interrupt or an alarm
            sleep_us(SAMPLE_PERIOD_US);
        }
        else
        {
            // do nothing
            on = false;
            sleep_us(SAMPLE_PERIOD_US);
        }
    }
}

//*************************************************************************************************

int main()
{
    stdio_init_all();
    adc_init();
    if (cyw43_arch_init())
    {
        printf("Wi-Fi init failed");
        return -1;
    }

    gpio_set_function(PWM_GPIO, GPIO_FUNC_PWM); // set GPIO 0 to PWM
    uint slice_num = pwm_gpio_to_slice_num(PWM_GPIO); // Find out which PWM slice is connected to GPIO 0

    adc_gpio_init(ADC_GPIO_PIN); // Make sure the pin is high-impedance, no pullups etc
    adc_select_input(ADC_CHANNEL); // Select ADC input 2 (GPIO28)
    adc_fifo_setup(
        true,    // Write each completed conversion to the sample FIFO
        true,    // Enable DMA data request (DREQ)
        1,       // DREQ (and IRQ) asserted when at least 1 sample present
        false,   // We won't see the ERR bit because of 8 bit reads; disable.
        true     // Shift each sample to 8 bits when pushing to FIFO
    );
    adc_set_clkdiv(0); // Divisor of 0 -> full speed.

    multicore_launch_core1(core1_entry);

    // main loop
    while (true)
    {
        char c = getchar();
        
        switch (c)
        {
            // LED ON
            case 'n':
                cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
                printf("k");
                break;
                
            // LED OFF
            case 'f':
                cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
                printf("k");
                break;

            // ADC read, return it as text
            case 'a':
            {
                uint16_t result = adc_read();
                printf("%i\n", result);
                break;
            }

            // ADC read, return it as hex string
            case 'b':
            {
                uint16_t result = adc_read();
                printf("%03x", result);
                break;
            }

            // PWM on
            case 'p':
            {
                clkDiv = getIntFromUser();

                pwm_config config = pwm_get_default_config();
                pwm_config_set_clkdiv_int(&config, clkDiv); // ex: 125MHz / 125 = 1MHz counter
                currentWrap = 10;
                pwm_config_set_wrap(&config, currentWrap); // set period of PWM, ex: 10000 counts at 1MHz = 10000 counts / 1E6 counts per sec = 10ms (100Hz)
                pwm_init(slice_num, &config, true);
                printf("%05uk", clkDiv);
                break;
            }

            // set duty cycle of PWM
            case 'd':
            {
                duty = getIntFromUser();
                pwm_set_gpio_level(PWM_GPIO, duty); // set number of cycles it will be high before dropping
                printf("%05uk", duty);
                break;
            }

            // PWM off
            case 'm':
                pwm_set_enabled(slice_num, false);
                printf("k");
                // in case it is high when we stop the PWM, set the duty cycle to 0 first before we use this command
                break;

            // Change the period/wrap for PWM
            case 'w':
            {
                currentWrap = getIntFromUser();
                pwm_set_wrap(slice_num, currentWrap);
                printf("%05uk", currentWrap);
                break;
            }

            case 's':
                sinFreqHz = getIntFromUser();
                generateSinWave = true;
                printf("%05uk", sinFreqHz);
                break;

            case 'i':
                generateSinWave = false;
                printf("k");
                break;

            case 'c':
            {
                printf("k");
                doDMA();
                currentBlock = 0;
                break;
            }

            case 'l':
                fwrite(capture_buf + currentBlock*SAMPLE_BLOCK_SIZE, 1, SAMPLE_BLOCK_SIZE, stdout);
                fflush(stdout);
                currentBlock++;
                if (currentBlock >= NUM_SAMPLE_BLOCKS)
                {
                    currentBlock = 0;
                }
                break;

            case 't':
            {
                //printf("%u\n", clock_get_hz(clk_sys));
                /*uint32_t* reg = (uint32_t*)PADS_BANK0_BASE + 4;
                uint32_t value = *reg;
                printf("0x%x\n", value);*/
                printf("%05u\n", clkDiv);
                printf("%05u\n", currentWrap);
                printf("%05u\n", duty);
                printf("%05u\n", sinFreqHz);
                break;
            }

            case 'z':
                printf("k");
                fflush(stdout);
                break;

            default:
                break;
        }
    }
}
