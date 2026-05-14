/***********************************************
 * tone.c
 *  Implementation for the speaker/buzzer driver
 *  connected to the microcontroller.
 *  This driver uses Timer2 to generate square waves
 *  at the desired frequency for the specified duration.
 *
 *  Author:  Unknown
 *  Date:    Unknown
 *
 *  Refactored by: Erland Larsen
 *  Date:    2026-03-17
 *  Project: SPE4_API
 **********************************************/
#include "tone.h"
#include <avr/io.h>
// #include <util/delay.h>

#define BUZ_BIT PA7
#define BUZ_DDR DDRA
#define BUZ_PORT PORTA

void tone_play(uint16_t frequency, uint16_t duration)
{
    uint8_t prescaler_bits = 0;
    uint16_t prescaler_value = 0;
    uint16_t num_ticks = 0;

    // Set BUZ_BIT as output
    BUZ_DDR |= (1 << BUZ_BIT);

    // Calculate the half-period delay in microseconds
    uint16_t delay_us = 500000 / frequency;

    // Calculate the number of cycles needed for the specified duration
    uint16_t duration_loop = (uint16_t)((uint32_t)duration * 1000 / (2 * delay_us));

    // Initialize Timer in normal mode
    TCCR2A = 0;
    TCCR2B = 0;

    // Choose prescaler based on delay
    if (delay_us > 4000)
    {
        prescaler_bits = (1 << CS22) | (1 << CS21) | (1 << CS20); // 1024
        prescaler_value = 1024;
    }
    else if (delay_us > 2000)
    {
        prescaler_bits = (1 << CS22) | (1 << CS21); // 256
        prescaler_value = 256;
    }
    else if (delay_us > 1000)
    {
        prescaler_bits = (1 << CS22) | (1 << CS20); // 128
        prescaler_value = 128;
    }
    else if (delay_us > 500)
    {
        prescaler_bits = (1 << CS22); // 64
        prescaler_value = 64;
    }
    else if (delay_us > 125)
    {
        prescaler_bits = (1 << CS21) | (1 << CS20); // 32
        prescaler_value = 32;
    }
    else
    {
        prescaler_bits = (1 << CS21); // 8
        prescaler_value = 8;
    }

    // Set the prescaler
    TCCR2B = prescaler_bits;

    // Calculate the number of timer ticks needed for the specified delay
    num_ticks = (F_CPU / 1000000UL) * delay_us / prescaler_value;

    // Generate the tone
    for (uint16_t i = 0; i < duration_loop; i++)
    {
        // Set BUZ_BIT high
        BUZ_PORT |= (1 << BUZ_BIT);
        // Reset the timer counter
        TCNT2 = 0;

        // Wait until the timer counter reaches the required ticks
        while (TCNT2 < num_ticks)
        {
            // Busy-wait
        }

        // Set BUZ_BIT low
        BUZ_PORT &= ~(1 << BUZ_BIT);
        // Reset the timer counter
        TCNT2 = 0;

        // Wait until the timer counter reaches the required ticks
        while (TCNT2 < num_ticks)
        {
            // Busy-wait
        }
    }
    // Stop the timer
    TCCR2B = 0;

    // Set BUZ_BIT as input to turn off the buzzer
    BUZ_DDR &= ~(1 << BUZ_BIT);
}

void tone_play_startup()
{
    // --- STIRRING ---
    tone_play(500, 200); // low, groggy
    tone_play(600, 200);
    tone_play(700, 150);
    tone_play(800, 150);

    // --- WAKING UP ---
    tone_play(1000, 120);
    tone_play(1200, 120);
    tone_play(1500, 120);

    // --- ONLINE ---
    tone_play(2000, 100);
    tone_play(2500, 100);
    tone_play(3000, 150); // bright, awake
    tone_play(3000, 80);  // double tap, confirmed
}

void tone_play_air_raid()
{
    // --- URGENT TRIPLE BEEP ---
    tone_play(3000, 80);
    tone_play(3000, 80);
    tone_play(3000, 80);

    // --- CRITICAL PULSE (faster, more insistent) ---
    tone_play(3200, 60);
    tone_play(3200, 60);
    tone_play(3200, 60);
    tone_play(3200, 60);

    // --- ALARM SCREAM (continuous high tone) ---
    tone_play(3500, 600);

    // --- DROP WARNING (something is very wrong) ---
    tone_play(3500, 80);
    tone_play(2800, 80);
    tone_play(3500, 80);
    tone_play(2800, 80);

    // --- FINAL SUSTAINED CRY ---
    tone_play(3500, 800);
}

void tone_play_smoke_detector()
{
    tone_play(3000, 80);
    tone_play(3000, 80);
    tone_play(3000, 80);
}

void tone_play_wifi_connected()
{
    tone_play(1500, 80);  // tap
    tone_play(2000, 80);  // step
    tone_play(2500, 80);  // step
    tone_play(3000, 200); // connected ✓
}