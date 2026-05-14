/***********************************************
 * tone.h
 *  Interface for the speaker/buzzer driver
 *  connected to the microcontroller.
 *  This driver uses Timer2 to generate square waves
 *  at the desired frequency for the specified duration.
 *
 *  Author:  Unknown
 *  Date:    Unknown
 *
 *  Refactored by: Erland Larsen
 *  Date:    2026-03-15
 *  Project: SPE4_API
 **********************************************/
#pragma once

#include <stdint.h>
void tone_play(uint16_t frequency, uint16_t duration);
void tone_play_startup();        // by BetonoMISHAlka
void tone_play_air_raid();       // by Ukraine Defense Forces
void tone_play_smoke_detector(); // by Bianca
void tone_play_wifi_connected(); // by Mace