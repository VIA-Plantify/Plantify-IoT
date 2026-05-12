/**
 * data_server.h  –  Plantify-IoT TCP data export server
 *
 * Include this in main.c, then call data_server_run() when button 3 is held.
 */

#pragma once

/**
 * Starts the WiFi AP, opens TCP server on port 9000, streams sensor JSON.
 * This function never returns – it loops forever sending sensor data.
 * Call it from main() right after button_init():
 *
 *   if (button_get(3))
 *       data_server_run();
 */
void data_server_run(void);
