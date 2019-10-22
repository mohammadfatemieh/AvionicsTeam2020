#ifndef XBEE_PROGRAMMER
#define XBEE_PROGRAMMER

#include <stdint.h>

#define BD_9600   3
#define BD_19200  4
#define BD_115200 6

#define RESET_PIN 39 

enum Baudrate {
    bd_2400 = 1,
    bd_4800,
    bd_9600,
    bd_19200,
    bd_38400,
    bd_57600,
    bd_115200,
    bd_230400
};

enum Power_level { dBm_7, dBm_15, dBm_18, dBm_21, dBm_24 };



uint8_t init(long baud);
uint8_t check_response(const char c1[], const char c2[]);
uint8_t ok(uint8_t time_out);
void send_command(const char command[]);
uint8_t set_baudrate(Baudrate baudrate);
uint8_t apply_changes();
uint8_t get_info();
uint8_t set_power_level(Power_level power);

uint8_t manual_mode();

void reset();



#endif