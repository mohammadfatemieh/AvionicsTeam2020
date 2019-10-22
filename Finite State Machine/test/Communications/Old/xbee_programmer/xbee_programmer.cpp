#include <arduino.h>
#include "xbee_programmer.h"
#include <cstring>
#include <stdio.h>

//Return 0 for success and 1 for failure.
uint8_t init(long baud) {
    Serial.begin(9600);
    Serial5.begin(baud);
    while(!Serial);
    while(!Serial5);
    Serial.println("Initializing programmer");

    reset();

    delay(1500);
    Serial.println("+++");
    Serial5.print("+++");

    if (ok(1)) {
        delay(1000);
        return 0;
    } 

    Serial.println("Failed to connect\n");
    return 1;
}

void reset() {
    pinMode(RESET_PIN, OUTPUT);
    digitalWrite(RESET_PIN, LOW);
    delay(1);
    pinMode(RESET_PIN, INPUT);
}

uint8_t ok(uint8_t time_out) {
    unsigned long timer = millis();
    char data[4] = {};
    while (Serial5.available() < 3) {
        if (time_out && millis()-timer > 1200) {
                Serial.println("time out");
                return 0;
        } 
    }
        Serial5.readBytes(data, 3);
        Serial.println(data);
        if(check_response(data, "OK\r")) {
            Serial.println("Response doesn't match");
            return 0;
        }
        Serial.println();
        return 1;

}

uint8_t check_response(const char c1[], const char c2[]) {
    if (strcmp(c1, c2)) {
        return 1;
    }
    return 0;
}

uint8_t apply_changes() {
    send_command("WR");
    if(ok(0)) {
        send_command("CN");
        ok(1);
        return 0;
    }
    return 1;
}

void send_command(const char command[]) {
    Serial.print("AT ");
    Serial.println(command);
    Serial5.print("AT ");
    Serial5.print(command);
    Serial5.print('\r');
}

uint8_t set_baudrate(Baudrate baudrate) {
    char cmd[5];
    sprintf(cmd, "BD %d",baudrate);
    send_command(cmd);
    return !ok(1);
}

uint8_t set_power_level(Power_level power) {
    char cmd[5];
    sprintf(cmd, "PL %d", power);
    send_command(cmd);
    return !ok(1);
}


uint8_t get_info() {
    long baudrates[] = { 2400, 4800, 9600, 19200, 38400, 57600, 115200, 230400};
    char *power_level[] = {"+7dBm", "+15dBm", "+18dBm", "+21dBm", "+24dBm."};
    char node_id[32];
    uint8_t baud;
    uint8_t power;


    //get name
    send_command("NI");
    while (Serial5.available() < 2);
    char c;
    int i = 0;
    while (c != '\r') {
        if (Serial5.available()) {
            c = Serial5.read();
            node_id[i] = c;
            i++;
        }
    }

    //get baudrate
    send_command("BD");
    while(Serial5.available() < 2);
    baud = Serial5.read()-49;
    Serial5.read();


    //get tx power level
    send_command("PL");
    while(Serial5.available() < 2);
    power = Serial5.read() - 48;
    Serial5.read();


    Serial.print("\n********************************\n");
    Serial.print("Name: ")
    Serial.println(node_id);
    Serial.print("Baudrate: ");
    Serial.println(baudrates[baud]);
    Serial.print("Power level: ");
    Serial.println(power_level[power]);
    Serial.print("********************************\n");


    return 0;
}

//Not working yet
uint8_t manual_mode() {
    char data;
    while(1) {

        if(Serial.available()) {
            data = Serial.read();
            Serial.print(data);
            Serial5.print((char)data);
        }
        if(Serial5.available()) {
            Serial.print(Serial5.read());
        }
    }
}