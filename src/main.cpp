#include <Arduino.h>
#include <nRF24L01.h>
#include <printf.h>
#include <RF24.h>
#include <RF24_config.h>

#define CE_pin 6
#define CSN_pin 7
#define R_EN 2
#define L_EN 3
#define R_PWM 9
#define L_PWM 10



void setup() {
  TCCR1B |= (1 << CS11);
}

void loop() {
  // put your main code here, to run repeatedly:
}