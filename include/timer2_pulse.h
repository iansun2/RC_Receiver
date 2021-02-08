#include <Arduino.h>

int8_t timer2_pulse_pin;

ISR(TIMER2_COMPA_vect){
  if(digitalRead(timer2_pulse_pin)){
    digitalWrite(timer2_pulse_pin, 0);
    TIMSK2 &= (B11111111 ^ (1 << OCIE2A));
  }else{
    digitalWrite(timer2_pulse_pin, 1);
  }
}

void timer2_pulse_init(){
    noInterrupts();
    TCCR2A = 0;
    TCCR2B = 0;
    TCNT2 = 0;
    TCCR2A |= (1 << WGM21);
    TCCR2B|= (1 << CS21) | (1 << CS22);
    OCR2A = 255;
    TCNT2 = 0;
    interrupts();
}
void timer2_pulse_attach(int8_t pin){
    pinMode(pin,OUTPUT);
}

void timer2_pulse_write(int8_t pin, int16_t pulse_us){
  TCNT2 = 0;
  timer2_pulse_pin = pin;
  OCR2A = map(pulse_us, 0, 4080, 0, 255);
  TIMSK2 |= (1 << OCIE2A);
}