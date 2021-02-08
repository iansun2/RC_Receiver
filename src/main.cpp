#include <Arduino.h>
#include <nRF24L01.h>
#include <printf.h>
#include <RF24.h>
#include <RF24_config.h>
#include <timer2_pulse.h>
//https://nrf24.github.io/RF24/classRF24.html

//Version 1.0 2021/2/7

//pin setting//=============================================
//NRF24L01
#define CE_pin 7
#define CSN_pin 6
//BTS7960
#define R_EN_pin 2
#define L_EN_pin 3
#define R_PWM_pin 9
#define L_PWM_pin 10
//Servo
#define servo_pin 8
//axis setting
#define x_axis_center 2000
#define y_axis_center 0
//protection setting
#define controller_timeout 2000 //(ms)

//declear//=====================================================
RF24 radio(CE_pin, CSN_pin);
const byte nrf24l01_address[6] = "shink";
typedef struct{
  int16_t x_axis;
  int16_t y_axis;
}rx_data;
rx_data main_data;
uint32_t last_receive;

//init nrf24l01 function//====================================================
void init_nrf24l01(){
  Serial.println("init nRF24L01");
  radio.begin();
  printf_begin(); 
  radio.openReadingPipe(0,nrf24l01_address);
  radio.setPALevel(RF24_PA_MAX,1);
  radio.setDataRate(RF24_250KBPS);
  radio.setChannel(120);
  radio.startListening();
  radio.printDetails();
}

//init motor function//====================================================
void init_motor(){
  Serial.println("init servo");
  timer2_pulse_init();
  timer2_pulse_attach(servo_pin);
  timer2_pulse_write(servo_pin, x_axis_center);
  Serial.println("init bts7960");
  TCCR1B |= (1 << CS11);
  main_data.x_axis = x_axis_center;
  main_data.y_axis = y_axis_center;
  pinMode(R_EN_pin,OUTPUT);
  pinMode(L_EN_pin,OUTPUT);
  pinMode(R_PWM_pin,OUTPUT);
  pinMode(L_PWM_pin,OUTPUT);
  analogWrite(R_PWM_pin,0);
  analogWrite(L_PWM_pin,0);
  digitalWrite(R_EN_pin,0);
  digitalWrite(L_EN_pin,0);
}

//shutdown motor function//=======================================================
void shutdown_motor(){
  Serial.println("controller timeout");
  analogWrite(R_PWM_pin,0);
  analogWrite(L_PWM_pin,0);
  digitalWrite(R_EN_pin,0);
  digitalWrite(L_EN_pin,0);
  timer2_pulse_write(servo_pin, x_axis_center);
}

//setup//=================================================================
void setup() {
  Serial.begin(115200);
  init_nrf24l01();
  init_motor();
  while(!radio.available()){
    Serial.println("no connect");
  }
  last_receive = millis();
  digitalWrite(R_EN_pin,1);
  digitalWrite(L_EN_pin,1);
}

//main loop//===================================================================
void loop() {
  //controller connected
  if((millis() - last_receive) <= controller_timeout){
    if (radio.available()) {
      last_receive = millis();
      radio.read(&main_data, sizeof(main_data));
      //y
      if(main_data.y_axis >= 0){
        analogWrite(0,L_PWM_pin);
        analogWrite(main_data.y_axis,R_PWM_pin);
      }else if(main_data.y_axis <= 0){
        analogWrite(0,R_PWM_pin);
        analogWrite(abs(main_data.y_axis),L_PWM_pin);
      }
      //x
      timer2_pulse_write(servo_pin,main_data.x_axis);
    }

  //controller reconnect
  }else if (radio.available()) {
    Serial.println("controller reconnect");
    last_receive = millis();
    digitalWrite(R_EN_pin,1);
    digitalWrite(L_EN_pin,1);

  //controller disconnect
  }else{
    shutdown_motor();
  }
}