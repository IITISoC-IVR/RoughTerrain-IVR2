/*
  This program identifies the tilt position
  of MPU6050 and assigns a value based on
  the position.

  This value is sent to the receiver module
  through NRF24L01

  This program is made by Shreyas for
  Electronics Champ YouTube Channel.
  Please subscribe to this channel
  Thank You
*/

//Include the libraries
#include <Wire.h>
#include <TinyMPU6050.h>
#include <SPI.h>
#include <NRFLite.h>

MPU6050 mpu (Wire);

int message;

const static uint8_t RADIO_ID = 1;             // Our radio's id.
const static uint8_t DESTINATION_RADIO_ID = 0; // Id of the radio we will transmit to.
const static uint8_t PIN_RADIO_CE = 7;
const static uint8_t PIN_RADIO_CSN = 8;

struct RadioPacket { // Any packet up to 32 bytes can be sent.

  uint8_t FromRadioId;
  uint32_t Data;
  uint32_t FailedTxCount;

};

//Create NRF24 object
NRFLite _radio;
RadioPacket _radioData;

void setup() {

  // Initialization
  mpu.Initialize();
  // Calibration (wait for about 20s to calibrate)
  mpu.Calibrate();

  //start up
  Serial.begin(9600);
  Serial.println("Done Clabration");

  if (!_radio.init(RADIO_ID, PIN_RADIO_CE, PIN_RADIO_CSN)) {

    Serial.println("Cannot communicate with radio");
    while (1); // Wait here forever.
  }

  _radioData.FromRadioId = RADIO_ID;

}

void loop() {

  mpu.Execute();

  while (mpu.GetRawAccX() <= -8000) {

    //send msg to move front
    message = 1;
    _radioData.Data = message;
    sendData();
    Serial.println("front");
    mpu.Execute();

  }

  while (mpu.GetRawAccX() >= 8000) {

    //send msg to move back
    message = 2;
    sendData();
    _radioData.Data = message;
    Serial.println("back");
    mpu.Execute();

  }

  while (mpu.GetRawAccY() <= -8000) {

    //send msg to move left
    message = 3;
    sendData();
    _radioData.Data = message;
    Serial.println("left");
    mpu.Execute();

  }

  while (mpu.GetRawAccY() >= 8000) {

    //send msg to move right
    message = 4;
    sendData();
    _radioData.Data = message;
    Serial.println("right");
    mpu.Execute();

  }

  while (mpu.GetRawAccX() < 8000 and mpu.GetRawAccX() > -8000 and mpu.GetRawAccY() < 8000 and mpu.GetRawAccY() > -8000) {

    //send msg to stop
    message = 0;
    sendData();
    _radioData.Data = message;
    Serial.println("none");
    mpu.Execute();

  }

}

void sendData() {

  if (_radio.send(DESTINATION_RADIO_ID, &_radioData, sizeof(_radioData))) { // Note how '&' must be placed in front of the variable name.

  }

  else {

    Serial.println("Failed");
    _radioData.FailedTxCount++;

  }

  delay(500);
  mpu.Execute();

}

