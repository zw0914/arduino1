#include <Arduino.h>

/* Defines of GPIO_Interrupt.ino */
const byte ledPin = 2;       // Builtin-LED pin
const byte interruptPin = 0; // BOOT/IO0 button pin
volatile byte state = LOW;

/* Defines of SendAndReceive.ino */
// select only NEC and the universal decoder for pulse width or pulse distance protocols
#define DECODE_NEC          // Includes Apple and Onkyo
#define DECODE_DISTANCE     // in case NEC is not received correctly

#include "PinDefinitionsAndMore.h"
//#define EXCLUDE_UNIVERSAL_PROTOCOLS // Saves up to 1000 bytes program space.
//#define EXCLUDE_EXOTIC_PROTOCOLS
//#define SEND_PWM_BY_TIMER
//#define USE_NO_SEND_PWM
//#define NO_LED_FEEDBACK_CODE // saves 500 bytes program space
//#define DEBUG // Activate this for lots of lovely debug output from the decoders.
#include <IRremote.hpp>
#define INFO
#define DELAY_AFTER_SEND 2000
#define DELAY_AFTER_LOOP 5000
uint16_t sAddress = 0x0102;
uint8_t sCommand = 0x34;
uint8_t sRepeats = 1;
 
void setup() {
  Serial.begin(9600);
  pinMode(ledPin, OUTPUT);
  pinMode(interruptPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPin), IRsender, CHANGE);
}

void loop() {
  Serial.println("IR receiving...");
if (IrReceiver.decode()) {
        Serial.print(F("Decoded protocol: "));
        Serial.print(getProtocolString(IrReceiver.decodedIRData.protocol));
        Serial.print(F("Decoded raw data: "));
        Serial.print(IrReceiver.decodedIRData.decodedRawData, HEX);
        Serial.print(F(", decoded address: "));
        Serial.print(IrReceiver.decodedIRData.address, HEX);
        Serial.print(F(", decoded command: "));
        Serial.println(IrReceiver.decodedIRData.command, HEX);
        IrReceiver.resume();
    }
  /* Receiver code of SendAndReceive.ino */
}

void IRsender() {
  Serial.println("IR sending...");
  Serial.print(F("Sending: 0x"));
    Serial.print(sAddress, HEX);
    Serial.print(sCommand, HEX);
    Serial.println(sRepeats, HEX);

    // clip repeats at 4
    if (sRepeats > 4) {
        sRepeats = 4;
    }
    // Results for the first loop to: Protocol=NEC Address=0x102 Command=0x34 Raw-Data=0xCB340102 (32 bits)
    IrSender.sendNEC(sAddress, sCommand, sRepeats);
  
  /* Sender of SendAndReceiv.ino */
}
