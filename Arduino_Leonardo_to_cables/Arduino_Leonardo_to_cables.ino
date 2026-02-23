// Code for Arduino Leonardo
// Connection with Cables.gl via MIDI protocol
// Supports servos, DMX shield, and Tinkerkit shield
// Raphaël Isdant - BaseDigitale des Beaux-Arts de Paris
// http://www.babd.fr/
// Version 1.2

/*
  MIDI Channel Usage:
  - Channel 0: Sends the lower 7 bits of analog sensor values (0-127).
  - Channel 1: Sends the upper 3 bits of analog sensor values (extending resolution to 10 bits).
  - Channel 4: Controls the output mode for each control number:
    - Value 0: PWM mode (direct analog output).
    - Value 1: Servo mode (controls servo position).
    - Value 2: DMX mode (sends data via DMX protocol).
*/

#include <Servo.h>
#include "src/MIDIUSB.h"
#include "src/DMXSerial.h"

const int numAnalogPins = 6;  // Number of analog pins used to read sensor values
const int numControlNumbers = 128;  // Maximum number of MIDI control numbers used (up to 128)
const unsigned long midiDelayInterval = 10;  // Minimum delay in milliseconds between MIDI messages to reduce load
const int analogThreshold = 2;  // Variation threshold to trigger sending new MIDI values (to avoid small fluctuations)
const float emaAlpha = 0.2;  // Smoothing coefficient for exponential moving average (between 0 and 1)


byte lower7Bits[numControlNumbers] = {0};
byte higher7Bits[numControlNumbers] = {0};

int analogValues[numAnalogPins];
int previousAnalogValues[numAnalogPins];
unsigned long lastMidiSendTime = 0;

Servo servos[numControlNumbers];
int outMode[numControlNumbers] = {0}; // 0 = PWM, 1 = Servo, 2 = DMX

void setup() {
  for (int i = 0; i < numAnalogPins; i++) {
    pinMode(A0 + i, INPUT);
    previousAnalogValues[i] = analogRead(A0 + i);
  }
  DMXSerial.init(DMXController);
}

inline void controlChange(byte channel, byte control, byte value) {
  midiEventPacket_t event = {0x0B, 0xB0 | channel, control, value};
  MidiUSB.sendMIDI(event);
}

void applyPWM(int controlNumber, int combinedValue) {
  if (outMode[controlNumber] == 1) {  // Detach servo if switching from Servo mode
    servos[controlNumber].detach();
  }
  outMode[controlNumber] = 0;
  analogWrite(controlNumber, combinedValue & 0xFF);
}

void applyServo(int controlNumber, int combinedValue) {
  if (outMode[controlNumber] != 1) {  // Attach servo if not already in Servo mode
    servos[controlNumber].attach(controlNumber);
  }
  outMode[controlNumber] = 1;
  servos[controlNumber].write(map(combinedValue, 0, 255, 0, 180));
}

void applyDMX(int controlNumber, int combinedValue) {
  if (outMode[controlNumber] == 1) {  // Detach servo if switching from Servo mode
    servos[controlNumber].detach();
  }
  outMode[controlNumber] = 2;
  DMXSerial.write(controlNumber, combinedValue & 0xFF);
}

void loop() {
  unsigned long currentTime = millis();

  if (currentTime - lastMidiSendTime >= midiDelayInterval) {
    for (int i = 0; i < numAnalogPins; i++) {
      int rawAnalogValue = analogRead(A0 + i);
      analogValues[i] = emaAlpha * rawAnalogValue + (1 - emaAlpha) * previousAnalogValues[i];

      if (abs(analogValues[i] - previousAnalogValues[i]) > analogThreshold) {
        previousAnalogValues[i] = analogValues[i];

        byte lower7 = analogValues[i] & 0x7F;
        byte upper3 = (analogValues[i] >> 7) & 0x07;

        controlChange(0, i, lower7);
        controlChange(1, i, upper3);
      }
    }
    MidiUSB.flush();
    lastMidiSendTime = currentTime;
  }

  midiEventPacket_t rx = MidiUSB.read();

  if (rx.header == 0x0B) {
    byte channel = rx.byte1 & 0x0F;
    byte controlNumber = rx.byte2;
    byte controlValue = rx.byte3;

    if (channel == 0) {
      lower7Bits[controlNumber] = controlValue & 0x7F;
    } else if (channel == 1) {
      higher7Bits[controlNumber] = controlValue & 0x7F;
    }

    int combinedValue = (higher7Bits[controlNumber] << 7) | lower7Bits[controlNumber];

    if (channel == 4) {  // Handle mode changes on channel 4
      if (controlValue == 2) {  // DMX mode
        applyDMX(controlNumber, combinedValue);
      } else if (controlValue == 1) {  // Servo mode
        applyServo(controlNumber, combinedValue);
      } else {  // Default to PWM mode
        applyPWM(controlNumber, combinedValue);
      }
    }
  }
}
