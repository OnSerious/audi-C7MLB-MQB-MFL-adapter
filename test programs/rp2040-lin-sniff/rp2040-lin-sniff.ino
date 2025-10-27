// https://github.com/zapta/linbus/tree/master/analyzer/arduino
#include "lin_frame.h"

#define LED_PIN 18
#define SWC_ID 0x0E                                     // set to 0xFF to ignore

// 0xD - backlight
// 0xE - buttons
// 0x3A - heater temperature

byte b, i, n;
LinFrame frame;

void setup() {
  Serial.begin(19200);
  while(!Serial);
  Serial.println("LIN sniffer started");
  Serial1.begin(19200);
  pinMode(LED_PIN, OUTPUT);
  frame = LinFrame();
}

void loop() {
  if (Serial1.available()) {
    b = Serial1.read();
    n = frame.num_bytes();

    // if (b == 0x55) {         // dump everything
    //   Serial.println();
    // }
    // Serial.print(" ");
    // Serial.print(b, HEX);
    // return;

    if (b == 0x55 && n > 0) {                         // Single byte ID only frames are sent by the master to request a response from slaves
      frame.pop_byte();
      handle_frame();
      frame.reset();
    } else if (n == LinFrame::kMaxBytes) {
      frame.reset();
    } else {
      frame.append_byte(b);
    }
  }
}

void handle_frame() {

  if (SWC_ID != 0xFF) {
    if ((frame.get_byte(0) & 0x3F) != SWC_ID)
      return;
  }

  // if (frame.num_bytes() > 1) {
  //   if (frame.get_byte(frame.num_bytes() + 1) != calculate_enh_checksum()) {      // validate cks
  //     return;
  //   }
  // }

  print_frame();
}

int calculate_enh_checksum() {
  int checksum = frame.get_byte(0);
  for (i = 1; i < frame.num_bytes(); i++) {
    checksum += frame.get_byte(i);
  }
  return 0xFF - (checksum % 0xFF);
}

void print_frame() {
  // uint8_t frame_[LinFrame::kMaxBytes];
  // for (i = 0; i < frame.num_bytes() + 1; i++) {
  //   frame_[i] = frame.get_byte(i);
  // }

  // if (frame_[2] == 0 ) {
  //   return;
  // }

  digitalWrite(LED_PIN, HIGH);
  if (frame.num_bytes() > 1) {
    for (i = 0; i < frame.num_bytes() + 1; i++) {           // +1 to show cks
      Serial.print(frame.get_byte(i), HEX);
      Serial.print(" ");
    }
    Serial.println();
  } else {
    Serial.print("request: ");
    Serial.println(frame.get_byte(0), HEX);
  }
  digitalWrite(LED_PIN, LOW);
}