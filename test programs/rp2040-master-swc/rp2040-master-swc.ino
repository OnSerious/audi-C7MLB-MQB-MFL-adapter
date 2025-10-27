// https://github.com/zapta/linbus/tree/master/analyzer/arduino
#include "lin_frame.h"
#define LED_PIN 18
#define SWC_ID 0x0E                                     // set to 0xFF to ignore

LinFrame frame;

unsigned long request_buttons_timer, request_heating_status_timer,
              backlight_status_timer;
uint8_t backlight_status[] = {0, 0x81, 0, 0, 0};
uint8_t backlight_value = 0x64;

void setup() {
  Serial.begin(38400);
  while(!Serial);
  Serial.println("LIN master SWC");
  pinMode(LED_PIN, OUTPUT);
  frame = LinFrame();
  pinMode(0, OUTPUT);                             // TX pin

  request_buttons_timer
  =request_heating_status_timer
  =backlight_status_timer
  =millis();
}

void loop() {
  if (Serial1.available()) {
    byte b = Serial1.read();
    byte n = frame.num_bytes();

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

  if ((millis() - request_buttons_timer) >= 30) {
    
    sendLinBreak();       // Send LIN break
    Serial1.write((uint8_t)0x55);   // Send sync byte (0x55)
    // delayMicroseconds(10);
    Serial1.write((uint8_t)0x8E);   // Send protected ID
    // Serial1.flush();

    request_buttons_timer = millis();
  }

  if ((millis() - request_heating_status_timer) >= 250) {
    sendLinBreak();
    Serial1.write((uint8_t)0x55);
    // delayMicroseconds(10);
    Serial1.write((uint8_t)0xBA);
    // Serial1.flush();

    request_heating_status_timer = millis();
  }

  if ((millis() - backlight_status_timer) >= 100) {
    sendLinBreak();
    Serial1.write((uint8_t)0x55);
    // delayMicroseconds(10);
    Serial1.write((uint8_t)0xD);
    Serial1.flush();
    // delayMicroseconds(10);

    backlight_status[0] = backlight_value;
    backlight_status[4] = calculate_enh_checksum(backlight_status, 0xD, 4);
    for (uint8_t i = 0; i < 5; i++) {
      Serial1.write(backlight_status[i]);
      // Serial.print(backlight_status[i], HEX);
      // Serial.print(" ");
    }
    backlight_status_timer = millis();
  }
}

void sendLinBreak() {
  // Send break: Set TX low for 13+ bit times
  // At 19200 baud, 1 bit = 52.09 µs, 13 bits = 677 µs
  
  Serial1.end();  // Release UART control of TX pin
  digitalWrite(0, LOW);  // Drive low for break
  delayMicroseconds(700);
  digitalWrite(0, HIGH);  // Release
  Serial1.begin(19200);  // Restart UART
  // delayMicroseconds(10);
}

void handle_frame() {
  if (SWC_ID != 0xFF) {
    if ((frame.get_byte(0) & 0x3F) != SWC_ID)
      return;
  }

  if (frame.get_byte(frame.num_bytes()) != check_enh_checksum()) {      // validate cks
    return;
  }

  print_frame();
}

uint8_t calculate_enh_checksum(uint8_t *data, uint8_t id, uint8_t size) {
  int checksum = id;
  for (uint8_t i = 0; i < size; i++) {
    checksum += data[i];
  }
  return 0xFF - (checksum % 0xFF);
}

int check_enh_checksum() {
  int checksum = frame.get_byte(0);
  for (uint8_t i = 1; i < frame.num_bytes(); i++) {
    checksum += frame.get_byte(i);
  }
  return 0xFF - (checksum % 0xFF);
}

void print_frame() {
  digitalWrite(LED_PIN, HIGH);
  if (frame.num_bytes() > 1) {
    for (uint8_t i = 0; i < frame.num_bytes() + 1; i++) {           // +1 to show cks
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
