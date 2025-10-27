// https://github.com/zapta/linbus/tree/master/analyzer/arduino
#include "lin_frame.h"

#define LED_PIN 18
#define SWC_ID 0x0E

// 0xD - backlight
// 0xE - buttons
// 0x3A - heater temperature

LinFrame frame;

uint8_t buttons_alive_counter = 0;
uint8_t active_button_id = 0;
uint8_t active_button_counter = 0;
uint8_t active_button_detect = 0x80;
uint8_t button_direction = 0;
uint8_t paddles = 0x20;
uint8_t buttons_status[] = {0, 0, 0, 0, 1, 0, 0, 0, 0};
uint8_t temperature = 0x52;                                                       // 32 C
uint8_t heater_button = 0xFE;                                                     // released
uint8_t heater_button_active_counter = 0;
uint8_t heater_status[] = {0, 0, 0};

uint8_t swc_status = 0;                                                           // Horn. Also reports faults with the steering wheel. Paddles missing, left buttons disconnected etc.

void setup() {
  Serial.begin(19200);
  while(!Serial);
  Serial.println("LIN slave SWC");
  Serial1.begin(19200);
  pinMode(LED_PIN, OUTPUT);
  frame = LinFrame();
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

    if (b == 0x55 && n > 0) {                                     // Single byte ID only frames are sent by the master to request a response from slaves
      frame.pop_byte();
      handle_frame();
      frame.reset();
    } else if (n == LinFrame::kMaxBytes) {
      frame.reset();
    } else {
      if (n == 0) {                                               // we're at the ID byte since 55 was popped
        if (b == 0x8E) {                                          // Button status
          buttons_status[0] = active_button_detect | buttons_alive_counter;
          buttons_alive_counter == 0xF ? buttons_alive_counter = 0 : buttons_alive_counter++;

          buttons_status[1] = active_button_id;
          buttons_status[3] = button_direction;
          buttons_status[6] = paddles;
          buttons_status[7] = swc_status;
          buttons_status[8] = calculate_enh_checksum(buttons_status, 0x8E, 8);
          for (uint8_t i = 0; i < 9; i++) {
            Serial1.write(buttons_status[i]);
            // Serial.print(buttons_status[i], HEX);
            // Serial.print(" ");
          }
          // Serial.println();
          Serial1.flush();
          if (active_button_counter > 0) {
            active_button_counter--;
          } else {
            active_button_id = button_direction = 0;
            active_button_detect = 0x80;
            bitWrite(paddles, 2, 0);
            bitWrite(paddles, 1, 0);
            bitWrite(swc_status, 0, 0);
          }
        }
        else if (b == 0xBA) {                                     // steering heater status
          heater_status[0] = temperature;
          heater_status[1] = heater_button;
          heater_status[2] = calculate_enh_checksum(heater_status, 0xBA, 2);
          for (uint8_t i = 0; i < 3; i++) {
            Serial1.write(heater_status[i]);
            // Serial.print(heater_status[i], HEX);
            // Serial.print(" ");
          }
          // Serial.println();
          Serial1.flush();
          if (heater_button_active_counter > 0) {
            heater_button_active_counter--;
          } else {
            bitWrite(heater_button, 0, 0);
          }
        }
      }
      frame.append_byte(b);
    }
  }

  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    handleCommand(cmd);
  }
}

void handle_frame() {
  // if ((frame.get_byte(0) & 0x3F) != SWC_ID)
  //   return;

  // print_frame();
}

uint8_t calculate_enh_checksum(uint8_t *frame, uint8_t id, uint8_t size) {
  int checksum = id;
  for (uint8_t i = 0; i < size; i++) {
    checksum += frame[i];
  }
  return 0xFF - (checksum % 0xFF);
}

void print_frame() {
  digitalWrite(LED_PIN, HIGH);
  for (uint8_t i = 0; i < frame.num_bytes() + 1; i++) {           // +1 to show cks
    Serial.print(frame.get_byte(i), HEX);
    Serial.print(" ");
  }
  Serial.println();
  digitalWrite(LED_PIN, LOW);
}

void handleCommand(String cmd) {
  if (cmd == "tlow") {
    Serial.println("Setting sw heat low (10C)");    
    temperature = 0x3C;
  }
  else if (cmd == "thigh") {
    Serial.println("Setting sw heat high (45C)");
    temperature = 0x5F;
  }
  else if (cmd == "heatbtn") {
    Serial.println("Sending sw heat button");
    temperature = 0x5F;                                             // force temp high to avoid runaway
    bitWrite(heater_button, 0, 1);
    heater_button_active_counter = 1;
  }

  else if (cmd == "paddle+") {
    Serial.println("Sending paddle+");
    active_button_counter = 1;
    bitWrite(paddles, 2, 1);
    bitWrite(paddles, 1, 0);
  }
  else if (cmd == "paddle-") {
    Serial.println("Sending paddle-");
    active_button_counter = 1;
    bitWrite(paddles, 2, 0);
    bitWrite(paddles, 1, 1);
  }

  else if (cmd == "scroll-") {
    Serial.println("Sending scroll-");
    active_button_id = 6;
    active_button_counter = 1;
    button_direction = 0xF;
    active_button_detect = 0x90;
  }
  else if (cmd == "scroll+") {
    Serial.println("Sending scroll+");
    active_button_id = 6;
    active_button_counter = 1;
    button_direction = 1;
    active_button_detect = 0x90;
  }
  else if (cmd == "ok") {
    Serial.println("Sending ok");
    active_button_id = 7;
    active_button_counter = 1;
    button_direction = 1;
    active_button_detect = 0x90;
  }
  else if (cmd == "left") {
    Serial.println("Sending left");
    active_button_id = 3;
    active_button_counter = 1;
    button_direction = 1;
    active_button_detect = 0x90;
  }
  else if (cmd == "right") {
    Serial.println("Sending right");
    active_button_id = 2;
    active_button_counter = 1;
    button_direction = 1;
    active_button_detect = 0x90;
  }
  else if (cmd == "menu") {
    Serial.println("Sending menu");
    active_button_id = 1;
    active_button_counter = 1;
    button_direction = 1;
    active_button_detect = 0x90;
  }
  else if (cmd == "nav") {
    Serial.println("Sending nav");
    active_button_id = 0x1B;
    active_button_counter = 1;
    button_direction = 1;
    active_button_detect = 0x90;
  }
  else if (cmd == "voice") {
    Serial.println("Sending voice");
    active_button_id = 0x19;
    active_button_counter = 1;
    button_direction = 1;
    active_button_detect = 0x90;
  }
  else if (cmd == "mute") {
    Serial.println("Sending mute");
    active_button_id = 0x20;
    active_button_counter = 1;
    button_direction = 1;
    active_button_detect = 0x90;
  }
  else if (cmd == "vol-") {
    Serial.println("Sending vol-");
    active_button_id = 0x12;
    active_button_counter = 1;
    button_direction = 0xF;
    active_button_detect = 0x90;
  }
  else if (cmd == "vol+") {
    Serial.println("Sending vol+");
    active_button_id = 0x12;
    active_button_counter = 1;
    button_direction = 1;
    active_button_detect = 0x90;
  }

  else if (cmd == "prev") {
    Serial.println("Sending prev");
    active_button_id = 0x16;
    active_button_counter = 1;
    button_direction = 1;
    active_button_detect = 0x90;
  }
  else if (cmd == "next") {
    Serial.println("Sending next");
    active_button_id = 0x15;
    active_button_counter = 1;
    button_direction = 1;
    active_button_detect = 0x90;
  }
  else if (cmd == "joker") {
    Serial.println("Sending joker");
    active_button_id = 0x21;
    active_button_counter = 1;
    button_direction = 1;
    active_button_detect = 0x90;
  }
  else if (cmd == "horn") {
    Serial.println("Sending horn");
    active_button_counter = 1;
    bitWrite(swc_status, 0, 1);
  }

  else if (cmd == "ds") {
    Serial.println("Sending Drive Select");
    active_button_counter = 1;
    active_button_id = 0x70;
    active_button_counter = 1;
    button_direction = 1;
    active_button_detect = 0x90;
  }
}
