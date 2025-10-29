// https://github.com/zapta/linbus/tree/master/analyzer/arduino
#include "lin_frame.h"

// [LIN message IDs]
// Steering wheel controls (Slave):
//    0xE  - Button status and errors.
//    0x3A - Steering wheel heater temperature and button status.
// J527 / Car side (Master):
//    0xD  - Backlight status
//    0x3B - ?
//    0x3D - ?

#define SW_TX_PIN PB10
#define DEBUG_MODE 1                                                                                                                // Enable USART1 debug interface. RX of TTL adapter needs to be connected to pin PA9 (TX).
#if DEBUG_MODE
#define DEBUG_BUTTON_PRESS 1                                                                                                        // Print which button is pressed on the SWC.
#define SERIAL_WAIT_TIMEOUT 10000
#define DEBUG_BAUD 230400
#endif
#define CORRECT_SW_TEMP 1                                                                                                           // For aftermarket steering wheels: pretend the temperature is lower than it is. J527 channel 10 adaptation is max 45C.
#define HORN_AS_DS 1                                                                                                                // Treat the horn message as Drive Select. Allows hardwiring R8 style buttons with a simple ground switch.
#if CORRECT_SW_TEMP
#define SW_TEMP_OFFSET -3                                                                                                           // Offset sensor by this value in degrees Celsius. Warning: this could damage the elements! J527 controls heating directly.
#endif
#define BACK_BUTTON_MEMORY 1                                                                                                        // Add memory for remapping the back button to revert the last action.

#define LINBUS_BAUD 19200
#define SYNC_BYTE 0x55
#define E_MESSAGE_INTERVAL 30
#define BA_MESSAGE_INTERVAL 120
#define D_MESSAGE_INTERVAL 100
#define SLAVE_COMM_TIMEOUT 2000
#define SLAVE_BOOT_DELAY 200
#define MASTER_COMM_TIMEOUT 60000

#if DEBUG_MODE
HardwareSerial serial_debug(USART1);
#endif
HardwareSerial car_lin(USART2);
HardwareSerial sw_lin(USART3);

unsigned long request_buttons_status_timer, request_heating_status_timer,
              backlight_status_message_timer, slave_comm_timer, master_comm_timer;

uint8_t backlight_status_message[] = {0, 0x81, 0, 0, 0x71},                                                                         // Lights OFF
        buttons_status_message[] = {0x80, 0xF0, 0, 0, 0x21, 0, 0, 0, 0xDE},                                                         // First message upon connection
        steering_heater_status_message[] = {0x32, 0xFE, 0x14},                                                                      // 0C, button released
        // unk_message[] = {0x76, 0x77, 0x37, 0x1A, 0xB7, 0xB4, 0xD7, 0x42, 0x3A},
        back_button_memory = 0;

uint8_t button_remap_array[] = {                                                                                                    // Label    MQB original value
        0x0,
        1,                                                                                                                          // Menu     (1)
        2,                                                                                                                          // Right    (2)
        3,                                                                                                                          // Left     (3)
        0x0, 0x0,
        6,                                                                                                                          // Scroll   (6)
        7,                                                                                                                          // OK       (7)
        8,                                                                                                                          // Back     (8)     - MQB only
        0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
        0x12,                                                                                                                       // Vol      (0x12)
        0x0, 0x0,
        0x15,                                                                                                                       // Next     (0x15)
        0x16,                                                                                                                       // Prev     (0x16)
        0x0, 0x0,
        0x19,                                                                                                                       // Voice    (0x19)
        0x0,
        0x1B,                                                                                                                       // Nav      (0x1B)
        0x19,                                                   // Mapped to Voice                                                  // Phone    (0x1C)  - MQB only.
        0x0, 0x0, 0x0,
        0x20,                                                                                                                       // Mute     (0x20)
        0x21,                                                                                                                       // Joker*   (0x21)
        0x0,
        1,                                                      // Mapped to Menu                                                   // View     (0x23)  - MQB only
        0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
        0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
        0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
        0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
        0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
        0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
        0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
        0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
        0x0, 0x0, 0x0, 0x0, 0x0,
        0x70                                                    // Mapped to Drive Select                                           // RS mode  (0x71)  - MQB only
};

LinFrame master_frame = LinFrame(), slave_frame = LinFrame();
bool e_message_initialized = false, ba_message_initialized = false, d_message_initialized = false,
     e_message_requested = false, ba_message_requested = false;

void setup() {
#if DEBUG_MODE
  serial_debug.begin(DEBUG_BAUD);
  while(!serial_debug) {
    if (millis() >= SERIAL_WAIT_TIMEOUT) {
      break;
    }
  }
  serial_debug.print("Audi MQB to MLB LIN adapter for STM32 started.");
  serial_debug.print(" Clock speed: ");
  serial_debug.print(F_CPU / 1000000);
  serial_debug.println(" MHz.");
#endif

  pinMode(SW_TX_PIN, OUTPUT);
  car_lin.begin(LINBUS_BAUD);
  send_lin_wakeup();
  delay(SLAVE_BOOT_DELAY);                                                                                                          // Needed for MQB buttons to initialize.

  request_buttons_status_timer
  =request_heating_status_timer
  =backlight_status_message_timer
  =slave_comm_timer
  =master_comm_timer
  =millis();

#if BACK_BUTTON_MEMORY
  button_remap_array[8] = 8;                                                                                                        // Back can't be remapped via the array if this option is on.
#endif
}

void loop() {

// MASTER - requests button status, steering heater status and provides backlight status to the steering wheel.
  if (sw_lin.available()) {
    slave_comm_timer = millis();
    byte n = slave_frame.num_bytes();
    byte b = sw_lin.read();

    if (e_message_requested) {
      if (n == 0) {
        slave_frame.append_byte(0x8E);
      }
      slave_frame.append_byte(b);
      n = slave_frame.num_bytes();
      if (n == 10) {
        handle_slave_frame();
        slave_frame.reset();
        e_message_requested = false;
      }
    }

    if (ba_message_requested) {
      if (n == 0) {
        slave_frame.append_byte(0xBA);
      }
      slave_frame.append_byte(b);
      n = slave_frame.num_bytes();
      if (n == 4) {
        handle_slave_frame();
        slave_frame.reset();
        ba_message_requested = false;
      }
    }
  }

  if (((millis() - request_heating_status_timer) >= BA_MESSAGE_INTERVAL) && !ba_message_requested && !e_message_requested) {
    send_lin_break();                                                                                                               // Send LIN break
    sw_lin.write((uint8_t)SYNC_BYTE);                                                                                               // Send sync byte
    // delayMicroseconds(10);
    sw_lin.write((uint8_t)0xBA);                                                                                                    // Send protected ID
    // sw_lin.flush();

    request_heating_status_timer = millis();
    ba_message_requested = true;
    return;
  }

  if (((millis() - request_buttons_status_timer) >= E_MESSAGE_INTERVAL) && !e_message_requested && !ba_message_requested) {
    if ((millis() - backlight_status_message_timer) >= D_MESSAGE_INTERVAL) {
      send_lin_break();
      sw_lin.write((uint8_t)SYNC_BYTE);
      // delayMicroseconds(10);
      sw_lin.write((uint8_t)0xD);
      // sw_lin.flush();

      for (uint8_t i = 0; i < 5; i++) {
        sw_lin.write(backlight_status_message[i]);
      }
      backlight_status_message_timer = millis();
      sw_lin.flush();
    }
   
    send_lin_break();
    sw_lin.write((uint8_t)SYNC_BYTE);
    // delayMicroseconds(10);
    sw_lin.write((uint8_t)0x8E);
    // sw_lin.flush();

    request_buttons_status_timer = millis();
    e_message_requested = true;
  }


// SLAVE - reports button status, steering heater status and receive backlight data from car.
  if (car_lin.available()) {
    master_comm_timer = millis();
    byte b = car_lin.read();
    byte n = master_frame.num_bytes();

    // dump everything
// #if DEBUG_MODE
//     if (b == SYNC_BYTE) {         
//       serial_debug.println();
//     }
//     serial_debug.print(" ");
//     serial_debug.print(b, HEX);
//     return;
// #endif

    if (b == SYNC_BYTE && n > 1 && master_frame.get_byte(n - 1) == 0) {                                                                    // Sync byte to detect start of new frame. 
      // if (n > 0) {
        master_frame.pop_byte();                                                                                                    // break "0" is present in the master frames :(
        handle_master_frame();
      // }
      master_frame.reset();
    } else if (n > LinFrame::kMaxBytes) {                                                                    // Overflow. Discard this frame.
      master_frame.reset();
    } else {
      master_frame.append_byte(b);
    }
  }

// Pseudo watchdog to reset LIN or the board.
  if ((millis() - slave_comm_timer) >= SLAVE_COMM_TIMEOUT) {
#if DEBUG_MODE
    serial_debug.println("Timeout of slave RX. Resetting sw_lin.");
#endif
    send_lin_wakeup();
    delay(SLAVE_BOOT_DELAY);
    slave_comm_timer = millis();
    request_heating_status_timer = millis();
    request_buttons_status_timer = millis();
    e_message_requested = false;
    ba_message_requested = false;
    e_message_initialized = false;
    ba_message_initialized = false;
  }
  if ((millis() - master_comm_timer) >= MASTER_COMM_TIMEOUT) {
    if (d_message_initialized) {
#if DEBUG_MODE
      serial_debug.println("Timeout of master RX - rebooting.");
      serial_debug.flush();
      delay(500);
#endif
      NVIC_SystemReset();
    }
  }
}


void send_lin_wakeup(void) {
  // Send wakeup: Set TX low for 250..5000 µs
  sw_lin.end();
  digitalWrite(SW_TX_PIN, 0);
  delayMicroseconds(500);
  digitalWrite(SW_TX_PIN, 1);
  sw_lin.begin(LINBUS_BAUD);
}


void send_lin_break(void) {
  // Send break: Set TX low for 13+ bit times
  // At 19200 baud, 1 bit = 52.09 µs, 13 bits = 677 µs
  sw_lin.end();                                                                                                                     // Release USART control of TX pin
  digitalWrite(SW_TX_PIN, 0);                                                                                                       // Drive low for break
  delayMicroseconds(780);                                                                                                           // ~14.97 periods
  digitalWrite(SW_TX_PIN, 1);                                                                                                       // Release
  delayMicroseconds(52);                                                                                                            // 1 Bit time
  sw_lin.begin(LINBUS_BAUD);                                                                                                        // Restart UART
}


void handle_slave_frame(void) {
  uint8_t expected_checksum = verify_frame_checksum(slave_frame);
  if (slave_frame.get_byte(slave_frame.num_bytes() - 1) != expected_checksum) {                                                     // Validate checksum
#if DEBUG_MODE
    serial_debug.print("slave_frame checksum validation failed for: ");
    print_frame(slave_frame);
    serial_debug.print("Got: ");
    serial_debug.print(slave_frame.get_byte(slave_frame.num_bytes() - 1), HEX);
    serial_debug.print(" expected: ");
    serial_debug.println(expected_checksum, HEX);
#endif
    return;
  } else {
// #if DEBUG_MODE
//     print_frame(slave_frame);
// #endif
  }

  if (slave_frame.get_byte(0) == 0x8E) {                                                                                            // Button status
    buttons_status_message[0] = slave_frame.get_byte(1);                                                                            // Counter / button pressed
    buttons_status_message[7] = slave_frame.get_byte(8);                                                                            // Horn and SWC error status (paddles disconnected, left buttons disconnected etc.)

#if HORN_AS_DS
    if (bitRead(buttons_status_message[7], 0)) {
      if ((buttons_status_message[0] >> 4) == 8) {
        buttons_status_message[0] = 0x90 | (buttons_status_message[0] & 0xF);                                                       // Force button pressed while preserving the counter
      }
      buttons_status_message[1] = 0x70;                                                                                             // Change Button ID to Drive Select
      buttons_status_message[3] = 1;                                                                                                // Change button direction to pressed
      bitWrite(buttons_status_message[7], 0, 0);                                                                                    // Force horn status to OFF
  #if DEBUG_BUTTON_PRESS
      serial_debug.println("[ Drive Select ]");
  #endif
    } else {
      buttons_status_message[1] = button_remap_array[slave_frame.get_byte(2)];                                                      // Button ID
      buttons_status_message[3] = slave_frame.get_byte(4);                                                                          // scroll wheel direction or button hold duration
    }
#else
    buttons_status_message[1] = button_remap_array[slave_frame.get_byte(2)];
    buttons_status_message[3] = slave_frame.get_byte(4);
  #if DEBUG_BUTTON_PRESS
    if (bitRead(buttons_status_message[7], 0)) {
      serial_debug.println("[ Horn ]");
    }
  #endif
#endif

#if BACK_BUTTON_MEMORY
    if (buttons_status_message[1] != 8 && buttons_status_message[3] > 0) {                                                          // A button other than back was pressed
      if (buttons_status_message[1] == 3) {
        back_button_memory = 2;
      } else if (buttons_status_message[1] == 2) {
        back_button_memory = 3;
      } else if (buttons_status_message[1] == 1) {
        back_button_memory = 1;
      }
    } else if (buttons_status_message[1] == 8 && back_button_memory != 0) {
#if DEBUG_BUTTON_PRESS
      serial_debug.print("Sending back button action: ");
      serial_debug.println(back_button_memory, HEX);
#endif
      buttons_status_message[1] = back_button_memory;
      back_button_memory = 0;
    }
#endif

    buttons_status_message[2] = slave_frame.get_byte(3);                                                                            // May need adjustment with new buttons
    buttons_status_message[4] = slave_frame.get_byte(5);
    buttons_status_message[5] = slave_frame.get_byte(6);

    buttons_status_message[6] = slave_frame.get_byte(7);                                                                            // Paddles
    buttons_status_message[8] = calculate_lin2_checksum(buttons_status_message, 0x8E, 8);

#if DEBUG_BUTTON_PRESS
    if (buttons_status_message[1] == slave_frame.get_byte(2)) {                                                                     // Print only if the button is not remapped
      switch (buttons_status_message[1]) {
        case 1:
          serial_debug.println("[ Menu ]");
          break;
        case 2:
          serial_debug.println("[ Right> ]");
          break;
        case 3:
          serial_debug.println("[ <Left ]");
          break;
        case 6:
          if (buttons_status_message[3] == 0xF) {
            serial_debug.println("[ Scroll- ]");
          } else if (buttons_status_message[3] == 1) {
            serial_debug.println("[ Scroll+ ]");
          }
          break;
        case 7:
          serial_debug.println("[ OK ]");
          break;
        case 8:
          serial_debug.println("[ Back ]");
          break;
        case 0x12:
          if (buttons_status_message[3] == 0xF) {
            serial_debug.println("[ Vol- ]");
          } else if (buttons_status_message[3] == 1) {
            serial_debug.println("[ Vol+ ]");
          }
          break;
        case 0x15:
          serial_debug.println("[ Track>> ]");
          break;
        case 0x16:
          serial_debug.println("[ <<Track ]");
          break;
        case 0x19:
          serial_debug.println("[ Voice ]");
          break;
        case 0x1B:
          serial_debug.println("[ Nav ]");
          break;
        case 0x1C:
          serial_debug.println("[ Phone ]");
          break;
        case 0x20:
          serial_debug.println("[ Mute ]");
          break;
        case 0x21:
          serial_debug.println("[ Joker* ]");
          break;
        case 0x23:
          serial_debug.println("[ View ]");
          break;
        case 0x71:
          serial_debug.println("[ RS Mode ]");
          break;
        default:
          break;
      }
    } else {                                                                                                                        // Print remapped value as HEX.
      serial_debug.print("[ remap 0x");
      serial_debug.print(buttons_status_message[1], HEX);
      serial_debug.println(" ]");
    }

    if (bitRead(buttons_status_message[6], 0)) {
      serial_debug.println("[ Paddle- ]");
    } else if (bitRead(buttons_status_message[6], 1)) {
      serial_debug.println("[ Paddle+ ]");
    }
#endif

#if DEBUG_MODE
    if (!e_message_initialized) {
      e_message_initialized = true;
      serial_debug.println("Button status message initialized.");
    }
#else
      e_message_initialized = true;
#endif
  }
  else if (slave_frame.get_byte(0) == 0xBA) {                                                                                       // Steering heater status
    steering_heater_status_message[0] = slave_frame.get_byte(1);
#if CORRECT_SW_TEMP
    if (SW_TEMP_OFFSET > 0) {
        if (steering_heater_status_message[0] + SW_TEMP_OFFSET < 0xFF) {
            steering_heater_status_message[0] += SW_TEMP_OFFSET;
        } else {
            steering_heater_status_message[0] = 0xFF;
        }
    } else if (SW_TEMP_OFFSET < 0) {
      if (steering_heater_status_message[0] >= abs(SW_TEMP_OFFSET)) {
          steering_heater_status_message[0] += SW_TEMP_OFFSET;
      } else {
          steering_heater_status_message[0] = 0;
      }
    }
    // steering_heater_status_message[2] = (steering_heater_status_message[2] + (SW_TEMP_OFFSET * -1)) % 0xFF;
    steering_heater_status_message[2] = calculate_lin2_checksum(steering_heater_status_message, 0xBA, 2);
#else
    steering_heater_status_message[2] = slave_frame.get_byte(3);
#endif

    steering_heater_status_message[1] = slave_frame.get_byte(2);
#if DEBUG_BUTTON_PRESS
    if (bitRead(steering_heater_status_message[1], 0)) {
      serial_debug.println("[ SWHeat ]");
    }
#endif

#if DEBUG_MODE
    if (!ba_message_initialized) {
      ba_message_initialized = true;
      serial_debug.println("Steering heater message initialized.");
    }
#else
    ba_message_initialized = true;
#endif
  }
}


void handle_master_frame(void) {
// #if DEBUG_MODE
//       print_frame(master_frame);
// #endif

  if (master_frame.num_bytes() > LinFrame::kMinBytes) {                                                                             // Must be a data frame
    uint8_t expected_checksum = verify_frame_checksum(master_frame);
    if (master_frame.get_byte(master_frame.num_bytes() - 1) != expected_checksum) {                                                 // Validate checksum
  #if DEBUG_MODE
      serial_debug.print("master_frame checksum validation failed for: ");
      print_frame(master_frame);
      serial_debug.print("Got: ");
      serial_debug.print(master_frame.get_byte(master_frame.num_bytes() - 1), HEX);
      serial_debug.print(" expected: ");
      serial_debug.println(expected_checksum, HEX);
  #endif
      return;
    } else {
// #if DEBUG_MODE
//       print_frame(master_frame);
// #endif
      if (master_frame.get_byte(0) == 0xD) {                                                                                        // Backlight status
        backlight_status_message[0] = master_frame.get_byte(1);
        backlight_status_message[1] = master_frame.get_byte(2);
        backlight_status_message[2] = master_frame.get_byte(3);
        backlight_status_message[3] = master_frame.get_byte(4);
        backlight_status_message[4] = master_frame.get_byte(5);
#if DEBUG_MODE
        if (!d_message_initialized) {
          d_message_initialized = true;
          serial_debug.println("Backlight message initialized.");
        }
#else
        d_message_initialized = true;
#endif
      }
    }
  } else {                                                                                                                          // Must be a request frame
      if (master_frame.get_byte(0) == 0x8E) {                                                                                       // Button status
        if (!e_message_initialized) {
          return;
        }
        for (uint8_t i = 0; i < 9; i++) {
          car_lin.write(buttons_status_message[i]);
          // serial_debug.print(buttons_status_message[i], HEX);
          // serial_debug.print(" ");
        }
        // serial_debug.println();
        car_lin.flush();
      }
      else if (master_frame.get_byte(0) == 0xBA) {                                                                                  // Steering heater status
        if (!ba_message_initialized) {
          return;
        }
        for (uint8_t i = 0; i < 3; i++) {
          car_lin.write(steering_heater_status_message[i]);
          // serial_debug.print(steering_heater_status_message[i], HEX);
          // serial_debug.print(" ");
        }
        // serial_debug.println();
        car_lin.flush();
      }
      // else if (master_frame.get_byte(0) == 0x7D) {                                                                                  // unk
        // for (uint8_t i = 0; i < 9; i++) {
        //   car_lin.write(unk_message[i]);
        //   // serial_debug.print(unk_message[i], HEX);
        //   // serial_debug.print(" ");
        // }
        // // serial_debug.println();
        // car_lin.flush();
      // }
#if DEBUG_MODE
      // else {
      //   serial_debug.print("Received unknown master request ID: ");
      //   serial_debug.println(master_frame.get_byte(0), HEX);
      // }
#endif
    return;
  }
}


uint8_t calculate_lin2_checksum(uint8_t *data, uint8_t id, uint8_t size) {
  uint16_t checksum = id;
  for (uint8_t i = 0; i < size; i++) {
		checksum += data[i];
		if (checksum >= 0x100) {
		  checksum -= 0xFF;
    }
	}
  return ~checksum & 0xFF;
}


uint8_t verify_frame_checksum(LinFrame frame) {
  uint16_t checksum = frame.get_byte(0);
  for (uint8_t i = 1; i < frame.num_bytes() - 1; i++) {
    checksum += frame.get_byte(i);
    if (checksum >= 0x100) {
		  checksum -= 0xFF;
    }
  }
  return (~checksum) & 0xFF;
}


#if DEBUG_MODE
void print_frame(LinFrame frame) {
  if (frame.num_bytes() > LinFrame::kMinBytes) {
    for (uint8_t i = 0; i < frame.num_bytes(); i++) {
      if (i == 0) {
        serial_debug.print("[");
      }
      serial_debug.print(frame.get_byte(i), HEX);
      if (i == 0) {
        serial_debug.print("]");
      }
      serial_debug.print(" ");
    }
    serial_debug.println();
  } else {
    serial_debug.print("req frame ");
    serial_debug.println(frame.get_byte(0), HEX);
  }
}
#endif
