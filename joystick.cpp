#include "Arduino.h"
#include "USBHost_t36.h"

USBHost myusb;
USBHub hub1(myusb);
USBHub hub2(myusb);
USBHIDParser hid1(myusb);
USBHIDParser hid2(myusb);
USBHIDParser hid3(myusb);
USBHIDParser hid4(myusb);
USBHIDParser hid5(myusb);
JoystickController joystick1(myusb);
//BluetoothController bluet(myusb, true, "0000");   // Version does pairing to
// device
BluetoothController bluet(myusb);  // version assumes it already was paired

uint32_t joystick_buttons_prev = 0;
uint32_t joystick_buttons = 0;

RawHIDController rawhid1(myusb);
RawHIDController rawhid2(myusb, 0xffc90004);

USBDriver *drivers[] = {&hub1, &hub2, &joystick1, &bluet, &hid1,
                        &hid2, &hid3, &hid4,      &hid5};

#define CNT_DEVICES (sizeof(drivers) / sizeof(drivers[0]))
const char *driver_names[CNT_DEVICES] = {
    "Hub1", "Hub2", "JOY1D", "Bluet", "HID1", "HID2", "HID3", "HID4", "HID5"};

bool driver_active[CNT_DEVICES] = {false, false, false, false};

// Lets also look at HID Input devices
USBHIDInput *hiddrivers[] = {&joystick1, &rawhid1, &rawhid2};

#define CNT_HIDDEVICES (sizeof(hiddrivers) / sizeof(hiddrivers[0]))
const char *hid_driver_names[CNT_DEVICES] = {"Joystick1", "RawHid1", "RawHid2"};

bool hid_driver_active[CNT_DEVICES] = {false, false, false};

BTHIDInput *bthiddrivers[] = {&joystick1};
#define CNT_BTHIDDEVICES (sizeof(bthiddrivers) / sizeof(bthiddrivers[0]))
const char *bthid_driver_names[CNT_HIDDEVICES] = {"joystick"};
bool bthid_driver_active[CNT_HIDDEVICES] = {false};

float joystick_axes[6] = {0.f};
bool first_joystick_message = true;
uint8_t last_bdaddr[6] = {0, 0, 0, 0, 0, 0};

void UpdateActiveDeviceInfo();

//=============================================================================
// Setup
//=============================================================================
void SetupJoystick() {
  myusb.begin();
  delay(2000);
}

bool IsAvailable() { return static_cast<bool>(joystick1); }

bool WasPressed(uint32_t button_bit) {
  // Serial.printf("WasPressed\n");
  return ((joystick_buttons & button_bit) &&
          !(joystick_buttons_prev & button_bit));
}

//=============================================================================
// Loop
//=============================================================================
void UpdateJoystick() {
  myusb.Task();

  // check to see if the device list has changed:
  UpdateActiveDeviceInfo();

  joystick_buttons_prev = joystick_buttons;
  if (joystick1.available()) {
    if (first_joystick_message) {
      Serial.printf("*** First Joystick message %x:%x ***\n",
                    joystick1.idVendor(), joystick1.idProduct());
      first_joystick_message = false;

      const uint8_t *psz = joystick1.manufacturer();
      if (psz && *psz) Serial.printf("  manufacturer: %s\n", psz);
      psz = joystick1.product();
      if (psz && *psz) Serial.printf("  product: %s\n", psz);
      psz = joystick1.serialNumber();
      if (psz && *psz) Serial.printf("  Serial: %s\n", psz);

      // lets try to reduce number of fields that update
      joystick1.axisChangeNotifyMask(0xFFFFFl);
    }

    for (uint8_t i = 0; i < 6; i++) {
      joystick_axes[i] = joystick1.getAxis(i) / 255.f;
    }
    joystick_buttons = joystick1.getButtons();

    // See about maybe pair...
    if ((joystick_buttons & 0x10000) && !(joystick_buttons_prev & 0x10000) &&
        (joystick_buttons & 0x0C01)) {
      // PS button just pressed and select button pressed act like PS4 share
      // like... Note: you can use either R1 or L1 with the PS button, to work
      // with Sony Move Navigation...
      Serial.print("\nPS3 Pairing Request");
      if (!last_bdaddr[0] && !last_bdaddr[1] && !last_bdaddr[2] &&
          !last_bdaddr[3] && !last_bdaddr[4] && !last_bdaddr[5]) {
        Serial.println(" - failed - no Bluetooth adapter has been plugged in");
      } else if (!hiddrivers[0]) {  // Kludge see if we are connected as HID?
        Serial.println(" - failed - PS3 device not plugged into USB");
      } else {
        Serial.printf(" - Attempt pair to: %x:%x:%x:%x:%x:%x\n", last_bdaddr[0],
                      last_bdaddr[1], last_bdaddr[2], last_bdaddr[3],
                      last_bdaddr[4], last_bdaddr[5]);

        if (!joystick1.PS3Pair(last_bdaddr)) {
          Serial.println("  Pairing call Failed");
        } else {
          Serial.println(
              "  Pairing complete (I hope), make sure Bluetooth adapter is "
              "plugged in and try PS3 without USB");
        }
      }
    }

    joystick1.joystickDataClear();
  }
}

//=============================================================================
// UpdateActiveDeviceInfo
//=============================================================================
void UpdateActiveDeviceInfo() {
  for (uint8_t i = 0; i < CNT_DEVICES; i++) {
    if (*drivers[i] != driver_active[i]) {
      if (driver_active[i]) {
        Serial.printf("*** Device %s - disconnected ***\n", driver_names[i]);
        driver_active[i] = false;
      } else {
        Serial.printf("*** Device %s %x:%x - connected ***\n", driver_names[i],
                      drivers[i]->idVendor(), drivers[i]->idProduct());
        driver_active[i] = true;

        const uint8_t *psz = drivers[i]->manufacturer();
        if (psz && *psz) Serial.printf("  manufacturer: %s\n", psz);
        psz = drivers[i]->product();
        if (psz && *psz) Serial.printf("  product: %s\n", psz);
        psz = drivers[i]->serialNumber();
        if (psz && *psz) Serial.printf("  Serial: %s\n", psz);

        if (drivers[i] == &bluet) {
          const uint8_t *bdaddr = bluet.myBDAddr();
          // remember it...
          Serial.printf("  BDADDR: %x:%x:%x:%x:%x:%x\n", bdaddr[0], bdaddr[1],
                        bdaddr[2], bdaddr[3], bdaddr[4], bdaddr[5]);
          for (uint8_t i = 0; i < 6; i++) last_bdaddr[i] = bdaddr[i];
        }
      }
    }
  }

  for (uint8_t i = 0; i < CNT_HIDDEVICES; i++) {
    if (*hiddrivers[i] != hid_driver_active[i]) {
      if (hid_driver_active[i]) {
        Serial.printf("*** HID Device %s - disconnected ***\n",
                      hid_driver_names[i]);
        hid_driver_active[i] = false;
      } else {
        Serial.printf("*** HID Device %s %x:%x - connected ***\n",
                      hid_driver_names[i], hiddrivers[i]->idVendor(),
                      hiddrivers[i]->idProduct());
        hid_driver_active[i] = true;

        const uint8_t *psz = hiddrivers[i]->manufacturer();
        if (psz && *psz) Serial.printf("  manufacturer: %s\n", psz);
        psz = hiddrivers[i]->product();
        if (psz && *psz) Serial.printf("  product: %s\n", psz);
        psz = hiddrivers[i]->serialNumber();
        if (psz && *psz) Serial.printf("  Serial: %s\n", psz);

        // See if this is our joystick object...
        if (hiddrivers[i] == &joystick1) {
          Serial.printf("  Joystick type: %d\n", joystick1.joystickType());
        }
      }
    }
  }
  // Then Bluetooth devices
  for (uint8_t i = 0; i < CNT_BTHIDDEVICES; i++) {
    if (*bthiddrivers[i] != bthid_driver_active[i]) {
      if (bthid_driver_active[i]) {
        Serial.printf("*** BTHID Device %s - disconnected ***\n",
                      hid_driver_names[i]);
        bthid_driver_active[i] = false;
      } else {
        Serial.printf("*** BTHID Device %s %x:%x - connected ***\n",
                      hid_driver_names[i], hiddrivers[i]->idVendor(),
                      hiddrivers[i]->idProduct());
        bthid_driver_active[i] = true;

        const uint8_t *psz = bthiddrivers[i]->manufacturer();
        if (psz && *psz) Serial.printf("  manufacturer: %s\n", psz);
        psz = bthiddrivers[i]->product();
        if (psz && *psz) Serial.printf("  product: %s\n", psz);
        psz = bthiddrivers[i]->serialNumber();
        if (psz && *psz) Serial.printf("  Serial: %s\n", psz);
      }
    }
  }
}

//=============================================================================
// displayPS3Data
//=============================================================================
// void displayPS3Data() {
//   buttons = joystick1.getButtons();
//   // buttons = psAxis[2] | ((uint16_t)psAxis[3] << 8);

//   // Use L3 (Left joystick button) to toggle Show Raw or not...
//   // if ((buttons & 0x02) && !(buttons_prev & 0x02)) show_raw_data =
//   // !show_raw_data; if ((buttons & 0x04) && !(buttons_prev & 0x04))
//   // show_changed_data = !show_changed_data;

//   // See about maybe pair...
//   if ((buttons & 0x10000) && !(buttons_prev & 0x10000) && (buttons & 0x0C01))
//   {
//     // PS button just pressed and select button pressed act like PS4 share
//     // like... Note: you can use either R1 or L1 with the PS button, to work
//     // with Sony Move Navigation...
//     Serial.print("\nPS3 Pairing Request");
//     if (!last_bdaddr[0] && !last_bdaddr[1] && !last_bdaddr[2] &&
//         !last_bdaddr[3] && !last_bdaddr[4] && !last_bdaddr[5]) {
//       Serial.println(" - failed - no Bluetooth adapter has been plugged in");
//     } else if (!hiddrivers[0]) {  // Kludge see if we are connected as HID?
//       Serial.println(" - failed - PS3 device not plugged into USB");
//     } else {
//       Serial.printf(" - Attempt pair to: %x:%x:%x:%x:%x:%x\n",
//       last_bdaddr[0],
//                     last_bdaddr[1], last_bdaddr[2], last_bdaddr[3],
//                     last_bdaddr[4], last_bdaddr[5]);

//       if (!joystick1.PS3Pair(last_bdaddr)) {
//         Serial.println("  Pairing call Failed");
//       } else {
//         Serial.println(
//             "  Pairing complete (I hope), make sure Bluetooth adapter is "
//             "plugged in and try PS3 without USB");
//       }
//     }
//   }

//   Serial.printf("LX: %d, LY: %d, RX: %d, RY: %d \r\n", psAxis[0], psAxis[1],
//                 psAxis[2], psAxis[5]);
//   Serial.printf("L-Trig: %d, R-Trig: %d\r\n", psAxis[3], psAxis[4]);
//   Serial.printf("Buttons: %x\r\n", buttons);
// }
