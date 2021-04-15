#pragma once

#include "Arduino.h"
#include "USBHost_t36.h"

extern float joystick_axes[6];
extern uint32_t joystick_buttons;
// extern uint32_t joystick_buttons_prev;
// extern JoystickController joystick1;

const uint32_t PS3_PS = 0x10000;
const uint32_t PS3_SELECT = 0x1;
const uint32_t PS3_START = 0x8;
const uint32_t PS3_LEFT_THUMB = 0x02;
const uint32_t PS3_RIGHT_THUMB = 0x04;

void SetupJoystick();
void UpdateJoystick();

bool IsAvailable();
bool WasPressed(uint32_t button_bit);