#pragma once

uint8_t map_4_8(uint8_t input) {
  // chatgpt solution :)
  if (input < 8)
    // Linear mapping from range 0-7 to 0-127
    return (input << 4) | (input << 1) | (input >> 2);
  else
    // Linear mapping from range 8-15 to 128-255
    return 128 + (((input - 8) << 4) | ((input - 8) << 1) | ((input - 8) >> 2));
  
  // if (input < input_mid)
  //   return map(input, input_min, input_mid-1, 0, output_mid);
  // else
  //   return map(input, input_mid, input_max, output_mid, output_max);
}


// https://github.com/Minimuino/thumbstick-deadzones
uint8_t dz_sloped_scaled_axial(uint8_t stick_input, uint8_t deadzone) {
  const uint8_t center = 128;
  const uint8_t dzmin = center - deadzone;
  const uint8_t dzmax = center + deadzone;

  if (stick_input >= dzmin && stick_input <= dzmax)
    stick_input = center;
  else if (stick_input < dzmin)
    stick_input = map(stick_input, 0, dzmin, 0, center);
  else if (stick_input > dzmax)
    stick_input = map(stick_input, dzmax, 255, center, 255);

 return stick_input;
}


// https://github.com/x360ce/x360ce/blob/master/x360ce.Engine/Common/ConvertHelper.cs
float LimitRange(float value, float _min, float _max)
{
  // If inverted then swap.
  if (_min > _max) (_min, _max) = (_max, _min);
  // Limit value between min and max.
  return max(_min, min(_max, value));

  // If inverted then swap.
  //if (min > max)
  //  (min, max) = (max, min);
  //if (value > max)
  //  return max;
  //if (value < min)
  //  return min;
  //return value;
}

float ConvertRangeF(float oldValue, float oldMin, float oldMax, float newMin, float newMax)
{
  // if (oldMin == oldMax)
  //   throw new ArgumentException($"The arguments {nameof(oldMin)} and {nameof(oldMax)} cannot be equal!");
  // if (newMin == newMax)
  //   throw new ArgumentException($"The arguments {nameof(newMin)} and {nameof(newMax)} cannot be equal!");

  // if (LimitRange(oldValue, oldMin, oldMax) != oldValue)
  //   throw new ArgumentOutOfRangeException($"Value {nameof(oldValue)} is out of {nameof(oldMin)} - {nameof(oldMax)} range!");

  auto oldSize = oldMax - oldMin;
  auto newSize = newMax - newMin;
  auto position = (oldValue - oldMin) / oldSize;
  auto newValue = position * newSize + newMin;
  return LimitRange(newValue, newMin, newMax);
}

//float GetThumbValue(float dInputValue, float deadZone, float antiDeadZone, float linear, bool isInverted, bool isHalf, bool isThumb = true)
//{
//  //
//  //        [  32768 steps | 32768 steps ]
//  // DInput [      0 32767 | 32768 65535 ] 
//  // XInput [ -32768    -1 | 0     32767 ]
//  //
//  auto dih = 32768.0f;
//  auto dInput = (float)dInputValue;
//  // If source axis must be inverted then...
//  if (isInverted)
//    dInput = (float)UINT16_MAX - dInput;
//  // if only upper half axis must be used then...
//  // Note: half axis is ignored if destination is thumb.
//  if (isHalf && !isThumb)
//  {
//    // Limit minimum value.
//    if (dInput < dih)
//      dInput = dih;
//    // Convert half Dinput range [32768;65535] range to DInput range (ushort[0;65535])
//    dInput = ConvertRangeF(dInput, dih, UINT16_MAX, 0.0f, UINT16_MAX);
//  }
//  auto min = isThumb ? -32768.f : 0.0f;
//  auto max = isThumb ? 32767.0f : 255.0f;
//
//  // Convert DInput range(ushort[0; 65535]) to XInput thumb range(ushort[min; max]).
//  auto xInput = ConvertRangeF(dInput, 0, UINT16_MAX, min, max);
//  // Check if value is negative (only thumb).
//  bool invert = xInput < 0.0f;
//  // Convert [-32768;-1] -> [32767;0]
//  if (invert)
//    xInput = -1.0f - xInput;
//
//  auto deadZoneApplied = false;
//  // If deadzone value is set then...
//  if (deadZone > 0.0f)
//  {
//    deadZoneApplied = xInput <= deadZone;
//    xInput = deadZoneApplied
//      ? 0.0f
//      // Convert range [deadZone;max] => [0;max];
//      : ConvertRangeF(xInput, deadZone, max, 0.0f, max);
//  }
//  // If anti-deadzone value is set then...
//  if (antiDeadZone > 0.0f && xInput > 0.0f)
//  {
//    // Convert range [0;max] => [antiDeadZone;max];
//    xInput = ConvertRangeF(xInput, 0.0f, max, antiDeadZone, max);
//  }
//  // If linear value is set then...
//  if (linear != 0.0f && xInput > 0.0f)
//  {
//    // [antiDeadZone;32767] => [0;1f];
//    auto valueF = ConvertRangeF(xInput, antiDeadZone, max, 0.0f, 1.0f);
//    auto linearF = (float)linear / 100.0f;
//    auto x = -valueF;
//    if (linearF < 0.0f) x = 1.0f + x;
//    auto v = (float)sqrt(1.0f - x * x);
//    if (linearF < 0.0f) v = 1.0f - v;
//    valueF = valueF + (2.0f - v - valueF - 1.0f) * abs(linearF);
//    // [0;1f] => [antiDeadZone;max];
//    xInput = ConvertRangeF(valueF, 0.0f, 1.0f, antiDeadZone, max);
//  }
//  // If inversion required (only thumb) and not in deadzone then...
//  // Checking for deadzone prevents XInput value jittering between 0 and -1.
//  if (invert && !deadZoneApplied)
//    // Convert [32767;0] -> [-32768;-1]
//    xInput = -1.0f - xInput;
//  // Set negative center value (-1) to 0 for thumb.
//  if (isThumb && xInput == -1)
//    xInput = 0;
//  // Return value.
//  return xInput;
//}

float GetThumbValue(float dInputValue, float deadZone, float antiDeadZone, float linear, bool isInverted, bool isHalf, bool isThumb = true)
{
  //
  //        [  32768 steps | 32768 steps ]
  // DInput [      0 32767 | 32768 65535 ] 
  // XInput [ -32768    -1 | 0     32767 ]
  //
  auto dih = 128.0f;
  auto dInput = (float)dInputValue;
  // If source axis must be inverted then...
  if (isInverted)
    dInput = (float)UINT8_MAX - dInput;
  // if only upper half axis must be used then...
  // Note: half axis is ignored if destination is thumb.
  if (isHalf && !isThumb)
  {
    // Limit minimum value.
    if (dInput < dih)
      dInput = dih;
    // Convert half Dinput range [32768;65535] range to DInput range (ushort[0;65535])
    dInput = ConvertRangeF(dInput, dih, UINT8_MAX, 0.0f, UINT8_MAX);
  }
  auto min = isThumb ? (float)INT8_MIN : 0.0f;
  auto max = isThumb ? (float)INT8_MAX : 255.0f;

  // Convert DInput range(ushort[0; 65535]) to XInput thumb range(ushort[min; max]).
  auto xInput = ConvertRangeF(dInput, 0, UINT8_MAX, min, max);
  // Check if value is negative (only thumb).
  bool invert = xInput < 0.0f;
  // Convert [-32768;-1] -> [32767;0]
  if (invert)
    xInput = -1.0f - xInput;

  auto deadZoneApplied = false;
  // If deadzone value is set then...
  if (deadZone > 0.0f)
  {
    deadZoneApplied = xInput <= deadZone;
    xInput = deadZoneApplied
      ? 0.0f
      // Convert range [deadZone;max] => [0;max];
      : ConvertRangeF(xInput, deadZone, max, 0.0f, max);
  }
  // If anti-deadzone value is set then...
  if (antiDeadZone > 0.0f && xInput > 0.0f)
  {
    // Convert range [0;max] => [antiDeadZone;max];
    xInput = ConvertRangeF(xInput, 0.0f, max, antiDeadZone, max);
  }
  // If linear value is set then...
  if (linear != 0.0f && xInput > 0.0f)
  {
    // [antiDeadZone;32767] => [0;1f];
    auto valueF = ConvertRangeF(xInput, antiDeadZone, max, 0.0f, 1.0f);
    auto linearF = (float)linear / 100.0f;
    auto x = -valueF;
    if (linearF < 0.0f) x = 1.0f + x;
    auto v = (float)sqrt(1.0f - x * x);
    if (linearF < 0.0f) v = 1.0f - v;
    valueF = valueF + (2.0f - v - valueF - 1.0f) * abs(linearF);
    // [0;1f] => [antiDeadZone;max];
    xInput = ConvertRangeF(valueF, 0.0f, 1.0f, antiDeadZone, max);
  }
  // If inversion required (only thumb) and not in deadzone then...
  // Checking for deadzone prevents XInput value jittering between 0 and -1.
  if (invert && !deadZoneApplied)
    // Convert [32767;0] -> [-32768;-1]
    xInput = -1.0f - xInput;
  // Set negative center value (-1) to 0 for thumb.
  if (isThumb && xInput == -1)
    xInput = 0;
  // Return value.
  return xInput;
}
