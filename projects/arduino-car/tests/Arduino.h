#pragma once

#include <cstdlib>
#include <sstream>

// Host-side API stub. This header is not used by Arduino builds.
constexpr int A0 = 14;
constexpr int A1 = 15;
constexpr int OUTPUT = 1;
constexpr int HIGH = 1;
constexpr int LOW = 0;

using std::abs;

struct SerialStub {
  long baud = 0;
  std::ostringstream output;
  void begin(long value) { baud = value; }
  template <typename T> void print(const T& value) { output << value; }
  template <typename T> void println(const T& value) { output << value << '\n'; }
};

extern SerialStub Serial;
void pinMode(int pin, int mode);
int analogRead(int pin);
void analogWrite(int pin, int value);
void digitalWrite(int pin, int value);

inline int constrain(int value, int low, int high) {
  return value < low ? low : (value > high ? high : value);
}
