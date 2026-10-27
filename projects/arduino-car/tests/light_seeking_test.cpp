#include "Arduino.h"

#include <array>
#include <iostream>
#include <stdexcept>
#include <string>

SerialStub Serial;
std::array<int, 20> modes{};
std::array<int, 20> inputs{};
std::array<int, 20> pwm{};
std::array<int, 20> levels{};

void pinMode(int pin, int mode) { modes.at(pin) = mode; }
int analogRead(int pin) { return inputs.at(pin); }
void analogWrite(int pin, int value) { pwm.at(pin) = value; }
void digitalWrite(int pin, int value) { levels.at(pin) = value; }

// Exercise the actual sketch, rather than a separate implementation.
#include "../firmware/light_seeking/light_seeking.ino"

void expect(bool condition, const std::string& message) {
  if (!condition) throw std::runtime_error(message);
}

int checked = 0;

void runCase(const char* name, int right, int left, int expectedLeft, int expectedRight) {
  inputs.at(A0) = right;
  inputs.at(A1) = left;
  pwm.fill(-1);
  levels.fill(-1);
  Serial.output.str("");
  Serial.output.clear();
  loop();
  expect(pwm.at(9) == expectedLeft && pwm.at(10) == expectedRight, name);
  expect(levels.at(6) == HIGH && levels.at(7) == LOW &&
         levels.at(4) == HIGH && levels.at(5) == LOW,
         std::string(name) + ": motor direction");
  ++checked;
}

int main() {
  try {
    setup();
    expect(Serial.baud == 9600, "serial baud");
    for (int pin : {4, 5, 6, 7, 9, 10}) expect(modes.at(pin) == OUTPUT, "output pin setup");

    runCase("sensor offset centers readings", 503, 500, 105, 119);
    expect(Serial.output.str() == "R:503 | L:500 | diff:0\n", "serial readings");
    runCase("equal readings remain in deadband", 500, 500, 105, 119);
    runCase("left brighter", 600, 300, 35, 119);
    runCase("right brighter", 300, 600, 105, 49);
    runCase("positive deadband inside", 515, 500, 105, 119);
    runCase("positive deadband boundary turns", 516, 500, 35, 119);
    runCase("negative deadband inside", 491, 500, 105, 119);
    runCase("negative deadband boundary turns", 490, 500, 105, 49);
    runCase("dark boundary is strict", 900, 900, 105, 119);
    runCase("only right exceeds dark threshold", 901, 900, 105, 119);
    runCase("only left exceeds dark threshold", 900, 901, 105, 119);
    runCase("both exceed dark threshold", 901, 901, 0, 0);
    expect(Serial.output.str() == "worring R:901 | L:901\n", "dark serial message");
    runCase("ADC maximum stops", 1023, 1023, 0, 0);
    runCase("ADC minimum does not stop", 0, 0, 105, 119);
    runCase("right dark alone turns left", 1023, 0, 35, 119);
    runCase("left dark alone turns right", 0, 1023, 105, 49);

    // Editable speed settings must still respect the hardware PWM range.
    speedBase = 300;
    speedTurn = -30;
    runCase("straight PWM upper clamp", 503, 500, 255, 255);
    runCase("left turn PWM lower clamp", 600, 300, 0, 255);
    runCase("right turn PWM lower clamp", 300, 600, 255, 0);
    runCase("dark stop precedes speed settings", 901, 901, 0, 0);

    std::cout << "PASS: setup, serial output, and " << checked << " control cases\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "FAIL: " << error.what() << '\n';
    return 1;
  }
}
