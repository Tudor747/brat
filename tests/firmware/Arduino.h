#pragma once
#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <deque>
#include <sstream>
#include <string>
using std::min;
constexpr int HIGH = 1, LOW = 0, INPUT = 0, OUTPUT = 1;
inline uint64_t mockTime = 0;
inline int pins[40] = {};
inline int pulses[40] = {};
inline uint32_t micros() { return (uint32_t)mockTime; }
inline uint32_t millis() { return (uint32_t)(mockTime / 1000); }
inline int digitalRead(int pin) { return pins[pin]; }
inline void digitalWrite(int pin, int value) { pins[pin] = value; if (value) ++pulses[pin]; }
inline void pinMode(int, int) {}
inline void delayMicroseconds(unsigned delay) { mockTime += delay; }
inline void ledcWrite(int, uint32_t) {}
inline void ledcSetup(int, int, int) {}
inline void ledcAttachPin(int, int) {}
struct MockSerial {
    std::string output;
    std::deque<char> input;
    void begin(int) {}
    int available() { return (int)input.size(); }
    char read() { char c = input.front(); input.pop_front(); return c; }
    template<class T> void print(T value) { std::ostringstream s; s << value; output += s.str(); }
    void print(double value, int) { print(value); }
    void println() { output += '\n'; }
    template<class T> void println(T value) { print(value); println(); }
    template<class... T> void printf(const char* format, T... args) {
        char buffer[512]; std::snprintf(buffer, sizeof(buffer), format, args...); output += buffer;
    }
};
inline MockSerial Serial;
