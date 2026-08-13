// Compile-only Arduino stubs.
//
// These exist so `src/` can be type-checked on a development machine that does
// not have the Xtensa toolchain installed — `tools/check_app_syntax.py`
// compiles the application sources against these declarations with no linking.
// They catch typos, wrong signatures, missing includes and const-correctness
// mistakes, which is most of what a firmware build catches.
//
// They are NOT a simulator and NOT a hardware abstraction: nothing here has a
// definition, and no real behaviour is modelled. The only build that produces
// a flashable image is the PlatformIO `esp32dev` environment.
#pragma once

#include <cstdarg>
#include <cstddef>
#include <cstdint>

#ifdef _MSC_VER
// MSVC's strtok_s takes the same arguments as POSIX strtok_r.
#include <cstring>
#define strtok_r strtok_s
#endif

#define F(string_literal) (string_literal)
#define PROGMEM

constexpr int HIGH = 1;
constexpr int LOW = 0;
constexpr int INPUT = 0x01;
constexpr int OUTPUT = 0x03;
constexpr int INPUT_PULLUP = 0x05;

constexpr uint32_t SERIAL_8N1 = 0x800001c;

void pinMode(uint8_t pin, uint8_t mode);
void pinMode(int pin, int mode);
void digitalWrite(uint8_t pin, uint8_t value);
void digitalWrite(int pin, int value);
int digitalRead(uint8_t pin);
int digitalRead(int pin);

uint32_t millis();
uint32_t micros();
void delay(uint32_t ms);

// --- ADC ---------------------------------------------------------------------
typedef enum {
    ADC_0db,
    ADC_2_5db,
    ADC_6db,
    ADC_11db,
} adc_attenuation_t;

uint16_t analogRead(uint8_t pin);
uint16_t analogRead(int pin);
uint32_t analogReadMilliVolts(uint8_t pin);
uint32_t analogReadMilliVolts(int pin);
void analogReadResolution(uint8_t bits);
void analogSetPinAttenuation(uint8_t pin, adc_attenuation_t attenuation);
void analogSetPinAttenuation(int pin, adc_attenuation_t attenuation);

// --- LEDC --------------------------------------------------------------------
uint32_t ledcSetup(uint8_t channel, uint32_t frequency, uint8_t resolutionBits);
void ledcAttachPin(uint8_t pin, uint8_t channel);
void ledcAttachPin(int pin, uint8_t channel);
void ledcWrite(uint8_t channel, uint32_t duty);

// --- Print / Stream ----------------------------------------------------------
class Print {
public:
    virtual ~Print() = default;
    virtual size_t write(uint8_t value) = 0;

    size_t print(const char* text);
    size_t print(char value);
    size_t print(int value);
    size_t print(unsigned int value);
    size_t print(long value);
    size_t print(unsigned long value);
    size_t print(double value, int digits = 2);

    size_t println();
    size_t println(const char* text);
    size_t println(char value);
    size_t println(int value);
    size_t println(unsigned long value);
    size_t println(double value, int digits = 2);

    int printf(const char* format, ...);
    void flush();
};

class Stream : public Print {
public:
    virtual int available() = 0;
    virtual int read() = 0;
    virtual int peek() = 0;
};

class HardwareSerial : public Stream {
public:
    size_t write(uint8_t value) override;
    int available() override;
    int read() override;
    int peek() override;

    void begin(unsigned long baud);
    void begin(unsigned long baud, uint32_t config, int8_t rxPin, int8_t txPin);
    void begin(unsigned long baud, uint32_t config, int rxPin, int txPin);
    void end();
};

extern HardwareSerial Serial;
extern HardwareSerial Serial1;
extern HardwareSerial Serial2;

// --- ESP ---------------------------------------------------------------------
class EspClass {
public:
    void restart();
    uint32_t getFreeHeap();
};

extern EspClass ESP;
