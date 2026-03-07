#pragma once
#include <Arduino.h>

// Quadrature encoder decoder using GPIO interrupts
//
// Attaches CHANGE interrupts to both channel A and B.
// Uses a 2-bit state machine (Gray code lookup) to count ±1 per edge.
// Up to 4 encoders are supported (MAX_ENCODERS).
//
// Usage:
//   Encoder enc(ENC1_A, ENC1_B);
//   enc.begin(0);          // index 0-3
//   int32_t cnt = enc.getCount();
//   enc.resetCount();

constexpr uint8_t MAX_ENCODERS = 4;

class Encoder {
public:
    Encoder(uint8_t pinA, uint8_t pinB);

    // index: 0 to MAX_ENCODERS-1
    void begin(uint8_t index);

    // Thread-safe (disables interrupts briefly)
    int32_t getCount() const;
    void    resetCount();

    // Called internally by ISRs - do not call directly
    void _onEdge();

private:
    uint8_t          _pinA;
    uint8_t          _pinB;
    volatile int32_t _count;
    volatile uint8_t _prevAB;

    // Gray-code quadrature decode table [prevAB << 2 | currAB] -> delta
    static const int8_t _qTable[16];

    // Static ISR dispatch
    static Encoder* _instances[MAX_ENCODERS];
    static void _isr0();
    static void _isr1();
    static void _isr2();
    static void _isr3();
};
