#include "Encoder.h"

// Gray-code quadrature decode table
// Index = (prevAB << 2) | currAB
// Value = +1 (CW), -1 (CCW), 0 (no change or error)
const int8_t Encoder::_qTable[16] = {
//  currAB: 00   01   10   11
     0,   +1,  -1,   0,   // prevAB: 00
    -1,    0,   0,  +1,   // prevAB: 01
    +1,    0,   0,  -1,   // prevAB: 10
     0,   -1,  +1,   0,   // prevAB: 11
};

Encoder* Encoder::_instances[MAX_ENCODERS] = { nullptr, nullptr, nullptr, nullptr };

Encoder::Encoder(uint8_t pinA, uint8_t pinB)
    : _pinA(pinA), _pinB(pinB), _count(0), _prevAB(0)
{}

void Encoder::begin(uint8_t index) {
    if (index >= MAX_ENCODERS) return;
    _instances[index] = this;

    pinMode(_pinA, INPUT_PULLUP);
    pinMode(_pinB, INPUT_PULLUP);

    // Read initial state after pull-ups settle
    delayMicroseconds(10);
    _prevAB = ((digitalRead(_pinA) << 1) | digitalRead(_pinB)) & 0x03;

    // Attach ISR for both channels
    static void (*isrs[MAX_ENCODERS])() = { _isr0, _isr1, _isr2, _isr3 };
    attachInterrupt(digitalPinToInterrupt(_pinA), isrs[index], CHANGE);
    attachInterrupt(digitalPinToInterrupt(_pinB), isrs[index], CHANGE);
}

void Encoder::_onEdge() {
    uint8_t currAB = ((digitalRead(_pinA) << 1) | digitalRead(_pinB)) & 0x03;
    _count += _qTable[(_prevAB << 2) | currAB];
    _prevAB = currAB;
}

int32_t Encoder::getCount() const {
    noInterrupts();
    int32_t c = _count;
    interrupts();
    return c;
}

void Encoder::resetCount() {
    noInterrupts();
    _count = 0;
    interrupts();
}

// Static ISR dispatch
void Encoder::_isr0() { if (_instances[0]) _instances[0]->_onEdge(); }
void Encoder::_isr1() { if (_instances[1]) _instances[1]->_onEdge(); }
void Encoder::_isr2() { if (_instances[2]) _instances[2]->_onEdge(); }
void Encoder::_isr3() { if (_instances[3]) _instances[3]->_onEdge(); }
