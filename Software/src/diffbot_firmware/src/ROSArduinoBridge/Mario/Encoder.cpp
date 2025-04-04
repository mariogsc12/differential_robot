#include "Arduino.h"
#include "Encoder.hpp"

Encoder::Encoder(const int pinA, const int pinB,const int PPR):
    Pin_A(pinA),
    Pin_B(pinB),
    Pulses_Per_Revolution(PPR),
    pulses(0)
{
}

void Encoder::initialize()
{
  pinMode(Pin_A, INPUT);
  pinMode(Pin_B, INPUT);

  // Attach interrupts (CHANGE, RISING, FALLING, etc. depending on your needs)
  attachInterrupt(digitalPinToInterrupt(Pin_A), CHANGE);
  attachInterrupt(digitalPinToInterrupt(Pin_B), CHANGE);
}

long Encoder::getPulses()
{
	return Encoder::pulses;
}

void Encoder::setPulses(long pulses_)
{
	Encoder::pulses = pulses_;
}

void Encoder::count1()
{
  Encoder::pulses += ((digitalRead(Pin_A)<<1) - 1) * ((digitalRead(Pin_B)<<1) - 1); 
}

void Encoder::count2()
{
  Encoder::pulses -= ((digitalRead(Pin_A)<<1) - 1) * ((digitalRead(Pin_B)<<1) - 1); 
}

int Encoder::getPPR() const
{
    return Encoder::Pulses_Per_Revolution;
}
