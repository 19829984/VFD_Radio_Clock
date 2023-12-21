#ifndef VFD_DIGITS_H
#define VFD_DIGITS_H
#include <inttypes.h>

const uint8_t NUM_VFD_DIGITS = 17;

const uint8_t vfd_digits[17] = {
    0b11111100, // 0 
    0b01100000, // 1
    0b11011010, // 2
    0b11110010, // 3
    0b01100110, // 4
    0b10110110, // 5
    0b10111110, // 6
    0b11100000, // 7
    0b11111110, // 8
    0b11110110, // 9
    0b11101110, // A
    0b00111110, // b
    0b10011100, // C
    0b01111010, // d
    0b10011110, // E
    0b10001110, // F
    0b00000001  // .
};

#endif