
#include <fonts.h>


//#################################
// Fonts used by graphicsFunctions
//#################################

const uint8_t missingGlyph[7] PROGMEM = {
  0b0000000,
  0b0111110,
  0b0111110,
  0b0111110,
  0b0111110,
  0b0111110,
  0b0111110,
};
const uint8_t GLYPH_P[7] PROGMEM = {
  0b0000000,
  0b0111100,
  0b0100010,
  0b0111100,
  0b0100000,
  0b0100000,
  0b0100000,
};
const uint8_t GLYPH_I[7] PROGMEM = {
  0b0000000,
  0b0111110,
  0b0001000,
  0b0001000,
  0b0001000,
  0b0001000,
  0b0111110,
};
const uint8_t GLYPH_N[7] PROGMEM = {
  0b0000000,
  0b0100010,
  0b0110010,
  0b0101010,
  0b0100110,
  0b0100010,
  0b0100010,
};
const uint8_t GLYPH_E[7] PROGMEM = {
  0b0000000,
  0b0111110,
  0b0100000,
  0b0111000,
  0b0100000,
  0b0100000,
  0b0111110,
};
const uint8_t GLYPH_C[7] PROGMEM = {
  0b0000000,
  0b0011110,
  0b0100000,
  0b0100000,
  0b0100000,
  0b0100000,
  0b0011110,
};
const uint8_t GLYPH_R[7] PROGMEM = {
  0b0000000,
  0b0111100,
  0b0100010,
  0b0111100,
  0b0101100,
  0b0100010,
  0b0100010,
};
const uint8_t GLYPH_S[7] PROGMEM = {
  0b0000000,
  0b0011110,
  0b0100000,
  0b0011100,
  0b0000010,
  0b0000010,
  0b0111100,
};
const uint8_t GLYPH_T[7] PROGMEM = {
  0b0000000,
  0b0111110,
  0b0001000,
  0b0001000,
  0b0001000,
  0b0001000,
  0b0001000,
};
const uint8_t GLYPH_0[7] PROGMEM = {
  0b0000000,
  0b0011100,
  0b0100010,
  0b0100010,
  0b0100010,
  0b0100010,
  0b0011100,
};
const uint8_t GLYPH_1[7] PROGMEM = {
  0b0000000,
  0b0011000,
  0b0101000,
  0b0001000,
  0b0001000,
  0b0001000,
  0b0011100,
};
const uint8_t GLYPH_2[7] PROGMEM = {
  0b0000000,
  0b0011100,
  0b0000010,
  0b0000100,
  0b0001000,
  0b0010000,
  0b0111100,
};
const uint8_t GLYPH_3[7] PROGMEM = {
  0b0000000,
  0b0011100,
  0b0000010,
  0b0001110,
  0b0000010,
  0b0000010,
  0b0011100,
};
const uint8_t GLYPH_4[7] PROGMEM = {
  0b0000000,
  0b0100010,
  0b0100010,
  0b0111110,
  0b0000010,
  0b0000010,
  0b0000010,
};
const uint8_t GLYPH_5[7] PROGMEM = {
  0b0000000,
  0b0111110,
  0b0100000,
  0b0011100,
  0b0000010,
  0b0000010,
  0b0111100,
};
const uint8_t GLYPH_6[7] PROGMEM = {
  0b0000000,
  0b0011000,
  0b0100000,
  0b0111100,
  0b0100010,
  0b0100010,
  0b0011100,
};
const uint8_t GLYPH_7[7] PROGMEM = {
  0b0000000,
  0b0111110,
  0b0000010,
  0b0000100,
  0b0001000,
  0b0010000,
  0b0100000,
};
const uint8_t GLYPH_8[7] PROGMEM = {
  0b0000000,
  0b0111110,
  0b0100010,
  0b0111110,
  0b0100010,
  0b0100010,
  0b0111110,
};
const uint8_t GLYPH_9[7] PROGMEM = {
  0b0000000,
  0b0111110,
  0b0100010,
  0b0111110,
  0b0000010,
  0b0000010,
  0b0000010,
};

// 7x7 packed for all 95 printable ASCII characters.  If we have not populated a given
// character, we use nullptr and getGlyph subs in a not-available symbol (Solid box)
const uint8_t* font7x7[95] = {
    nullptr,       // space
    nullptr,       // !
    nullptr,       // "
    nullptr,       // #
    nullptr,       // $
    nullptr,       // %
    nullptr,       // &
    nullptr,       // '
    nullptr,       // (
    nullptr,       // )
    nullptr,       // *
    nullptr,       // +
    nullptr,       // ,
    nullptr,       // -
    nullptr,       // .
    nullptr,       // /
    GLYPH_0,       // 0
    GLYPH_1,       // 1
    GLYPH_2,       // 2
    GLYPH_3,       // 3
    GLYPH_4,       // 4
    GLYPH_5,       // 5
    GLYPH_6,       // 6
    GLYPH_7,       // 7
    GLYPH_8,       // 8
    GLYPH_9,       // 9
    nullptr,       // :
    nullptr,       // ;
    nullptr,       // <
    nullptr,       // =
    nullptr,       // >
    nullptr,       // ?
    nullptr,       // @
    nullptr,       // A
    nullptr,       // B
    GLYPH_C,       // C
    nullptr,       // D
    GLYPH_E,       // E
    nullptr,       // F
    nullptr,       // G
    nullptr,       // H
    GLYPH_I,       // I
    nullptr,       // J
    nullptr,       // K
    nullptr,       // L
    nullptr,       // M
    GLYPH_N,       // N
    nullptr,       // O
    GLYPH_P,       // P
    nullptr,       // Q
    GLYPH_R,       // R
    GLYPH_S,       // S
    GLYPH_T,       // T
    nullptr,       // U
    nullptr,       // V
    nullptr,       // W
    nullptr,       // X
    nullptr,       // Y
    nullptr,       // Z
    nullptr,       // [
    nullptr,       // \
    nullptr,       // ]
    nullptr,       // ^
    nullptr,       // _
    nullptr,       // `
    nullptr,       // a
    nullptr,       // b
    nullptr,       // c
    nullptr,       // d
    nullptr,       // e
    nullptr,       // f
    nullptr,       // g
    nullptr,       // h
    nullptr,       // i
    nullptr,       // j
    nullptr,       // k
    nullptr,       // l
    nullptr,       // m
    nullptr,       // n
    nullptr,       // o
    nullptr,       // p
    nullptr,       // q
    nullptr,       // r
    nullptr,       // s
    nullptr,       // t
    nullptr,       // u
    nullptr,       // v
    nullptr,       // w
    nullptr,       // x
    nullptr,       // y
    nullptr,       // z
    nullptr,       // {
    nullptr,       // |
    nullptr,       // }
};


// Function to read the glyph table and return a pointer to the specified character
// We pass the ascii index (i.e. for A that would be 65)
// If a character wasn't defined, return a solid-1 bitmap to indicate error
const uint8_t* getGlyph(char c) {
    if(c < 32 || c > 126)
        return missingGlyph;

    const uint8_t* glyph = font7x7[c - 32];

    if(glyph == nullptr)
        return missingGlyph;

    return glyph;
}