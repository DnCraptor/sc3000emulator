// Color table
// <-MSB
// G2 G1 G0 R2 R1 R0 B1 B0   

const uint8_t colors[]={
    0b00000000  ,  // #000000 Black (Transparent)
    0b00000000  ,  // #000000 Black 
    0b10100000  ,  // #00AC00 medium green 
    0b01110010  ,  // #24DB55 light green
    0b10010011  ,  // #2424FF dark blue
    0b01001011  ,  // #4949FF light blue
    0b10010100  ,  // #AC2400 dark red 
    0b01110011  ,  // #24DBFF cyan
    0b10011100  ,  // #FF2400 medium red
    0b11011110  ,  // #FF6D55 light red
    0b10110100  ,  // #ACAC00 dark yellow
    0b10101100  ,  // #DBAC55 light yellow
    0b00100000  ,  // #009200 dark green
    0b10010101  ,  // #AC24AA magenta
    0b10110111  ,  // #ACACAA gray
    0b11111111     // #FFFFFF White
};
