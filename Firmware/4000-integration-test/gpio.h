#ifndef GPIO_H
#define GPIO_H
#include <Arduino.h>

// GPIO Pin Definitions

#define chA 26
#define chB 27
#define chC 25

#define btnRec 21
#define ledRec 24

#define sevenSegCLK 18
#define sevenSegDAT 19
#define sevenSegCS 20

// Function Prototypes
void gpio_initialise(void);
void disable_linear_encoder(void);
uint8_t readEncoderState(void);




#endif