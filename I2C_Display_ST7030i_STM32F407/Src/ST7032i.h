#ifndef __ST7032I_H
#define __ST7032I_H

#include <stdio.h>
#include "main.h"

//#include "main.h"

#define LCD_CO_MORE_CONTROL_BYTE_OFF		0b00000000
#define LCD_CO_MORE_CONTROL_BYTE_ON			0b10000000

#define LCD_DATA_TYPE						0b01000000
#define LCD_INSTRUCTION_TYPE				0b00000000

/* Normal Mode */
#define LCD_CLEAR_DISPLAY 					0b00000001
#define LCD_RETURN_HOME 					0b00000010
#define LCD_ENTRY_MODE_SET 					0b00000100
#define LCD_DISPLAY_ON_OFF					0b00001000
#define LCD_CURSOR_OR_DISPLAY_SHIFT			0b00010000
#define LCD_FUNCTION_SET					0b00100000
#define LCD_SET_CGRAM_ADDRESS				0b01000000
#define LCD_SET_DDRAM_ADDRESS				0b10000000

/* LCD_ENTRY_MODE_SET */
#define LCD_ENTRY_MODE_SET_INCREMENT			0b00000010
#define LCD_ENTRY_MODE_SET_DECREMENT			0b00000000
#define LCD_ENTRY_MODE_SET_SHIFT_DISPLAY_ON		0b00000010
#define LCD_ENTRY_MODE_SET_SHIFT_DISPLAY_OFF	0b00000000

/* LCD_FUNCTION_SET */
#define LCD_FUNCTION_SET_DL_8BITS				0b00010000
#define LCD_FUNCTION_SET_DL_4BITS				0b00000000
#define LCD_FUNCTION_SET_N_2LINE				0b00001000
#define LCD_FUNCTION_SET_N_1LINE				0b00000000
#define LCD_FUNCTION_SET_DH_DOUBLE_HEIGHT_ON	0b00000100
#define LCD_FUNCTION_SET_DH_DOUBLE_HEIGHT_OFF	0b00000000

#define LCD_FUNCTION_SET_TEST_BIT_ON		0b00000010
#define LCD_FUNCTION_SET_TEST_BIT_OFF		0b00000000

#define LCD_FUNCTION_SET_EXTENSION_MODE		0b00000001
#define LCD_FUNCTION_SET_NORMAL_MODE		0b00000000


/* LCD_DISPLAY_ON_OFF */
#define LCD_DISPLAY_ON_BIT					0b00000100
#define LCD_DISPLAY_OFF_BIT					0b00000000
#define LCD_CURSOR_ON_BIT					0b00000010
#define LCD_CURSOR_OFF_BIT					0b00000000
#define LCD_CURSOR_POSITION_ON_BIT			0b00000001
#define LCD_CURSOR_POSITION_OFF_BIT			0b00000000

/* LCD_CURSOR_OR_DISPLAY_SHIFT */
#define LCD_DISPLAY_SHIFT						0b00001000
#define LCD_CURSOR_SHIFT						0b00000000
#define LCD_CURSOR_OR_DISPLAY_DIRECTION_RIGHT	0b00000100
#define LCD_CURSOR_OR_DISPLAY_DIRECTION_LEFT	0b00000000


/* LCD_SET_DDRAM_ADDRESS */
//#define LCD_SET_DDRAM_ADDRESS				0b10000000

#define LCD_EXT_MODE_IS0_CURSOR_OR_DISPLAY_SHIFT 		0b00010000
#define LCD_EXT_MODE_IS0_SET_CGRAM 						0b01000000

#define LCD_EXT_MODE_IS1_INTERNAL_OSC_FREQUENCY 		0b00010000
#define LCD_EXT_MODE_IS1_SEWT_ICON_ADDRESS	 			0b01000000
#define LCD_EXT_MODE_IS1_POWER_ICON_CONTRAST_SET		0b01010000
#define LCD_EXT_MODE_IS1_FOLLOW_CONTROL					0b01100000
#define LCD_EXT_MODE_IS1_CONTRAST_SET					0b01110000

/* Cursor or Display Shift  LCD_EXT_MODE_IS0_CURSOR_OR_DISPLAY_SHIFT*/

#define LCD_DIRECTION_OF_CURSOR  0
#define LCD_DIRECTION_OF_SCREEN  1

#define LCD_CURSOR_DIRECTION_RL_TO_RIGHT 1
#define LCD_CURSOR_DIRECTION_RL_TO_LEFT  0

/* LCD_EXT_MODE_IS1_INTERNAL_OSC_FREQUENCY */
#define LCD_EXT_MODE_IS1_INTERNAL_OSC_FREQUENCY_BS_1_4BIAS_ON	0b00001000
#define LCD_EXT_MODE_IS1_INTERNAL_OSC_FREQUENCY_BS_1_5BIAS_ON	0b00000000
#define LCD_EXT_MODE_IS1_INTERNAL_OSC_FREQUENCY_F2_ON			0b00000100
#define LCD_EXT_MODE_IS1_INTERNAL_OSC_FREQUENCY_F1_ON			0b00000010
#define LCD_EXT_MODE_IS1_INTERNAL_OSC_FREQUENCY_F0_ON			0b00000001


#define LCD_SYMBOLS 		21  // 21*6=126 (+2) =128
#define LCD_DOTS 			128
#define LCD_SYMBOL_WIDTH 	5
#define LCD_SPACE 			1
#define LCD_ADDR 0x7C //3E //(0x27 << 1)

#define LCD_SYMBOLS 21  // 21*6=126 (+2) =128
#define LCD_REVERSE_SYMBOLS_ON 1
#define LCD_REVERSE_SYMBOLS_OFF 0

#define LCD_REVERSE_SYMBOL_ON 1
#define LCD_REVERSE_SYMBOL_OFF 0

void before_LCD_print(uint8_t numString);
void LCD_print_String(uint8_t *str,uint8_t numString,uint8_t reverseSymbols,uint8_t reverseSymbol);
uint8_t LCD_print_String_prefix();
void LCD_clear(uint8_t numString);
void I2C_Scan(char *str,I2C_HandleTypeDef *pH);
void LCD_Init(I2C_HandleTypeDef *h);
void LCD_SendInternal(uint8_t data,uint8_t flags) ;
void LCD_SendCommands(uint8_t *buf,uint8_t size);
void add(uint8_t* buf,uint8_t p,uint8_t cmd,uint8_t val);
void LCD_SendCommand(uint8_t cmd);
void LCD_SendString(char *str);
void LCD_SendData(uint8_t data);
void LCD_SendDates(uint8_t* buf, uint8_t size);
void writeDR(uint8_t val);
void writeIR(uint8_t val);
void I2C1_DataTransfer(uint8_t *aTxBuffer,int TXBUFFERSIZE);
uint8_t readI2C();
void rst_LCD();
void LCD_enable(void);


#endif

