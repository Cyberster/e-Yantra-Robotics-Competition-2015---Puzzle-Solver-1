

/***************************************************************************************************
                             Revision History
 ****************************************************************************************************
15.0: Initial version 
 ***************************************************************************************************/
#ifndef _GLCD_H_
#define _GLCD_H_

#include <avr\io.h>
#include "stdutils.h"



/***************************************************************************************************
                                 GLCD PORT Configuration
 ***************************************************************************************************/
#define M_GlcdDataBus PORTJ
#define M_GlcdDataBusDirection DDRJ
#define M_GlcdDataBusInput PINJ

#define M_GlcdControlBus PORTD
#define M_GlcdControlBusDirection DDRD
#define M_GlcdControlBus1 PORTL
#define M_GlcdControlBusDirection1 DDRL

#define GLCD_RS 5
#define GLCD_RW 6
#define GLCD_EN 7
#define GLCD_CS1 0
#define GLCD_CS2 1
#define GLCD_D7 7


#define M_GlcdClearBit(y,x)   util_BitClear((y),(x))
#define M_GlcdSetBit(y,x)     util_BitSet((y),(x))

/**************************************************************************************************/










/***************************************************************************************************
                 PreCompile configurations to enable/disable the functions
 ***************************************************************************************************
PreCompile configuration to enable or disable the API's.
 1.Required interfaces can be enabled/disabled by configuring its respective macros to 1/0.
 2. By default all the API's are enabled except for FLOAT display.
 3. Displaying of floating number takes huge controller resources and need to be enabled only 
    if required. This implies for other interfaces as well. 
 ***************************************************************************************************/
#define    Enable_GLCD_DisplayString          1
#define    Enable_GLCD_DisplayDecimalNumber   1
#define    Enable_GLCD_DisplayHexNumber       1
#define    Enable_GLCD_DisplayBinaryNumber    1
#define    Enable_GLCD_DisplayFloatNumber     1
#define    Enable_GLCD_Printf                 1
#define    Enable_GLCD_DisplayLogo            1
/**************************************************************************************************/





/***************************************************************************************************
                             Commonly used LCD macros/Constants
***************************************************************************************************/
#define BlankSpace ' '

#define C_GlcdDisplayDefaultDigits_U8            0xffu // Will display the exact digits in the number
#define C_GlcdMaxDigitsToDisplay_U8              10u   // Max decimal/hexadecimal digits to be displayed
#define C_GlcdNumOfBinDigitsToDisplay_U8         16u   // Max bits of a binary number to be displayed
#define C_GlcdMaxDigitsToDisplayUsingPrintf_U8   C_GlcdDisplayDefaultDigits_U8 /* Max dec/hexadecimal digits to be displayed using printf */


#define C_GlcdFirstLine_U8 0x00u
#define C_GlcdLastLine_U8 0x07u
#define C_FirstLineNumberAddress_U8 0xB8
#define C_LastLineNumberAddress_U8  0xBF


#define C_MaxBarGraphs_U8 0x04
#define C_LookUpOffset_U8 0x20
/**************************************************************************************************/





/***************************************************************************************************
                                 Struct/Enums used
 ***************************************************************************************************/
typedef struct{
	uint8_t PageNum;
	uint8_t LineNum;
	uint8_t CursorPos;
	uint8_t Invertdisplay;
}GLCD_Config;
/**************************************************************************************************/





/***************************************************************************************************
                             Function Prototypes
 ***************************************************************************************************/
void GLCD_Init();
void GLCD_Clear();
void GLCD_SetCursor(uint8_t pageNumber,uint8_t lineNumber,uint8_t CursorPosition);
void GLCD_GetCursor(uint8_t *page_ptr,uint8_t *line_ptr,uint8_t *cursor_ptr);
void GLCD_GoToPage(uint8_t pageNumber);
void GLCD_GoToLine(uint8_t var_lineNumber_u8);
void GLCD_GoToNextLine();
void GLCD_EnableDisplayInversion();
void GLCD_DisableDisplayInversion();
void GLCD_DisplayChar(uint8_t var_lcdData_u8);
void GLCD_DisplayString(char *ptr_stringPointer_u8);
void GLCD_ScrollMessage(uint8_t var_lineNumber_u8, char *ptr_msgPointer_u8);
void GLCD_DisplayDecimalNumber(uint32_t var_DecNumber_u32, uint8_t var_numOfDigitsToDisplay_u8);
void GLCD_DisplayHexNumber(uint32_t var_hexNumber_u32,uint8_t var_numOfDigitsToDisplay_u8);
void GLCD_DisplayBinaryNumber(uint32_t var_binNumber_u32, uint8_t var_numOfBitsToDisplay_u8);
void GLCD_DisplayFloatNumber(double var_floatNum_f32);
void GLCD_Printf(const char *argList, ...);
void GLCD_DisplayLogo();
void GLCD_DisplayVerticalGraph(uint8_t var_barGraphNumber_u8, uint8_t var_percentageValue_u8);
void GLCD_DisplayHorizontalGraph(uint8_t var_barGraphNumber_u8, uint8_t var_percentageValue_u8);
/**************************************************************************************************/
#endif
