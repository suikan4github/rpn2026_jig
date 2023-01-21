/*
 * calc.cpp
 *
 *  Created on: Jan 8, 2022
 *      Author: takemasa
 */

#define ENABLE_DEBUG_PRINT 0

#include "main.h"

#if ENABLE_DEBUG_PRINT
#include <cstring>
#include <stdio.h>
#endif
#include <rpnengine.hpp>
#include "calcparam.hpp"

extern SPI_HandleTypeDef hspi1;			// Defined by the CubeMX.

static const int kDigitsNum = 9;				// Number of digits in the VFD.
static const int kDigitsSize = 4;				// Number of the bytes for the MAX6921 data

static const int kRowCount = 9;					// size of row in the key matrix
static const int kColCount = 3;					// size of col in the key matrix
static const int kChatteringThreashold = 10;	// how many continuous input is needed to change the state

/******************************************************************
 * MAX6934 output signal mapping
 * OUT | bit | Description
 * ---------------------------------------------------------
 *   0 |  31 | D8
 *   1 |  30 | Seg_g
 *   2 |  29 | D7
 *   3 |  28 | Seg_f
 *   4 |  27 | D6
 *   5 |  26 | Seg_e
 *   6 |  25 | D5
 *   7 |  24 | D4
 *   8 |  23 | Seg_d
 *   9 |  22 | D3
 *  10 |  21 | DP
 *  11 |  20 | D2
 *  12 |  19 | Seg_c
 *  13 |  18 | D1
 *  14 |  17 | Seg_b
 *  15 |  16 | D0
 *  16 |  15 | Seg_a
 *
 *  The MAX6934 shift register requires the microprocessor send from
 *  the data of the OUT31 signal. That mean, it is MSB first.
 *  Assume we use 32bit word ( 4bytes ) to store the shift register
 *  image. In the little endian architecture,there is no simple way
 *  to send the data at once.
 *  Thus, we store the data to the up side down of the bit order.
 *  In the other word, OUT0-31 will be stored in the bit31-0. In this
 *  way, we can transmit four byte at once, if we set the SPI as
 *  LSB first.
 ******************************************************************/

#define D8_POS 31
#define D7_POS 29
#define D6_POS 27
#define D5_POS 25
#define D4_POS 24
#define D3_POS 22
#define D2_POS 20
#define D1_POS 18
#define D0_POS 16

#define SEG_A_POS 15
#define SEG_B_POS 17
#define SEG_C_POS 19
#define SEG_D_POS 23
#define SEG_E_POS 26
#define SEG_F_POS 28
#define SEG_G_POS 30
#define SEG_DP_POS 21

// Aggregate the calculator component object.
namespace calc {
	rpn_engine::Console *console;
	rpn_engine::AntiChattering *anti_chatter[kRowCount][kColCount];	// We need AntiChattering object for each key matrix position.
	rpn_engine::SegmentDecoder *segment_decoder;
}


// This function is call from the AntiChattering class, if the
// Key is surely pressed after anti-chattering process.
// row, col are physical key matrix position( zero origin ).
static void KeyPerssedCallBackFunction(unsigned row, unsigned col)
{
	bool is_func_key_pressed = calc::console->GetIsFuncKeyPressed();
	bool is_hex_mode = calc::console->GetIsHexMode();

	// Convert the position of key to the object.
	rpn_engine::Op opcode =  rpn_engine::EncodeKey(row, col, is_func_key_pressed, is_hex_mode);

	// And then, pass the opcode to the Console object.
	calc::console->Input(opcode);

}

// Convert 8bit segment data to the 32bit shift register pattern.
static uint32_t Segment2Shiftreg(uint8_t segment)
{
	uint32_t value = 0;
	if ( segment & (1<<0))
		value |= 1 << SEG_A_POS;
	if ( segment & (1<<1))
		value |= 1 << SEG_B_POS;
	if ( segment & (1<<2))
		value |= 1 << SEG_C_POS;
	if ( segment & (1<<3))
		value |= 1 << SEG_D_POS;
	if ( segment & (1<<4))
		value |= 1 << SEG_E_POS;
	if ( segment & (1<<5))
		value |= 1 << SEG_F_POS;
	if ( segment & (1<<6))
		value |= 1 << SEG_G_POS;
	if ( segment & (1<<7))
		value |= 1 << SEG_DP_POS;

	return value;
}

// Set specific key matrix row to active
static void AssertKeyMatrixRow( uint8_t row )
{
	// Clear all row.
	// Because the line is open drain, the clear means set.
	if ( 0 == row)
		HAL_GPIO_WritePin(KR0_GPIO_Port, KR0_Pin, GPIO_PIN_RESET);
	if ( 1 == row)
		HAL_GPIO_WritePin(KR1_GPIO_Port, KR1_Pin, GPIO_PIN_RESET);
	if ( 2 == row)
		HAL_GPIO_WritePin(KR2_GPIO_Port, KR2_Pin, GPIO_PIN_RESET);
	if ( 3 == row)
		HAL_GPIO_WritePin(KR3_GPIO_Port, KR3_Pin, GPIO_PIN_RESET);
	if ( 4 == row)
		HAL_GPIO_WritePin(KR4_GPIO_Port, KR4_Pin, GPIO_PIN_RESET);
	if ( 5 == row)
		HAL_GPIO_WritePin(KR5_GPIO_Port, KR5_Pin, GPIO_PIN_RESET);
	if ( 6 == row)
		HAL_GPIO_WritePin(KR6_GPIO_Port, KR6_Pin, GPIO_PIN_RESET);
	if ( 7 == row)
		HAL_GPIO_WritePin(KR7_GPIO_Port, KR7_Pin, GPIO_PIN_RESET);
	if ( 8 == row)
		HAL_GPIO_WritePin(KR8_GPIO_Port, KR8_Pin, GPIO_PIN_RESET);
}

// Set specific key matrix row to inactive
static void DeassertKeyMatrixRow( uint8_t row )
{
	// Clear all row.
	// Because the line is open drain, the clear means set.
	if ( 0 == row)
		HAL_GPIO_WritePin(KR0_GPIO_Port, KR0_Pin, GPIO_PIN_SET);
	if ( 1 == row)
		HAL_GPIO_WritePin(KR1_GPIO_Port, KR1_Pin, GPIO_PIN_SET);
	if ( 2 == row)
		HAL_GPIO_WritePin(KR2_GPIO_Port, KR2_Pin, GPIO_PIN_SET);
	if ( 3 == row)
		HAL_GPIO_WritePin(KR3_GPIO_Port, KR3_Pin, GPIO_PIN_SET);
	if ( 4 == row)
		HAL_GPIO_WritePin(KR4_GPIO_Port, KR4_Pin, GPIO_PIN_SET);
	if ( 5 == row)
		HAL_GPIO_WritePin(KR5_GPIO_Port, KR5_Pin, GPIO_PIN_SET);
	if ( 6 == row)
		HAL_GPIO_WritePin(KR6_GPIO_Port, KR6_Pin, GPIO_PIN_SET);
	if ( 7 == row)
		HAL_GPIO_WritePin(KR7_GPIO_Port, KR7_Pin, GPIO_PIN_SET);
	if ( 8 == row)
		HAL_GPIO_WritePin(KR8_GPIO_Port, KR8_Pin, GPIO_PIN_SET);
}

// Execution entity of the calculator. Drive the VFD by the dynamic way. Scan the key and give it to Console object.
extern "C" void exec_calc()
{

	// VFD data with blank segment
	// This data contains.the segment drive signal for the MAX6921.
	uint32_t vfd_data[kDigitsNum];

	char old_text[10] = "00000000";		// For debug. Initialize the buffer to store the previous state.
	int old_decimal_point_position=7;	// For debug. Initialize the buffer to store the previous state.


	printf("\n\nHello!\n");

	// create calc::console object
   calc::console = new rpn_engine::Console(calc::splash_string);

	// initialize the anti chattering objects with threshold, callback and its position.
	for (unsigned int row=0; row<kRowCount; row++)
		for ( unsigned int col=0; col<kColCount; col++)
			calc::anti_chatter[row][col]= new rpn_engine::AntiChattering(
															kChatteringThreashold,
															kChatteringThreashold,
															KeyPerssedCallBackFunction,
															row,
															col);

	// hardware dependent segment mapping.
	calc::segment_decoder = new rpn_engine::SegmentDecoder(
			1<<0,
			1<<1,
			1<<2,
			1<<3,
			1<<4,
			1<<5,
			1<<6,
			1<<7
			);

	// Set Blank pin to "L" to show the display.
	HAL_GPIO_WritePin(BLANK_GPIO_Port, BLANK_Pin, GPIO_PIN_RESET);

	// Deassert all Key matrix row.
	for ( int i= 0; i<kRowCount; i++)
		DeassertKeyMatrixRow(i);

	while(1)
	{
		  char digits_str[kDigitsNum+1];	// with zero termination
		  int decimal_point_position;
		  uint32_t shiftreg;

		  // Scan all 9 digits
		  for ( int digit = 0; digit < kDigitsNum; digit ++)
		  {
			  // Assert the key matrix to forcus.
			  AssertKeyMatrixRow(digit);

			  // clean up the digit data
			  vfd_data[0] = 1<<D8_POS;	// digit 8,
			  vfd_data[1] = 1<<D7_POS,	// digit 7,
			  vfd_data[2] = 1<<D6_POS,	// digit 6,
			  vfd_data[3] = 1<<D5_POS,	// digit 5,
			  vfd_data[4] = 1<<D4_POS,	// digit 4,
			  vfd_data[5] = 1<<D3_POS,	// digit 3,
			  vfd_data[6] = 1<<D2_POS,	// digit 2,
			  vfd_data[7] = 1<<D1_POS,	// digit 1,
			  vfd_data[8] = 1<<D0_POS,	// digit 0,

			  // Obtain the text representative of the calculated result.
			  calc::console->GetText(digits_str);
			  // Obtain the decimal point position.
			  decimal_point_position = calc::console->GetDecimalPointPosition();	// decimal point 0 is right of the digit 0. ( Right most )

			  if (decimal_point_position != calc::console->kDecimalPointNotDisplayed)		// Do we need to display decimal point?
			  {
				  decimal_point_position = 8 - decimal_point_position; 						// Convert the DP position to the array index.
				  vfd_data[decimal_point_position] |= 1<< SEG_DP_POS;						// set decimal point.
			  }

			  uint8_t segment;								// segment pattern.

			  if  ( digit == 0 && calc::console->GetIsFuncKeyPressed()){	// Is func key pressed && left most digit?
				  segment = calc::segment_decoder->decode('_'); 				// So, let's display func key status sign.
				  vfd_data[digit] |= Segment2Shiftreg(segment);
			  }

			  segment=	calc::segment_decoder->decode(digits_str[digit]) ;	// Obtain the segment pattern of this digit.
			  vfd_data[digit] |= Segment2Shiftreg(segment);					// Set the appropriate bit according to the segment pattern.

			  // Transmit a digits data.
			  // This poling transmission define the duration of the display of a digit.
			  // To change the duration, change the clock configuration of SPI.
			  HAL_SPI_Transmit(&hspi1, (uint8_t*)&vfd_data[digit], kDigitsSize, 100);

			  // Assert load pulse
			  HAL_GPIO_WritePin(LOAD_GPIO_Port, LOAD_Pin, GPIO_PIN_SET);
			  // We assume there is enough pulse width to load.
			  // Deassert load pulse.
			  HAL_GPIO_WritePin(LOAD_GPIO_Port, LOAD_Pin, GPIO_PIN_RESET);

			  {
				  // Blank to adjust the brightness
				  HAL_GPIO_WritePin(BLANK_GPIO_Port, BLANK_Pin, GPIO_PIN_SET);
				  // Dummy transmission to give the settling time of key.
				  HAL_SPI_Transmit(&hspi1, (uint8_t *)&vfd_data[digit], kDigitsSize, 100);
				  // show display again
				  HAL_GPIO_WritePin(BLANK_GPIO_Port, BLANK_Pin, GPIO_PIN_RESET);
			  }

			  // read key state and store the state to the calc::anti_chatter[] objects.
			  // Anti chattering object will call the Console object, if needed.
			  calc::anti_chatter[digit][0]->Input(
					  ( HAL_GPIO_ReadPin(KC0_GPIO_Port, KC0_Pin) == GPIO_PIN_RESET)?
							  rpn_engine::KeyLevel::high :
							  rpn_engine::KeyLevel::low
			  );
			  calc::anti_chatter[digit][1]->Input(
					  ( HAL_GPIO_ReadPin(KC1_GPIO_Port, KC1_Pin) == GPIO_PIN_RESET)?
							  rpn_engine::KeyLevel::high :
							  rpn_engine::KeyLevel::low
			  );
			  calc::anti_chatter[digit][2]->Input(
					  ( HAL_GPIO_ReadPin(KC2_GPIO_Port, KC2_Pin) == GPIO_PIN_RESET)?
							  rpn_engine::KeyLevel::high :
							  rpn_engine::KeyLevel::low
			  );

#if ENABLE_DEBUG_PRINT

			  // Is display data changed, print out the data for debug.
			  if (std::strcmp(digits_str,old_text) || (decimal_point_position != old_decimal_point_position))
			  {
				  std::strcpy(old_text,digits_str);						// save digits_str;
				  old_decimal_point_position = decimal_point_position;  // save decimal point position
				  char print_txt[12];

				  // For readability, we need to insert the decimal point to the appropriate position of number.
				  if (old_decimal_point_position == calc::console->kDecimalPointNotDisplayed)	// if decimal point is not displayed.
					  std::strcpy(print_txt, old_text); 												// copy the number as is
				  else														// if decimal point exist
				  {															// need to copy with decimal point
					  int dppos = old_decimal_point_position + 1;
					  for ( int i=0; i<dppos; i++)						// Copy the first half of the string.
						  print_txt[i] = old_text[i];
					  print_txt[dppos] = '.';							// Insert decimal point
					  for ( int i= dppos; i<9; i++)						// Copy the second half of the string
						  print_txt[i+1] = old_text[i];
					  print_txt[10] = 0;									// Terminate by null.
				  }
				  printf("%s\n", print_txt);
			  }
#endif

			  // Clear the asserted Row to move to the next row.
			  DeassertKeyMatrixRow(digit);
		  } // for

	}


}
