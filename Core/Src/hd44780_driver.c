#include "hd44780_driver.h"

int RussianFont(uint8_t RusWord){
	switch (RusWord){
		case 0xC0:		//�
			RusWord=0x41;
			break;
		case 0xC1:		//�
			RusWord=0xA0;
			break;
		case 0xC2:		//�
			RusWord=0x42;
			break;
		case 0xC3:		//�
			RusWord=0xA1;
			break;
		case 0xC4:		//�
			RusWord=0xE0;
			break;
		case 0xC5:		//E
			RusWord=0x45;
			break;
		case 0xA8:		//�
			RusWord=0xA2;
			break;
		case 0xC6:		//�
			RusWord=0xA3;
			break;
		case 0xC7:		//3
			RusWord=0xA4;
			break;
		case 0xC8:		//�
			RusWord=0xA5;
			break;
		case 0xC9:		//�
			RusWord=0xA6;
			break;
		case 0xCA:		//�
			RusWord=0x4B;
			break;
		case 0xCB:		//�
			RusWord=0xA7;
			break;
		case 0xCC:		//�
			RusWord=0x4D;
			break;
		case 0xCD:		//H
			RusWord=0x48;
			break;
		case 0xCE:		//O
			RusWord=0x4F;
			break;
		case 0xCF:		//�
			RusWord=0xA8;
			break;
		case 0xD0:		//P
			RusWord=0x50;
			break;
		case 0xD1:		//C
			RusWord=0x43;
			break;
		case 0xD2:		//�
			RusWord=0x54;
			break;
		case 0xD3:		//�
			RusWord=0xA9;
			break;
		case 0xD4:		//�
			RusWord=0xAA;
			break;
		case 0xD5:		//X
			RusWord=0x58;
			break;
		case 0xD6:		//�
			RusWord=0xE1;
			break;
		case 0xD7:		//�
			RusWord=0xAB;
			break;
		case 0xD8:		//�
			RusWord=0xAC;
			break;
		case 0xD9:		//�
			RusWord=0xE2;
			break;
		case 0xDA:		//�
			RusWord=0xAD;
			break;
		case 0xDB:		//�
			RusWord=0xAE;
			break;
		case 0xDC:		//�
			RusWord=0x62;
			break;
		case 0xDD:		//�
			RusWord=0xAF;
			break;
		case 0xDE:		//�
			RusWord=0xB0;
			break;
		case 0xDF:		//�
			RusWord=0xB1;
			break;

		case 0xE0:		//�
			RusWord=0x61;
			break;
		case 0xE1:		//�
			RusWord=0xB2;
			break;
		case 0xE2:		//�
			RusWord=0xB3;
			break;
		case 0xE3:		//�
			RusWord=0xB4;
			break;
		case 0xE4:		//�
			RusWord=0xE3;
			break;
		case 0xE5:		//�
			RusWord=0x65;
			break;
		case 0xB8:		//�
			RusWord=0xA2;
			break;
		case 0xE6:		//�
			RusWord=0xB6;
			break;
		case 0xE7:		//�
			RusWord=0xB7;
			break;
		case 0xE8:		//�
			RusWord=0xB8;
			break;
		case 0xE9:		//�
			RusWord=0xB9;
			break;
		case 0xEA:		//�
			RusWord=0xBA;
			break;
		case 0xEB:		//�
			RusWord=0xBB;
			break;
		case 0xEC:		//�
			RusWord=0xBC;
			break;
		case 0xED:		//�
			RusWord=0xBD;
			break;
		case 0xEE:		//o
			RusWord=0x6F;
			break;
		case 0xEF:		//�
			RusWord=0xBE;
			break;
		case 0xF0:		//�
			RusWord=0x70;
			break;
		case 0xf1:		//c
			RusWord=0x63;
			break;
		case 0xf2:		//�
			RusWord=0xBF;
			break;
		case 0xf3:		//�
			RusWord=0x79;
			break;
		case 0xf4:		//�
			RusWord=0xE4;
			break;
		case 0xf5:		//x
			RusWord=0x78;
			break;
		case 0xf6:		//�
			RusWord=0xE5;
			break;
		case 0xf7:		//�
			RusWord=0xC0;
			break;
		case 0xf8:		//�
			RusWord=0xC1;
			break;
		case 0xf9:		//�
			RusWord=0xE6;
			break;
		case 0xfa:		//�
			RusWord=0xC2;
			break;
		case 0xfb:		//�
			RusWord=0xC3;
			break;
		case 0xfc:		//�
			RusWord=0xC4;
			break;
		case 0xfd:		//�
			RusWord=0xC5;
			break;
		case 0xfe:		//�
			RusWord=0xC6;
			break;
		case 0xff:		//�
			RusWord=0xC7;
			break;
	}
	return RusWord;

}


void lcd_delay(void) {
	const uint32_t period = HAL_RCC_GetSysClockFreq() / 10000; // 100us = 1/10000sec

	volatile uint32_t delay;
	for (delay = period; delay > 0; delay--);
//	volatile uint32_t tmpvar;
//	for (tmpvar=10000;tmpvar!=0;tmpvar--);	//4000
}




void lcd_init() {
	LCD_PORT->CRH |= LCD_PORT_CRH_S;
	LCD_PORT->CRL |= LCD_PORT_CRL_S;
	LCD_PORT->CRH &= ~(LCD_PORT_CRH_C);
	LCD_PORT->CRL &= ~(LCD_PORT_CRL_C);
	lcd_set_4bit_mode();
	lcd_set_state(LCD_ENABLE,CURSOR_DISABLE,NO_BLINK);
	lcd_clear();
	lcd_send(0x06,COMMAND);
}

void lcd_set_user_char(uint8_t char_num, uint8_t * char_data) {
	uint8_t i;
	lcd_send(((1<<6) | (char_num * 8) ), COMMAND);
	for (i=0;i<=7;i++) {
		lcd_send(char_data[i],DATA);
	}
	lcd_send((1<<7), COMMAND);
}

void lcd_set_xy(uint8_t x, uint8_t y)  {
	if (y==0) {
		lcd_send( ((1<<7) | (x)),COMMAND);
	} else {
		lcd_send( ((3<<6) | (x)),COMMAND);
	}
}


void lcd_out(char * txt) {
	while(*txt) {
		lcd_send(*txt,DATA);
		txt++;
	}
}

void lcd_clear(void) {
	lcd_send(0x01,COMMAND);
}

void lcd_set_state(lcd_state state, cursor_state cur_state, cursor_mode cur_mode)  {
	if (state==LCD_DISABLE)  {
		lcd_send(0x08,COMMAND);
	} else {
		if (cur_state==CURSOR_DISABLE) {
			if (cur_mode==NO_BLINK)  {
				lcd_send(0x0C,COMMAND);
			} else {
				lcd_send(0x0D,COMMAND);
			}
		} else  {
			if (cur_mode==NO_BLINK)  {
				lcd_send(0x0E,COMMAND);
			} else {
				lcd_send(0x0F,COMMAND);
			}
		}
	}
}

void lcd_set_4bit_mode(void) {
	lcd_delay();
	lcd_delay();
	lcd_delay();

	LCD_PORT->BSRR=(LCD_DB7_BC | LCD_DB6_BC | LCD_DB5_BC | LCD_DB4_BC | LCD_CD_BC | LCD_EN_BC);
	LCD_PORT->BSRR=(LCD_DB5_BS|LCD_DB4_BS);

	LCD_PORT->BSRR=LCD_EN_BS;
	lcd_delay();
	LCD_PORT->BSRR=LCD_EN_BC;
	lcd_delay();
	lcd_delay();
	lcd_delay();
	
	LCD_PORT->BSRR=(LCD_DB7_BC | LCD_DB6_BC | LCD_DB5_BC | LCD_DB4_BC | LCD_CD_BC | LCD_EN_BC);
	LCD_PORT->BSRR=(LCD_DB5_BS|LCD_DB4_BS);

	LCD_PORT->BSRR=LCD_EN_BS;
	lcd_delay();
	LCD_PORT->BSRR=LCD_EN_BC;
	lcd_delay();
	lcd_delay();
	
	
	LCD_PORT->BSRR=(LCD_DB7_BC | LCD_DB6_BC | LCD_DB5_BC | LCD_DB4_BC | LCD_CD_BC | LCD_EN_BC);
	LCD_PORT->BSRR=(LCD_DB5_BS|LCD_DB4_BS);

	LCD_PORT->BSRR=LCD_EN_BS;
	lcd_delay();
	LCD_PORT->BSRR=LCD_EN_BC;
	lcd_delay();

	LCD_PORT->BSRR=(LCD_DB7_BC | LCD_DB6_BC | LCD_DB5_BC | LCD_DB4_BC | LCD_CD_BC | LCD_EN_BC);
	LCD_PORT->BSRR=(LCD_DB5_BS);

	LCD_PORT->BSRR=LCD_EN_BS;
	lcd_delay();
	LCD_PORT->BSRR=LCD_EN_BC;
	lcd_delay();

	LCD_PORT->BSRR=(LCD_DB7_BC | LCD_DB6_BC | LCD_DB5_BC | LCD_DB4_BC | LCD_CD_BC | LCD_EN_BC);
	LCD_PORT->BSRR=(LCD_DB5_BS);

	LCD_PORT->BSRR=LCD_EN_BS;
	lcd_delay();
	LCD_PORT->BSRR=LCD_EN_BC;
	lcd_delay();

	LCD_PORT->BSRR=(LCD_DB7_BC | LCD_DB6_BC | LCD_DB5_BC | LCD_DB4_BC | LCD_CD_BC | LCD_EN_BC);
	LCD_PORT->BSRR=(LCD_DB7_BS);

	LCD_PORT->BSRR=LCD_EN_BS;
	lcd_delay();
	LCD_PORT->BSRR=LCD_EN_BC;
	lcd_delay();

}

void lcd_send(uint8_t byte, dat_or_comm dc)  {

	if(dc)
	{
		if (byte == '\r')
			return;
		if (byte == '\n')
		{
			lcd_set_xy(0, 1);
			return;
		}
	}

	LCD_PORT->BSRR=(LCD_DB7_BC | LCD_DB6_BC | LCD_DB5_BC | LCD_DB4_BC | LCD_CD_BC | LCD_EN_BC);

	if (dc) {
		LCD_PORT->BSRR=LCD_CD_BS;
		if(byte>0xA0){
			byte=RussianFont(byte);
		}

	}

	if (byte & 0x10) {
		LCD_PORT->BSRR=LCD_DB4_BS;
	}
	if (byte & 0x20) {
		LCD_PORT->BSRR=LCD_DB5_BS;
	}
	if (byte & 0x40) {
		LCD_PORT->BSRR=LCD_DB6_BS;
	}
	if (byte & 0x80) {
		LCD_PORT->BSRR=LCD_DB7_BS;
	}

	LCD_PORT->BSRR=LCD_EN_BS;
	lcd_delay();
	LCD_PORT->BSRR=LCD_EN_BC;
	lcd_delay();


	LCD_PORT->BSRR=(LCD_DB7_BC | LCD_DB6_BC | LCD_DB5_BC | LCD_DB4_BC );

	if (byte & 0x01) {
		LCD_PORT->BSRR=LCD_DB4_BS;
	}
	if (byte & 0x02) {
		LCD_PORT->BSRR=LCD_DB5_BS;
	}
	if (byte & 0x04) {
		LCD_PORT->BSRR=LCD_DB6_BS;
	}
	if (byte & 0x08) {
		LCD_PORT->BSRR=LCD_DB7_BS;
	}

	LCD_PORT->BSRR=LCD_EN_BS;
	lcd_delay();
	LCD_PORT->BSRR=LCD_EN_BC;
	lcd_delay();


}
