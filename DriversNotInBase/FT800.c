/*
 * FT800.c
 *
 * Created: 05.01.2015 13:43:47
 *  Author: will
 */ 

#include "FT800.h"
#include "spi.h"

#include <string.h>
unsigned short cmd_offset = 0;
unsigned short dli = 0;


/*
uint8_t spi_rd8() {
}
t16_t spi_rd16() {
}*/
//uint32_t tbuffer[3] = {0x00000004, 0x0000000A,0x01000004 };

void tr8last(uint8_t value) {
		
	uint32_t tbuffer[1] = { 0x01000000 |value};
	uint32_t rbuffer[1] = {0};
	spi_freeRTOSTranceive(tbuffer, 1, 0, rbuffer);
	
}
void tr8(uint8_t value) {
	
	uint32_t tbuffer[1] = { 0x00000000 |value};
	uint32_t rbuffer[1] = {0};
	spi_freeRTOSTranceive(tbuffer, 1, 0, rbuffer);
}
void tr16(uint16_t value) {
	uint32_t tbuffer[2] = { 0x00000000 | (value & 0x00FF), 0x01000000 | ((value >> 8) & 0x00FF)};
	uint32_t rbuffer[2];
	//tbuffer[0] = value;
	//tbuffer[1] = value >> 8;
	spi_freeRTOSTranceive(tbuffer, 2, 0,  rbuffer);
	/*
	tr8((value) & 0xFF);
	tr8(((value) >> 8) & 0xFF);*/
}

void tr32(uint32_t value) {
	uint32_t tbuffer[4] = {0x00000000 | (value & 0x00FF), 0x00000000 | ((value >> 8) & 0x00FF), 0x00000000 | ((value >> 16) & 0x00FF),  0x01000000 | ((value >> 24) & 0x00FF)};	
	uint32_t rbuffer[4];
	spi_freeRTOSTranceive(tbuffer, 4, 0 ,  rbuffer);
}

void host_command(ft_uint8_t command) {
	
	uint32_t tbuffer[3] = {command, 0x00, 0x01000000};
	uint32_t rbuffer[3];
	spi_freeRTOSTranceive(tbuffer, 3, 0, rbuffer);
	/*
	tr8(command);
	tr8(0x00);
	tr8(0x00);*/
}

void wr8(unsigned long addr, ft_uint8_t value) {
	
	
// 	uint32_t tbuffer[4] = {  (0x80 | (addr >> 16)) & 0xFF,  (addr >> 8) & 0xFF, addr & 0xFF, 0x01000000 | (value & 0xFF)};
// 	uint32_t rbuffer[4];
	uint32_t tbuffer[4] = {spi_word(false,0,(0x80 | (addr >> 16))), spi_word(false,0,(addr >> 8)), spi_word(false,0, addr),spi_word(true,0,value)};
	uint32_t rbuffer[4];
	spi_freeRTOSTranceive(tbuffer, 4, 0,  rbuffer);
	/*
	tr8(0x80 | (addr >> 16));
	tr8(addr >> 8);
	tr8(addr);
	tr8(value);
	*/
}

void wr16(unsigned long addr, ft_uint16_t value) {
	uint32_t tbuffer[5] = {(0x80 | (addr >> 16)) & 0xFF, (addr >> 8) & 0xFF, addr & 0xFF, value & 0xFF, 0x01000000 | ((value >> 8) & 0xFF)};
	uint32_t rbuffer[5];
	spi_freeRTOSTranceive(tbuffer, 5, 0,  rbuffer);
	/*
	tr8(0x80 | (addr >> 16));
	tr8(addr >> 8);
	tr8(addr);
	tr16(value);
	tr8((value) & 0xFF);
	tr8(((value) >> 8) & 0xFF);*/
}
void wr32(unsigned long addr, ft_uint32_t value) {
	uint32_t tbuffer[7] = {(0x80 | (addr >> 16)) & 0xFF,  (addr >> 8) & 0xFF, addr & 0xFF, value & 0xFF, ((value >> 8) & 0x00FF),((value >> 16) & 0xFF) , 0x01000000 | ((value >> 24) & 0x00FF) };
	uint32_t rbuffer[7];
	spi_freeRTOSTranceive(tbuffer, 7, 0,  rbuffer);
}

ft_uint8_t wr8s(unsigned long addr, const ft_char8_t *s) {	
	//Get length
	uint16_t counter = 0;
	ft_uint8_t i;
	ft_uint8_t l = strlen(s);
	for(i=0;i<=l;i++){
		counter += 1;
	}
	for(;i%4>0;i++){
		counter += 1;
	}
	uint32_t tbuffer[counter+3];
	uint32_t rbuffer[counter+3];
	tbuffer[0] = (0x80 | (addr >> 16)) & 0x00FF;
	tbuffer[1] = (addr >> 8) & 0x00FF;
	tbuffer[2] = addr & 0x00FF;
	for(i=0;i<=l;i++){
		tbuffer[i+3] = s[i];
	}
	for(;i%4>0;i++){
		tbuffer[i+3] = 0;
		
		/*if (i%4 <= 0) {
			tbuffer[i+3] =0x01000000;
		}
		else {
			tbuffer[i+3] = 0;
		}*/
	}
	tbuffer[counter +2 ] = 0x01000000;
	spi_freeRTOSTranceive(tbuffer, counter+3, 0,  rbuffer);
	return i;
}

void writeString(char *s, ft_uint8_t fontsize, uint8_t x, uint8_t y ){
	cmd(CMD_DLSTART);
	cmd(CLEAR(1, 1, 1)); // clear screen
	cmd(COLOR_RGB(255,255,255));
	cmd_text(x, y, fontsize, OPT_CENTER, s);
	cmd(DISPLAY()); // display the image
	cmd(CMD_SWAP);
	cmd_exec();
}

ft_uint8_t rd8(unsigned long addr) { 
	uint32_t tbuffer[5] = { (addr >> 16) & 0xFF,  (addr >> 8) & 0xFF, addr & 0xFF, 0,0x01000000 };
	uint32_t rbuffer[5] = {0};
	spi_freeRTOSTranceive(tbuffer, 5, 0,  rbuffer);
	return (rbuffer[4] & 0x00ff);
	//return (uint8_t)data;
}

ft_uint16_t rd16(unsigned long addr, uint32_t *rbuffer){
	uint32_t tbuffer[5];	
	tbuffer[0] = 0x80 | (addr >> 16);
	tbuffer[1] = addr >> 8;
	tbuffer[2] = addr;
	tbuffer[3] = 0;
	tbuffer[4] = 0; // LSB receive
	tbuffer[5] = 0; // MSB receive
	spi_freeRTOSTranceive(tbuffer, 6, 0,  rbuffer);
}

void cmd_incrementn(unsigned char n){
	cmd_offset=(cmd_offset+n)%4096;
}

void cmd(ft_uint32_t command){
	wr32(RAM_CMD + cmd_offset, command);cmd_increment4();
}

#define ACTIVE  0x00
#define STANDBY 0x41
#define SLEEP   0x42
#define PWRDOWN 0x50
#define CLKEXT  0x44
#define CLK48M  0x62
#define CLK36M  0x61
#define CORERST 0x68

void FT800_Init(void){
	//Initialize the screen controller

	
	host_command(ACTIVE); //send host command "ACTIVE" to FT800
	host_command(CLKEXT); //send command to "CLKEXT" to FT800
	host_command(CLK48M); //send command to "CLKEXT" to FT800
	wr16(REG_HSIZE, 480); // width resolution
	wr16(REG_VSIZE, 272); // height resolution
	wr16(REG_HCYCLE, 531); // number if horizontal cycles for display
	wr16(REG_HSYNC0, 0); // hsync falls
	wr16(REG_HSYNC1, 41); // hsync rise
	wr16(REG_HOFFSET, 43); // horizontal offset from starting signal
	wr16(REG_VCYCLE, 288); // number of vertical cycles for display
	wr16(REG_VSYNC0, 0); // vsync falls
	wr16(REG_VSYNC1, 10); // vsync rise
	wr16(REG_VOFFSET, 12); // vertical offset from start signal
	wr8(REG_CSPREAD, 0); // output clock spread enable -CHanged from 0 - 1
	//wr8(REG_DITHER, 0); // output number of bits
	//wr16(REG_OUTBITS, 0x01B6); // output bits resolution
	wr8(REG_SWIZZLE, 0); // output swizzle
	wr8(REG_PCLK_POL, 1); // clock polarity: 0 - rising edge, 1 - falling edge
	// write first display list
	wr32(RAM_DL+0,CLEAR_COLOR_RGB(0,0,0));
	wr32(RAM_DL+4,CLEAR(1,1,1));
	wr32(RAM_DL+8,DISPLAY());
	wr8(REG_DLSWAP,DLSWAP_FRAME);//display list swap
	wr8(REG_GPIO_DIR, 0x80);
	//wr8(REG_GPIO,0x00); // disable display bit
	wr8(REG_GPIO, 0x80);
	wr8(REG_PCLK,5); // clock prescaler (0: disable, >0: 48MHz/pclock)

}