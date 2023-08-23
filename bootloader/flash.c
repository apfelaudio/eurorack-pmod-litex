/*  
 *  Copyright 2020 Gregory Davill <greg.davill@gmail.com>
 *
 * Adapted from: https://github.com/norbertthiel
 * src: https://github.com/litex-hub/litespi/issues/52#issuecomment-890787356
 */

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <generated/csr.h>

#include "flash.h"

static uint32_t transfer_byte(uint8_t b)
{
	// wait for tx ready
	while(!spiflash_core_master_status_tx_ready_read())
	;

	spiflash_core_master_rxtx_write((uint32_t)b);

	//wait for rx ready
	while(!spiflash_core_master_status_rx_ready_read())
	;

	return spiflash_core_master_rxtx_read();
}

static void transfer_cmd(uint8_t *bs, uint8_t *resp, int len)
{
	spiflash_core_master_phyconfig_len_write(8);
	spiflash_core_master_phyconfig_width_write(1);
	spiflash_core_master_phyconfig_mask_write(1);
	spiflash_core_master_cs_write(1);

	if(resp != 0){
		if(bs != 0){
			for(int i=0; i < len; i++)
				resp[i] = transfer_byte(bs[i]);
		}else{
			for(int i=0; i < len; i++)
				resp[i] = transfer_byte(0xFF);
		}
	}else{
		for(int i=0; i < len; i++)
			transfer_byte(bs[i]);
	}

	spiflash_core_master_cs_write(0);
}

uint32_t spiflash_read_status_register(void)
{
	uint8_t buf[2];
    transfer_cmd((uint8_t[]){0x05, 0}, buf, 2);
	return buf[1];
}

uint32_t spiflash_read_status2_register(void)
{
	uint8_t buf[2];
    transfer_cmd((uint8_t[]){0x35, 0}, buf, 2);
	return buf[1];
}

void spiflash_write_enable(void)
{
    transfer_cmd((uint8_t[]){0x06}, 0, 1);
}

void spiflash_page_program(uint32_t addr, uint8_t *data, int len)
{
	spiflash_core_master_phyconfig_len_write(8);
	spiflash_core_master_phyconfig_width_write(1);
	spiflash_core_master_phyconfig_mask_write(1);
	spiflash_core_master_cs_write(1);

	transfer_byte(0x32);
	transfer_byte(addr >> 16);
	transfer_byte(addr >> 8);
	transfer_byte(addr >> 0);

	spiflash_core_master_phyconfig_len_write(8);
	spiflash_core_master_phyconfig_width_write(4);
	spiflash_core_master_phyconfig_mask_write(0x0F);

	for(int i = 0; i < len; i++){
	
		while(!spiflash_core_master_status_tx_ready_read()){}
		spiflash_core_master_rxtx_write((uint32_t)data[i]);		

		while(!spiflash_core_master_status_rx_ready_read()){}
		spiflash_core_master_rxtx_read();
	}

	spiflash_core_master_cs_write(0);
}

void spiflash_sector_erase(uint32_t addr)
{
	spiflash_core_master_phyconfig_len_write(8);
	spiflash_core_master_phyconfig_width_write(1);
	spiflash_core_master_phyconfig_mask_write(1);
	spiflash_core_master_cs_write(1);

	transfer_byte(0xd8);
	transfer_byte(addr >> 16);
	transfer_byte(addr >> 8);
	transfer_byte(addr >> 0);

	spiflash_core_master_cs_write(0);
}

#define min(x, y) (((x) < (y)) ? (x) : (y))

int spiflash_write_stream(uint32_t addr, uint8_t *stream, int len)
{
	int res = 0;
        if( ((uint32_t)addr & ((64*1024) - 1)) == 0)
	{
		int w_len = min(len, SPIFLASH_MODULE_PAGE_SIZE);
		int offset = 0;
		while(w_len)
		{
			if(((uint32_t)addr+offset) & ((64*1024) - 1) == 0)
			{
				spiflash_write_enable();
				spiflash_sector_erase(addr+offset);

				while (spiflash_read_status_register() & 1){}
			}

			spiflash_write_enable();
			spiflash_page_program(addr+offset, stream+offset, w_len);

			while(spiflash_read_status_register() & 1){}

			offset += w_len;
			w_len = min(len-offset,SPIFLASH_MODULE_PAGE_SIZE);
			res = offset;
		}
	}
	return res;
}



/* 25Q128JV FLASH, Read Unique ID Number (4Bh) 
	The Read Unique ID Number instruction accesses a factory-set read-only 64-bit number that is unique to
	each W25Q128JV device. The ID number can be used in conjunction with user software methods to help
	prevent copying or cloning of a system. The Read Unique ID instruction is initiated by driving the /CS pin
	low and shifting the instruction code “4Bh” followed by a four bytes of dummy clocks. After which, the 64-
	bit ID is shifted out on the falling edge of CLK
*/
void spiflash_read_uuid(uint8_t* uuid) {

	spiflash_core_master_phyconfig_len_write(8);
	spiflash_core_master_phyconfig_width_write(1);
	spiflash_core_master_phyconfig_mask_write(1);
	spiflash_core_master_cs_write(1);

	transfer_byte(0x4B);
	transfer_byte(0xFF);
	transfer_byte(0xFF);
	transfer_byte(0xFF);
	transfer_byte(0xFF);

	for(int i=0; i < 8; i++)
		uuid[i] = transfer_byte(0xFF);

	spiflash_core_master_cs_write(0);
}

bool spiflash_protection_read(void){

	uint8_t status = spiflash_read_status_register();
	if((status & 0b11111100) != 0b00110000){
		return false;
	}

	uint8_t status2 = spiflash_read_status2_register();
	if((status2 & 0b01000011) != 0b00000010){
		return false;
	}

	return true;
}

void spiflash_protection_write(bool lock){


	uint8_t status1 = 0;
	if(((status1 = spiflash_read_status_register()) & 0b11111100) != (lock ? 0b00110000 : 0b00000000)){
		spiflash_write_enable();
		
		spiflash_core_master_phyconfig_len_write(8);
		spiflash_core_master_phyconfig_width_write(1);
		spiflash_core_master_phyconfig_mask_write(1);
		spiflash_core_master_cs_write(1);

		transfer_byte(0x01);
		if(lock){
			transfer_byte(0b00110000);
		}else{
			transfer_byte(0b00000000);
		}

		spiflash_core_master_cs_write(0);
	}


	uint8_t status2 = 0;
	if(((status2 = spiflash_read_status2_register()) & 0x02) == 0){
	
		spiflash_write_enable();
		
		spiflash_core_master_phyconfig_len_write(8);
		spiflash_core_master_phyconfig_width_write(1);
		spiflash_core_master_phyconfig_mask_write(1);
		spiflash_core_master_cs_write(1);

		transfer_byte(0x31);
		transfer_byte(0b00000010 | status2);

		spiflash_core_master_cs_write(0);
	}
}

void spiflash_read_security_register(uint8_t security_page, uint8_t* buff){
	spiflash_core_master_phyconfig_len_write(8);
	spiflash_core_master_phyconfig_width_write(1);
	spiflash_core_master_phyconfig_mask_write(1);
	spiflash_core_master_cs_write(1);

	transfer_byte(0x48);
	transfer_byte(0x00);
	transfer_byte(security_page << 4);
	transfer_byte(0x00);

	transfer_byte(0x00);

	for(int byte_index = 0; byte_index < 256; byte_index++){
		*buff++ = transfer_byte(0);
	}

	spiflash_core_master_cs_write(0);	
}

void spiflash_write_security_register(uint8_t security_page, uint8_t* buff){
	spiflash_core_master_phyconfig_len_write(8);
	spiflash_core_master_phyconfig_width_write(1);
	spiflash_core_master_phyconfig_mask_write(1);
	spiflash_core_master_cs_write(1);

	transfer_byte(0x42);
	transfer_byte(0x00);
	transfer_byte(security_page << 4);
	transfer_byte(0x00);

	for(int byte_index = 0; byte_index < 256; byte_index++){
		transfer_byte(*buff++);
	}

	spiflash_core_master_cs_write(0);	

	while(spiflash_read_status_register() & 1){}
}

void spiflash_erase_security_register(uint8_t security_page){
	spiflash_core_master_phyconfig_len_write(8);
	spiflash_core_master_phyconfig_width_write(1);
	spiflash_core_master_phyconfig_mask_write(1);
	spiflash_core_master_cs_write(1);

	transfer_byte(0x44);
	transfer_byte(0x00);
	transfer_byte(security_page << 4);
	transfer_byte(0x00);

	spiflash_core_master_cs_write(0);	

	while(spiflash_read_status_register() & 1){}
}

