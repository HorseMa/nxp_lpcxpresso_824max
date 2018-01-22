/*
 * Copyright (c) 2014-2016 IBM Corporation.
 * All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *  * Neither the name of the <organization> nor the
 *    names of its contributors may be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#include <stdlib.h>
#include <stdint.h>
#include "board.h"
#include "chip.h"
#include "utilities.h"
#include "modem.h"

uint32_t src_iap_array_data[ARRAY_ELEMENTS];
#if 0
void eeprom_erase (void) {
    uint8_t ret_code;
    hal_disableIRQs();
    /* IAP Flash programming */
    /* Prepare to write/erase the last sector */
    ret_code = Chip_IAP_PreSectorForReadWrite(IAP_LAST_SECTOR, IAP_LAST_SECTOR);

    /* Error checking */
    if (ret_code != IAP_CMD_SUCCESS) {
        //Print_Val("Command failed to execute, return code is: ", ret_code);
    }

    /* Erase the last sector */
    ret_code = Chip_IAP_EraseSector(IAP_LAST_SECTOR, IAP_LAST_SECTOR);
    //ret_code = Chip_IAP_ErasePage(498,499);

    /* Error checking */
    if (ret_code != IAP_CMD_SUCCESS) {
        //Print_Val("Command failed to execute, return code is: ", ret_code);
    }
    /* Re-enable interrupt mode */
    hal_enableIRQs();

	/* Start the signature generator for the last sector */
    Chip_FMC_ComputeSignatureBlocks(START_ADDR_LAST_SECTOR, (SECTOR_SIZE / 16));

    /* Check for signature geenration completion */
    while (Chip_FMC_IsSignatureBusy()) {}
}
#endif
// write 32-bit word to EEPROM memory
void eeprom_write (void) {
    //int i;
    uint8_t ret_code;
    uint32_t part_id;
    uint32_t unique_id[4];
    uint16_t joincfgcrc;
    uint16_t sesscfgcrc;

    joincfgcrc = os_crc16((uint8_t*)&persist.joinpar, sizeof(joinparam_t));
    sesscfgcrc = os_crc16((uint8_t*)&persist.sesspar, sizeof(sessparam_t));
    persist.cfghash = (joincfgcrc << 16) | sesscfgcrc;
    memset(src_iap_array_data,0,IAP_NUM_BYTES_TO_WRITE);
    memcpy(src_iap_array_data,&persist,sizeof(persist));
    /* Read Part Identification Number*/
    part_id = Chip_IAP_ReadPID();
    //Print_Val("Part ID is: 0x", part_id);

    /* Read Part Unique Identification Number*/
    Chip_IAP_ReadUID(unique_id);
    //DEBUGSTR("Unique Part ID is:\r\n");
    //for (i=0; i<4; i++) {
        //Print_Val("0x", unique_id[i]);
    //}

    /* Disable interrupt mode so it doesn't fire during FLASH updates */
    hal_disableIRQs();
#if 1
    /* IAP Flash programming */
    /* Prepare to write/erase the last sector */
    ret_code = Chip_IAP_PreSectorForReadWrite(IAP_LAST_SECTOR, IAP_LAST_SECTOR);

    /* Error checking */
    if (ret_code != IAP_CMD_SUCCESS) {
        //Print_Val("Command failed to execute, return code is: ", ret_code);
    }

    /* Erase the last sector */
    //ret_code = Chip_IAP_EraseSector(IAP_LAST_SECTOR, IAP_LAST_SECTOR);
    ret_code = Chip_IAP_ErasePage(EEPROM_BASE / 64,EEPROM_BASE / 64 + 1);
    /* Error checking */
    if (ret_code != IAP_CMD_SUCCESS) {
        //Print_Val("Command failed to execute, return code is: ", ret_code);
    }
#endif
#if 1
    /* Prepare to write/erase the last sector */
    ret_code = Chip_IAP_PreSectorForReadWrite(IAP_LAST_SECTOR, IAP_LAST_SECTOR);

    /* Error checking */
    if (ret_code != IAP_CMD_SUCCESS) {
        //Print_Val("Command failed to execute, return code is: ", ret_code);
    }

    /* Write to the last sector */
    //ret_code = Chip_IAP_CopyRamToFlash(START_ADDR_LAST_SECTOR, &val, 4);
    ret_code = Chip_IAP_CopyRamToFlash(EEPROM_BASE, src_iap_array_data, IAP_NUM_BYTES_TO_WRITE);

    /* Error checking */
    if (ret_code != IAP_CMD_SUCCESS) {
        //Print_Val("Command failed to execute, return code is: ", ret_code);
    }
#endif
    /* Re-enable interrupt mode */
    hal_enableIRQs();

	/* Start the signature generator for the last sector */
    Chip_FMC_ComputeSignatureBlocks(START_ADDR_LAST_SECTOR, (SECTOR_SIZE / 16));

    /* Check for signature geenration completion */
    while (Chip_FMC_IsSignatureBusy()) {}

    /* Get the generated FLASH signature value */
    //Print_Val("Generated signature for the last sector is: 0x", Chip_FMC_GetSignature(0));
}

/*void eeprom_copy (void* dst, const void* src, uint16_t len) {
    while(((uint32_t)dst & 3) || ((uint32_t)src & 3) || (len & 3)); // halt if not multiples of 4
    uint32_t* d = (uint32_t*)dst;
    uint32_t* s = (uint32_t*)src;
    uint16_t  l = len/4;

    while(l--) {
        eeprom_write(d++, *s++);
    }
}*/
