/*
  Copyright (c) 2015 Arduino LLC.  All right reserved.
  Copyright (c) 2015 Atmel Corporation/Thibaut VIARD.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  See the GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include "sam.h"
#include <string.h>
#include "sam_ba_monitor.h"
#include "sam_ba_serial.h"
#include "board_driver_serial.h"
#include "board_driver_usb.h"
#include "board_driver_jtag.h"
#include "sam_ba_usb.h"
#include "sam_ba_cdc.h"

const char RomBOOT_Version[] = SAM_BA_VERSION;
const char RomBOOT_ExtendedCapabilities[] = "[Arduino:XYZ]";

/* Provides one common interface to handle both USART and USB-CDC */
typedef struct
{
  /* send one byte of data */
  int (*put_c)(int value);
  /* Get one byte */
  int (*get_c)(void);
  /* Receive buffer not empty */
  bool (*is_rx_ready)(void);
  /* Send given data (polling) */
  uint32_t (*putdata)(void const* data, uint32_t length);
  /* Get data from comm. device */
  uint32_t (*getdata)(void* data, uint32_t length);
  /* Send given data (polling) using xmodem (if necessary) */
  uint32_t (*putdata_xmd)(void const* data, uint32_t length);
  /* Get data from comm. device using xmodem (if necessary) */
  uint32_t (*getdata_xmd)(void* data, uint32_t length);
} t_monitor_if;

#if defined(SAM_BA_UART_ONLY)  ||  defined(SAM_BA_BOTH_INTERFACES)
/* Initialize structures with function pointers from supported interfaces */
const t_monitor_if uart_if =
{
  .put_c =       serial_putc,
  .get_c =       serial_getc,
  .is_rx_ready = serial_is_rx_ready,
  .putdata =     serial_putdata,
  .getdata =     serial_getdata,
  .putdata_xmd = serial_putdata_xmd,
  .getdata_xmd = serial_getdata_xmd
};
#endif

#if defined(SAM_BA_USBCDC_ONLY)  ||  defined(SAM_BA_BOTH_INTERFACES)
//Please note that USB doesn't use Xmodem protocol, since USB already includes flow control and data verification
//Data are simply forwarded without further coding.
const t_monitor_if usbcdc_if =
{
  .put_c =         cdc_putc,
  .get_c =         cdc_getc,
  .is_rx_ready =   cdc_is_rx_ready,
  .putdata =       cdc_write_buf,
  .getdata =       cdc_read_buf,
  .putdata_xmd =   cdc_write_buf,
  .getdata_xmd =   cdc_read_buf_xmd
};
#endif

/* The pointer to the interface object use by the monitor */
t_monitor_if * ptr_monitor_if;

/* b_terminal_mode mode (ascii) or hex mode */
volatile bool b_terminal_mode = false;
volatile bool b_sam_ba_interface_usart = false;

void sam_ba_monitor_init(uint8_t com_interface)
{
#if defined(SAM_BA_UART_ONLY)  ||  defined(SAM_BA_BOTH_INTERFACES)
  //Selects the requested interface for future actions
  if (com_interface == SAM_BA_INTERFACE_USART)
  {
    ptr_monitor_if = (t_monitor_if*) &uart_if;
    b_sam_ba_interface_usart = true;
  }
#endif
#if defined(SAM_BA_USBCDC_ONLY)  ||  defined(SAM_BA_BOTH_INTERFACES)
  if (com_interface == SAM_BA_INTERFACE_USBCDC)
  {
    ptr_monitor_if = (t_monitor_if*) &usbcdc_if;
  }
#endif
}

/**
 * \brief This function allows data rx by USART
 *
 * \param *data  Data pointer
 * \param length Length of the data
 */
void sam_ba_putdata_term(uint8_t* data, uint32_t length)
{
  uint8_t temp, buf[12], *data_ascii;
  uint32_t i, int_value;

  if (b_terminal_mode)
  {
    if (length == 4)
      int_value = *(uint32_t *) data;
    else if (length == 2)
      int_value = *(uint16_t *) data;
    else
      int_value = *(uint8_t *) data;

    data_ascii = buf + 2;
    data_ascii += length * 2 - 1;

    for (i = 0; i < length * 2; i++)
    {
      temp = (uint8_t) (int_value & 0xf);

      if (temp <= 0x9)
        *data_ascii = temp | 0x30;
      else
        *data_ascii = temp + 0x37;

      int_value >>= 4;
      data_ascii--;
    }
    buf[0] = '0';
    buf[1] = 'x';
    buf[length * 2 + 2] = '\n';
    buf[length * 2 + 3] = '\r';
    ptr_monitor_if->putdata(buf, length * 2 + 4);
  }
  else
    ptr_monitor_if->putdata(data, length);
  return;
}

volatile uint32_t sp;
void call_applet(uint32_t address)
{
  uint32_t app_start_address;

  __disable_irq();

  sp = __get_MSP();

  /* Rebase the Stack Pointer */
  __set_MSP(*(uint32_t *) address);

  /* Load the Reset Handler address of the application */
  app_start_address = *(uint32_t *)(address + 4);

  /* Jump to application Reset Handler in the application */
  asm("bx %0"::"r"(app_start_address));
}

uint32_t current_number;
uint32_t i, length;
uint8_t command, *ptr_data, *ptr, data[SIZEBUFMAX];
uint8_t j;
uint32_t u32tmp;

uint32_t PAGE_SIZE, PAGES, MAX_FLASH;

// Prints a 32-bit integer in hex.
static void put_uint32(uint32_t n)
{
  char buff[8];
  int i;
  for (i=0; i<8; i++)
  {
    int d = n & 0XF;
    n = (n >> 4);

    buff[7-i] = d > 9 ? 'A' + d - 10 : '0' + d;
  }
  ptr_monitor_if->putdata(buff, 8);
}

#ifdef ENABLE_JTAG_LOAD
static uint32_t offset = __UINT32_MAX__;
static bool flashNeeded = false;
#endif

static void sam_ba_monitor_loop(void)
{
  length = ptr_monitor_if->getdata(data, SIZEBUFMAX);
  ptr = data;

  for (i = 0; i < length; i++, ptr++)
  {
    if (*ptr == 0xff) continue;

    if (*ptr == '#')
    {
      if (b_terminal_mode)
      {
        ptr_monitor_if->putdata("\n\r", 2);
      }
      if (command == 'S')
      {
        //Check if some data are remaining in the "data" buffer
        if(length>i)
        {
          //Move current indexes to next avail data (currently ptr points to "#")
          ptr++;
          i++;

          //We need to add first the remaining data of the current buffer already read from usb
          //read a maximum of "current_number" bytes
          if ((length-i) < current_number)
          {
            u32tmp=(length-i);
          }
          else
          {
            u32tmp=current_number;
          }

          memcpy(ptr_data, ptr, u32tmp);
          i += u32tmp;
          ptr += u32tmp;
          j = u32tmp;
        }
        //update i with the data read from the buffer
        i--;
        ptr--;
        //Do we expect more data ?
        if(j<current_number)
          ptr_monitor_if->getdata_xmd(ptr_data, current_number-j);

        __asm("nop");
      }
      else if (command == 'R')
      {
        ptr_monitor_if->putdata_xmd(ptr_data, current_number);
      }
      else if (command == 'O')
      {
        *ptr_data = (char) current_number;
      }
      else if (command == 'H')
      {
        *((uint16_t *) ptr_data) = (uint16_t) current_number;
      }
      else if (command == 'W')
      {
        *((int *) ptr_data) = current_number;
      }
      else if (command == 'o')
      {
        sam_ba_putdata_term(ptr_data, 1);
      }
      else if (command == 'h')
      {
        current_number = *((uint16_t *) ptr_data);
        sam_ba_putdata_term((uint8_t*) &current_number, 2);
      }
      else if (command == 'w')
      {
        current_number = *((uint32_t *) ptr_data);
        sam_ba_putdata_term((uint8_t*) &current_number, 4);
      }
      else if (command == 'G')
      {
        call_applet(current_number);
        /* Rebase the Stack Pointer */
        __set_MSP(sp);
        __enable_irq();
        if (b_sam_ba_interface_usart) {
          ptr_monitor_if->put_c(0x6);
        }
      }
      else if (command == 'T')
      {
        b_terminal_mode = 1;
        ptr_monitor_if->putdata("\n\r", 2);
      }
      else if (command == 'N')
      {
        if (b_terminal_mode == 0)
        {
          ptr_monitor_if->putdata("\n\r", 2);
        }
        b_terminal_mode = 0;
      }
      else if (command == 'V')
      {
        ptr_monitor_if->putdata("v", 1);
        ptr_monitor_if->putdata((uint8_t *) RomBOOT_Version, strlen(RomBOOT_Version));
        ptr_monitor_if->putdata(" ", 1);
        ptr_monitor_if->putdata((uint8_t *) RomBOOT_ExtendedCapabilities, strlen(RomBOOT_ExtendedCapabilities));
        ptr_monitor_if->putdata(" ", 1);
        ptr = (uint8_t*) &(__DATE__);
        i = 0;
        while (*ptr++ != '\0')
          i++;
        ptr_monitor_if->putdata((uint8_t *) &(__DATE__), i);
        ptr_monitor_if->putdata(" ", 1);
        i = 0;
        ptr = (uint8_t*) &(__TIME__);
        while (*ptr++ != '\0')
          i++;
        ptr_monitor_if->putdata((uint8_t *) &(__TIME__), i);
        ptr_monitor_if->putdata("\n\r", 2);
      }
      else if (command == 'X')
      {
        // Syntax: X[ADDR]#
        // Erase the flash memory starting from ADDR to the end of flash.

        // Note: the flash memory is erased in ROWS, that is in block of 4 pages.
        //       Even if the starting address is the last byte of a ROW the entire
        //       ROW is erased anyway.

        uint32_t dst_addr = current_number; // starting address

        while (dst_addr < MAX_FLASH)
        {
          // Execute "ER" Erase Row
          NVMCTRL->ADDR.reg = dst_addr / 2;
          NVMCTRL->CTRLA.reg = NVMCTRL_CTRLA_CMDEX_KEY | NVMCTRL_CTRLA_CMD_ER;
          while (NVMCTRL->INTFLAG.bit.READY == 0)
            ;
          dst_addr += PAGE_SIZE * 4; // Skip a ROW
        }

        // Notify command completed
        ptr_monitor_if->putdata("X\n\r", 3);
      }
      else if (command == 'Y')
      {
        // This command writes the content of a buffer in SRAM into flash memory.

        // Syntax: Y[ADDR],0#
        // Set the starting address of the SRAM buffer.

        // Syntax: Y[ROM_ADDR],[SIZE]#
        // Write the first SIZE bytes from the SRAM buffer (previously set) into
        // flash memory starting from address ROM_ADDR

        static uint32_t *src_buff_addr = NULL;

        if (current_number == 0)
        {
          // Set buffer address
          src_buff_addr = (uint32_t*)ptr_data;
        }
        else
        {
          // Write to flash
          uint32_t size = current_number/4;
          uint32_t *src_addr = src_buff_addr;
          uint32_t *dst_addr = (uint32_t*)ptr_data;

#ifdef ENABLE_JTAG_LOAD

          if ((uint32_t)dst_addr == 0x40000) {
              if (jtagInit() != 0) {
                // fail!
                sam_ba_putdata( ptr_monitor_if, "y\n\r", 3);
                return;
              }

              // content of the first flash page:
              // offset (32) : length(32) : sha256sum(256) : type (32) : force (32) = 48 bytes
              // for every section; check last sector of the flash to understand if reflash is needed
              externalFlashSignatures data[3];
              jtagFlashReadBlock(LAST_FLASH_PAGE, sizeof(data), (uint8_t*)data);
              externalFlashSignatures* newData = (externalFlashSignatures*)src_addr;
              for (int k=0; k<3; k++) {
                if (newData[k].force != 0) {
                  offset = newData[k].offset;
                  flashNeeded = true;
                  break;
                }
                if ((data[k].type == newData[k].type) || (data[k].type == 0xFFFFFFFF)) {
                  if (newData[k].offset < offset) {
                    offset = newData[k].offset;
                  }
                  if (memcmp(data[k].sha256sum, newData[k].sha256sum, 32) != 0) {
                    flashNeeded = true;
                    break;
                  }
                }
              }

              // merge old page and new page
              for (int k=0; k<3; k++) {
                if (newData[k].type != 0xFFFFFFFF) {
                  memcpy(&data[k], &newData[k], sizeof(newData[k]));
                }
              }

              jtagFlashEraseBlock(SCRATCHPAD_FLASH_PAGE);
              // write first page to SCRATCHPAD_FLASH_PAGE (to allow correct verification)
              for (int j =0; j<size; ) {
                jtagFlashWriteBlock(SCRATCHPAD_FLASH_PAGE + j*4, 512, (uint32_t*)&src_addr[j]);
                j += 128;
              }

              // write real structure with checksums to LAST_FLASH_PAGE
              jtagFlashWriteBlock(LAST_FLASH_PAGE, sizeof(data),  (uint32_t*)data);
              goto end;
          }


          if ((uint32_t)dst_addr >= 0x41000) {

            if (flashNeeded == false) {
              goto end;
            }

            uint32_t rebasedAddress = (uint32_t)dst_addr - 0x41000 + offset;
            if (rebasedAddress % 0x10000 == 0) {
              jtagFlashEraseBlock(rebasedAddress);
            }

            for (int j =0; j<size; ) {
              jtagFlashWriteBlock(rebasedAddress + j*4, 512, (uint32_t*)&src_addr[j]);
              j += 128;
            }
            goto end;
          }
#endif
          // Set automatic page write
          NVMCTRL->CTRLB.bit.MANW = 0;

          // Do writes in pages
          while (size)
          {
            // Execute "PBC" Page Buffer Clear
            NVMCTRL->CTRLA.reg = NVMCTRL_CTRLA_CMDEX_KEY | NVMCTRL_CTRLA_CMD_PBC;
            while (NVMCTRL->INTFLAG.bit.READY == 0)
              ;

            // Fill page buffer
            uint32_t i;
            for (i=0; i<(PAGE_SIZE/4) && i<size; i++)
            {
              dst_addr[i] = src_addr[i];
            }

            // Execute "WP" Write Page
            //NVMCTRL->ADDR.reg = ((uint32_t)dst_addr) / 2;
            NVMCTRL->CTRLA.reg = NVMCTRL_CTRLA_CMDEX_KEY | NVMCTRL_CTRLA_CMD_WP;
            while (NVMCTRL->INTFLAG.bit.READY == 0)
              ;

            // Advance to next page
            dst_addr += i;
            src_addr += i;
            size     -= i;
          }
        }

end:

        // Notify command completed
        ptr_monitor_if->putdata("Y\n\r", 3);
      }
      else if (command == 'Z')
      {
        // This command calculate CRC for a given area of memory.
        // It's useful to quickly check if a transfer has been done
        // successfully.

        // Syntax: Z[START_ADDR],[SIZE]#
        // Returns: Z[CRC]#

        uint8_t *data;
        uint32_t size = current_number;
        uint16_t crc = 0;
        uint32_t i = 0;

#ifdef ENABLE_JTAG_LOAD
        uint8_t buf[4096];
#endif

#ifdef ENABLE_JTAG_LOAD
        if ((uint32_t)ptr_data == 0x40000) {
          data = (uint8_t*)buf;
          for (int j =0; j<size; ) {
            jtagFlashReadBlock(SCRATCHPAD_FLASH_PAGE + j, 256, &data[j]);
            j += 256;
          }
        } else if ((uint32_t)ptr_data >= 0x41000) {
          data = (uint8_t*)buf;
          for (int j =0; j<size; ) {
            jtagFlashReadBlock((uint32_t)ptr_data + offset - 0x41000 + j, 256, &data[j]);
            j += 256;
          }
        } else {
          data = (uint8_t *)ptr_data;
        }
#else
        data = (uint8_t *)ptr_data;
#endif

        for (i=0; i<size; i++)
          crc = serial_add_crc(*data++, crc);

        // Send response
        ptr_monitor_if->putdata("Z", 1);
        put_uint32(crc);
        ptr_monitor_if->putdata("#\n\r", 3);
      }

      command = 'z';
      current_number = 0;

      if (b_terminal_mode)
      {
        ptr_monitor_if->putdata(">", 1);
      }
    }
    else
    {
      if (('0' <= *ptr) && (*ptr <= '9'))
      {
        current_number = (current_number << 4) | (*ptr - '0');
      }
      else if (('A' <= *ptr) && (*ptr <= 'F'))
      {
        current_number = (current_number << 4) | (*ptr - 'A' + 0xa);
      }
      else if (('a' <= *ptr) && (*ptr <= 'f'))
      {
        current_number = (current_number << 4) | (*ptr - 'a' + 0xa);
      }
      else if (*ptr == ',')
      {
        ptr_data = (uint8_t *) current_number;
        current_number = 0;
      }
      else
      {
        command = *ptr;
        current_number = 0;
      }
    }
  }
}

/**
 * \brief This function starts the SAM-BA monitor.
 */
void sam_ba_monitor_run(void)
{
  uint32_t pageSizes[] = { 8, 16, 32, 64, 128, 256, 512, 1024 };
  PAGE_SIZE = pageSizes[NVMCTRL->PARAM.bit.PSZ];
  PAGES = NVMCTRL->PARAM.bit.NVMP;
  MAX_FLASH = PAGE_SIZE * PAGES;

  ptr_data = NULL;
  command = 'z';
  while (1)
  {
    sam_ba_monitor_loop();
  }
}
