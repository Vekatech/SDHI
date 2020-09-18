/* mbed Microcontroller Library
 * Copyright (c) 2006-2012 ARM Limited
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
 
/* If the target has no SDHI support then SDCard is not supported */
#if DEVICE_SDHI

#include "SDHIBlockDevice.hpp"
#include "mbed_debug.h"
#include <errno.h>

#include "iodefine.h"
#include "PeripheralPins.h"

/* Required version: 5.15.5 and above */
#ifdef MBED_MAJOR_VERSION
#if (MBED_VERSION < MBED_ENCODE_VERSION(5,15,5))
#error "Incompatible mbed-os version detected! Required 5.15.5 and above"
#endif
#else
#warning "mbed-os version 5.15.5 or above required"
#endif



//#define SD_COMMAND_TIMEOUT                       5000   /*!< Timeout in ms for response */
//#define SD_CMD0_GO_IDLE_STATE_RETRIES            5      /*!< Number of retries for sending CMDO */
#define SD_CMD_TRACE                             0      /*!< 1 - Enable SD command tracing */

//#define SD_BLOCK_DEVICE_ERROR_WOULD_BLOCK        -5001    /*!< operation would block */
#define SD_BLOCK_DEVICE_ERROR_UNSUPPORTED        -5002  /*!< unsupported operation */
#define SD_BLOCK_DEVICE_ERROR_PARAMETER          -5003  /*!< invalid parameter */
#define SD_BLOCK_DEVICE_ERROR_NO_INIT            -5004  /*!< uninitialized */
#define SD_BLOCK_DEVICE_ERROR_NO_DEVICE          -5005  /*!< device is missing or not connected */
#define SD_BLOCK_DEVICE_ERROR_WRITE_PROTECTED    -5006  /*!< write protected */
#define SD_BLOCK_DEVICE_ERROR_UNUSABLE           -5007  /*!< unusable card */
#define SD_BLOCK_DEVICE_ERROR_NO_RESPONSE        -5008  /*!< No response from device */
#define SD_BLOCK_DEVICE_ERROR_CRC                -5009  /*!< CRC error */
#define SD_BLOCK_DEVICE_ERROR_ERASE              -5010  /*!< Erase error: reset/sequence */
#define SD_BLOCK_DEVICE_ERROR_WRITE              -5011  /*!< SPI Write error: !SPI_DATA_ACCEPTED */

#define BLOCK_SIZE_HC                            512    /*!< Block size supported for SD card is 512 bytes  */
#define WRITE_BL_PARTIAL                         0      /*!< Partial block write - Not supported */
//
// Types
#define SDCARD_NONE              0           /**< No card is present */
#define SDCARD_V1                1           /**< v1.x Standard Capacity */
#define SDCARD_V2                2           /**< v2.x Standard capacity SD card */
#define SDCARD_V2HC              3           /**< v2.x High capacity SD card */
#define CARD_UNKNOWN             4           /**< Unknown or unsupported card */

/* R3 Response : OCR Register */
#define OCR_HCS_CCS             (0x1 << 30)
#define OCR_LOW_VOLTAGE         (0x01 << 24)
#define OCR_3_3V                (0x1 << 20)

#define ESD_SECTOR_SIZE         BLOCK_SIZE_HC

#define REG_CSD_HACK

#define CSD_CID_LENGTH        16

#define ARRAYSIZE(arr)  (size_t)(sizeof(arr)/sizeof(arr[0]))

static uint32_t _sd_workbuf[SD_SIZE_OF_INIT/ sizeof(uint32_t)] __attribute__ ((section ("NC_BSS")));
static uint32_t _sd_rw_buf[ESD_SECTOR_SIZE/sizeof(uint32_t)] __attribute__ ((section ("NC_BSS")));
//static uint8_t  _sectorBuff[ESD_SECTOR_SIZE] __attribute__ ((section ("NC_BSS")));


static uint32_t ext_bits(unsigned char *data, size_t datasize, unsigned int msb, unsigned int lsb) {
    uint32_t bits = 0;

    if ((datasize > 0) && (msb < (datasize<<3)))
    {
        uint32_t size = 1 + msb - lsb;
        for (uint32_t i = 0; i < size; i++)
        {
            uint32_t position = lsb + i;
            int32_t byte = (datasize-1) - (position >> 3);
            if( byte < 0 )
                break;
            uint32_t bit = position & 0x7;
            uint32_t value = (data[byte] >> bit) & 1;
            bits |= value << i;
        }
    }
    return bits;
}

#if SDHI_DBG
const sderr_t SDHIBlockDevice::_sd_error_msg[]= {
        {"SD_OK",                       0},        /* OK */
        {"SD_ERR",                     -1},        /* general error */
        {"SD_ERR_WP",                  -2},        /* write protect error */
        {"SD_ERR_RO",                  -3},        /* read only error */
        {"SD_ERR_RES_TOE",             -4},        /* response time out error */
        {"SD_ERR_CARD_TOE",            -5},        /* card time out error */
        {"SD_ERR_END_BIT",             -6},        /* end bit error */
        {"SD_ERR_CRC",                 -7},        /* CRC error */
        {"SD_ERR_CARD_RES",            -8},        /* card response error */
        {"SD_ERR_HOST_TOE",            -9},        /* host time out error */
        {"SD_ERR_CARD_ERASE",         -10},        /* card erase error */
        {"SD_ERR_CARD_LOCK",          -11},        /* card lock error */
        {"SD_ERR_CARD_UNLOCK",        -12},        /* card unlock error */
        {"SD_ERR_HOST_CRC",           -13},        /* host CRC error */
        {"SD_ERR_CARD_ECC",           -14},        /* card internal ECC error */
        {"SD_ERR_CARD_CC",            -15},        /* card internal error */
        {"SD_ERR_CARD_ERROR",         -16},        /* unknown card error */
        {"SD_ERR_CARD_TYPE",          -17},        /* non support card type */
        {"SD_ERR_NO_CARD",            -18},        /* no card */
        {"SD_ERR_ILL_READ",           -19},        /* illegal buffer read */
        {"SD_ERR_ILL_WRITE",          -20},        /* illegal buffer write */
        {"SD_ERR_AKE_SEQ",            -21},        /* the sequence of authentication process */
        {"SD_ERR_OVERWRITE",          -22},        /* CID/CSD overwrite error */
    /* 23-29 */
        {"SD_ERR_CPU_IF",             -30},        /* target CPU interface function error  */
        {"SD_ERR_STOP",               -31},        /* user stop */
    /* 32-49 */
        {"SD_ERR_CSD_VER",            -50},        /* CSD register version error */
        {"SD_ERR_SCR_VER",            -51},        /* SCR register version error */
        {"SD_ERR_FILE_FORMAT",        -52},        /* CSD register file format error  */
        {"SD_ERR_NOTSUP_CMD",         -53},        /* not supported command  */
    /* 54-59 */
        {"SD_ERR_ILL_FUNC",           -60},        /* invalid function request error */
        {"SD_ERR_IO_VERIFY",          -61},        /* direct write verify error */
        {"SD_ERR_IO_CAPAB",           -62},        /* IO capability error */
    /* 63-69 */
        {"SD_ERR_IFCOND_VER",         -70},        /* Interface condition version error */
        {"SD_ERR_IFCOND_VOLT",        -71},        /* Interface condition voltage error */
        {"SD_ERR_IFCOND_ECHO",        -72},        /* Interface condition echo back pattern error */
    /* 73-79 */
        {"SD_ERR_OUT_OF_RANGE",       -80},        /* the argument was out of range */
        {"SD_ERR_ADDRESS_ERROR",      -81},
        {"SD_ERR_BLOCK_LEN_ERROR",    -82},
        {"SD_ERR_ILLEGAL_COMMAND",    -83},
        {"SD_ERR_RESERVED_ERROR18",   -84},
        {"SD_ERR_RESERVED_ERROR17",   -85},
        {"SD_ERR_CMD_ERROR",          -86},
        {"SD_ERR_CBSY_ERROR",         -87},
        {"SD_ERR_NO_RESP_ERROR",      -88},
    /* 89 */
    /* 90-95 */
        {"SD_ERR_ERROR",              -96},
        {"SD_ERR_FUNCTION_NUMBER",    -97},
        {"SD_ERR_COM_CRC_ERROR",      -98},
        {"SD_ERR_INTERNAL",           -99},        /* driver software internal error */
};

#endif

SDHIBlockDevice::SDHIBlockDevice(uint32_t sdport)
{
    _init_ref_count = 0;
    Initialize(sdport);
//    for( uint32_t n=0; n < ARRAYSIZE(_sd_error_msg); n++ )
//    {
//      _sd_err_map[_sd_error_msg[n].errorno] = _sd_error_msg[n].msg;
//    }
//
//    if ( sdport < SDHI_COUNT )
//    {
//      _sd_channel = sdport;
//      _regbase = (uint32_t)((_sd_channel == 0ul) ? &SDHI0 : &SDHI1);
//
//      _card_type = SDCARD_NONE;
//
//      // Set default to 100kHz for initialisation and 1MHz for data transfer
//      _init_sck = 100000;
//      _transfer_sck = hz;
//
//      // Only HC block size is supported.
//      _block_size = BLOCK_SIZE_HC;
//      _erase_size = BLOCK_SIZE_HC;
//      _mtx_lock = false;
//
//      int32_t sd_err = sd_init( _sd_channel, _regbase, &_sd_workbuf[0], SD_CD_SOCKET );
//      if ( sd_err != SD_OK)
//      {
//          _error(sd_err);
////            return SD_BLOCK_DEVICE_ERROR_NO_DEVICE;
//      }
//
//      void *rw_buff = (void *)&_sd_rw_buf[0];
//
//      sd_err = sd_set_buffer( _sd_channel, rw_buff, (unsigned long)sizeof(_sd_rw_buf) );
//      if ( sd_err != SD_OK )
//      {
//          _error(sd_err);
//      }
//
//    }
//    else
//    {
//      _sd_channel = -1;
//      _regbase = 0ul;
//
//    }
}

SDHIBlockDevice::SDHIBlockDevice(PinName sd_CLK, PinName sd_CMD, PinName sd_CD, PinName sd_WP,
                                 PinName sd_D0, PinName sd_D1, PinName sd_D2, PinName sd_D3 )
{
    _init_ref_count = 0;

    uint32_t sd_wp  = pinmap_peripheral(sd_WP, PinMap_SDHI_WP);
    uint32_t sd_cd  = pinmap_peripheral(sd_CD, PinMap_SDHI_CD);
    uint32_t sd_clk = pinmap_peripheral(sd_CLK, PinMap_SDHI_CLK);
    uint32_t sd_cmd = pinmap_peripheral(sd_CMD, PinMap_SDHI_CMD);
    uint32_t sd_d0  = pinmap_peripheral(sd_D0, PinMap_SDHI_D0);
    uint32_t sd_d1  = pinmap_peripheral(sd_D1, PinMap_SDHI_D1);
    uint32_t sd_d2  = pinmap_peripheral(sd_D2, PinMap_SDHI_D2);
    uint32_t sd_d3  = pinmap_peripheral(sd_D3, PinMap_SDHI_D3);

    uint32_t sd_cntl_1 = pinmap_merge(sd_wp, sd_cd);
    uint32_t sd_cntl_2 = pinmap_merge(sd_clk, sd_cmd);
    uint32_t sd_cntl = pinmap_merge(sd_cntl_1, sd_cntl_2);

    uint32_t sd_data_1 = pinmap_merge(sd_d0, sd_d1);
    uint32_t sd_data_2 = pinmap_merge(sd_d2, sd_d3);
    uint32_t sd_data = pinmap_merge(sd_data_1, sd_data_2);

    uint32_t sdport = pinmap_merge(sd_cntl, sd_data);

    MBED_ASSERT((int)sdport != NC);

    Initialize(sdport);
}



SDHIBlockDevice::~SDHIBlockDevice()
{
    if (_is_initialized) {
        deinit();
    }
}

void SDHIBlockDevice::Initialize( uint32_t sdport )
{
#if SDHI_DBG
    for( uint32_t n=0; n < ARRAYSIZE(_sd_error_msg); n++ )
    {
        _sd_err_map[_sd_error_msg[n].errorno] = _sd_error_msg[n].msg;
    }
#endif
    _sectors = 0;
    _is_initialized = 0;
    if ( sdport < SDHI_COUNT )
    {
        _sd_channel = sdport;
        _regbase = (uint32_t)((_sd_channel == SDHI_0) ? &SDHI0 : &SDHI1);

        _card_type = SDCARD_NONE;

        // Only HC block size is supported.
        _block_size = BLOCK_SIZE_HC;
        _erase_size = BLOCK_SIZE_HC;

        int32_t sd_err = sd_init( _sd_channel, _regbase, &_sd_workbuf[0], SD_CD_SOCKET );
        if ( sd_err != SD_OK)
        {
            _error(sd_err);
        }

        void *rw_buff = (void *)&_sd_rw_buf[0];

        sd_err = sd_set_buffer( _sd_channel, rw_buff, (unsigned long)sizeof(_sd_rw_buf) );
        if ( sd_err != SD_OK )
        {
            _error(sd_err);
        }
    }
    else
    {
        _sd_channel = -1;
        _regbase = 0ul;

    }

}

int SDHIBlockDevice::_initialise_card()
{
    // Detail debugging is for commands
    _dbg = SDHI_DBG ? SD_CMD_TRACE : 0;

    int32_t status = BD_ERROR_OK;

    int32_t sd_err;

    if( _regbase )
    {
        sd_err = sd_mount(_sd_channel, SDCFG_DRIVER_MODE, SD_VOLT_3_3);

        if ( sd_err != SD_OK )
        {
            _error(sd_err);
            return SD_BLOCK_DEVICE_ERROR_UNUSABLE;
        }
        uint8_t    card_type;
        uint8_t    card_speed;
        uint8_t    card_capa;

        sd_err=sd_get_type(_sd_channel, &card_type, &card_speed, &card_capa);
        if( sd_err != SD_OK)
        {
            return _error(sd_err);
        }

        if( (card_type & SD_MEDIA_SD) != SD_MEDIA_SD )
        {
            return SD_BLOCK_DEVICE_ERROR_UNUSABLE;
        }

        uint32_t regOCR;
        uint8_t regCID[CSD_CID_LENGTH];
        uint8_t regCSD[CSD_CID_LENGTH];
        uint8_t regDSR[2];
        uint8_t regSCR[8];

        sd_err = sd_get_reg(_sd_channel, (uint8_t *)&regOCR, regCID, regCSD, regDSR, regSCR);
        if (sd_err != SD_OK)
        {
            return _error(sd_err);
        }
        regOCR = __REV(regOCR);
        // Check if card supports voltage range: 3.3V
        if (!(regOCR & OCR_3_3V)) {
            _card_type = CARD_UNKNOWN;
            status = SD_BLOCK_DEVICE_ERROR_UNUSABLE;
            return status;
        }
    }
    else
    {
        status = SD_BLOCK_DEVICE_ERROR_NO_INIT;
    }

    return status;
}


int SDHIBlockDevice::init()
{
    vMutex l(&_mutex);

    if(!_is_initialized)
        _init_ref_count = 0;

    _init_ref_count++;

    if(_init_ref_count != 1)
        return BD_ERROR_OK;

    int err = _initialise_card();
    _is_initialized = (err == BD_ERROR_OK);
    if (!_is_initialized) {
        debug_if(SDHI_DBG, "Fail to initialize card\n");
        return err;
    }
    debug_if(SDHI_DBG, "init card = %d\r\n", _is_initialized);
    _sectors = _sd_sectors();

    if (0 == _sectors) {
        return BD_ERROR_DEVICE_ERROR;
    }

    return BD_ERROR_OK;
}

int SDHIBlockDevice::deinit()
{
    vMutex l(&_mutex);

    if(!_is_initialized)
        _init_ref_count = 0;

    _init_ref_count--; //!!!

    if(_init_ref_count)
        return BD_ERROR_OK;

    _sectors = 0;
    if ( _is_initialized )
    {
        sd_unmount(_sd_channel);
        debug_if(SDHI_DBG, "card deinited![%d]\r\n", _is_initialized);
        _is_initialized = false;
    }
    return 0;
}


int SDHIBlockDevice::program(const void *b, bd_addr_t addr, bd_size_t size)
{
    if (!is_valid_program(addr, size)) {
        return SD_BLOCK_DEVICE_ERROR_PARAMETER;
    }

    vMutex l(&_mutex);

    if (!_is_initialized) {
        return SD_BLOCK_DEVICE_ERROR_NO_INIT;
    }

    uint8_t* buffer = const_cast<uint8_t*>(static_cast<const uint8_t*>(b));
    int status = BD_ERROR_OK;

    // Get block count
    bd_addr_t blockCnt = size / _block_size;

    addr = addr / _block_size;

    // Send command to perform write operation
    int32_t sd_err = sd_write_sect( _sd_channel, buffer, addr, blockCnt, SD_WRITE_OVERWRITE);

    if (sd_err != SD_OK)
    {
        _error(sd_err);
        status = SD_BLOCK_DEVICE_ERROR_WRITE;
    }

    return status;
}

int SDHIBlockDevice::read(void *b, bd_addr_t addr, bd_size_t size)
{
    if (!is_valid_read(addr, size)) {
        return SD_BLOCK_DEVICE_ERROR_PARAMETER;
    }

    vMutex l(&_mutex);

    if (!_is_initialized) {
        return SD_BLOCK_DEVICE_ERROR_NO_INIT;
    }
    uint8_t *buffer = static_cast<uint8_t*>(b);
    int status = BD_ERROR_OK;
    bd_addr_t blockCnt =  size / _block_size;

    addr = addr / _block_size;

    int32_t sd_err = sd_read_sect(_sd_channel, buffer, addr, blockCnt);

    if (sd_err != SD_OK)
    {
        _error(sd_err);
        status = SD_BLOCK_DEVICE_ERROR_NO_RESPONSE;
    }

    return status;
}

int SDHIBlockDevice::erase(bd_addr_t addr, bd_size_t size)
{
    return 0;
}


bool SDHIBlockDevice::_is_valid_trim(bd_addr_t addr, bd_size_t size)
{
    return (
        addr % _erase_size == 0 &&
        size % _erase_size == 0 &&
        addr + size <= this->size());
}

int SDHIBlockDevice::trim(bd_addr_t addr, bd_size_t size)
{
    if (!_is_valid_trim(addr, size)) {
        return SD_BLOCK_DEVICE_ERROR_PARAMETER;
    }

    vMutex l(&_mutex);

    if (!_is_initialized) {
        return SD_BLOCK_DEVICE_ERROR_NO_INIT;
    }
    int status = BD_ERROR_OK;

    size -= _block_size;
    // SDSC Card (CCS=0) uses byte unit address
    // SDHC and SDXC Cards (CCS=1) use block unit address (512 Bytes unit)
    if (SDCARD_V2HC == _card_type) {
        size = size / _block_size;
        addr = addr / _block_size;
    }

    return status;
}

bd_size_t SDHIBlockDevice::get_read_size() const
{
    return _block_size;
}

bd_size_t SDHIBlockDevice::get_program_size() const
{
    return _block_size;
}

/*
bd_size_t SDHIBlockDevice::get_erase_size() const
{
    return _block_size;
}
*/

bd_size_t SDHIBlockDevice::size() const
{
    return _block_size*_sectors;
}

void SDHIBlockDevice::debug(bool dbg)
{
    _dbg = dbg;
}

const char *SDHIBlockDevice::get_type() const
{
    return "RZ-SDHI";
}

// PRIVATE FUNCTIONS

bd_size_t SDHIBlockDevice::_sd_sectors() {
    uint32_t c_size, c_size_mult, read_bl_len;
    uint32_t block_len, mult, blocknr;
    uint32_t hc_c_size;
    bd_size_t blocks = 0, capacity = 0;
    uint8_t csd[CSD_CID_LENGTH];

    int32_t sd_err = sd_get_reg(_sd_channel, NULL, NULL, csd, NULL, NULL);
    if ( sd_err != SD_OK )
    {
        debug_if(SDHI_DBG, "Couldn't read csd response from disk\r\n");
        _error(sd_err);
        return 0;
    }
    for(int i = 0; i < (CSD_CID_LENGTH-1); i++)
    {
        csd[i] = csd[i+1];
    }

    debug_if(SDHI_DBG,"CSD is ");
    for(unsigned int i = 0; i < sizeof(csd); i++)
    {
        debug_if(SDHI_DBG, "%02X ", csd[i]);
    }
    debug_if(SDHI_DBG,"\r\n");

    // csd_structure : csd[127:126]
    int csd_structure = ext_bits(csd, CSD_CID_LENGTH, 127, 126);
    switch (csd_structure) {
        case 0:
            c_size = ext_bits(csd, CSD_CID_LENGTH, 73, 62);              // c_size        : csd[73:62]
            c_size_mult = ext_bits(csd, CSD_CID_LENGTH, 49, 47);         // c_size_mult   : csd[49:47]
            read_bl_len = ext_bits(csd, CSD_CID_LENGTH, 83, 80);         // read_bl_len   : csd[83:80] - the *maximum* read block length
            block_len = 1 << read_bl_len;                // BLOCK_LEN = 2^READ_BL_LEN
            mult = 1 << (c_size_mult + 2);               // MULT = 2^C_SIZE_MULT+2 (C_SIZE_MULT < 8)
            blocknr = (c_size + 1) * mult;               // BLOCKNR = (C_SIZE+1) * MULT
            capacity = blocknr * block_len;              // memory capacity = BLOCKNR * BLOCK_LEN
            blocks = capacity / _block_size;
            debug_if(SDHI_DBG, "Standard Capacity: c_size: %lu\r\n", c_size);
            debug_if(SDHI_DBG, "Sectors: 0x%llx : %llu\r\n", blocks, blocks);
            debug_if(SDHI_DBG, "Capacity: 0x%llx : %llu MB\r\n", capacity, (capacity/(1024U*1024U)));

            // ERASE_BLK_EN = 1: Erase in multiple of 512 bytes supported
            if (ext_bits(csd, CSD_CID_LENGTH, 46, 46)) {
                _erase_size = BLOCK_SIZE_HC;
            } else {
                // ERASE_BLK_EN = 1: Erase in multiple of SECTOR_SIZE supported
                _erase_size = BLOCK_SIZE_HC * (ext_bits(csd, CSD_CID_LENGTH, 45, 39) + 1);
            }
            break;

        case 1:
            hc_c_size = ext_bits(csd, CSD_CID_LENGTH, 69, 48);            // device size : C_SIZE : [69:48]
            blocks = (hc_c_size+1) << 10;                 // block count = C_SIZE+1) * 1K byte (512B is block size)
            debug_if(SDHI_DBG, "SDHC/SDXC Card: hc_c_size: %lu\r\n", hc_c_size);
            debug_if(SDHI_DBG, "Sectors: 0x%llx : %llu\r\n", blocks, blocks);
            debug_if(SDHI_DBG, "Capacity: %llu MB\r\n", (blocks/(2048U)));
            // ERASE_BLK_EN is fixed to 1, which means host can erase one or multiple of 512 bytes.
            _erase_size = BLOCK_SIZE_HC;
            break;

        default:
            debug_if(SDHI_DBG, "CSD struct unsupported\r\n");
            return 0;
    };
    return blocks;
}

const char * SDHIBlockDevice::_sderr_msg(int32_t errorno)
{
#if SDHI_DBG
    map<int32_t, const char *>::iterator it = _sd_err_map.find(errorno);

    if ( it != _sd_err_map.end() )
        return it->second;
    else
        return "SD UNKNWON ERROR NO\n";
#else
    return (const char *)0;
#endif
}

int SDHIBlockDevice::_error(int32_t errcode)
{
    int32_t sd_err = errcode;
    if(_sd_channel >= 0 && _sd_channel < SDHI_COUNT )
    {
        int32_t err = sd_get_error(_sd_channel);
        if ( err != SD_OK)
            sd_err = err;
        debug_if(SDHI_DBG, _sderr_msg(sd_err));
    }
    return sd_err;
}

#endif /* DEVICE_SDHI */
