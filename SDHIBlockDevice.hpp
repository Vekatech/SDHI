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
#ifndef MBED_RZ_SDHI_BLOCK_DEVICE_HPP
#define MBED_RZ_SDHI_BLOCK_DEVICE_HPP

/* If the target has no SDHI support then SDCard is not supported */
#if DEVICE_SDHI

#include "BlockDevice.h"
#include "mbed.h"
#include "platform/PlatformMutex.h"
#include <map>

//#include "r_typedefs.h"

#include "sdif.h"
#include "sd_cfg.h"

#define SDHI_DBG                                 1      /*!< 1 - Enable debugging */

/** Access an SD Card using SDHI
 *
 * @code
 * #include "mbed.h"
 * #include "SDHIBlockDevice.h"
 *
 * SDHIBlockDevice rz_sdhi(p5, p6, p7, p12, p13, p14, p15, p2); // CLK, CMD, CD, WP, D0, D1, D2, D3
 * -- or --
 * #define SDHI_CHANNEL 0
 * SDHIBlockDevice rz_sdhi(SDHI_CHANNEL); // use channel 0
 *
 * uint8_t block[512] = "Hello World!\n";
 *
 * int main() {
 *     rz_sdhi.init();
 *     rz_sdhi.write(block, 0, 512);
 *     rz_sdhi.read(block, 0, 512);
 *     printf("%s", block);
 *     rz_sdhi.deinit();
 * }
 */

class vMutex
{
public:
    vMutex(PlatformMutex * mtx) :
    _mutex(mtx)
    {
        if (_mutex)
            _mutex->lock();
    }
    virtual ~vMutex()
    {
        if(_mutex)
            _mutex->unlock();
    }

private:
    PlatformMutex * _mutex;
};

typedef struct {
    const char    *msg;
    int32_t        errorno;
} sderr_t;



class SDHIBlockDevice : public BlockDevice {
public:
    /** Lifetime of an SD card
     */
    SDHIBlockDevice(uint32_t sdport);
    SDHIBlockDevice(PinName sd_CLK, PinName sd_CMD, PinName sd_CD, PinName sd_WP,
                    PinName sd_D0, PinName sd_D1, PinName sd_D2, PinName sd_D3 );
    virtual ~SDHIBlockDevice();

    /** Initialize a block device
     *
     *  @return         0 on success or a negative error code on failure
     */
    virtual int init();

    /** Deinitialize a block device
     *
     *  @return         0 on success or a negative error code on failure
     */
    virtual int deinit();

    /** Read blocks from a block device
     *
     *  @param buffer   Buffer to write blocks to
     *  @param addr     Address of block to begin reading from
     *  @param size     Size to read in bytes, must be a multiple of read block size
     *  @return         0 on success, negative error code on failure
     */
    virtual int read(void *buffer, bd_addr_t addr, bd_size_t size);

    /** Program blocks to a block device
     *
     *  The blocks must have been erased prior to being programmed
     *
     *  @param buffer   Buffer of data to write to blocks
     *  @param addr     Address of block to begin writing to
     *  @param size     Size to write in bytes, must be a multiple of program block size
     *  @return         0 on success, negative error code on failure
     */
    virtual int program(const void *buffer, bd_addr_t addr, bd_size_t size);

    /** Erase blocks on a block device
     *
     *  The state of an erased block is undefined until it has been programmed
     *
     *  @param addr     Address of block to begin erasing
     *  @param size     Size to erase in bytes, must be a multiple of erase block size
     *  @return         0 on success, negative error code on failure
     */
    virtual int erase(bd_addr_t addr, bd_size_t size);


    /** Mark blocks as no longer in use
     *
     *  This function provides a hint to the underlying block device that a region of blocks
     *  is no longer in use and may be erased without side effects. Erase must still be called
     *  before programming, but trimming allows flash-translation-layers to schedule erases when
     *  the device is not busy.
     *
     *  @param addr     Address of block to mark as unused
     *  @param size     Size to mark as unused in bytes, must be a multiple of erase block size
     *  @return         0 on success, negative error code on failure
     */
    virtual int trim(bd_addr_t addr, bd_size_t size);

    /** Get the size of a readable block
     *
     *  @return         Size of a readable block in bytes
     */
    virtual bd_size_t get_read_size() const;

    /** Get the size of a programable block
     *
     *  @return         Size of a programable block in bytes
     *  @note Must be a multiple of the read size
     */
    virtual bd_size_t get_program_size() const;

    /** Get the size of a eraseable block
     *
     *  @return         Size of a eraseable block in bytes
     *  @note Must be a multiple of the program size
     */
    /*virtual bd_size_t get_erase_size() const; */


    /** Get the total size of the underlying device
     *
     *  @return         Size of the underlying device in bytes
     */
    virtual bd_size_t size() const;

    /** Enable or disable debugging
     *
     *  @param          State of debugging
     */
    virtual void debug(bool dbg);

    /** Get the BlockDevice class type.
     *
     *  @return         A string represent the BlockDevice class type.
     */

    virtual const char *get_type() const;

protected:
    void Initialize( uint32_t sdport );

private:

    uint8_t _card_type;

    /*  Move the SDCard into the SPI Mode idle state
     *
     *  The card is transitioned from SDCard mode to SPI mode by sending the
     *  CMD0 (GO_IDLE_STATE) command with CS asserted. See the notes in the
     *  "SPI Startup" section of the comments at the head of the
     *  implementation file for further details and specification references.
     *
     *  @return         Response form the card. R1_IDLE_STATE (0x1), the successful
     *                  response from CMD0. R1_XXX_XXX for more response
     */
//    uint32_t _go_idle_state();
    int _initialise_card();

    bd_size_t _sectors;
    bd_size_t _sd_sectors();

    bool _is_valid_trim(bd_addr_t addr, bd_size_t size);

    const char * _sderr_msg(int32_t errorno);

    int _error(int32_t errcode);

    PlatformMutex _mutex;

    bd_size_t _block_size;
    bd_size_t _erase_size;
    bool _is_initialized;
    bool _dbg;
    uint32_t _init_ref_count;

    int32_t _sd_channel;
    uint32_t _regbase;
#if SDHI_DBG
    static const sderr_t _sd_error_msg[];

    map<int32_t, const char *> _sd_err_map;
#endif
};

#endif  /* DEVICE_SDHI */

#endif  /* MBED_RZ_SDHI_BLOCK_DEVICE_HPP */

