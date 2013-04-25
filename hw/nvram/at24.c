/*
 * AT24Cxx EEPROM emulation
 *
 * Copyright (c) Siemens AG, 2012, 2013
 * Author: Jan Kiszka
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
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include "hw/hw.h"
#include "hw/i2c/i2c.h"
#include "hw/block/block.h"
#include "sysemu/blockdev.h"

#define TYPE_AT24 "at24"
#define AT24(obj) OBJECT_CHECK(AT24State, (obj), TYPE_AT24)

#define AT24_BASE_ADDRESS   0x50
#define AT24_MAX_PAGE_LEN   256

typedef struct AT24Parameters {
    unsigned int size;
    unsigned int page_size;
    unsigned int device_bits;
    unsigned int hi_addr_bits;
    bool addr16;
} AT24Parameters;

typedef enum AT24TransferState {
    AT24_IDLE,
    AT24_RD_ADDR,
    AT24_WR_ADDR_HI,
    AT24_WR_ADDR_LO,
    AT24_RW_DATA0,
    AT24_RD_DATA,
    AT24_WR_DATA,
} AT24TransferState;

typedef struct AT24State {
    I2CSlave parent_obj;

    BlockConf block_conf;
    BlockDriverState *bs;
    uint32_t size;
    bool wp;
    unsigned int addr_mask;
    unsigned int page_mask;
    bool addr16;
    unsigned int hi_addr_mask;
    uint8_t device;
    AT24TransferState transfer_state;
    uint8_t sector_buffer[BDRV_SECTOR_SIZE];
    int cached_sector;
    bool cache_dirty;
    uint32_t transfer_addr;
} AT24State;

static void at24_flush_transfer_buffer(AT24State *s)
{
    if (s->cached_sector < 0 || !s->cache_dirty) {
        return;
    }
    bdrv_write(s->bs, s->cached_sector, s->sector_buffer, 1);
    s->cache_dirty = false;
}

static void at24_event(I2CSlave *i2c, enum i2c_event event, uint8_t param)
{
    AT24State *s = AT24(i2c);

    switch (event) {
    case I2C_START_SEND:
        switch (s->transfer_state) {
        case AT24_IDLE:
            if (s->addr16) {
                s->transfer_addr = (param & s->hi_addr_mask) << 16;
                s->transfer_state = AT24_WR_ADDR_HI;
            } else {
                s->transfer_addr = (param & s->hi_addr_mask) << 8;
                s->transfer_state = AT24_WR_ADDR_LO;
            }
            break;
        default:
            s->transfer_state = AT24_IDLE;
            break;
        }
        break;
    case I2C_START_RECV:
        switch (s->transfer_state) {
        case AT24_IDLE:
            s->transfer_state = AT24_RD_ADDR;
            break;
        case AT24_RW_DATA0:
            s->transfer_state = AT24_RD_DATA;
            break;
        default:
            s->transfer_state = AT24_IDLE;
            break;
        }
        break;
    case I2C_FINISH:
        switch (s->transfer_state) {
        case AT24_WR_DATA:
            at24_flush_transfer_buffer(s);
            /* fall through */
        default:
            s->transfer_state = AT24_IDLE;
            break;
        }
        break;
    default:
        s->transfer_state = AT24_IDLE;
        break;
    }
}

static int at24_cache_sector(AT24State *s, int sector)
{
    int ret;

    if (s->cached_sector == sector) {
        return 0;
    }
    ret = bdrv_read(s->bs, sector, s->sector_buffer, 1);
    if (ret < 0) {
        s->cached_sector = -1;
        return ret;
    }
    s->cached_sector = sector;
    s->cache_dirty = false;
    return 0;
}

static int at24_tx(I2CSlave *i2c, uint8_t data)
{
    AT24State *s = AT24(i2c);

    switch (s->transfer_state) {
    case AT24_WR_ADDR_HI:
        s->transfer_addr |= (data << 8) & s->addr_mask;
        s->transfer_state = AT24_WR_ADDR_LO;
        break;
    case AT24_WR_ADDR_LO:
        s->transfer_addr |= data & s->addr_mask;
        s->transfer_state = AT24_RW_DATA0;
        break;
    case AT24_RW_DATA0:
        s->transfer_state = AT24_WR_DATA;
        if (at24_cache_sector(s, s->transfer_addr >> BDRV_SECTOR_BITS) < 0) {
            s->transfer_state = AT24_IDLE;
            return -1;
        }
        /* fall through */
    case AT24_WR_DATA:
        if (!s->wp) {
            s->sector_buffer[s->transfer_addr & ~BDRV_SECTOR_MASK] = data;
            s->cache_dirty = true;
        }
        s->transfer_addr = (s->transfer_addr & s->page_mask) |
            ((s->transfer_addr + 1) & ~s->page_mask);
        break;
    default:
        s->transfer_state = AT24_IDLE;
        return -1;
    }
    return 0;
}

static int at24_rx(I2CSlave *i2c)
{
    AT24State *s = AT24(i2c);
    unsigned int sector, offset;
    int result;

    switch (s->transfer_state) {
    case AT24_RD_ADDR:
        s->transfer_state = AT24_IDLE;
        result = s->transfer_addr;
        break;
    case AT24_RD_DATA:
        sector = s->transfer_addr >> BDRV_SECTOR_BITS;
        offset = s->transfer_addr & ~BDRV_SECTOR_MASK;
        s->transfer_addr = (s->transfer_addr + 1) & s->addr_mask;
        if (at24_cache_sector(s, sector) < 0) {
            result = 0;
            break;
        }
        result = s->sector_buffer[offset];
        break;
    default:
        s->transfer_state = AT24_IDLE;
        result = 0;
        break;
    }
    return result;
}

static void at24_reset(DeviceState *d)
{
    AT24State *s = AT24(d);

    s->transfer_state = AT24_IDLE;
    s->cached_sector = -1;
}

static const AT24Parameters at24_parameters[] = {
    {    1*1024,   8, 3, 0, false },
    {    2*1024,   8, 3, 0, false },
    {    4*1024,  16, 2, 1, false },
    {    8*1024,  16, 1, 2, false },
    {   16*1024,  16, 0, 3, false },
    {   32*1024,  32, 3, 0, true },
    {   64*1024,  32, 3, 0, true },
    {  128*1024,  64, 2, 0, true },
    {  256*1024,  64, 2, 0, true },
    {  512*1024, 128, 2, 0, true },
    { 1024*1024, 256, 1, 1, true },
    { },
};

static void at24_realize(DeviceState *d, Error **errp)
{
    I2CSlave *i2c = I2C_SLAVE(d);
    AT24State *s = AT24(d);
    const AT24Parameters *params;
    int64_t image_size;
    int dev_no;

    s->bs = s->block_conf.bs;
    if (!s->bs) {
        error_setg(errp, "drive property not set");
        return;
    }

    s->wp = bdrv_is_read_only(s->bs);

    image_size = bdrv_getlength(s->bs);
    if (s->size == 0) {
        s->size = image_size;
    } else if (image_size < s->size) {
        error_setg(errp, "image file smaller than specified EEPROM size");
        return;
    }

    for (params = at24_parameters; params->size != 0; params++) {
        if (s->size * 8 == params->size) {
            break;
        }
    }
    if (params->size == 0) {
        error_setg(errp, "invalid EEPROM size, must be 2^(10..17)");
        return;
    }
    s->addr_mask = s->size - 1;
    s->page_mask = ~(params->page_size - 1);
    s->hi_addr_mask = (1 << params->hi_addr_bits) - 1;
    s->addr16 = params->addr16;

    dev_no = (s->device & ((1 << params->device_bits) - 1)) <<
        params->hi_addr_bits;
    i2c_set_slave_address(i2c, AT24_BASE_ADDRESS | dev_no);
    i2c_set_slave_address_mask(i2c, 0x7f & ~s->hi_addr_mask);
}

static void at24_pre_save(void *opaque)
{
    AT24State *s = opaque;

    at24_flush_transfer_buffer(s);
}

static int at24_post_load(void *opaque, int version_id)
{
    AT24State *s = opaque;

    s->cached_sector = -1;
    return 0;
}

static const VMStateDescription vmstate_at24 = {
    .name = "at24",
    .version_id = 1,
    .minimum_version_id = 1,
    .minimum_version_id_old = 1,
    .pre_save = at24_pre_save,
    .post_load = at24_post_load,
    .fields = (VMStateField[]) {
        VMSTATE_UINT32(transfer_state, AT24State),
        VMSTATE_UINT32(transfer_addr, AT24State),
        VMSTATE_END_OF_LIST()
    }
};

static Property at24_properties[] = {
    DEFINE_BLOCK_PROPERTIES(AT24State, block_conf),
    DEFINE_PROP_UINT8("device", AT24State, device, 0),
    DEFINE_PROP_UINT32("size", AT24State, size, 0),
    DEFINE_PROP_END_OF_LIST(),
};

static void at24_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    I2CSlaveClass *sc = I2C_SLAVE_CLASS(klass);

    sc->event = at24_event;
    sc->recv = at24_rx;
    sc->send = at24_tx;
    dc->desc = "AT24Cxx EEPROM";
    dc->reset = at24_reset;
    dc->realize = at24_realize;
    dc->vmsd = &vmstate_at24;
    dc->props = at24_properties;
}

static const TypeInfo at24_info = {
    .name          = "at24",
    .parent        = TYPE_I2C_SLAVE,
    .instance_size = sizeof(AT24State),
    .class_init    = at24_class_init,
};

static void at24_register_types(void)
{
    /* buffering is based on this, enforce it early */
    assert(AT24_MAX_PAGE_LEN <= BDRV_SECTOR_SIZE);

    type_register_static(&at24_info);
}

type_init(at24_register_types)
