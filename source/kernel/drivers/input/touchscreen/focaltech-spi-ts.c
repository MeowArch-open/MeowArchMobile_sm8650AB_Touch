// SPDX-License-Identifier: GPL-2.0-only
/*
 * FocalTech touchscreen on SPI, as found on the Xiaomi Redmi K80 ("zorn",
 * SM8650), where the stock DT calls it "focal,touch-spi".
 *
 * The touch report format is the ordinary FocalTech one that edt-ft5x06.c
 * already speaks.  What differs -- and the only reason this cannot just be a
 * bus variant of that driver -- is the framing.  FocalTech's SPI parts do not
 * expose registers as plain SPI reads; every access is one full-duplex frame
 * with a 4-byte header, three dummy bytes, and on reads a trailing CRC:
 *
 *   write:  addr  0x00  len_hi len_lo  [3 dummy  payload...]
 *   read:   addr  0xA0  len_hi len_lo   3 dummy  data...  crc_lo crc_hi
 *                   ^ 0x80 = read, 0x20 = append CRC to the data
 *
 * The chip answers inside that same frame: rx[3] holds a status whose 0xA0
 * bits must both be clear, the payload appears at rx[7], and the CRC is
 * CRC-16/KERMIT over the payload only.  Chip select must then stay high for
 * ~150us before the next frame or the part loses byte sync.  On a write the
 * three dummy bytes are present only when there is a payload, so a bare
 * command is a 4-byte frame.
 *
 * The framing and the register numbers are taken from Xiaomi's own SPI glue
 * (vendor_xiaomi_proprietary_touch-driver, branch popsicle-w-oss, file
 * q200/focaltech_3383/focaltech_spi.c); the input side is written against
 * mainline conventions instead of the vendor's xiaomi_touch/THP plumbing.
 */

/*
 * The vendor's crckermit() is a reflected CRC-16 over poly 0x8408 seeded with
 * 0xffff, which is precisely crc_ccitt(0xffff, ...).
 */
#include <linux/crc-ccitt.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/input/touchscreen.h>
#include <linux/interrupt.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/regulator/consumer.h>
#include <linux/spi/spi.h>
#include <linux/unaligned.h>

#define FTS_CMD_WRITE		0x00
#define FTS_CMD_READ		0x80
#define FTS_CMD_DATA_CRC	0x20

#define FTS_HDR_LEN		4	/* addr, ctrl, len_hi, len_lo */
#define FTS_DUMMY_LEN		3
#define FTS_DATA_OFF		(FTS_HDR_LEN + FTS_DUMMY_LEN)
#define FTS_CRC_LEN		2
#define FTS_STATUS_OFF		3
#define FTS_STATUS_ERR		0xa0

/* Chip select has to idle this long between frames. */
#define FTS_CS_HIGH_DELAY_US	150
#define FTS_XFER_RETRIES	3

#define FTS_REG_WORKMODE	0x00
#define FTS_REG_TOUCHDATA	0x01	/* reads straight through into 0x02.. */
#define FTS_REG_POWER_MODE	0xa5
#define FTS_REG_CHIP_ID		0xa3
#define FTS_REG_FW_VER		0xa6
#define FTS_REG_CHIP_ID2	0x9f
#define FTS_REG_VENDOR_ID	0xa8

/*
 * Xiaomi's firmware comes up streaming raw AFE frames for their userspace
 * touch-host-processing algorithm to turn into contacts -- 138 frames a second
 * with nothing on the glass, in the layout of the vendor's struct frame_afe_data
 * (protocol_type 0xa5, a pair of complementary CRCs, numCol/numRow at the end).
 * There are no contacts in that stream at all. One register switches the part
 * back to working them out itself and reporting them the ordinary way.
 */
#define FTS_REG_THP_MODE	0x9e
#define FTS_THP_MODE_TIC	0x00	/* touch IC computes contacts */
#define FTS_THP_MODE_THP	0x41	/* raw AFE frames to the host */

#define FTS_POWER_MODE_ACTIVE	0x00
#define FTS_POWER_MODE_SLEEP	0x03

/*
 * Reading from FTS_REG_TOUCHDATA walks the classic FocalTech register file:
 * byte 0 is the gesture id (0x01), byte 1 is TD_STATUS (0x02) holding the
 * event type in the high nibble and the contact count in the low one.
 *
 * Two contact layouts follow, and which one a firmware uses is announced by
 * that event type:
 *
 *   v1 (type 0)  6 bytes per contact from 0x03, 12-bit whole-pixel coordinates,
 *                a pressure byte, and the touch area in the top nibble of the
 *                last byte.  This is what edt-ft5x06.c parses.
 *   v2 (type 2)  8 bytes per contact starting two registers later.  The
 *                coordinates are 16-bit in 1/16 of a pixel -- the low nibble of
 *                each axis lives in the byte v1 uses for pressure -- and the
 *                last two bytes are the major and minor axes.  No pressure is
 *                reported; the vendor substitutes a constant, and so do we.
 *
 * zorn's firmware (chip 0x36/0x83) speaks v2. The vendor calls it
 * TOUCH_PROTOCOL_v2 and keeps the sub-pixel value, scaling it in their HAL by
 * focaltech,super-resolution-factors; mainline wants pixels, so it is divided
 * out here.
 */
#define FTS_MAX_POINTS		10
#define FTS_TOUCH_HDR_LEN	2

#define FTS_V1_STRIDE		6
#define FTS_V1_BASE		2
#define FTS_V2_STRIDE		8
#define FTS_V2_BASE		4
#define FTS_V2_SUBPIXEL		16
#define FTS_V2_PRESSURE		0x3f	/* the constant the vendor reports */

#define FTS_TOUCHDATA_LEN	(FTS_V2_BASE + FTS_MAX_POINTS * FTS_V2_STRIDE)

#define FTS_EVT_TYPE(td)	((td) >> 4)
#define FTS_EVT_TOUCH_V1	0x0
#define FTS_EVT_TOUCH_V2	0x2
#define FTS_NR_CONTACTS(td)	((td) & 0x0f)

#define FTS_TCH_DOWN		0
#define FTS_TCH_UP		1
#define FTS_TCH_CONTACT		2

static unsigned int raw_dump;
module_param(raw_dump, uint, 0644);
MODULE_PARM_DESC(raw_dump, "hex dump this many bytes of each touch frame (0 = off)");

static bool thp_mode;
module_param(thp_mode, bool, 0444);
MODULE_PARM_DESC(thp_mode, "leave the firmware streaming raw AFE frames (no contacts are reported)");

struct fts_ts {
	struct spi_device *spi;
	struct input_dev *input;
	struct touchscreen_properties props;
	struct gpio_desc *reset_gpio;
	/* Serialises the header/dummy/payload frames against each other. */
	struct mutex bus_lock;
	u8 *tx_buf;
	u8 *rx_buf;
	size_t buf_len;
};

/* Longest payload we ever move; the frame adds the header, dummies and CRC. */
#define FTS_MAX_PAYLOAD		256
#define FTS_MAX_FRAME		(FTS_DATA_OFF + FTS_MAX_PAYLOAD + FTS_CRC_LEN)

static int fts_xfer(struct fts_ts *ts, size_t len)
{
	struct spi_transfer xfer = {
		.tx_buf = ts->tx_buf,
		.rx_buf = ts->rx_buf,
		.len = len,
	};
	int ret;

	ret = spi_sync_transfer(ts->spi, &xfer, 1);

	/* Owed unconditionally: the part resynchronises while CS is high. */
	udelay(FTS_CS_HIGH_DELAY_US);
	if (ret)
		return ret;

	/* The chip clocks its status out underneath the length byte. */
	if (ts->rx_buf[FTS_STATUS_OFF] & FTS_STATUS_ERR)
		return -EIO;

	return 0;
}

static void fts_fill_hdr(struct fts_ts *ts, u8 addr, u8 ctrl, size_t datalen)
{
	ts->tx_buf[0] = addr;
	ts->tx_buf[1] = ctrl;
	ts->tx_buf[2] = (datalen >> 8) & 0xff;
	ts->tx_buf[3] = datalen & 0xff;
}

static int fts_read(struct fts_ts *ts, u8 addr, u8 *data, size_t len)
{
	size_t frame = FTS_DATA_OFF + len + FTS_CRC_LEN;
	u16 crc_calc, crc_read;
	u8 *payload = ts->rx_buf + FTS_DATA_OFF;
	int ret = -EINVAL;
	int i;

	if (!len || frame > ts->buf_len)
		return -EINVAL;

	guard(mutex)(&ts->bus_lock);

	for (i = 0; i < FTS_XFER_RETRIES; i++) {
		memset(ts->tx_buf, 0, frame);
		fts_fill_hdr(ts, addr, FTS_CMD_READ | FTS_CMD_DATA_CRC, len);

		ret = fts_xfer(ts, frame);
		if (ret)
			continue;

		crc_calc = crc_ccitt(0xffff, payload, len);
		crc_read = get_unaligned_le16(payload + len);
		if (crc_calc != crc_read) {
			ret = -EBADMSG;
			continue;
		}

		memcpy(data, payload, len);
		return 0;
	}

	dev_err_ratelimited(&ts->spi->dev,
			    "read %zu at %#04x failed: %d (status %#04x)\n",
			    len, addr, ret, ts->rx_buf[FTS_STATUS_OFF]);
	return ret;
}

static int fts_write(struct fts_ts *ts, u8 addr, const u8 *data, size_t len)
{
	size_t frame = len ? FTS_DATA_OFF + len : FTS_HDR_LEN;
	int ret = -EINVAL;
	int i;

	if (frame > ts->buf_len)
		return -EINVAL;

	guard(mutex)(&ts->bus_lock);

	for (i = 0; i < FTS_XFER_RETRIES; i++) {
		memset(ts->tx_buf, 0, frame);
		fts_fill_hdr(ts, addr, FTS_CMD_WRITE, len);
		if (len)
			memcpy(ts->tx_buf + FTS_DATA_OFF, data, len);

		ret = fts_xfer(ts, frame);
		if (!ret)
			return 0;
	}

	dev_err(&ts->spi->dev, "write to %#04x failed: %d (status %#04x)\n",
		addr, ret, ts->rx_buf[FTS_STATUS_OFF]);
	return ret;
}

static int fts_read_reg(struct fts_ts *ts, u8 addr, u8 *val)
{
	return fts_read(ts, addr, val, 1);
}

static int fts_write_reg(struct fts_ts *ts, u8 addr, u8 val)
{
	return fts_write(ts, addr, &val, 1);
}

struct fts_contact {
	unsigned int id, flag, x, y, pressure, major, minor;
};

static void fts_decode_point(const u8 *p, bool v2, struct fts_contact *c)
{
	c->id = p[2] >> 4;
	c->flag = p[0] >> 6;

	if (v2) {
		c->x = ((p[0] & 0x0f) << 12 | p[1] << 4 | p[4] >> 4) /
		       FTS_V2_SUBPIXEL;
		c->y = ((p[2] & 0x0f) << 12 | p[3] << 4 | (p[4] & 0x0f)) /
		       FTS_V2_SUBPIXEL;
		c->pressure = FTS_V2_PRESSURE;
		c->major = p[5];
		c->minor = p[6];
	} else {
		c->x = (p[0] & 0x0f) << 8 | p[1];
		c->y = (p[2] & 0x0f) << 8 | p[3];
		c->pressure = p[4];
		c->major = p[5] >> 4;
		c->minor = 0;
	}
}

static void fts_report(struct fts_ts *ts, const u8 *buf, bool v2)
{
	unsigned int stride = v2 ? FTS_V2_STRIDE : FTS_V1_STRIDE;
	unsigned int base = v2 ? FTS_V2_BASE : FTS_V1_BASE;
	unsigned int nr_contacts = FTS_NR_CONTACTS(buf[1]);
	unsigned int i, nr_down = 0;
	struct fts_contact c;

	/*
	 * Count the contacts the firmware claims are down before touching the
	 * input device, because a frame whose per-contact flags disagree with
	 * its own count is corrupt and must be dropped whole rather than
	 * half-reported. The vendor driver makes the same check.
	 */
	for (i = 0; i < FTS_MAX_POINTS; i++) {
		fts_decode_point(buf + base + i * stride, v2, &c);
		if (c.id >= FTS_MAX_POINTS)
			break;
		if (c.flag == FTS_TCH_DOWN || c.flag == FTS_TCH_CONTACT)
			nr_down++;
	}

	if (nr_down && !nr_contacts) {
		dev_warn_ratelimited(&ts->spi->dev,
				     "frame claims 0 contacts but %u are down\n",
				     nr_down);
		return;
	}

	for (i = 0; i < FTS_MAX_POINTS; i++) {
		bool down;

		fts_decode_point(buf + base + i * stride, v2, &c);

		/*
		 * A contact id at or past the maximum marks the end of the
		 * valid entries rather than an error; the firmware leaves the
		 * remaining slots at 0xff.
		 */
		if (c.id >= FTS_MAX_POINTS)
			break;

		down = c.flag == FTS_TCH_DOWN || c.flag == FTS_TCH_CONTACT;

		input_mt_slot(ts->input, c.id);
		if (!input_mt_report_slot_state(ts->input, MT_TOOL_FINGER, down))
			continue;

		touchscreen_report_pos(ts->input, &ts->props, c.x, c.y, true);
		input_report_abs(ts->input, ABS_MT_PRESSURE, c.pressure);
		input_report_abs(ts->input, ABS_MT_TOUCH_MAJOR, c.major);
		input_report_abs(ts->input, ABS_MT_TOUCH_MINOR, c.minor);

		if (raw_dump)
			dev_info_ratelimited(&ts->spi->dev,
					     "slot %u %s at (%u, %u) major %u minor %u\n",
					     c.id, down ? "down" : "up",
					     c.x, c.y, c.major, c.minor);
	}

	input_mt_sync_frame(ts->input);
	input_sync(ts->input);
}

static irqreturn_t fts_irq_handler(int irq, void *dev_id)
{
	struct fts_ts *ts = dev_id;
	u8 buf[FTS_TOUCHDATA_LEN];
	unsigned int evt;

	if (fts_read(ts, FTS_REG_TOUCHDATA, buf, sizeof(buf)))
		return IRQ_NONE;

	if (raw_dump)
		print_hex_dump(KERN_INFO, "fts frame: ", DUMP_PREFIX_OFFSET,
			       16, 1, buf, min_t(size_t, raw_dump, sizeof(buf)),
			       false);

	/*
	 * An all-ones header means the firmware has restarted and has not
	 * reloaded its parameters yet; anything other than one of the two touch
	 * layouts (gesture, proximity, the vendor's frame-data channels) is not
	 * something this driver asked the firmware to produce.
	 */
	evt = FTS_EVT_TYPE(buf[1]);
	if (buf[0] == 0xff && buf[1] == 0xff) {
		dev_warn_ratelimited(&ts->spi->dev, "firmware needs re-init\n");
		return IRQ_HANDLED;
	}
	if (evt != FTS_EVT_TOUCH_V1 && evt != FTS_EVT_TOUCH_V2) {
		dev_warn_ratelimited(&ts->spi->dev,
				     "ignoring event type %#x (hdr %02x %02x)\n",
				     evt, buf[0], buf[1]);
		return IRQ_HANDLED;
	}

	if (FTS_NR_CONTACTS(buf[1]) > FTS_MAX_POINTS) {
		dev_err_ratelimited(&ts->spi->dev, "bogus contact count %u\n",
				    FTS_NR_CONTACTS(buf[1]));
		return IRQ_HANDLED;
	}

	fts_report(ts, buf, evt == FTS_EVT_TOUCH_V2);

	return IRQ_HANDLED;
}

/* avdd before iovdd, as the vendor's power_source_ctrl does. */
static const char * const fts_supply_names[] = { "avdd", "iovdd" };

static void fts_reset(struct fts_ts *ts)
{
	if (!ts->reset_gpio)
		return;

	gpiod_set_value_cansleep(ts->reset_gpio, 1);
	msleep(1);
	gpiod_set_value_cansleep(ts->reset_gpio, 0);
	/* Firmware boot; nothing on the bus answers before this elapses. */
	msleep(200);
}

static int fts_identify(struct fts_ts *ts)
{
	u8 id[2], fw, vendor;
	int ret;

	/*
	 * First bus traffic of the session.  A failure here is a wiring or
	 * framing problem, not a missing feature, so say which register died.
	 */
	ret = fts_read_reg(ts, FTS_REG_CHIP_ID, &id[0]);
	if (ret)
		return ret;

	ret = fts_read_reg(ts, FTS_REG_CHIP_ID2, &id[1]);
	if (ret)
		return ret;

	if (fts_read_reg(ts, FTS_REG_FW_VER, &fw))
		fw = 0;
	if (fts_read_reg(ts, FTS_REG_VENDOR_ID, &vendor))
		vendor = 0;

	dev_info(&ts->spi->dev,
		 "FocalTech chip id %#04x/%#04x, firmware %#04x, vendor %#04x\n",
		 id[0], id[1], fw, vendor);

	if (id[0] == 0x00 || id[0] == 0xff)
		return dev_err_probe(&ts->spi->dev, -ENODEV,
				     "implausible chip id, not talking to the part\n");

	return 0;
}

/*
 * Leave Xiaomi's raw-AFE streaming mode. Read back with retries, because this
 * is the one register whose effect cannot be observed any other way: get it
 * wrong and the driver simply never sees a contact, which on the bench looks
 * exactly like broken wiring.
 */
static int fts_set_report_mode(struct fts_ts *ts)
{
	u8 want = thp_mode ? FTS_THP_MODE_THP : FTS_THP_MODE_TIC;
	int ret = -EIO;
	u8 got;
	int i;

	for (i = 0; i < 3; i++) {
		ret = fts_write_reg(ts, FTS_REG_THP_MODE, want);
		if (!ret) {
			ret = fts_read_reg(ts, FTS_REG_THP_MODE, &got);
			if (!ret && got != want)
				ret = -EIO;
			if (!ret)
				break;
		}
		msleep(1);
	}
	if (ret) {
		dev_err(&ts->spi->dev, "cannot set report mode %#04x: %d\n",
			want, ret);
		return ret;
	}

	dev_info(&ts->spi->dev, "report mode: %s\n",
		 thp_mode ? "THP, raw AFE frames (no contacts will be reported)"
			  : "TIC, contacts computed on the touch controller");

	return 0;
}

static int fts_probe(struct spi_device *spi)
{
	struct device *dev = &spi->dev;
	u8 discard[FTS_TOUCHDATA_LEN];
	struct fts_ts *ts;
	int ret;

	spi->mode = SPI_MODE_0;
	spi->bits_per_word = 8;
	ret = spi_setup(spi);
	if (ret)
		return dev_err_probe(dev, ret, "spi_setup failed\n");

	ts = devm_kzalloc(dev, sizeof(*ts), GFP_KERNEL);
	if (!ts)
		return -ENOMEM;

	ts->spi = spi;
	ts->buf_len = FTS_MAX_FRAME;
	ts->tx_buf = devm_kzalloc(dev, ts->buf_len, GFP_KERNEL);
	ts->rx_buf = devm_kzalloc(dev, ts->buf_len, GFP_KERNEL);
	if (!ts->tx_buf || !ts->rx_buf)
		return -ENOMEM;

	ret = devm_mutex_init(dev, &ts->bus_lock);
	if (ret)
		return ret;

	spi_set_drvdata(spi, ts);

	/* Hold the part in reset across the power-up. */
	ts->reset_gpio = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ts->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(ts->reset_gpio),
				     "cannot get reset gpio\n");

	ret = devm_regulator_bulk_get_enable(dev, ARRAY_SIZE(fts_supply_names),
					     fts_supply_names);
	if (ret)
		return dev_err_probe(dev, ret, "cannot enable supplies\n");

	fts_reset(ts);

	ret = fts_identify(ts);
	if (ret)
		return ret;

	ret = fts_set_report_mode(ts);
	if (ret)
		return ret;

	ts->input = devm_input_allocate_device(dev);
	if (!ts->input)
		return -ENOMEM;

	ts->input->name = "FocalTech Touchscreen";
	ts->input->phys = "input/ts";
	ts->input->id.bustype = BUS_SPI;

	/*
	 * Defaults matching the panel this part is laminated to; a DT with
	 * touchscreen-size-x/y overrides them in parse_properties below.
	 */
	input_set_abs_params(ts->input, ABS_MT_POSITION_X, 0, 1440, 0, 0);
	input_set_abs_params(ts->input, ABS_MT_POSITION_Y, 0, 3200, 0, 0);
	input_set_abs_params(ts->input, ABS_MT_PRESSURE, 0, 255, 0, 0);
	input_set_abs_params(ts->input, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(ts->input, ABS_MT_TOUCH_MINOR, 0, 255, 0, 0);

	touchscreen_parse_properties(ts->input, true, &ts->props);

	ret = input_mt_init_slots(ts->input, FTS_MAX_POINTS,
				  INPUT_MT_DIRECT | INPUT_MT_DROP_UNUSED);
	if (ret)
		return dev_err_probe(dev, ret, "cannot init mt slots\n");

	ret = input_register_device(ts->input);
	if (ret)
		return dev_err_probe(dev, ret, "cannot register input device\n");

	/*
	 * The interrupt is level-sensitive in hardware but requested as an edge
	 * so that a frame left pending by whatever ran before Linux does not
	 * wedge us: drain one frame first, otherwise the line can already be
	 * low and no edge would ever arrive.
	 */
	fts_read(ts, FTS_REG_TOUCHDATA, discard, sizeof(discard));

	ret = devm_request_threaded_irq(dev, spi->irq, NULL, fts_irq_handler,
					IRQF_ONESHOT | IRQF_TRIGGER_FALLING,
					"focaltech-spi-ts", ts);
	if (ret)
		return dev_err_probe(dev, ret, "cannot request irq %d\n", spi->irq);

	return 0;
}

static int fts_suspend(struct device *dev)
{
	struct fts_ts *ts = spi_get_drvdata(to_spi_device(dev));

	disable_irq(ts->spi->irq);

	return fts_write_reg(ts, FTS_REG_POWER_MODE, FTS_POWER_MODE_SLEEP);
}

static int fts_resume(struct device *dev)
{
	struct fts_ts *ts = spi_get_drvdata(to_spi_device(dev));

	fts_reset(ts);
	/* Reset drops the firmware back to its default raw-streaming mode. */
	fts_set_report_mode(ts);
	enable_irq(ts->spi->irq);

	return 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(fts_pm_ops, fts_suspend, fts_resume);

static const struct of_device_id fts_of_match[] = {
	{ .compatible = "focal,touch-spi" },
	{ }
};
MODULE_DEVICE_TABLE(of, fts_of_match);

static const struct spi_device_id fts_spi_id[] = {
	{ "touch-spi" },
	{ }
};
MODULE_DEVICE_TABLE(spi, fts_spi_id);

static struct spi_driver fts_driver = {
	.driver = {
		.name = "focaltech-spi-ts",
		.of_match_table = fts_of_match,
		.pm = pm_sleep_ptr(&fts_pm_ops),
	},
	.id_table = fts_spi_id,
	.probe = fts_probe,
};
module_spi_driver(fts_driver);

MODULE_DESCRIPTION("FocalTech SPI touchscreen");
MODULE_LICENSE("GPL");
