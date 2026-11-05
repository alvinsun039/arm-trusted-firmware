/*
 * Copyright (c) 2026, A733 open-source BL31 contributors
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * SID/efuse service for the vendor kernel ABI.
 */
#include <assert.h>

#include <platform_def.h>

#include <arch_helpers.h>
#include <common/debug.h>
#include <lib/mmio.h>
#include <lib/utils_def.h>

#include <sunxi_mmap.h>

/* DRAM window an NS caller may hand us pointers into */
static bool ns_dram_ptr(uintptr_t p, size_t len)
{
	return p >= SUNXI_DRAM_BASE && p < (SUNXI_DRAM_BASE + 0x80000000ULL) &&
	       len < 0x1000U && (p + len) <= (SUNXI_DRAM_BASE + 0x80000000ULL);
}

/* ---- EFUSE controller  --------- */
#define SID_EFUSE_CTRL			(SUNXI_SID_BASE + 0x40)
#define SID_EFUSE_DATA			(SUNXI_SID_BASE + 0x60)
#define SID_EFUSE_OP_READ		0xAC02U
#define SID_EFUSE_CTRL_BUSY		BIT(1)

/* ---- key table (the vendor BSP rodata 0x130B4) -------------------------------
 * Field layout matches the firmware's 64-byte entries: +40 = extra-flag
 * bit used with bitmap word 0x80, +44 = burn bit used with word 0x7C,
 * +48 = bit0==1 means the key is readable.
 */
struct sid_key {
	const char	*name;
	uint32_t	word_off;	/* efuse array index base	*/
	uint32_t	size_bits;	/* key length in bits		*/
	int32_t		flag_shift;	/* bitmap 0x80 bit, -1 = none	*/
	int32_t		burn_shift;	/* bitmap 0x7C bit, -1 = none	*/
	uint32_t	protect;	/* bit0 = 1 -> readable		*/
};

static const struct sid_key sid_keys[] = {
	{ "chipid",		0x0000, 128, -1, -1, 0x1 },
	{ "customer_reserved",	0x0068, 160, -1, -1, 0x3 },
	{ "huk",		0x0098, 192, -1,  12, 0x0 },
	{ "rotpk",		0x00b8, 256, -1,  15, 0x0 },
	{ "rssk",		0x00f8, 256, 17,  17, 0x0 },
	{ "ssk",		0x00d8, 256, 16,  16, 0x0 },
	{ "hdcphash",		0x0118, 128, -1,  18, 0x3 },
	{ "hdcppkf",		0x01a8, 128, -1,  23, 0x0 },
	{ "npu_status",		0x0088,  32, -1,  -1, 0x3 },
	{ "hdcp_edp_l128",	0x0138, 128, -1,  -1, 0x0 },
	{ "oem",		0x0068, 160, -1,  -1, 0x3 },
};

#define SID_KEYS_COUNT		ARRAY_SIZE(sid_keys)

/* ---- hardware: one word out of the efuse array (0xAD38) ----------------
 * The CTRL index field is 9 bits wide ([24:16], mask 0xFE00FFFF).
 */
static uint32_t efuse_read_word(uint32_t word_index)
{
	uint32_t ctl = mmio_read_32(SID_EFUSE_CTRL);
	uint32_t ctl_op;

	ctl &= ~0x3U;
	ctl &= ~0xFE00FFFFU;
	ctl |= word_index << 16;
	ctl_op = ctl & 0xFFFF00FCU;
	mmio_write_32(SID_EFUSE_CTRL, ctl);
	mmio_write_32(SID_EFUSE_CTRL, ctl_op | SID_EFUSE_OP_READ);

	/* wait busy clear (bit1) */
	while ((mmio_read_32(SID_EFUSE_CTRL) & SID_EFUSE_CTRL_BUSY) != 0U)
		;

	/* post-op cleanup: clear OP + index (0xAD70-0xAD78) */
	ctl &= 0xFFFFFFFCU;
	ctl &= 0xFE0000FFU;
	mmio_write_32(SID_EFUSE_CTRL, ctl);

	return mmio_read_32(SID_EFUSE_DATA);
}

/*
 * One word into the efuse array - hardware write sequence, faithful to
 * :
 *   EFUSE_HV_PWRSWT (RTC+0x204) power on
 *   data -> 0x03006050, index -> CTRL[23:16], OP 0xAC01, busy bit0
 *   then power off.
 */
static void efuse_write_word(uint32_t word_index, uint32_t value)
{
	uint32_t ctl;

	mmio_setbits_32(SUNXI_RTC_BASE + 0x204, BIT(0));	/* HV power on */

	mmio_write_32(SUNXI_SID_BASE + 0x50, value);	/* write data */
	ctl = mmio_read_32(SID_EFUSE_CTRL);
	ctl &= ~0x3U;
	ctl &= ~0xFE00FFFFU;
	ctl |= word_index << 16;
	ctl &= 0xFFFF00FCU;
	mmio_write_32(SID_EFUSE_CTRL, ctl);
	mmio_write_32(SID_EFUSE_CTRL, ctl | 0xAC01U);	/* write op */

	while ((mmio_read_32(SID_EFUSE_CTRL) & BIT(0)) != 0U)
		;

	ctl &= 0xFFFFFFFCU;
	ctl &= 0xFE0000FFU;
	mmio_write_32(SID_EFUSE_CTRL, ctl);

	mmio_clrbits_32(SUNXI_RTC_BASE + 0x204, BIT(0));	/* HV power off */
}

/* ---- key lookup (0xBFB8 walk) ------------------------------------------ */
static const struct sid_key *sid_find_key(uintptr_t name_buf)
{
	char name[32];
	unsigned int i;
	unsigned int j;

	for (i = 0U; i < sizeof(name) - 1U; i++) {
		uint8_t c = *(volatile uint8_t *)(name_buf + i);

		if (c == 0U)
			break;
		name[i] = (char)c;
	}
	name[i] = 0;
	if (i == 0U)
		return NULL;

	for (j = 0U; j < SID_KEYS_COUNT; j++) {
		if (strcmp(name, sid_keys[j].name) == 0)
			return &sid_keys[j];
	}
	return NULL;
}

/* ---- key contents into a 128-byte buffer  -------------
 * Data word j is read from hardware index word_off + 4*j (the key array
 * uses a stride of four per data word; confirmed by 0xC290/0xC2E0 and the
 * matching write loop at 0xC4E4/0xC5A8).
 */
static void efuse_fill_key(const struct sid_key *key, uint8_t *buf)
{
	uint32_t words = (key->size_bits / 8U + 3U) / 4U;
	uint32_t j;

	for (j = 0U; j < 128U; j++)
		buf[j] = 0;

	for (j = 0U; j < words; j++) {
		uint32_t v = efuse_read_word(key->word_off + 4U * j);

		buf[4U * j] = (uint8_t)v;
		buf[4U * j + 1U] = (uint8_t)(v >> 8);
		buf[4U * j + 2U] = (uint8_t)(v >> 16);
		buf[4U * j + 3U] = (uint8_t)(v >> 24);
	}
}

/* ---- EFUSE_READ (0xBFB8 + 0xC1C4) --------------------------------------
 * Checks in the firmware's order: burn bit in word 0x7C (+44), readability
 * (protect bit0 == 1), extra flag in word 0x80 (+40).  Returns the number
 * of bytes copied to out_buf, or -1.
 */
int sunxi_efuse_read(uintptr_t key_buf, uintptr_t out_buf)
{
	const struct sid_key *key;
	uint8_t data[128];
	uint32_t bytes, i;

	key = sid_find_key(key_buf);
	if (key == NULL) {
		WARN("EFUSE: unknown key\n");
		return -1;
	}

	/* not-burned? bitmap word 0x7C, entry field +44 */
	if (key->burn_shift > 0) {
		uint32_t bmp = efuse_read_word(0x7CU);

		if (((bmp >> key->burn_shift) & 1U) == 0U) {
			WARN("EFUSE: key '%s' NOT burned\n", key->name);
			return -1;
		}
	}

	/* readable? need protect bit0 == 1 (0xC064-0xC078) */
	if ((key->protect & 0x1U) == 0U) {
		WARN("EFUSE: key '%s' is read protected\n", key->name);
		return -1;
	}

	/* extra flag? bitmap word 0x80, entry field +40 (0xC07C-0xC0A4) */
	if (key->flag_shift >= 0) {
		uint32_t bmp = efuse_read_word(0x80U);

		if (((bmp >> key->flag_shift) & 1U) != 0U) {
			WARN("EFUSE: key '%s' read blocked (flag)\n", key->name);
			return -1;
		}
	}

	bytes = key->size_bits / 8U;
	efuse_fill_key(key, data);
	for (i = 0U; i < bytes; i++)
		mmio_write_8(out_buf + i, data[i]);
	flush_dcache_range(out_buf, bytes);

	return (int)bytes;
}

/* ---- EFUSE_WRITE (0xC2EC): full burn path, faithful to the vendor BSP --------
 * key_buf points to the vendor struct: name at +0, +64 = byte length,
 * +68 = byte offset (must be 4-aligned), +72 = data pointer (64-bit when
 * the second SMC argument's low byte is non-zero, else 32-bit).
 * The burn is irreversible; the vendor BSP verifies by reading the
 * key back and comparing before setting the burn bit.
 */
int sunxi_efuse_write(uintptr_t key_buf, uintptr_t ptr_flag)
{
	const struct sid_key *key;
	uint8_t src[128], chk[128];
	uint32_t len, off, base, done, j;
	uintptr_t data;

	if (key_buf == 0U) {
		ERROR("EFUSE: null burn request\n");
		return -1;
	}

	/* second argument low byte: 1 -> 64-bit data pointer (0xC31C-0xC338) */
	if ((ptr_flag & 0xffU) != 0U)
		data = mmio_read_64(key_buf + 72);
	else
		data = mmio_read_32(key_buf + 72);

	key = sid_find_key(key_buf);
	if (key == NULL) {
		WARN("EFUSE: unknown key\n");
		return -1;
	}

	len = mmio_read_32(key_buf + 64);
	off = mmio_read_32(key_buf + 68);

	/* size / alignment gates (0xC368-0xC3C8) */
	if (key->size_bits == 0U) {
		WARN("EFUSE: key has no size\n");
		return -1;
	}
	if ((key->size_bits / 8U) < (len + off)) {
		WARN("EFUSE: burn length exceeds key size\n");
		return -1;
	}
	if ((off & 0x3U) != 0U) {
		WARN("EFUSE: burn offset not 4-aligned\n");
		return -1;
	}

	/* already burned? (0xC3CC-0xC400) */
	if (key->burn_shift > 0) {
		uint32_t bmp = efuse_read_word(0x7CU);

		if (((bmp >> key->burn_shift) & 1U) != 0U) {
			WARN("EFUSE: key '%s' already burned\n", key->name);
			return -1;
		}
	}

	if (len == 0U || len > sizeof(src)) {
		WARN("EFUSE: invalid burn length %u\n", len);
		return -1;
	}
	if (!ns_dram_ptr(data, len)) {
		WARN("EFUSE: burn data not in DRAM\n");
		return -1;
	}

	ERROR("EFUSE: burning key '%s' (irreversible!)\n", key->name);
	for (j = 0U; j < len; j++)
		src[j] = mmio_read_8(data + j);

	/* data word j goes to hardware index base + 4*j (0xC5A8) */
	base = key->word_off + off;
	done = 0U;
	while (done + 4U <= len) {
		uint32_t v = (uint32_t)src[done] |
			     ((uint32_t)src[done + 1U] << 8) |
			     ((uint32_t)src[done + 2U] << 16) |
			     ((uint32_t)src[done + 3U] << 24);

		efuse_write_word(base + 4U * (done / 4U), v);
		done += 4U;
	}
	if (done < len) {
		uint32_t v = 0U;

		for (j = done; j < len; j++)
			v |= (uint32_t)src[j] << ((j - done) * 8U);
		efuse_write_word(base + 4U * (done / 4U), v);
	}

	/* read-back verify against the NS source (0xC560-0xC5A4) */
	efuse_fill_key(key, chk);
	for (j = 0U; j < len; j++) {
		if (chk[j] != src[j]) {
			ERROR("EFUSE: verify failed at byte %u\n", j);
			return -1;
		}
	}

	/* mark burned in bitmap word 0x7C (0xC5CC-0xC5F8) */
	if (key->burn_shift > 0) {
		uint32_t bmp = efuse_read_word(0x7CU);

		bmp |= 1U << key->burn_shift;
		efuse_write_word(0x7CU, bmp);
		(void)efuse_read_word(0x7CU);
	}

	return 0;
}

/* ---- READ_SEC_REG (0xB238 + 5-interval whitelist @0x0F320) -------------
 * Whitelist recovered from the firmware: SID key window, secure status,
 * rotpk, and two single words inside CE / DMAC1 register space.
 */
struct sec_reg_range {
	uintptr_t base;
	uint32_t size;
};

static const struct sec_reg_range sec_reg_whitelist[] = {
	{ SUNXI_SID_BASE + 0x200, 0x68 },	/* key mapping window	*/
	{ SUNXI_SID_BASE + 0x0A0, 0x04 },	/* secure status	*/
	{ SUNXI_SID_BASE + 0x140, 0x04 },	/* rotpk		*/
	{ 0x04601020UL,		  0x04 },	/* CE			*/
	{ 0x04024020UL,		  0x04 },	/* DMAC1		*/
};

int sunxi_read_sec_reg(uintptr_t addr, uint32_t *value)
{
	unsigned int i;

	for (i = 0U; i < ARRAY_SIZE(sec_reg_whitelist); i++) {
		const struct sec_reg_range *r = &sec_reg_whitelist[i];

		if (addr >= r->base && addr < (r->base + r->size)) {
			*value = mmio_read_32(addr);
			return 1;
		}
	}
	WARN("EFUSE: sec-reg read 0x%lx not whitelisted\n", addr);
	return 0;
}

/* ---- WRITE_SEC_REG (0xB2B4): only two whitelisted words ---------------- */
int sunxi_write_sec_reg(uintptr_t addr, uint32_t value)
{
	/* : byte-granular ranges
	 * [0x04601020..23] and [0x04024020..23] */
	if ((addr >= 0x04601020UL && addr <= 0x04601023UL) ||
	    (addr >= 0x04024020UL && addr <= 0x04024023UL)) {
		mmio_write_32(addr, value);
		return 1;
	}
	return 0;
}

/* ---- FF07: SoC id xored with 0xA55A (0xC114) ---------------------------
 * ABI: (key_name, out_buf, len).  The firmware requires the key name to
 * be "rgn0" and the length to be exactly 4.
 */
int sunxi_efuse_read_socid_xored(uintptr_t key_name, uintptr_t out_buf,
				 uintptr_t len)
{
	static const char rgn0[4] = { 'r', 'g', 'n', '0' };
	unsigned int i;

	for (i = 0U; i < 4U; i++) {
		if (((const volatile char *)key_name)[i] != rgn0[i]) {
			WARN("EFUSE: ff07 key name mismatch\n");
			return -1;
		}
	}
	/* the firmware compares the full C string (strlen-max memcmp):
	 * "rgn0x" must be rejected, not treated as a prefix match */
	if (((const volatile char *)key_name)[4] != '\0') {
		WARN("EFUSE: ff07 key name mismatch\n");
		return -1;
	}
	if (len != 4U) {
		WARN("EFUSE: ff07 length %lu != 4\n", (unsigned long)len);
		return -1;
	}

	mmio_write_32(out_buf,
		      mmio_read_32(SUNXI_SID_BASE + 0x200) ^ 0xA55AU);
	return 4;
}

int sunxi_efuse_probe_secure(void)
{
	return (mmio_read_32(SUNXI_SID_BASE + 0x0A0) & 0x1U) != 0U ? 1 : 0;
}
