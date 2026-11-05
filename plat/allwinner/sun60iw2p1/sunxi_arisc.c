/*
 * Copyright (c) 2026, A733 open-source BL31 contributors
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * A733 arisc (E902 "CPUS") firmware loader and hwmsgbox transport.
 *
 * The wire format below follows the E902 firmware's message definitions and
 * the A733 User Manual 6.1 (MSGBOX).
 *
 * Wire format (one u32 word each into the message FIFO):
 *   word1 = state | attr<<8  | type<<16 | result<<24
 *   word2 = count
 *   word3 = paras[0] ...
 *
 * The E902 side sees MBOX_CPUX (0x03004000) as its RX box and posts
 * replies into MBOX_CPUS (0x07094000); BL31 uses channel 3 in both.
 */

#include <assert.h>

#include <platform_def.h>

#include <arch_helpers.h>
#include <common/debug.h>
#include <drivers/delay_timer.h>
#include <lib/mmio.h>
#include <lib/spinlock.h>
#include <lib/utils_def.h>

#include <sunxi_mmap.h>
#include <sunxi_private.h>

/* hwmsgbox channel used for BL31 <-> arisc */
#define ARISC_MBOX_CH			U(3)

#define MBOX_CPUX_MSG_STATUS		(SUNXI_MSGBOX_CPUX_BASE + 0x60 + ARISC_MBOX_CH * 4)
#define MBOX_CPUX_MSG_REG		(SUNXI_MSGBOX_CPUX_BASE + 0x70 + ARISC_MBOX_CH * 4)
#define MBOX_CPUS_MSG_STATUS		(SUNXI_MSGBOX_CPUS_BASE + 0x60 + ARISC_MBOX_CH * 4)
#define MBOX_CPUS_MSG_REG		(SUNXI_MSGBOX_CPUS_BASE + 0x70 + ARISC_MBOX_CH * 4)

#define MBOX_STATUS_FULL		U(8)

/* message frame */
#define ARISC_STATE_USED		U(2)
#define ARISC_ATTR_HARDSYN		U(2)	/* MESSAGE_ATTR_HARDSYN */
#define ARISC_ATTR_ASYNC		U(0)

/* E902 message types */
#define ARISC_MSG_STARTUP_NOTIFY	U(0x90)
#define ARISC_MSG_SYS_OP		U(0x24)
#define ARISC_MSG_CPU_OP		U(0x22)
/* SSTANDBY_RESTORE_NOTIFY: BL31 -> arisc after a system resume */
#define ARISC_MSG_STANDBY_RESTORE	U(0x11)
#define ARISC_MSG_LOOPBACK		U(0x61)
#define ARISC_MSG_DDRFREQ		U(0x96)
#define ARISC_MSG_DRAM_CRC		U(0x64)

/*  firmware/parameter staging (container offsets) */
#define ARISC_FW_DST			U(0x00044000)
#define ARISC_FW_SIZE			U(0x00028000)
#define ARISC_PARA_DST			SUNXI_ARISC_LOAD_BASE
#define ARISC_PARA_SIZE			U(0x00008000)
#define ARISC_ENTRY_VIEW		U(0x40004000)

#define ARISC_POLL_US			100U
#ifndef SUNXI_ARISC_TIMEOUT_MS
/* 4s was too short - on a full/bursty queue the bounded wait
 * aborted mid-frame, leaving PARTIAL frames in the FIFO.  The ARISC's
 * bounded query then dropped them, permanently desyncing the word
 * stream (zero receipts from the kernel era onwards).  The reference
 * firmware waits unbounded; use a huge-but-finite bound to keep the
 * parity without a true hang. */
#define SUNXI_ARISC_TIMEOUT_MS		120000U
#endif
#define ARISC_SEND_TIMEOUT_US		(SUNXI_ARISC_TIMEOUT_MS * 1000U)

/*
 * The E902 answers a synchronous frame within a few milliseconds.  Its own
 * feedback write is bounded by SEND_MSG_TIMEOUT (100 s), so waiting that
 * long here only turns a frame that was never sent into a two-minute
 * system freeze - a suspend that dies in this window never gets its
 * console back.  Give up early and resynchronise the queue instead.
 */
#define ARISC_REPLY_TIMEOUT_US		U(2000000)

/*
 * The resume notify is written while the E902's wait_cpu0_resume() polls the
 * AP->ARISC FIFO, so a slot frees within milliseconds.  The budget only
 * exists so a co-processor that has already left that loop cannot park the
 * AP in the resume path forever.
 */
#define ARISC_NOTIFY_TIMEOUT_US		U(5000000)

static bool arisc_ready;

/*
 * The E902 posts a frame every few seconds (its debug/monitor path) and,
 * while its own TX queue is full, stops draining the AP->ARISC queue it is
 * being sent to.  A sender that never reads therefore deadlocks against it:
 * the reboot path used to sit in arisc_mbox_wait_not_full() for ~44 s until
 * the E902's send timed out.  Draining here keeps both queues moving.
 *
 * A frame is a header, a count and `count` parameters.  The queue holds
 * eight words, so a reader that walks away in the middle of a frame leaves
 * the tail behind and the next read takes a parameter for a header.  From
 * then on both sides read past each other and every later reply is lost.
 * Remember the remainder of an abandoned frame and finish discarding it
 * before the next header is parsed.
 */
static uint32_t arisc_rx_tail;		/* parameter words still to drop */
static bool arisc_rx_tail_count;	/* the next word is that frame's count */

static bool arisc_rx_ready(void)
{
	return mmio_read_32(MBOX_CPUX_MSG_STATUS) != 0U;
}

/* Discard the tail of an abandoned frame; false when it is not all here. */
static void arisc_mbox_ack(void)
{
	mmio_write_32(SUNXI_MSGBOX_CPUX_BASE + 0x24U, 0x40U);
}

static bool arisc_rx_drop_tail(void)
{
	bool dropped = false;

	while (arisc_rx_tail_count || (arisc_rx_tail != 0U)) {
		if (!arisc_rx_ready())
			return false;
		if (arisc_rx_tail_count) {
			arisc_rx_tail_count = false;
			arisc_rx_tail = mmio_read_32(MBOX_CPUX_MSG_REG) & 0xffU;
		} else {
			(void)mmio_read_32(MBOX_CPUX_MSG_REG);
			arisc_rx_tail--;
		}
		dropped = true;
	}
	if (dropped)
		arisc_mbox_ack();
	return true;
}

static void arisc_mbox_drain_one(void)
{
	uint32_t count, i;

	if (!arisc_rx_drop_tail() || !arisc_rx_ready())
		return;

	(void)mmio_read_32(MBOX_CPUX_MSG_REG);		/* header word */

	if (!arisc_rx_ready()) {
		arisc_rx_tail_count = true;
		return;
	}

	count = mmio_read_32(MBOX_CPUX_MSG_REG) & 0xffU;
	for (i = 0U; i < count; i++) {
		if (!arisc_rx_ready()) {
			arisc_rx_tail = count - i;
			return;
		}
		(void)mmio_read_32(MBOX_CPUX_MSG_REG);
	}

	arisc_mbox_ack();
}

/* Bounded wait for one free slot, without reading the other direction:
 * the vendor's 0x1C9C send spins exactly like this, and the resume notify
 * must not consume a frame the E902 is holding for the AP. */
static bool arisc_mbox_wait_slot(uint64_t timeout_us)
{
	uint64_t t = timeout_init_us(timeout_us);

	while (mmio_read_32(MBOX_CPUS_MSG_STATUS) == MBOX_STATUS_FULL) {
		if (timeout_elapsed(t))
			return false;
		udelay(ARISC_POLL_US);
	}
	return true;
}

static bool arisc_mbox_wait_not_full(void)
{
	uint64_t t = timeout_init_us(ARISC_SEND_TIMEOUT_US);

	while (mmio_read_32(MBOX_CPUS_MSG_STATUS) == MBOX_STATUS_FULL) {
		if (timeout_elapsed(t))
			return false;
		/*
		 * The E902 stops reading our queue while its own send queue
		 * is full, and its feedback send gives up only after
		 * SEND_MSG_TIMEOUT (100 s).  Waiting here without taking
		 * its frames therefore deadlocks as soon as both queues
		 * are full - which is exactly where a suspend hand-off
		 * strands: the AP waits for room, the ARISC waits for
		 * room.  Keep reading its queue while we wait.
		 */
		arisc_mbox_drain_one();
		udelay(ARISC_POLL_US);
	}
	return true;
}

static bool arisc_mbox_wait_rx(uint64_t deadline)
{
	while (!arisc_rx_ready()) {
		if (timeout_elapsed(deadline))
			return false;
		udelay(ARISC_POLL_US);
	}
	return true;
}

/* Wait for an abandoned frame to be discarded completely. */
static bool arisc_rx_drop_tail_wait(uint64_t deadline)
{
	while (!arisc_rx_drop_tail()) {
		if (timeout_elapsed(deadline))
			return false;
		udelay(ARISC_POLL_US);
	}
	return true;
}

static spinlock_t arisc_mbox_lock;

/*
 * Read one complete reply message and ACK it: header word, count word,
 * `count` parameter words, then write 0x40 to MBOX_CPUX+0x24 (the vendor BSP
 * 0x1C30-0x1C68).  Returns the header word or 0xffffffff on timeout.
 */
static uint32_t arisc_mbox_recv(void)
{
	uint32_t word1, count, i;
	uint64_t deadline = timeout_init_us(ARISC_REPLY_TIMEOUT_US);

	/* never parse a header in the middle of an abandoned frame */
	if (!arisc_rx_drop_tail_wait(deadline) ||
	    !arisc_mbox_wait_rx(deadline)) {
		return 0xffffffffU;
	}
	word1 = mmio_read_32(MBOX_CPUX_MSG_REG);

	if (!arisc_mbox_wait_rx(deadline)) {
		arisc_rx_tail_count = true;
		return 0xffffffffU;
	}
	count = mmio_read_32(MBOX_CPUX_MSG_REG) & 0xffU;
	for (i = 0U; i < count; i++) {
		if (!arisc_mbox_wait_rx(deadline)) {
			arisc_rx_tail = count - i;
			return 0xffffffffU;
		}
		(void)mmio_read_32(MBOX_CPUX_MSG_REG);
	}

	arisc_mbox_ack();
	return word1;
}

/* flow control - wait until the ARISC has dequeued the frame.
 * The CPUX->CPUS FIFO is 8 words deep; both sides use MSG_NUM (a
 * *message* count) as the "full" test, so word-level overflow silently
 * covers old data.  Waiting for MSG_NUM == 0 after each frame keeps the
 * sender in step with the ARISC's drain (the vendor BSP's ~3ms
 * frame spacing achieves the same). */
/* sticky send-failure telemetry in RTC+0x120 (A5E1_pp_ss:
 * pp = the failure point, ss = the mailbox status low byte at the
 * failure).  The suspend path's stage writes overwrite RTC+0x11C, so
 * send failures used to become invisible right after a failed suspend
 * (and 0x120 is free - the rtc2 window is all zero).  Sticky: only the
 * first failure is kept. */
/*
 * Blocking send of one message frame; returns the reply word1 (which
 * carries state/attr/type/result - BL31 reads the result byte).
 * Serialised with the same kind of spin lock the firmware uses.
 */
/*
 * Reserve room for a whole frame before its header goes out.  A frame that
 * is started and then abandoned leaves its tail in the E902's eight-word
 * receive queue; from then on the firmware parses a parameter as the next
 * header and every later request, the suspend hand-off included, is read
 * out of position.  Frames longer than the queue can only ever fit while
 * the receiver drains them as we write, so there is nothing to reserve for
 * those.
 */
static bool arisc_mbox_reserve(unsigned int words)
{
	uint64_t t = timeout_init_us(ARISC_SEND_TIMEOUT_US);

	if ((words + 2U) > MBOX_STATUS_FULL)
		return true;

	while ((mmio_read_32(MBOX_CPUS_MSG_STATUS) + words + 2U) >
	       MBOX_STATUS_FULL) {
		if (timeout_elapsed(t))
			return false;
		arisc_mbox_drain_one();
		udelay(ARISC_POLL_US);
	}
	return true;
}

static uint32_t arisc_msg_send(uint32_t type, uint32_t attr,
			       const uint32_t *paras, unsigned int count)
{
	uint32_t word1 = ARISC_STATE_USED | (attr << 8) | (type << 16);
	uint32_t reply;
	unsigned int i;

	/*
	 * The E902 posts a frame every few seconds and stops draining the
	 * AP->ARISC queue while its own TX queue is full.  Take its pending
	 * messages first so it keeps reading ours.
	 */
	for (i = 0U; i < 16U; i++)
		arisc_mbox_drain_one();

	spin_lock(&arisc_mbox_lock);

	if (!arisc_mbox_reserve(count) ||
	    !arisc_mbox_wait_not_full()) {
		spin_unlock(&arisc_mbox_lock);
		return 0xffffffffU;
	}
	mmio_write_32(MBOX_CPUS_MSG_REG, word1);

	if (!arisc_mbox_wait_not_full()) {
		spin_unlock(&arisc_mbox_lock);
		return 0xffffffffU;
	}
	mmio_write_32(MBOX_CPUS_MSG_REG, count);

	for (i = 0; i < count; i++) {
		if (!arisc_mbox_wait_not_full()) {
			spin_unlock(&arisc_mbox_lock);
			return 0xffffffffU;
		}
		mmio_write_32(MBOX_CPUS_MSG_REG, paras[i]);
	}

	/* drain-wait REMOVED for vendor parity.  The reference send
	 * helper only gates each
	 * write on "status == 8" and never waits for the ARISC to dequeue
	 * the frame.  Our  addition polled the very status register
	 * the ARISC reader uses; the -219 hand-off failures (arming
	 * 0x26 and cpu_op 0x22 frames vanishing, mailbox st=0 at the
	 * failure point) point at this extra polling perturbing the
	 * single-queue interleave. */

	if ((attr & ARISC_ATTR_HARDSYN) == 0U) {
		spin_unlock(&arisc_mbox_lock);
		return 0U;
	}

	reply = arisc_mbox_recv();
	spin_unlock(&arisc_mbox_lock);

	return reply;
}

/*
 * Drop everything the E902 has posted and the normal world has not taken.
 * The queue is only eight words deep and the kernel does not always read
 * what it is sent; once it is full the E902 blocks for its whole feedback
 * timeout and the reply the kernel is waiting for never leaves the
 * co-processor.  Called from the CPU_OFF and system-suspend paths, i.e.
 * while the AP is about to hand over and no reply is outstanding.
 */
void sunxi_arisc_rx_drain(void)
{
	unsigned int i;

	if (!arisc_ready)
		return;

	spin_lock(&arisc_mbox_lock);
	for (i = 0U; i < 32U; i++)
		arisc_mbox_drain_one();
	spin_unlock(&arisc_mbox_lock);
}

/* async frame with a chosen type (see sunxi_private.h).
 * fire-and-forget without fifo waits - in the warm path (immediately
 * after the cluster powers up) the AP system counter is stalled, so the
 * generic send's wait_drained would spin forever while the ARISC pump is
 * in its ~10s post-kick stall. */
/*
 * Wait until the E902 has taken the words the AP wrote: the AP->ARISC
 * queue status drops back to 0.  Bounded, because a wedged co-processor
 * must not keep the caller spinning.
 */
static bool arisc_mbox_wait_drained(uint32_t timeout_us)
{
	uint64_t t = timeout_init_us(timeout_us);

	while (mmio_read_32(MBOX_CPUS_MSG_STATUS) != 0U) {
		if (timeout_elapsed(t))
			return false;
		udelay(ARISC_POLL_US);
	}
	return true;
}

static uint32_t arisc_attr_for_type(uint32_t type)
{
	switch (type) {
	case ARISC_MSG_SYS_OP:
	case ARISC_MSG_LOOPBACK:
	case ARISC_MSG_DDRFREQ:
	case ARISC_MSG_DRAM_CRC:
		return ARISC_ATTR_ASYNC;
	default:
		return ARISC_ATTR_HARDSYN;
	}
}

int sunxi_arisc_msg_send(uint32_t type, const uint32_t *paras,
			 unsigned int count)
{
	uint32_t reply = arisc_msg_send(type, arisc_attr_for_type(type),
					paras, count);

	/* result is the top byte of word1 */
	return (int)((reply >> 24) & 0xffU);
}

int sunxi_arisc_sys_op(uint32_t state)
{
	unsigned int i;

	/*
	 * SYS_OP is ASYNC: the E902 sends no reply, so the best sign of
	 * delivery is that it took the words off the queue.  Its message
	 * pump stops reading while its own TX queue is full, and a frame
	 * whose header is lost is discarded as an unknown message, so try
	 * a few times; arisc_msg_send() drains the other direction first.
	 */
	for (i = 0U; i < 3U; i++) {
		/*
		 * Wait for the AP->ARISC queue to run empty before writing:
		 * the E902 parses a frame as header/count/params, so words
		 * left over from an earlier frame would shift ours and make
		 * it discard the whole thing as an unknown message.
		 */
		(void)arisc_mbox_wait_drained(500000U);	/* 500 ms */
		(void)arisc_msg_send(ARISC_MSG_SYS_OP, ARISC_ATTR_ASYNC,
				     &state, 1U);
		if (arisc_mbox_wait_drained(100000U))	/* 100 ms */
			break;
		udelay(1000U);
	}

	return 0;
}

/*
 * CPU_OP_REQ (type 0x22), async - the exact frame the vendor BSP
 * builds at 0x19F0 before a system suspend:
 *   paras = {mpidr, resume_entry, cpu_state, cluster_state, system_state}
 * all three states = 3 (arisc_power_off) tell the arisc to run its
 * super-standby flow (DRAM self refresh, clock switch, wake sources)
 * and to resume the boot core from `resume_entry` afterwards.
 */
/*
 * SSTANDBY_RESTORE_NOTIFY: tell the arisc the AP-side resume is done so
 * it can clear its standby bookkeeping. The reply word (state/attr/type/
 * result packed) is returned so callers can expose the result code.
 * 
 */
int sunxi_arisc_standby_restore_notify(void)
{
	/*
	 * The arisc's wait_cpu0_resume() passes a message with paras == NULL
	 * to hwmsgbox_query_message(), which stores every parameter word
	 * through that pointer.  count must therefore stay 0: the notify
	 * carries no parameters (the vendor frame does the same).
	 */
	const uint32_t frame[2] = {
		0x02U | (0U << 8) | (0x11U << 16),	/* state=2, attr=0(ASYNC), type=0x11 */
		0U,
	};

	/*
	 * Match the vendor 0x1C9C send: MBOX_STATUS_FULL is the FIFO depth
	 * (8 = full), and the vendor busy-waits for a free slot before each
	 * word.  The ARISC's wait_cpu0_resume() polls this FIFO, so a slot
	 * always frees.  Dropping the notification when the FIFO is full
	 * leaves the ARISC stuck in wait_cpu0_resume; it then never prints
	 * "cpu0 restore finished" and never releases the clocks/UART.
	 */
	if (!arisc_mbox_wait_slot(ARISC_NOTIFY_TIMEOUT_US)) {
		WARN("BL31: arisc restore notify: queue stayed full\n");
		return -1;
	}
	mmio_write_32(MBOX_CPUS_MSG_REG, frame[0]);

	if (!arisc_mbox_wait_slot(ARISC_NOTIFY_TIMEOUT_US)) {
		WARN("BL31: arisc restore notify: queue stayed full\n");
		return -1;
	}
	mmio_write_32(MBOX_CPUS_MSG_REG, frame[1]);
	return 0;
}

int sunxi_arisc_cpu_op(u_register_t mpidr, uintptr_t resume_entry,
		       uint32_t cpu_state, uint32_t cluster_state,
		       uint32_t system_state)
{
	const uint32_t paras[5] = {
		(uint32_t)mpidr,
		(uint32_t)resume_entry,
		cpu_state,
		cluster_state,
		system_state,
	};

	/* CPU_OP attr back to ASYNC.   tried HARDSYN to match
	 * the vendor frame, but our arisc_mbox_recv() waits for a *reply*
	 * while the vendor 0x1C9C HARDSYN only waits for the RX-ack
	 * (0x40 -> 0x03004024); the ARISC enters standby and never sends a
	 * reply, so HARDSYN wedges 4s (0xffffffff) before the wfi - wrong
	 * direction.  ASYNC fire-and-forget matches the observable vendor
	 * timing (BL31 reaches the PSCI wfi immediately after CPU_OP). */
	return arisc_msg_send(ARISC_MSG_CPU_OP, ARISC_ATTR_ASYNC, paras, 5U);
}

/*
 * E902 bring-up, mirroring :
 *   1. copy the 160KiB firmware 0x48100000 -> 0x44000 (E902 view 0x40004000)
 *   2. copy the 32KiB parameter block 0x4812C000 -> 0x48100000
 *   3. de-assert RISCV_BGR resets, program E902_RST_START_ADDR
 *   4. open RISCV_24M clock gate, release the undocumented hold @0x50005120
 *   5. wait for the 0x90 startup notify and answer its HARDSYN reply
 */
int sunxi_arisc_init(uint32_t boot_arg)
{
	const uint32_t *fw = (const uint32_t *)(uintptr_t)SUNXI_ARISC_LOAD_BASE;
	const uint32_t *para = (const uint32_t *)(uintptr_t)SUNXI_ARISC_PARA_LOAD_BASE;
	uint32_t *fw_dst = (uint32_t *)(uintptr_t)ARISC_FW_DST;
	uint32_t *para_dst = (uint32_t *)(uintptr_t)ARISC_PARA_DST;
	unsigned int i;

	/*
	 * Safety hardening over the vendor BSP: the loader re-runs its full
	 * copy/release sequence on every FF10 call, which is only safe when
	 * the E902 is still held in reset.  With a platform-side starter
	 * (our EDK2 AriscStartupDxe) a later FF10 - e.g. the test battery's
	 * F2 - must not overwrite the firmware of the already running core.
	 * Report the existing session instead.
	 */
	if (arisc_ready) {
		NOTICE("BL31: arisc already running\n");
		return 0;
	}

	/* The reference loader (0xAF64) re-runs the full copy/reset/startup
	 * sequence on every FF10 call; the handshake wait is bounded by our
	 * timeout so a stale flag cannot hang the core. */

	NOTICE("BL31: arisc: stage rtc-arg\n");
	/* RTC+0x10C: boot argument consumed by the E902 startup code */
	mmio_write_32(SUNXI_RTC_BASE + 0x10C, boot_arg);

	/* Enable the MSGBOX clocks exactly as the ARISC mclk.c/reset.c does
	 * (hwmsgbox_init): CCU_MSGBOX_BGR (0x02002744) + R_MSGBOX_GATE_RST
	 * (0x0701017C), bit0 = clock gate on, bit16 = reset de-assert.  The
	 * E902 enables its side in hwmsgbox_init; without the CPUX side
	 * running the RX status register (0x0300406C) stays 0 and the 0x90
	 * startup notify below is never seen. */
	mmio_setbits_32(0x02002744UL, BIT(0));   /* CCU_MSGBOX_BGR: gate on */
	mmio_setbits_32(0x0701017CUL, BIT(0));   /* R_MSGBOX_GATE_RST: gate on */
	mmio_clrbits_32(0x02002744UL, BIT(16));  /* CCU_MSGBOX_BGR: reset de-assert */
	mmio_clrbits_32(0x0701017CUL, BIT(16));  /* R_MSGBOX_GATE_RST: reset de-assert */

	NOTICE("BL31: arisc: stage fw-copy\n");
	/* firmware: 0x48100000 -> 0x44000, 160 KiB  */
	for (i = 0; i < ARISC_FW_SIZE / sizeof(uint32_t); i++)
		fw_dst[i] = fw[i];
	flush_dcache_range(ARISC_FW_DST, ARISC_FW_SIZE);

	NOTICE("BL31: arisc: stage para-copy\n");
	/* parameters: 0x4812C000 -> 0x48100000, 32 KiB (0xAFB0-0xAFC8) */
	for (i = 0; i < ARISC_PARA_SIZE / sizeof(uint32_t); i++)
		para_dst[i] = para[i];
	flush_dcache_range(ARISC_PARA_DST, ARISC_PARA_SIZE);

	NOTICE("BL31: arisc: stage start\n");
	/* RISCV_BGR: de-assert CFG reset, open CFG + core bus gates */
	mmio_write_32(SUNXI_S_PRCM_BASE + 0x21C, 0x00010003U);
	/* E902_RST_START_ADDR: E902 view of SRAM_A2 entry (manual 5.2.4.8;
	 * the vendor BSP writes 0x40004000 = 0x00044000 physical) */
	mmio_write_32(SUNXI_CPUS_E902_CFG_BASE + 0x204, ARISC_ENTRY_VIEW);
	/* CPUIDLE: enable the E902 clock/power switch */
	mmio_setbits_32(SUNXI_CPUIDLE_BASE + 0x8, BIT(0));
	/* RISCV_24M clock gate */
	mmio_setbits_32(SUNXI_S_PRCM_BASE + 0x210, BIT(31));
	/* undocumented hold register, not in the manual memory map */
	mmio_clrbits_32(0x50005120U, BIT(0));

	dsb();
	isb();

	/* wait for the startup notification - bounded: if the E902 firmware was not
	 * staged (scp TOC1 item missing) or fails to boot, do not hang the
	 * whole BL31 here; skip and let BL33 run. */
	NOTICE("BL31: waiting for arisc startup notification\n");
	{
		unsigned int arisc_wait_loops = 2000000U;  /* ~1s bounded */

		while (arisc_wait_loops-- != 0U) {
			uint32_t word1 = arisc_mbox_recv();

			if (word1 == 0xffffffffU)
				break;
			if (((word1 >> 16) & 0xffU) == ARISC_MSG_STARTUP_NOTIFY) {
				/* HARDSYN notify (E902 daemon.c): reply with the
				 * echoed header + 1-word payload (0x1A98) unless
				 * attr is 0 */
				if (((word1 >> 8) & 0x3U) != 0U) {
					if (arisc_mbox_wait_not_full())
						mmio_write_32(MBOX_CPUS_MSG_REG, word1);
					if (arisc_mbox_wait_not_full())
						mmio_write_32(MBOX_CPUS_MSG_REG, 1U);
					/* -0xB1D0: one payload word */
					if (arisc_mbox_wait_not_full())
						mmio_write_32(MBOX_CPUS_MSG_REG, 0U);
				}
				NOTICE("BL31: arisc started, version 0x%x\n", word1);
				arisc_ready = true;
				break;
			}
		}
		if (!arisc_ready)
			NOTICE("BL31: arisc startup timeout (fw not staged?), continuing\n");
	}

	return arisc_ready ? 0 : -1;
}
