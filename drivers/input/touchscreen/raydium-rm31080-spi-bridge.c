// SPDX-License-Identifier: GPL-2.0
/*
 * raydium-rm31080-spi-bridge.c
 *
 * Mainline swap-in for the downstream NVIDIA/Raydium "rm31080a_ts.c" misc
 * device driver (drivers/input/touchscreen/rm31080a_ts.c +
 * rm31080a_ctrl.c in android_kernel_nvidia_shield-lineage-18.1), rewritten
 * against modern DT/gpiod/regulator/clk/spi APIs.
 *
 * This driver is NOT a reimplementation of the Raydium touch algorithm.
 * All of that logic (register maps, scan tuning, coordinate calculus,
 * calibration) lives in the closed-source userspace stack (ts.default.so +
 * librm31080.so, invoked through the "rm-wrapper" shim) that is used
 * unmodified. This driver exists purely to reproduce, at the kernel/ABI
 * boundary, the exact protocol that userspace already speaks to the
 * downstream driver:
 *
 *   - a misc device node named "touch" (-> /dev/touch)
 *   - read()/write() as near-literal single-transaction SPI accesses
 *   - an ioctl surface (RM_IOCTL_*) for HAL-pid registration, touch-point
 *     injection, scalar get/set variables, raw scan buffer draining, and
 *     uploading 16 tables of small opcodes ("KRL" tables) that this driver
 *     must be able to execute against real hardware (regulators, GPIOs,
 *     the touch clock, and the SPI bus itself) whenever userspace asks for
 *     resume/suspend/watchdog/scan-start/etc.
 *   - realtime-signal (SIGRTMIN+12, i.e. signal 44) delivery to a
 *     registered HAL pid whenever new scan data is ready
 *
 * Every piece of protocol behaviour below was verified against the
 * downstream kernel source (functions named in comments) and, where
 * relevant, against a decompilation of the userspace binaries that
 * actually drive this hardware. Anywhere this port simplifies or
 * defers downstream behaviour, it is called out explicitly with a
 * "NOTE:" comment -- search for NOTE: before relying on this driver
 * for anything beyond basic touch + resume/suspend.
 *
 * Devicetree bindings expected on the SPI child node (see the
 * "raydium,rm31080" node already present in tegra124-shieldtablet.dtsi):
 *
 *   compatible      = "raydium,rm31080";
 *   reg             = <0>;
 *   spi-max-frequency = <18000000>;
 *   interrupts (or interrupts-extended) = the IRQ GPIO, IRQ_TYPE_EDGE_RISING
 *   reset-gpios     = the reset GPIO, GPIO_ACTIVE_LOW
 *   avdd-supply     = 3.3V analog rail
 *   dvdd-supply     = 1.8V digital rail
 *   avdd-gpios      = optional; discrete-GPIO alternative to avdd-supply on
 *                      boards that switch this rail via GPIO instead of a
 *                      regulator. Not used by this board's DT (avdd-supply
 *                      above covers it) -- see krl_config_3v3().
 *   dvdd-gpios      = same, for the 1.8V rail / dvdd-supply
 *   clocks / clock-names = "extern2" (the touch clock parent chain)
 *   raydium,platform-id  = <0x0b>   (RM_PLATFORM_T008; REQUIRED, see below)
 *   raydium,gpio-select  = <0x00>  (optional, defaults to 0)
 *
 * raydium,platform-id and raydium,gpio-select are NOT part of the
 * downstream binding doc (which only documented spidev-style properties);
 * downstream supplied these two values from board-file C code
 * (board-ardbeg.c: rm31080ts_tn8_data.platform_id = RM_PLATFORM_T008).
 * Since we have no board file, they must come from DT instead. Userspace
 * fetches them via RM_IOCTL_GET_VARIABLE and uses them to pick which
 * "para_XX_YY_ZZ_WW.so" config blob to dlopen() -- get this wrong and the
 * HAL silently loads the wrong board's tuning data.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/atomic.h>
#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/sched/signal.h>
#include <linux/pid.h>
#include <linux/signal.h>
#include <linux/uaccess.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/miscdevice.h>
#include <linux/kobject.h>
#include <linux/pm_wakeup.h>
#include <linux/fs.h>
#include <linux/poll.h>
#include <linux/spi/spi.h>
#include <linux/gpio/consumer.h>
#include <linux/regulator/consumer.h>
#include <linux/clk.h>
#include <linux/of.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/pm.h>
#include <linux/property.h>

/*
 * MSC_ACTIVITY isn't in mainline uapi/linux/input.h (it stops at
 * MSC_TIMESTAMP/MSC_MAX=0x06); downstream's Android-common-kernel-derived
 * uapi/linux/input.h adds MSC_TIMESEC, MSC_TIMEUSEC, and this at 0x08.
 * Defined locally rather than depending on kernel headers we don't
 * control, matching the confirmed downstream value exactly.
 */
#ifndef MSC_ACTIVITY
#define MSC_ACTIVITY			0x08
#endif

#define DRV_NAME "raydium-rm31080-spi-bridge"

/* =========================================================================
 * Protocol constants
 * Ported verbatim from include/linux/spi/rm31080a_ts.h and
 * include/linux/spi/rm31080a_ctrl.h in the downstream tree. These are the
 * ABI between this driver and the userspace HAL and must not be changed.
 * ========================================================================= */

#define RM_TS_SIGNAL			44	/* realtime signal used for events */

#define RM_SIGNAL_INTR			0x00000001
#define RM_SIGNAL_SUSPEND		0x00000002
#define RM_SIGNAL_RESUME		0x00000003
#define RM_SIGNAL_CHANGE_PARA		0x00000004
#define RM_SIGNAL_WATCH_DOG_CHECK	0x00000005
#define RM_SIGNAL_REPORT_MODE_CHANGE	0x00000006

/* scan_mode_state values, ported from rm31080a_ts.c */
#define RM_SCAN_ACTIVE_MODE		0x00
#define RM_SCAN_PRE_IDLE_MODE		0x01
#define RM_SCAN_IDLE_MODE		0x02

/* rm31080_ctrl_configure() return flags, ported from rm31080a_ts.c */
#define RM_NEED_NONE			0x00
#define RM_NEED_TO_SEND_SCAN		0x01
#define RM_NEED_TO_READ_RAW_DATA	0x02
#define RM_NEED_TO_SEND_SIGNAL		0x04

#define RM_IOCTL_REPORT_POINT		0x1001
#define RM_IOCTL_SET_HAL_PID		0x1002
#define RM_IOCTL_INIT_START		0x1003
#define RM_IOCTL_INIT_END		0x1004
#define RM_IOCTL_FINISH_CALC		0x1005
#define RM_IOCTL_SCRIBER_CTRL		0x1006
#define RM_IOCTL_READ_RAW_DATA		0x1007
#define RM_IOCTL_SET_PARAMETER		0x100A
#define RM_IOCTL_SET_VARIABLE		0x1010
#define RM_IOCTL_GET_VARIABLE		0x1011
#define RM_IOCTL_GET_SCAN_MODE		0x1012	/* downstream calls it "SACN" */
#define RM_IOCTL_SET_KRL_TBL		0x1013
#define RM_IOCTL_WATCH_DOG		0x1014
#define RM_IOCTL_SET_BASELINE		0x1015
#define RM_IOCTL_INIT_SERVICE		0x1016
#define RM_IOCTL_SET_CLK		0x1017

/* RM_IOCTL_SET_VARIABLE / RM_IOCTL_GET_VARIABLE index space (packed into
 * the upper 16 bits of the ioctl cmd by userspace: (index << 16) | ioctl) */
#define RM_VARIABLE_SELF_TEST_RESULT	0x01
#define RM_VARIABLE_SCRIBER_FLAG	0x02
#define RM_VARIABLE_AUTOSCAN_FLAG	0x03
#define RM_VARIABLE_VERSION		0x04
#define RM_VARIABLE_IDLEMODECHECK	0x05
#define RM_VARIABLE_REPEAT		0x06
#define RM_VARIABLE_WATCHDOG_FLAG	0x07
#define RM_VARIABLE_TEST_VERSION	0x08
#define RM_VARIABLE_SET_SPI_UNLOCK	0x09
#define RM_VARIABLE_SET_WAKE_UNLOCK	0x0A
#define RM_VARIABLE_DPW			0x0B
#define RM_VARIABLE_NS_MODE		0x0C
#define RM_VARIABLE_TOUCHFILE_STATUS	0x0D
#define RM_VARIABLE_TOUCH_EVENT		0x0E

/* RM_VARIABLE_TOUCH_EVENT's arg values, ported from enum tch_update_reason
 * in rm31080a_ts.h. Confirmed live: librm31080.so's WaterAreaDetection()
 * and its noise-detection counterpart call SendTouchEvent2Kernel(1) / (2)
 * / (0xff) exactly matching these. */
#define RM_TOUCH_EVENT_STYLUS_DISABLE_BY_WATER	0x01
#define RM_TOUCH_EVENT_STYLUS_DISABLE_BY_NOISE	0x02
#define RM_TOUCH_EVENT_STYLUS_IS_ENABLED	0xFF

/* GET_VARIABLE has its own, differently-numbered index space */
#define RM_VARIABLE_PLATFORM_ID	0x01
#define RM_VARIABLE_GPIO_SELECT	0x02
#define RM_VARIABLE_CHECK_SPI_LOCK	0x03

#define MASK_USER_SPACE_POINTER	0x00000000FFFFFFFFULL

/* KRL ("kernel table") bytecode -- executed by this driver whenever
 * userspace asks for resume/suspend/watchdog/scan-start/etc. Table content
 * is uploaded at runtime by userspace (RM_IOCTL_SET_KRL_TBL); we only need
 * to implement the interpreter, ported from rm_tch_cmd_process() in
 * rm31080a_ts.c. */
#define KRL_TBL_FIELD_POS_LEN_H	0
#define KRL_TBL_FIELD_POS_LEN_L	1
#define KRL_TBL_FIELD_POS_CASE_NUM	2
#define KRL_TBL_FIELD_POS_CMD_NUM	3
#define KRL_TBL_CMD_LEN			3
#define KRL_TBL_MAX_LEN			0x680	/* 1664 bytes, per-index buffer */
#define KRL_TBL_COUNT			17	/* indices 0..16 */

#define KRL_CMD_READ			0x11
#define KRL_CMD_WRITE_W_DATA		0x12
#define KRL_CMD_WRITE_WO_DATA		0x13
#define KRL_CMD_IF_AND_OR		0x14
#define KRL_CMD_AND			0x18
#define KRL_CMD_OR			0x19
#define KRL_CMD_NOT			0x1A
#define KRL_CMD_XOR			0x1B
#define KRL_CMD_WRITE_W_COUNT		0x1C
#define KRL_CMD_RETURN_RESULT		0x1D
#define KRL_CMD_RETURN_VALUE		0x1E
#define KRL_CMD_DRAM_INIT		0x1F
#define KRL_CMD_SEND_SIGNAL		0x20
#define KRL_CMD_CONFIG_RST		0x21
#define KRL_CMD_SET_TIMER		0x22
#define KRL_CMD_CONFIG_3V3		0x23
#define KRL_CMD_CONFIG_1V8		0x24
#define KRL_CMD_CONFIG_CLK		0x25
#define KRL_CMD_CONFIG_CS		0x26
#define KRL_CMD_MSLEEP			0x40
#define KRL_CMD_FLUSH_QU		0x52
#define KRL_CMD_READ_IMG		0x60
#define KRL_CMD_WRITE_IMG		0x61
#define KRL_CMD_CONFIG_IRQ		0x70
#define KRL_CMD_DUMMY			0xFF

#define KRL_SUB_CMD_SET_RST_GPIO	0x00
#define KRL_SUB_CMD_SET_RST_VALUE	0x01

#define KRL_SUB_CMD_SET_3V3_GPIO	0x00
#define KRL_SUB_CMD_SET_3V3_REGULATOR	0x01
#define KRL_SUB_CMD_SET_1V8_GPIO	0x00
#define KRL_SUB_CMD_SET_1V8_REGULATOR	0x01

#define KRL_SUB_CMD_SET_CLK		0x00
#define KRL_SUB_CMD_SET_CS_LOW		0x00
#define KRL_SUB_CMD_SET_IRQ		0x00

#define KRL_INDEX_FUNC_SET_IDLE	0
#define KRL_INDEX_FUNC_PAUSE_AUTO	1
#define KRL_INDEX_RM_RESUME		2
#define KRL_INDEX_RM_SUSPEND		3
#define KRL_INDEX_RM_READ_IMG		4
#define KRL_INDEX_RM_WATCHDOG		5
#define KRL_INDEX_RM_TESTMODE		6
#define KRL_INDEX_RM_SLOWSCAN		7
#define KRL_INDEX_RM_CLEARINT		8
#define KRL_INDEX_RM_SCANSTART		9
#define KRL_INDEX_RM_WAITSCANOK	10
#define KRL_INDEX_RM_SETREPTIME	11
#define KRL_INDEX_RM_NSPARA		12
#define KRL_INDEX_RM_WRITE_IMG		13
#define KRL_INDEX_RM_TLK		14
#define KRL_INDEX_RM_KL_TESTMODE	15
#define KRL_INDEX_RM_NS_SCF		16

#define RETURN_OK	0
#define RETURN_FAIL	1

/* Touch-point ABI struct -- MUST match struct rm_touch_event in
 * include/linux/spi/rm31080a_ts.h byte-for-byte; userspace copy_from_user's
 * this directly. Do not add/remove/reorder fields. */
#define RM_TS_MAX_POINTS	16

struct rm_touch_event {
	unsigned char uc_touch_count;
	unsigned char uc_id[RM_TS_MAX_POINTS];
	unsigned char uc_tool_type[RM_TS_MAX_POINTS];
	unsigned short us_x[RM_TS_MAX_POINTS];
	unsigned short us_y[RM_TS_MAX_POINTS];
	unsigned short us_z[RM_TS_MAX_POINTS];
	unsigned short us_tilt_x[RM_TS_MAX_POINTS];
	unsigned short us_tilt_y[RM_TS_MAX_POINTS];
	unsigned char uc_slot[RM_TS_MAX_POINTS];
	unsigned char uc_pre_tool_type[RM_TS_MAX_POINTS];
};

#define INPUT_SLOT_RESET	0x80
#define INPUT_ID_RESET		0xFF
#define POINT_TYPE_NONE		0x00
#define POINT_TYPE_STYLUS	0x01
#define POINT_TYPE_ERASER	0x02
#define POINT_TYPE_FINGER	0x03
#define POINT_TYPE_THUMB	0x04

#define MAX_SLOT_AMOUNT		10	/* MAX_REPORT_TOUCHED_POINTS downstream */
/* Compile-time fallback ONLY, used when the DT node has no
 * touchscreen-size-x/y properties (see rm->default_res_x/y and the DTS
 * NOTE on those properties). Not this board's real panel resolution --
 * deliberately generic so a board without DT properties still gets a
 * consistent (if uncalibrated) axis range instead of an unset one. */
#define RM_INPUT_RESOLUTION_X	4096
#define RM_INPUT_RESOLUTION_Y	4096

/* RM_IOCTL_SET_PARAMETER ABI struct -- MUST match struct rm_tch_ctrl_para
 * in include/linux/spi/rm31080a_ctrl.h byte-for-byte. Userspace copies the
 * whole struct in one shot (rm_tch_ctrl_set_parameter()). */
struct rm_tch_ctrl_para {
	unsigned short u16_data_length;
	unsigned short u16_ic_version;
	unsigned short u16_resolution_x;
	unsigned short u16_resolution_y;
	unsigned char u8_active_digital_repeat_times;
	unsigned char u8_analog_repeat_times;
	unsigned char u8_idle_digital_repeat_times;
	unsigned char u8_time2idle;
	unsigned char u8_kernel_msg;
	unsigned char u8_timer_trigger_scale;
	unsigned char u8_idle_mode_check;
	unsigned char u8_watch_dog_normal_cnt;
	unsigned char u8_ns_func_enable;
	unsigned char u8_event_report_mode;
	unsigned char u8_idle_mode_thd;
};

/* Raw-scan-image ring buffer feeding RM_IOCTL_READ_RAW_DATA.
 * QUEUE_COUNT / RM_RAW_DATA_LENGTH match the downstream #defines exactly
 * (rm31080a_ts.c) -- userspace has no way to learn these, so they must be
 * identical or the ring buffer sizing silently mismatches. */
#define QUEUE_COUNT		128
#define RM_RAW_DATA_LENGTH	6144

/*
 * 8-byte per-scan header prepended to each raw-data buffer, ported from
 * the ENABLE_FREQ_HOPPING ("ENABLE_SCAN_DATA_HEADER") branch of
 * rm_tch_read_image_data() in the L4T-forked driver. Confirmed needed:
 * userspace's RM_IOCTL_READ_RAW_DATA request length is
 * ctrl.u16_data_length + QUEUE_HEADER_NUM exactly (observed 0x1228 = 4648
 * requested vs. 4640 reported data_length -- an 8-byte gap). The
 * noise-scan-channel byte (offset 3) reflects the actually-selected
 * channel when frequency hopping is enabled -- see the ns_* fields on
 * struct rm31080_data and rm31080_ctrl_scan_start().
 */
#define QUEUE_HEADER_NUM	8
#define SCAN_TYPE_MT		1

/* =========================================================================
 * Driver private state
 * ========================================================================= */

struct rm31080_data {
	struct spi_device *spi;
	struct miscdevice miscdev;
	/*
	 * NOT a downstream field -- deliberate hardening beyond faithful
	 * parity. Downstream's dev_release() (and our original port of it)
	 * unconditionally clears b_init_finish/init_finished and force-exits
	 * idle mode on ANY close() of ANY fd, because there's a single
	 * global driver instance shared by every opener. Downstream gets
	 * away with this because nothing but the HAL is ever expected to
	 * touch the device node. That assumption doesn't hold once anything
	 * else (a debug tool, a udev rule, a stray `stat`-then-open from
	 * some unrelated userspace helper) opens and closes it while the
	 * HAL's own long-lived session is still active: that incidental
	 * close silently kills live touch reporting until the HAL restarts
	 * -- observed directly during AUTOSCAN_FLAG testing. This refcount
	 * makes rm31080_release()'s reset logic only run on the LAST close,
	 * which is what downstream's design clearly intends even though its
	 * code doesn't enforce it.
	 */
	atomic_t open_count;
	struct input_dev *input;

	struct gpio_desc *gpio_reset;
	struct gpio_desc *gpio_irq;	/* only used if the DT node doesn't
					 * already give us spi->irq */
	int irq;

	struct regulator *avdd;	/* 3.3V analog supply */
	struct regulator *dvdd;	/* 1.8V digital supply */
	/*
	 * Optional discrete-GPIO alternative to avdd-supply/dvdd-supply,
	 * ported from downstream's pdata->gpio_3v3/gpio_1v8 (consulted by
	 * KRL_CMD_CONFIG_3V3/1V8's GPIO sub-cmd, as opposed to the
	 * REGULATOR sub-cmd that avdd/dvdd above serve). NULL on this
	 * board -- our DT always models both rails as real regulators, so
	 * this is dead weight here -- but not every board with this
	 * touchscreen necessarily does; wired up for that portability, not
	 * because this board needs it. See krl_config_3v3()/_1v8().
	 */
	struct gpio_desc *gpio_avdd;
	struct gpio_desc *gpio_dvdd;
	struct clk *clk;		/* touch clock (extern2/clk_out_2 chain) */

	u32 platform_id;		/* from DT raydium,platform-id */
	u32 gpio_select;		/* from DT raydium,gpio-select */

	/*
	 * ABS_(MT_)X/Y max we advertise at input_register_device() time and
	 * fall back to in rm31080_report_pointer() until RM_IOCTL_SET_PARAMETER
	 * (if ever) supplies ctrl.u16_resolution_x/y instead. From DT
	 * touchscreen-size-x/y when present, else RM_INPUT_RESOLUTION_X/Y.
	 * This -- not a runtime ioctl from a closed-source HAL that may not
	 * fire before some other process has already enumerated the input
	 * device -- is what makes the axis range correct from first
	 * enumeration onward, which is the whole point: it's what lets
	 * userspace (libinput, and therefore both X11 and Wayland
	 * compositors) auto-calibrate without a manual Xorg "Calibration"
	 * quirk. See the NOTE on touchscreen-size-x/y in the DTS.
	 */
	u16 default_res_x;
	u16 default_res_y;

	/*
	 * Replaces the old Xorg configuration.
	 */
	bool invert_x;
	bool invert_y;

	/* KRL bytecode tables, uploaded by userspace via
	 * RM_IOCTL_SET_KRL_TBL. Index space is KRL_INDEX_*. */
	u8 *krl_tbl[KRL_TBL_COUNT];
	struct mutex krl_lock;		/* serializes rm_tch_cmd_process() */

	/* raw-scan ring buffer for RM_IOCTL_READ_RAW_DATA */
	u8 *queue;
	u16 q_front, q_rear;
	struct mutex q_lock;

	/* staging buffer for RM_IOCTL_SET_BASELINE, burst out to the chip
	 * by KRL_CMD_WRITE_IMG the next time a table containing it runs
	 * (downstream: g_u8_update_baseline / rm_tch_write_image_data()) */
	u8 *baseline;
	bool baseline_pending;

	struct workqueue_struct *wq;
	struct work_struct irq_work;

	struct pid *hal_pid;		/* registered via RM_IOCTL_SET_HAL_PID */

	struct rm_tch_ctrl_para ctrl;	/* from RM_IOCTL_SET_PARAMETER */

	bool init_service;		/* RM_IOCTL_INIT_SERVICE seen */
	bool init_finished;		/* between INIT_START/INIT_END */
	bool calc_finished;		/* RM_IOCTL_FINISH_CALC seen */
	bool is_suspended;
	bool spi_locked;
	u8 last_touch_count;

	/*
	 * downstream's u8_scan_mode_state state machine (ACTIVE/PRE_IDLE/
	 * IDLE), ported from rm_tch_ctrl_configure() / _enter_auto_mode() /
	 * _leave_auto_mode() in rm31080a_ts.c. Confirmed live on real
	 * hardware -- NOT dead code: librm31080.so calls
	 * SET_VARIABLE(AUTOSCAN_FLAG, 1) once its no-touch idle counter
	 * crosses a configured threshold (raydium_spi_ioctl(0x31010, 1),
	 * decompiled right next to a "HAL sent enter auto scan mode" log
	 * string). RM_IOCTL_SET_VARIABLE's index<<16 packing puts that at
	 * RM_VARIABLE_AUTOSCAN_FLAG (0x03) on RM_IOCTL_SET_VARIABLE
	 * (0x1010): (0x03 << 16) | 0x1010 == 0x31010.
	 *
	 * PRE_IDLE_MODE and IDLE_MODE are entirely driven from
	 * rm31080_ctrl_configure(), called once per IRQ from
	 * rm31080_irq_work(): PRE_IDLE runs the enter-auto-mode sequence
	 * and skips that cycle's scan/read/signal (the chip free-runs the
	 * idle KRL table on its own from here); IDLE runs the
	 * leave-auto-mode sequence and resumes normal scanning the cycle
	 * after. There is deliberately no path back to PRE_IDLE from
	 * userspace calling AUTOSCAN_FLAG a second time while already
	 * idle -- downstream's switch case only fires the PRE_IDLE
	 * transition from ACTIVE_MODE, matching that exactly.
	 *
	 * rm31080_ctrl_enter_manual_mode() (downstream:
	 * rm_tch_enter_manual_mode()) force-exits idle mode from
	 * rm31080_release() and RM_IOCTL_INIT_START -- see those sites.
	 *
	 * KNOWN GAP, not ported: raydium_tlk_ns_touch_suspend(), downstream's
	 * third caller of rm_tch_enter_manual_mode(), gated behind
	 * CONFIG_TRUSTED_LITTLE_KERNEL (which the Shield Tablet defconfig
	 * does enable) and EXPORT_SYMBOL'd for some other TrustZone-side
	 * kernel module to call into. We have no visibility into what calls
	 * it or when, so it's left unimplemented rather than guessed at.
	 */
	u8 scan_mode_state;		/* g_st_ts.u8_scan_mode_state */
	struct mutex scan_mode_lock;	/* g_st_ts.mutex_scan_mode */

	/*
	 * Ported from ts_timer_triggle/ts_timer_triggle_function +
	 * rm_timer_work_handler + rm_watchdog_work_function in
	 * rm31080a_ts.c. This is NOT an optional resilience feature: the
	 * *first* execution of the WATCHDOG table is, in practice, part of
	 * bringing the chip up into active scanning after
	 * SET_VARIABLE(WATCHDOG_FLAG, enable) -- disabling this subsystem
	 * outright (as an earlier diagnostic build of this driver did)
	 * leaves the chip never scanning at all. The 1Hz tick itself runs
	 * unconditionally from probe() to remove(), exactly like
	 * add_timer(&ts_timer_triggle) at the end of rm_tch_spi_probe();
	 * only the watchdog-table execution inside it is gated by
	 * watchdog_enable (== downstream's u8_watch_dog_enable).
	 */
	struct delayed_work timer_work;	/* ts_timer_triggle_function, HZ period */
	u32 timer_trigger_cnt;			/* rm_timer_trigger_function's u32TimerCnt,
						 * downsampled by ctrl.u8_timer_trigger_scale */
	bool watchdog_enable;			/* g_st_ts.u8_watch_dog_enable */
	bool watchdog_flg;			/* g_st_ts.u8_watch_dog_flg: "run the
						 * table on the next tick" request,
						 * settable directly via
						 * RM_IOCTL_WATCH_DOG too */
	u32 watchdog_cnt;			/* g_st_ts.u32_watch_dog_cnt */
	u32 watchdog_time;			/* g_st_ts.u32_watch_dog_time, in
						 * post-downsampling ticks; arg>>16
						 * from SET_VARIABLE(WATCHDOG_FLAG) */
	bool watchdog_check;			/* g_st_ts.b_watch_dog_check: set by
						 * the timer tick when the watchdog
						 * period elapses while
						 * scan_mode_state != ACTIVE_MODE;
						 * defers the actual watchdog table
						 * run to userspace via
						 * RM_SIGNAL_WATCH_DOG_CHECK instead
						 * of running it directly, since the
						 * idle KRL table owns the SPI bus
						 * autonomously while idle */

	/*
	 * Both confirmed live: librm31080.so calls
	 * SET_VARIABLE(TOUCHFILE_STATUS/TOUCH_EVENT) directly (see the
	 * RM_VARIABLE_TOUCH_EVENT NOTE above for TOUCH_EVENT specifically).
	 */
	u8 touchfile_check;	/* g_st_ts.u8_touchfile_check: opaque
				 * calibration-file-load status/error byte.
				 * Downstream's only consumer is a sysfs show
				 * handler we don't have, so this is stored and
				 * dev_dbg-logged but otherwise inert here --
				 * the specific error-code meanings (checksum
				 * mismatch, bad version, ...) are HAL-internal,
				 * not part of this driver's protocol. */
	u8 touch_event;		/* g_st_ts.u8_touch_event: see
				 * rm31080_set_variable()'s
				 * RM_VARIABLE_TOUCH_EVENT case, which is the
				 * actual functional consumer (a uevent, not
				 * just storage). */

	/*
	 * g_st_ts.wakelock_initialization, ported to the modern wakeup_source
	 * API (struct wake_lock/wake_lock_init/wake_lock_timeout/wake_unlock
	 * were removed from mainline years ago). Acquired with a bounded
	 * timeout in rm31080_resume() -- matching downstream's
	 * TCH_WAKE_LOCK_TIMEOUT (HZ/2, i.e. 500ms regardless of HZ) -- so the
	 * system can't suspend again mid-reinit; released early once
	 * userspace confirms reinit is done, via either RM_IOCTL_INIT_END or
	 * RM_VARIABLE_SET_WAKE_UNLOCK (downstream's two release call sites,
	 * both ported as-is). __pm_relax() is safe to call whether or not a
	 * timeout is currently outstanding, so unlike downstream's explicit
	 * wake_lock_active() guard, both release sites here call it
	 * unconditionally. NULL if registration failed at probe (treated as
	 * non-fatal -- this is a power-management nicety, not worth failing
	 * probe over); all three use sites guard on that.
	 */
	struct wakeup_source *wakelock_init;

	/*
	 * Frequency hopping / noise-scan channel cycling. Ported from
	 * rm_tch_ctrl_scan_start()'s ENABLE_FREQ_HOPPING branch,
	 * rm_set_ns_para(), rm_set_repeat_times(), and
	 * rm_tch_ctrl_wait_for_scan_finish() in rm31080a_ts.c. Unlike
	 * idle/auto-scan mode, this is NOT dead code: ctrl.u8_ns_func_enable
	 * is derived from bits 6-7 of SET_VARIABLE(WATCHDOG_FLAG)'s low byte
	 * (see rm31080_watchdog_configure()), and confirmed live on real
	 * hardware (arg=0x140041, captured repeatedly, decodes to
	 * ns_func_enable=1). When enabled, every scan_start cycles through a
	 * table of up to 3 noise-scan parameter columns instead of just
	 * running the plain SCANSTART table.
	 */
	u8 ns_para[9];		/* g_st_ts.u8_ns_para: 3 columns x 3 bytes,
				 * set via RM_VARIABLE_DPW (a user pointer,
				 * not a scalar -- see rm31080_set_variable()) */
	u8 ns_mode;		/* g_st_ts.u8_ns_mode: highest valid column
				 * index, via RM_VARIABLE_NS_MODE */
	u8 ns_rpt;		/* g_st_ts.u8_ns_rpt: repeat count for ns mode,
				 * via RM_VARIABLE_REPEAT */
	u8 ns_sel;		/* g_st_ts.u8_ns_sel: currently selected
				 * column's value, echoed into the raw-scan
				 * header's noise-scan-channel byte */
	u8 ns_sel_idx;		/* static u8NsSel in rm_tch_ctrl_scan_start():
				 * which column we're on, 0..ns_mode */
	u8 ns_last_rpt;		/* static u8Rpt in rm_tch_ctrl_scan_start();
				 * downstream initialises this to 1, not 0 --
				 * see probe() */
	struct mutex ns_mode_lock;	/* g_st_ts.mutex_ns_mode */

	u8 repeat_counter;	/* ts->u8_repeat_counter: scratch value
				 * KRL_CMD_WRITE_W_COUNT ORs into the byte it
				 * writes. Only meaningful for KRL table calls
				 * made on behalf of the noise-scan/repeat-time
				 * machinery below -- clear_int/scan_start's own
				 * base table and read_image never touch it. */
	u16 read_para;		/* g_st_ts.u16_read_para: written by
				 * KRL_CMD_RETURN_RESULT/RETURN_VALUE, read by
				 * rm31080_wait_for_scan_finish(). */

	/*
	 * Downstream's u8_resume_cnt: gates SET_SPI_UNLOCK against
	 * a race condition where multiple system-resume signals might be
	 * outstanding simultaneously. Incremented once per resume event
	 * (in rm31080_resume()). Decremented twice: once in INIT_END
	 * ("In case issued by boot-up") and once on each call to
	 * RM_VARIABLE_CHECK_SPI_LOCK's GET_VARIABLE handler (the polling
	 * site where userspace checks lock status). SET_SPI_UNLOCK only
	 * unlocks if resume_cnt == 1, deferring the unlock until all
	 * outstanding resumes have been serviced.
	 */
	u8 resume_cnt;
};

/* Only one touch chip in the system; the KRL interpreter and a couple of
 * legacy-shaped helpers need a single global pointer the way the
 * downstream driver used g_spi/g_input_dev/g_st_ts. */
static struct rm31080_data *g_rm;

/* =========================================================================
 * Low-level SPI transfers
 *
 * Ported from rm_tch_spi_read() / rm_tch_spi_write() / dev_read() /
 * dev_write() in rm31080a_ts.c. The read path is NOT a plain passthrough:
 * downstream ORs 0x80 into the address byte to mark it as a read, and
 * performs the address-write and data-read as two phases of a single
 * spi_sync() (chip select stays asserted across both). The write path is a
 * literal single spi_write() of whatever bytes are handed to it.
 * ========================================================================= */

static int rm31080_spi_read(struct rm31080_data *rm, u8 addr, u8 *rxbuf, size_t len)
{
	struct spi_transfer x[2] = { };
	struct spi_message m;
	u8 addr_byte = addr | 0x80;
	int ret;

	if (rm->spi_locked) {
		dev_dbg_ratelimited(&rm->spi->dev,
			"spi_read(addr=0x%02x len=%zu) skipped: spi_locked\n", addr, len);
		memset(rxbuf, 0, len);
		return RETURN_OK;
	}

	spi_message_init(&m);
	x[0].tx_buf = &addr_byte;
	x[0].len = 1;
	spi_message_add_tail(&x[0], &m);
	x[1].rx_buf = rxbuf;
	x[1].len = len;
	spi_message_add_tail(&x[1], &m);

	ret = spi_sync(rm->spi, &m);
	if (ret) {
		dev_err(&rm->spi->dev, "%s: spi_sync failed: %d\n", __func__, ret);
		return RETURN_FAIL;
	}
	return RETURN_OK;
}

static int rm31080_spi_write(struct rm31080_data *rm, u8 *txbuf, size_t len)
{
	int ret;

	if (rm->spi_locked) {
		dev_dbg_ratelimited(&rm->spi->dev, "spi_write(len=%zu) skipped: spi_locked\n", len);
		return RETURN_OK;
	}

	ret = spi_write(rm->spi, txbuf, len);
	if (ret) {
		dev_err(&rm->spi->dev, "%s: spi_write failed: %d\n", __func__, ret);
		return RETURN_FAIL;
	}
	return RETURN_OK;
}

static int rm31080_spi_byte_write(struct rm31080_data *rm, u8 addr, u8 val)
{
	u8 buf[2] = { addr, val };

	return rm31080_spi_write(rm, buf, 2);
}

static int rm31080_spi_burst_write(struct rm31080_data *rm, u8 reg, u8 *txbuf, size_t len)
{
	u8 *tmp;
	int ret;

	tmp = kmalloc(len + 1, GFP_KERNEL);
	if (!tmp)
		return -ENOMEM;
	tmp[0] = reg;
	memcpy(tmp + 1, txbuf, len);
	ret = rm31080_spi_write(rm, tmp, len + 1);
	kfree(tmp);
	return ret;
}

/* =========================================================================
 * Realtime signal delivery to the registered HAL pid
 * Ported from rm_tch_ts_send_signal(). si_code MUST be SI_QUEUE (not
 * SI_KERNEL) or the realtime payload (si_int) does not reach the
 * userspace handler -- this is exactly the downstream comment/trick,
 * kept intact.
 * ========================================================================= */

static int rm31080_send_signal(struct rm31080_data *rm, int info)
{
	struct kernel_siginfo sig = { };
	struct task_struct *t;
	int ret;

	if (!rm->hal_pid)
		return RETURN_FAIL;

	sig.si_signo = RM_TS_SIGNAL;
	sig.si_code = SI_QUEUE;
	sig.si_int = info;

	rcu_read_lock();
	t = pid_task(rm->hal_pid, PIDTYPE_PID);
	if (!t) {
		rcu_read_unlock();
		dev_err(&rm->spi->dev, "%s: HAL pid is gone\n", __func__);
		return RETURN_FAIL;
	}
	get_task_struct(t);
	rcu_read_unlock();

	ret = send_sig_info(RM_TS_SIGNAL, &sig, t);
	put_task_struct(t);

	if (ret)
		dev_err(&rm->spi->dev, "%s: send_sig_info failed: %d\n", __func__, ret);
	return ret;
}

/* =========================================================================
 * KRL bytecode interpreter
 *
 * Ported from rm_tch_cmd_process() plus KRL_CMD_CONFIG_3V3_Handler() /
 * KRL_CMD_CONFIG_1V8_Handler() in rm31080a_ts.c. This executes one "case"
 * (a named sub-sequence, e.g. "resume") out of one uploaded table
 * (identified by KRL_INDEX_*). Table layout, faithfully preserved:
 *
 *   byte 0        : length high byte (0 if table < 256 bytes)
 *   byte 1        : length low byte
 *   byte 2        : case_count - 1 (number of "case" sub-sequences)
 *   byte 3..N     : one length-prefixed byte per case, giving the
 *                   instruction count for that case
 *   thereafter    : case_count sequences of (instruction_count * 3)-byte
 *                   instructions, each instruction = { cmd, addr/sub_cmd,
 *                   data }
 *
 * u8reg is the interpreter's single scratch register, exactly as
 * downstream (a full register file was never needed for these tables).
 * ========================================================================= */

static int krl_config_3v3(struct rm31080_data *rm, u8 sub_cmd, u8 on_off)
{
	int ret = RETURN_FAIL;

	if (sub_cmd == KRL_SUB_CMD_SET_3V3_REGULATOR) {
		if (!rm->avdd)
			return RETURN_FAIL;
		ret = on_off ? regulator_enable(rm->avdd) : regulator_disable(rm->avdd);
	} else if (sub_cmd == KRL_SUB_CMD_SET_3V3_GPIO) {
		/* downstream: gpio_direction_output(pdata->gpio_3v3, on_off).
		 * Optional -- see the NOTE on gpio_avdd above. A board whose
		 * KRL tables actually exercise this sub-cmd needs avdd-gpios
		 * in its DT; one that doesn't (ours) never reaches here in
		 * practice, since our tables only ever use the REGULATOR
		 * sub-cmd above. */
		if (!rm->gpio_avdd) {
			dev_warn(&rm->spi->dev,
				 "KRL CONFIG_3V3 GPIO sub-cmd requested but no avdd-gpios in DT\n");
			return RETURN_FAIL;
		}
		gpiod_direction_output_raw(rm->gpio_avdd, on_off);
		ret = RETURN_OK;
	}
	return ret;
}

static int krl_config_1v8(struct rm31080_data *rm, u8 sub_cmd, u8 on_off)
{
	int ret = RETURN_FAIL;

	if (sub_cmd == KRL_SUB_CMD_SET_1V8_REGULATOR) {
		if (!rm->dvdd)
			return RETURN_FAIL;
		ret = on_off ? regulator_enable(rm->dvdd) : regulator_disable(rm->dvdd);
	} else if (sub_cmd == KRL_SUB_CMD_SET_1V8_GPIO) {
		/* downstream: gpio_direction_output(pdata->gpio_1v8, on_off).
		 * Same NOTE as krl_config_3v3()'s GPIO branch. */
		if (!rm->gpio_dvdd) {
			dev_warn(&rm->spi->dev,
				 "KRL CONFIG_1V8 GPIO sub-cmd requested but no dvdd-gpios in DT\n");
			return RETURN_FAIL;
		}
		gpiod_direction_output_raw(rm->gpio_dvdd, on_off);
		ret = RETURN_OK;
	}
	return ret;
}

/* KRL_CMD_READ_IMG needs a destination buffer that isn't part of the
 * generic 3-byte instruction encoding (downstream stashes it in a global,
 * g_pu8_burstread_buf, set just before calling into the interpreter). Do
 * the same thing here, scoped as file-static state guarded by krl_lock
 * (rm31080_cmd_process() holds it for the whole call). */
static u8 *rm31080_img_dest;
static size_t rm31080_img_len;

/*
 * rm31080_cmd_process - execute one case of one uploaded KRL table
 * @rm:       driver state
 * @sel_case: which case (0-based) within the table to run
 * @tbl:      table bytes (as uploaded via RM_IOCTL_SET_KRL_TBL)
 *
 * Returns RETURN_OK (0) on success, RETURN_FAIL (1) on the first failing
 * instruction (matching downstream's early-exit behaviour).
 */
static int rm31080_cmd_process(struct rm31080_data *rm, u8 sel_case, u8 *tbl)
{
	u16 j, str_idx, tbl_len;
	u8 i, reg = 0;
	int ret = RETURN_FAIL;

	if (!tbl)
		return RETURN_FAIL;

	mutex_lock(&rm->krl_lock);

	if (tbl[KRL_TBL_FIELD_POS_LEN_H]) {
		tbl_len = tbl[KRL_TBL_FIELD_POS_LEN_H];
		tbl_len <<= 8;
		tbl_len |= tbl[KRL_TBL_FIELD_POS_LEN_L];
	} else {
		tbl_len = tbl[KRL_TBL_FIELD_POS_LEN_L];
	}

	if (tbl_len < 3) {
		mutex_unlock(&rm->krl_lock);
		return RETURN_FAIL;
	}

	str_idx = KRL_TBL_FIELD_POS_CASE_NUM + tbl[KRL_TBL_FIELD_POS_CASE_NUM] + 1;
	for (i = 0; i < sel_case; i++)
		str_idx += tbl[i + KRL_TBL_FIELD_POS_CMD_NUM] * KRL_TBL_CMD_LEN;

	for (i = 0; i < tbl[sel_case + KRL_TBL_FIELD_POS_CMD_NUM]; i++) {
		u8 cmd, p_addr, p_sub, p_data;

		j = str_idx + (KRL_TBL_CMD_LEN * i);
		cmd = tbl[j];
		p_addr = tbl[j + 1];
		p_sub = tbl[j + 1];
		p_data = tbl[j + 2];
		ret = RETURN_FAIL;

		switch (cmd) {
		case KRL_CMD_READ:
			ret = rm31080_spi_read(rm, p_addr, &reg, 1);
			break;
		case KRL_CMD_WRITE_WO_DATA:
			ret = rm31080_spi_byte_write(rm, p_addr, reg);
			break;
		case KRL_CMD_WRITE_W_DATA:
			ret = rm31080_spi_byte_write(rm, p_addr, p_data);
			break;
		case KRL_CMD_IF_AND_OR:
			if (reg & p_addr)
				reg |= p_data;
			ret = RETURN_OK;
			break;
		case KRL_CMD_AND:
			reg &= p_data;
			ret = RETURN_OK;
			break;
		case KRL_CMD_OR:
			reg |= p_data;
			ret = RETURN_OK;
			break;
		case KRL_CMD_NOT:
			reg = ~reg;
			ret = RETURN_OK;
			break;
		case KRL_CMD_XOR:
			reg ^= p_data;
			ret = RETURN_OK;
			break;
		case KRL_CMD_DRAM_INIT:
			rm31080_spi_byte_write(rm, 0x01, 0x00);
			ret = rm31080_spi_byte_write(rm, 0x02, 0x00);
			break;
		case KRL_CMD_READ_IMG:
			/* Callers that need this (rm31080_read_image()) must
			 * pre-stage rm31080_img_dest/rm31080_img_len before
			 * invoking this table; mirrors downstream stashing
			 * the destination in the global g_pu8_burstread_buf
			 * before calling rm_tch_cmd_process(). */
			if (!rm31080_img_dest) {
				ret = RETURN_FAIL;
				break;
			}
			ret = rm31080_spi_read(rm, p_addr, rm31080_img_dest, rm31080_img_len);
			break;
		case KRL_CMD_WRITE_IMG:
			/* Streams the baseline image most recently staged by
			 * RM_IOCTL_SET_BASELINE (downstream:
			 * g_u8_update_baseline / rm_tch_write_image_data()).
			 * If nothing has been staged this is a no-op success,
			 * matching downstream's b_bl_updated guard in
			 * rm_tch_ctrl_enter_auto_mode(). */
			if (!rm->baseline_pending) {
				ret = RETURN_OK;
				break;
			}
			ret = rm31080_spi_burst_write(rm, p_addr, rm->baseline,
						       rm->ctrl.u16_data_length ?
						       rm->ctrl.u16_data_length : RM_RAW_DATA_LENGTH);
			if (!ret)
				rm->baseline_pending = false;
			break;
		case KRL_CMD_SEND_SIGNAL:
			ret = rm31080_send_signal(rm, p_data);
			break;
		case KRL_CMD_CONFIG_RST:
			/* p_data here is a literal physical pin level, exactly
			 * as downstream's raw gpio_direction_output()/
			 * gpio_set_value() treated it (0=drive low,
			 * 1=drive high) -- these tables were authored against
			 * that non-polarity-aware legacy API, not gpiod's
			 * active-low-translating one. Use the _raw variants
			 * so a table byte of 0 always means "physically low"
			 * regardless of the reset-gpios ACTIVE_LOW flag in DT,
			 * matching what downstream actually did on the wire.
			 * (Our own probe()-driven reset pulse, above, is
			 * hand-written and uses the polarity-aware API
			 * instead -- don't conflate the two.) */
			if (p_sub == KRL_SUB_CMD_SET_RST_GPIO) {
				gpiod_direction_output_raw(rm->gpio_reset, p_data);
				ret = RETURN_OK;
			} else if (p_sub == KRL_SUB_CMD_SET_RST_VALUE) {
				gpiod_set_raw_value_cansleep(rm->gpio_reset, p_data);
				ret = RETURN_OK;
			}
			break;
		case KRL_CMD_CONFIG_3V3:
			ret = krl_config_3v3(rm, p_sub, p_data);
			break;
		case KRL_CMD_CONFIG_1V8:
			ret = krl_config_1v8(rm, p_sub, p_data);
			break;
		case KRL_CMD_CONFIG_CLK:
			if (p_sub == KRL_SUB_CMD_SET_CLK && rm->clk) {
				if (p_data)
					ret = clk_prepare_enable(rm->clk);
				else {
					clk_disable_unprepare(rm->clk);
					ret = RETURN_OK;
				}
			}
			break;
		case KRL_CMD_CONFIG_CS:
			/* NOTE: downstream's manual chip-select toggle is
			 * gated behind CS_SUPPORT (off by default upstream
			 * too); treat as a no-op success like downstream's
			 * #else branch. */
			ret = RETURN_OK;
			break;
		case KRL_CMD_SET_TIMER:
			/* NOTE: downstream's slow-scan re-arm timer
			 * (init/add/del) belongs to ENABLE_SLOW_SCAN, a
			 * separate progressive-scan-rate feature from the
			 * PRE_IDLE/IDLE auto-scan mode (see the NOTE on
			 * scan_mode_state above, which IS implemented) --
			 * slow-scan itself isn't wired up; accept and no-op. */
			ret = RETURN_OK;
			break;
		case KRL_CMD_MSLEEP: {
			u32 ms = (u16)(p_data | (p_sub << 8));

			usleep_range(ms * 1000, ms * 1000 + 200);
			ret = RETURN_OK;
			break;
		}
		case KRL_CMD_FLUSH_QU:
			/* our workqueue is flushed synchronously by design
			 * (see rm31080_irq_work); nothing to do here */
			ret = RETURN_OK;
			break;
		case KRL_CMD_WRITE_W_COUNT:
			ret = rm31080_spi_byte_write(rm, p_addr, reg | rm->repeat_counter);
			break;
		case KRL_CMD_RETURN_RESULT:
			rm->read_para = reg;
			ret = RETURN_OK;
			break;
		case KRL_CMD_RETURN_VALUE:
			rm->read_para = ((u16)p_addr << 8) | p_data;
			ret = RETURN_OK;
			break;
		case KRL_CMD_CONFIG_IRQ:
			if (p_sub == KRL_SUB_CMD_SET_IRQ && rm->irq) {
				if (p_data)
					enable_irq(rm->irq);
				else
					disable_irq(rm->irq);
				ret = RETURN_OK;
			}
			break;
		case KRL_CMD_DUMMY:
			ret = RETURN_OK;
			break;
		default:
			break;
		}

		if (ret) {
			dev_err(&rm->spi->dev,
				"KRL cmd 0x%x failed (addr/sub=0x%x data=0x%x) in case %u\n",
				cmd, p_addr, p_data, sel_case);
			break;
		}
	}

	mutex_unlock(&rm->krl_lock);
	return ret;
}

static int rm31080_read_image(struct rm31080_data *rm, u8 *dest, size_t data_len)
{
	int ret;

	dest[0] = SCAN_TYPE_MT;
	dest[1] = (u8)(data_len >> 8);
	dest[2] = (u8)data_len;
	dest[3] = (rm->ctrl.u8_ns_func_enable & 0x01) ? rm->ns_sel : 0;
	dest[4] = 0;	/* self-test mode type; 0 (RM_TEST_MODE_NULL) since we
			 * don't implement the self-test ioctls */
	dest[5] = 0;
	dest[6] = 0;
	dest[7] = 0;

	rm31080_img_dest = dest + QUEUE_HEADER_NUM;
	rm31080_img_len = data_len;
	ret = rm31080_cmd_process(rm, 0, rm->krl_tbl[KRL_INDEX_RM_READ_IMG]);
	rm31080_img_dest = NULL;
	return ret;
}

/* =========================================================================
 * Raw-scan-image ring buffer (backs RM_IOCTL_READ_RAW_DATA)
 * Ported from rm_tch_queue_*() in rm31080a_ts.c. One slot kept empty to
 * distinguish full/empty without a separate counter, exactly as downstream.
 * ========================================================================= */

static bool rm31080_q_empty(struct rm31080_data *rm)
{
	return rm->q_rear == rm->q_front;
}

static bool rm31080_q_full(struct rm31080_data *rm)
{
	if (rm->q_rear + 1 == rm->q_front)
		return true;
	if (rm->q_rear == QUEUE_COUNT - 1 && rm->q_front == 0)
		return true;
	return false;
}

static u8 *rm31080_q_enqueue_start(struct rm31080_data *rm)
{
	if (!rm->queue || rm31080_q_full(rm))
		return NULL;
	return &rm->queue[rm->q_rear * RM_RAW_DATA_LENGTH];
}

static void rm31080_q_enqueue_finish(struct rm31080_data *rm)
{
	rm->q_rear = (rm->q_rear == QUEUE_COUNT - 1) ? 0 : rm->q_rear + 1;
}

static u8 *rm31080_q_dequeue_start(struct rm31080_data *rm)
{
	if (rm31080_q_empty(rm))
		return NULL;
	return &rm->queue[rm->q_front * RM_RAW_DATA_LENGTH];
}

static void rm31080_q_dequeue_finish(struct rm31080_data *rm)
{
	rm->q_front = (rm->q_front == QUEUE_COUNT - 1) ? 0 : rm->q_front + 1;
}

static long rm31080_q_read_raw_data(struct rm31080_data *rm, void __user *p, u32 len)
{
	u8 *slot;
	long ret;

	mutex_lock(&rm->q_lock);
	slot = rm31080_q_dequeue_start(rm);
	if (!slot) {
		mutex_unlock(&rm->q_lock);
		return RETURN_FAIL;
	}
	ret = copy_to_user(p, slot, len) ? RETURN_FAIL : RETURN_OK;
	if (!ret)
		rm31080_q_dequeue_finish(rm);
	mutex_unlock(&rm->q_lock);
	return ret;
}

/* =========================================================================
 * Touch-point injection (RM_IOCTL_REPORT_POINT)
 * Ported from raydium_report_pointer() in rm31080a_ts.c, Type-B multitouch
 * path only (INPUT_PROTOCOL_CURRENT_SUPPORT == INPUT_PROTOCOL_TYPE_B is the
 * only variant downstream actually built).
 * ========================================================================= */

static void rm31080_report_pointer(struct rm31080_data *rm, struct rm_touch_event *tp)
{
	unsigned int tool = MT_TOOL_FINGER;
	unsigned int btn = BTN_TOOL_FINGER;
	int max_x, max_y, count, i;

	if (rm->ctrl.u16_resolution_x && rm->ctrl.u16_resolution_y) {
		max_x = rm->ctrl.u16_resolution_x;
		max_y = rm->ctrl.u16_resolution_y;
	} else {
		max_x = rm->default_res_x;
		max_y = rm->default_res_y;
	}

	count = max(rm->last_touch_count, tp->uc_touch_count);

	dev_dbg_ratelimited(&rm->spi->dev,
		"report_pointer: last_touch_count=%u uc_touch_count=%u count=%d\n",
		rm->last_touch_count, tp->uc_touch_count, count);

	if (count && !tp->uc_touch_count) {
		rm->last_touch_count = 0;
		for (i = 0; i < MAX_SLOT_AMOUNT; i++) {
			input_mt_slot(rm->input, i);
			input_mt_report_slot_state(rm->input, MT_TOOL_FINGER, false);
			input_report_key(rm->input, BTN_TOOL_RUBBER, false);
		}
		input_sync(rm->input);
		return;
	}

	if (!count)
		return;

	for (i = 0; i < count && i < MAX_SLOT_AMOUNT; i++) {
		if (i >= tp->uc_touch_count)
			continue;

		input_mt_slot(rm->input, tp->uc_slot[i] & ~INPUT_SLOT_RESET);

		if ((tp->uc_slot[i] & INPUT_SLOT_RESET) || tp->uc_id[i] == INPUT_ID_RESET) {
			switch (tp->uc_pre_tool_type[i]) {
			case POINT_TYPE_FINGER:
				tool = MT_TOOL_FINGER;
				break;
			case POINT_TYPE_STYLUS:
				tool = MT_TOOL_PEN;
				break;
			case POINT_TYPE_ERASER:
				tool = MT_TOOL_PEN;
				btn = BTN_TOOL_RUBBER;
				break;
			default:
				break;
			}
			input_mt_report_slot_state(rm->input, tool, false);
			if (tp->uc_pre_tool_type[i] == POINT_TYPE_ERASER)
				input_report_key(rm->input, btn, false);
		}

		if (tp->uc_id[i] == INPUT_ID_RESET)
			continue;

		switch (tp->uc_tool_type[i]) {
		case POINT_TYPE_FINGER:
			tool = MT_TOOL_FINGER;
			break;
		case POINT_TYPE_STYLUS:
			tool = MT_TOOL_PEN;
			break;
		case POINT_TYPE_ERASER:
			tool = MT_TOOL_PEN;
			btn = BTN_TOOL_RUBBER;
			break;
		default:
			break;
		}

		input_mt_report_slot_state(rm->input, tool, true);
		{
			/* Orientation transform, see the NOTE on invert_x/y
			 * in the struct above. */
			u16 x = min_t(u16, tp->us_x[i], max_x - 1);
			u16 y = min_t(u16, tp->us_y[i], max_y - 1);

			if (rm->invert_x)
				x = max_x - 1 - x;
			if (rm->invert_y)
				y = max_y - 1 - y;

			input_report_abs(rm->input, ABS_MT_POSITION_X, x);
			input_report_abs(rm->input, ABS_MT_POSITION_Y, y);
		}
		input_report_abs(rm->input, ABS_MT_PRESSURE, tp->us_z[i]);

		if (tp->uc_tool_type[i] == POINT_TYPE_ERASER)
			input_report_key(rm->input, btn, true);
	}

	rm->last_touch_count = tp->uc_touch_count;
	input_sync(rm->input);
}

/* =========================================================================
 * Frequency hopping / noise-scan channel cycling
 *
 * Ported from rm_set_repeat_times(), rm_set_ns_para(),
 * rm_tch_ctrl_wait_for_scan_finish(), and rm_tch_ctrl_scan_start()'s
 * ENABLE_FREQ_HOPPING branch in rm31080a_ts.c. See the NOTE on the ns_*
 * struct fields above for why this -- unlike idle/auto-scan mode -- is not
 * dead code and needed real implementation.
 * ========================================================================= */

static void rm31080_set_repeat_times(struct rm31080_data *rm, u8 times)
{
	if (times <= 1)
		times = 0;
	else
		times -= 1;
	if (times > 127)
		times = 127;

	rm->repeat_counter = times & 0x7F;
	rm31080_cmd_process(rm, times == 0 ? 0 : 1, rm->krl_tbl[KRL_INDEX_RM_SETREPTIME]);
}

static void rm31080_set_ns_para(struct rm31080_data *rm, u8 col)
{
	u8 *tbl = rm->krl_tbl[KRL_INDEX_RM_NSPARA];
	u8 case_count = tbl[KRL_TBL_FIELD_POS_CASE_NUM];
	int ii;

	for (ii = 0; ii < case_count; ii++) {
		rm->repeat_counter = rm->ns_para[ii * 3 + col];
		rm31080_cmd_process(rm, ii, tbl);
	}
}

/*
 * rm_tch_ctrl_wait_for_scan_finish(1) downstream always executes the
 * WAITSCANOK table exactly once and returns 0 unconditionally for our
 * caller (u8Idx=1): the loop either breaks immediately (scan already done)
 * or returns immediately on the first "still busy" check (u8Idx truthy
 * skips the retry-sleep branch entirely) -- so its 50-iteration polling
 * loop is unreachable beyond the first pass in this call pattern. We only
 * need the one side-effecting table execution; the result is genuinely
 * unused by the only caller we have (rm31080_ctrl_scan_start()).
 */
static void rm31080_wait_for_scan_finish(struct rm31080_data *rm)
{
	rm31080_cmd_process(rm, 0, rm->krl_tbl[KRL_INDEX_RM_WAITSCANOK]);
}

/*
 * rm_tch_ctrl_wait_for_scan_finish(0): genuinely different from the
 * u8Idx=1 variant above, NOT just a stylistic alternative -- with
 * u8Idx=0, downstream re-runs the WAITSCANOK table and sleeps 1-2ms
 * between checks, up to 50 times (~100ms total), stopping as soon as the
 * scan-busy bit (read_para bit 0, set by the table's own
 * KRL_CMD_RETURN_RESULT/RETURN_VALUE opcode) clears -- an actual
 * polling wait, unlike u8Idx=1's single check-and-return. Used by
 * suspend (both call sites in rm_ctrl_suspend()) to make sure any
 * in-flight scan has genuinely finished before the chip is reconfigured
 * for suspend / powered down.
 */
static void rm31080_wait_for_scan_finish_blocking(struct rm31080_data *rm)
{
	int i;

	for (i = 0; i < 50; i++) {
		rm31080_cmd_process(rm, 0, rm->krl_tbl[KRL_INDEX_RM_WAITSCANOK]);
		if (rm->read_para & 0x01)
			usleep_range(1000, 2000);
		else
			break;
	}
}

/*
 * rm_tch_ctrl_scan_start(): replaces the bare SCANSTART call when
 * frequency hopping is enabled. Note the exact ordering downstream uses,
 * preserved here rather than "simplified": ns_sel is latched from the
 * *current* column before advancing the index, but rm31080_set_ns_para()
 * is then called with the *already-advanced* index -- these are not the
 * same value, and that appears intentional (matches specific chip-side
 * pipelining), not a bug to fix.
 */
static int rm31080_ctrl_scan_start(struct rm31080_data *rm)
{
	if (rm->ctrl.u8_ns_func_enable & 0x01) {
		u8 prev_idx;

		rm31080_wait_for_scan_finish(rm);

		mutex_lock(&rm->ns_mode_lock);
		prev_idx = rm->ns_sel_idx;
		rm->ns_sel = rm->ns_para[rm->ns_sel_idx];
		if (rm->ns_sel_idx < rm->ns_mode)
			rm->ns_sel_idx++;
		else
			rm->ns_sel_idx = 0;

		if (rm->ns_last_rpt != rm->ns_rpt) {
			rm->ns_last_rpt = rm->ns_rpt;
			rm31080_set_repeat_times(rm, rm->ns_last_rpt);
		}
		rm31080_set_ns_para(rm, rm->ns_sel_idx);

		dev_dbg_ratelimited(&rm->spi->dev,
			"freq hop: ns_mode=%u col %u->%u ns_sel=%u ns_rpt=%u\n",
			rm->ns_mode, prev_idx, rm->ns_sel_idx, rm->ns_sel, rm->ns_last_rpt);

		mutex_unlock(&rm->ns_mode_lock);
	} else {
		rm->ns_sel_idx = 0;
		rm->ns_sel = 0;
	}

	return rm31080_cmd_process(rm, 0, rm->krl_tbl[KRL_INDEX_RM_SCANSTART]);
}

/*
 * rm_tch_ctrl_enter_auto_mode(): hand scanning over to the chip's
 * autonomous idle KRL table. Ported directly, including the ordering
 * (baseline flush, then repeat-times, then the table itself).
 *
 * The baseline write here is unconditional on purpose: downstream gates
 * it on a separate b_bl_updated flag before calling rm_tch_write_image_data(),
 * but that flag and our baseline_pending serve the exact same "has a new
 * baseline been staged" role, and KRL_CMD_WRITE_IMG (see rm31080_cmd_process())
 * already no-ops successfully when baseline_pending is false -- so checking
 * it twice would be redundant, not more correct.
 */
static void rm31080_ctrl_enter_auto_mode(struct rm31080_data *rm)
{
	rm->ctrl.u8_idle_mode_check &= ~0x01;

	dev_dbg(&rm->spi->dev, "scan_mode: entering auto (idle) scan mode\n");

	rm31080_cmd_process(rm, 0, rm->krl_tbl[KRL_INDEX_RM_WRITE_IMG]);
	rm31080_set_repeat_times(rm, rm->ctrl.u8_idle_digital_repeat_times);
	rm31080_cmd_process(rm, 1, rm->krl_tbl[KRL_INDEX_FUNC_SET_IDLE]);
}

/*
 * rm_tch_ctrl_leave_auto_mode(): pull scanning back under IRQ control.
 * The ns_para column reset to 0 here is deliberate and downstream-exact:
 * leaving idle mode always restarts frequency hopping (if enabled) from
 * column 0, regardless of which column ns_sel_idx was sitting on when
 * AUTOSCAN_FLAG put us into idle mode.
 */
static void rm31080_ctrl_leave_auto_mode(struct rm31080_data *rm)
{
	rm->ctrl.u8_idle_mode_check |= 0x01;

	if (rm->ctrl.u8_ns_func_enable & 0x01) {
		mutex_lock(&rm->ns_mode_lock);
		rm31080_set_ns_para(rm, 0);
		mutex_unlock(&rm->ns_mode_lock);
	}

	rm31080_cmd_process(rm, 0, rm->krl_tbl[KRL_INDEX_FUNC_SET_IDLE]);
	rm31080_set_repeat_times(rm, rm->ctrl.u8_active_digital_repeat_times);

	dev_dbg(&rm->spi->dev, "scan_mode: left auto (idle) scan mode\n");
}

/*
 * rm_tch_enter_manual_mode(): force scan_mode_state back to ACTIVE_MODE
 * outright, used wherever downstream needs guaranteed IRQ-driven scanning
 * before doing something else (closing the device node, restarting init)
 * rather than waiting for the next IRQ to naturally pump the PRE_IDLE ->
 * IDLE -> ACTIVE cycle. The flush_workqueue() must run with scan_mode_lock
 * NOT held -- it drains rm31080_irq_work(), which takes that same lock via
 * rm31080_ctrl_configure() -- so this must never itself be called from
 * inside rm31080_irq_work().
 */
static void rm31080_ctrl_enter_manual_mode(struct rm31080_data *rm)
{
	flush_workqueue(rm->wq);

	mutex_lock(&rm->scan_mode_lock);
	switch (rm->scan_mode_state) {
	case RM_SCAN_PRE_IDLE_MODE:
		rm->scan_mode_state = RM_SCAN_ACTIVE_MODE;
		mutex_unlock(&rm->scan_mode_lock);
		return;
	case RM_SCAN_IDLE_MODE:
		rm31080_ctrl_leave_auto_mode(rm);
		rm->scan_mode_state = RM_SCAN_ACTIVE_MODE;
		mutex_unlock(&rm->scan_mode_lock);
		/* Downstream's msleep(10)-equivalent settling time after a
		 * manually-forced (i.e. not IRQ-cycle-paced) mode exit. */
		usleep_range(10000, 10050);
		return;
	default:
		mutex_unlock(&rm->scan_mode_lock);
		return;
	}
}

/*
 * rm_tch_ctrl_configure(): called once per IRQ (after clear-int, before
 * scan-start) to decide what this cycle needs to do. ACTIVE_MODE is the
 * steady state and is what every cycle did before this state machine
 * existed. PRE_IDLE/IDLE are one-shot transitions requested by
 * SET_VARIABLE(AUTOSCAN_FLAG) (see rm31080_set_variable()) and by this
 * function itself advancing PRE_IDLE -> IDLE -> (next AUTOSCAN_FLAG call
 * or chip activity) -> ACTIVE.
 */
static u32 rm31080_ctrl_configure(struct rm31080_data *rm)
{
	u32 flag;

	mutex_lock(&rm->scan_mode_lock);
	switch (rm->scan_mode_state) {
	case RM_SCAN_ACTIVE_MODE:
		flag = RM_NEED_TO_SEND_SCAN | RM_NEED_TO_READ_RAW_DATA |
		       RM_NEED_TO_SEND_SIGNAL;
		break;
	case RM_SCAN_PRE_IDLE_MODE:
		rm31080_ctrl_enter_auto_mode(rm);
		rm->scan_mode_state = RM_SCAN_IDLE_MODE;
		flag = RM_NEED_NONE;
		break;
	case RM_SCAN_IDLE_MODE:
		rm31080_ctrl_leave_auto_mode(rm);
		rm->scan_mode_state = RM_SCAN_ACTIVE_MODE;
		flag = RM_NEED_TO_SEND_SCAN;
		break;
	default:
		flag = RM_NEED_NONE;
		break;
	}
	mutex_unlock(&rm->scan_mode_lock);

	return flag;
}

/* =========================================================================
 * IRQ handling
 *
 * Ported from rm_tch_irq() + rm_work_handler() in rm31080a_ts.c. Clear-int
 * runs unconditionally every IRQ, same as downstream; what happens after
 * that is gated by rm31080_ctrl_configure()'s scan_mode_state read, which
 * is RM_NEED_TO_SEND_SCAN | RM_NEED_TO_READ_RAW_DATA | RM_NEED_TO_SEND_SIGNAL
 * in the steady-state ACTIVE case (scan-start, pull one image into the
 * ring buffer, signal the HAL) and RM_NEED_NONE or RM_NEED_TO_SEND_SCAN
 * alone during the one-shot PRE_IDLE/IDLE transitions -- see the NOTE on
 * scan_mode_state above.
 * ========================================================================= */

static void rm31080_irq_work(struct work_struct *work)
{
	struct rm31080_data *rm = container_of(work, struct rm31080_data, irq_work);
	u8 *slot;
	u32 flag;
	int clear_ret, scan_ret = 0, img_ret = -1;

	if (!rm->init_finished || rm->is_suspended) {
		dev_dbg_ratelimited(&rm->spi->dev,
			"irq_work: skipped (init_finished=%d is_suspended=%d)\n",
			rm->init_finished, rm->is_suspended);
		return;
	}

	clear_ret = rm31080_cmd_process(rm, 0, rm->krl_tbl[KRL_INDEX_RM_CLEARINT]);
	flag = rm31080_ctrl_configure(rm);

	if (flag & RM_NEED_TO_SEND_SCAN)
		scan_ret = rm31080_ctrl_scan_start(rm);

	if (flag & RM_NEED_TO_READ_RAW_DATA) {
		mutex_lock(&rm->q_lock);
		slot = rm31080_q_enqueue_start(rm);
		mutex_unlock(&rm->q_lock);

		if (slot) {
			img_ret = rm31080_read_image(rm, slot, rm->ctrl.u16_data_length ?
						      rm->ctrl.u16_data_length : RM_RAW_DATA_LENGTH);
			if (!img_ret) {
				mutex_lock(&rm->q_lock);
				rm31080_q_enqueue_finish(rm);
				mutex_unlock(&rm->q_lock);
			}
		}
	}

	dev_dbg_ratelimited(&rm->spi->dev,
		"irq_work: clear_int=%d scan_mode_state=%u flag=0x%x scan_start=%d read_image=%d data_length=%u\n",
		clear_ret, rm->scan_mode_state, flag, scan_ret, img_ret,
		rm->ctrl.u16_data_length);

	if ((flag & RM_NEED_TO_SEND_SIGNAL) && rm->calc_finished) {
		rm->calc_finished = false;
		rm31080_send_signal(rm, RM_SIGNAL_INTR);
	}
}

static irqreturn_t rm31080_irq(int irq, void *data)
{
	struct rm31080_data *rm = data;
	bool will_process = rm->init_service && rm->init_finished && !rm->is_suspended;

	/* rm_tch_irq(): a real touch IRQ landing at all is itself proof of
	 * life, same as a successful watchdog table run -- reset the
	 * countdown unconditionally, not just from the timer tick. */
	rm->watchdog_cnt = 0;

	/* Unprotected read of scan_mode_state, matching downstream: this is
	 * a best-effort wake/activity hint for Android's input stack, not
	 * a correctness-critical decision, so it doesn't need scan_mode_lock
	 * (rm31080_irq() runs threaded/sleepable here, but downstream's
	 * equivalent doesn't, and taking the lock on every single touch IRQ
	 * just for a hint isn't worth the contention). The real transition
	 * out of IDLE_MODE still happens properly locked, in
	 * rm31080_ctrl_configure() via rm31080_irq_work() below. */
	if (rm->scan_mode_state == RM_SCAN_IDLE_MODE) {
		input_event(rm->input, EV_MSC, MSC_ACTIVITY, 1);
		input_sync(rm->input);
	}

	dev_dbg_ratelimited(&rm->spi->dev,
		"hw irq fired (init_service=%d init_finished=%d is_suspended=%d -> %s)\n",
		rm->init_service, rm->init_finished, rm->is_suspended,
		will_process ? "queued" : "DROPPED");

	if (will_process)
		queue_work(rm->wq, &rm->irq_work);

	return IRQ_HANDLED;
}

/* =========================================================================
 * Timer tick / watchdog
 *
 * Ported from ts_timer_triggle_function() + rm_timer_work_handler() +
 * rm_watchdog_work_function() in rm31080a_ts.c. Runs unconditionally every
 * HZ (1s, matching downstream's TS_TIMER_PERIOD) for the lifetime of the
 * device -- this is NOT gated by watchdog_enable; only the table execution
 * inside it is. See the long NOTE on the struct fields above for why this
 * matters: this is where the very first "start scanning" kick happens.
 * ========================================================================= */

static void rm31080_timer_work(struct work_struct *work)
{
	struct rm31080_data *rm = container_of(to_delayed_work(work),
						struct rm31080_data, timer_work);
	bool triggered;

	dev_dbg_ratelimited(&rm->spi->dev,
		"timer tick: init_finished=%d is_suspended=%d watchdog_enable=%d watchdog_cnt=%u watchdog_time=%u\n",
		rm->init_finished, rm->is_suspended, rm->watchdog_enable,
		rm->watchdog_cnt, rm->watchdog_time);

	/* rm_timer_trigger_function(): downsample the 1Hz tick by
	 * ctrl.u8_timer_trigger_scale (0 = run every tick). */
	if (rm->timer_trigger_cnt++ < rm->ctrl.u8_timer_trigger_scale) {
		triggered = false;
	} else {
		rm->timer_trigger_cnt = 0;
		triggered = true;
	}

	if (triggered && rm->init_finished && !rm->is_suspended) {
		/*
		 * rm_watchdog_work_function(): when scan_mode_state is
		 * ACTIVE, a watchdog-time expiry requests a direct table
		 * run (watchdog_flg) same as always. When it's PRE_IDLE or
		 * IDLE, the idle KRL table owns the SPI bus autonomously,
		 * so running KRL_INDEX_RM_WATCHDOG here would race it --
		 * instead we just flag watchdog_check and let userspace
		 * decide via RM_SIGNAL_WATCH_DOG_CHECK, exactly like
		 * downstream's RM_SCAN_IDLE_MODE branch.
		 */
		if (rm->watchdog_enable) {
			if (rm->watchdog_cnt++ >= rm->watchdog_time) {
				rm->watchdog_cnt = 0;

				mutex_lock(&rm->scan_mode_lock);
				if (rm->scan_mode_state == RM_SCAN_ACTIVE_MODE)
					rm->watchdog_flg = true;
				else
					rm->watchdog_check = true;
				mutex_unlock(&rm->scan_mode_lock);
			}

			if (rm->watchdog_flg) {
				int wd_ret;

				dev_dbg(&rm->spi->dev,
					"watchdog: executing KRL_INDEX_RM_WATCHDOG table now (watchdog_time=%u)\n",
					rm->watchdog_time);

				/*
				 * Mutual exclusion against a real scan cycle
				 * here needs to be real hardware IRQ masking,
				 * not the software is_suspended flag downstream
				 * uses (and this port originally matched):
				 * rm31080_irq()'s is_suspended check drops a
				 * real touch IRQ outright with no way to
				 * recover it, and if the chip's INT line is
				 * level-held until explicitly cleared, a
				 * dropped event leaves it stuck asserted
				 * forever -- no further *rising* edges for our
				 * edge-triggered IRQ, ever (confirmed by
				 * testing: two independent runs where the chip
				 * went permanently silent immediately after a
				 * watchdog cycle, every time).
				 *
				 * disable_irq()/enable_irq() has none of that
				 * problem. Verified against
				 * drivers/gpio/gpio-tegra.c: irq_mask() only
				 * clears the GPIO_INT_ENB bit, never touches
				 * GPIO_INT_STA; the dispatcher ANDs
				 * (STA & ENB), and STA latches on a real edge
				 * regardless of ENB. A touch landing while
				 * masked stays latched in hardware and fires
				 * correctly the instant we unmask -- exactly
				 * the "pause without losing events" behaviour
				 * we actually need, provided by the interrupt
				 * controller instead of reimplemented (badly)
				 * in software.
				 */
				disable_irq(rm->irq);
				wd_ret = rm31080_cmd_process(rm, 0, rm->krl_tbl[KRL_INDEX_RM_WATCHDOG]);
				enable_irq(rm->irq);
				rm->watchdog_flg = false;

				dev_dbg(&rm->spi->dev,
					"watchdog: table execution finished, ret=%d\n", wd_ret);
			}
		}

		if (rm->watchdog_check) {
			dev_dbg(&rm->spi->dev,
				"watchdog: deferring to userspace via RM_SIGNAL_WATCH_DOG_CHECK (scan_mode_state=%u)\n",
				rm->scan_mode_state);
			rm31080_send_signal(rm, RM_SIGNAL_WATCH_DOG_CHECK);
			rm->watchdog_check = false;
		}
	}

	schedule_delayed_work(&rm->timer_work, HZ);
}

/* RM_IOCTL_WATCH_DOG: just requests a table run on the next tick, exactly
 * like downstream -- does NOT run the table synchronously. */
static void rm31080_watchdog_request(struct rm31080_data *rm)
{
	dev_dbg(&rm->spi->dev, "watchdog: table run requested via RM_IOCTL_WATCH_DOG\n");
	rm->watchdog_flg = true;
}

/*
 * Exact port of rm_ctrl_watchdog_func(): arg bit 0 = enable, arg >> 16 =
 * watchdog period in (downsampled) ticks. Disabling sets the period to
 * "never". Split out from rm31080_watchdog_configure() below because
 * downstream calls this SAME function from two different places with two
 * different intents -- the SET_VARIABLE(WATCHDOG_FLAG) path (which also
 * has a second, unrelated side effect on ns_func_enable -- see below) and
 * rm_tch_init_ts_structure_part()'s unconditional arg=0 reset on every
 * resume (which must NOT touch ns_func_enable, since that's an
 * independently-HAL-configured feature that shouldn't silently reset
 * itself every time the system wakes up).
 *
 * NOTE: this previously did not reset watchdog_check, unlike downstream's
 * b_watch_dog_check = 0 -- found while cross-checking against the real
 * downstream source for the suspend/resume audit below. Fixed here; low
 * risk since it only ever mattered as a stale flag left over from a prior
 * watchdog cycle, and this reset already runs on every (re)configure.
 */
static void rm31080_watchdog_reset(struct rm31080_data *rm, unsigned long arg)
{
	rm->watchdog_flg = false;
	rm->watchdog_cnt = 0;
	rm->watchdog_check = false;

	if (arg & 0x01) {
		rm->watchdog_enable = true;
		rm->watchdog_time = arg >> 16;
	} else {
		rm->watchdog_enable = false;
		rm->watchdog_time = 0xFFFFFFFF;
	}
}

/*
 * RM_VARIABLE_WATCHDOG_FLAG (SET_VARIABLE): ported from
 * rm_ctrl_watchdog_func() plus the ns_func_enable side effect that
 * downstream's SET_VARIABLE(WATCHDOG_FLAG) case layers on top of it
 * immediately afterward (see rm31080_watchdog_reset()'s comment).
 */
static void rm31080_watchdog_configure(struct rm31080_data *rm, unsigned long arg)
{
	rm31080_watchdog_reset(rm, arg);

	/* Same packed arg also carries u8_ns_func_enable in bits 6-7 of its
	 * low byte downstream (g_st_ctrl.u8_ns_func_enable is a single
	 * variable also written wholesale by RM_IOCTL_SET_PARAMETER; this is
	 * the second, later writer -- confirmed live on real hardware via
	 * arg=0x140041 decoding to ns_func_enable=1). */
	rm->ctrl.u8_ns_func_enable = (u8)(arg & 0xFF) >> 6;

	dev_dbg(&rm->spi->dev,
		"watchdog: configured via SET_VARIABLE, arg=0x%lx -> enable=%d period=%u ns_func_enable=%u (timer_trigger_scale=%u)\n",
		arg, rm->watchdog_enable, rm->watchdog_time, rm->ctrl.u8_ns_func_enable,
		rm->ctrl.u8_timer_trigger_scale);
}

/* =========================================================================
 * misc device file operations
 * ========================================================================= */

static int rm31080_open(struct inode *inode, struct file *filp)
{
	filp->private_data = g_rm;
	atomic_inc(&g_rm->open_count);
	return 0;
}

static int rm31080_release(struct inode *inode, struct file *filp)
{
	struct rm31080_data *rm = filp->private_data;

	/* Only reset on the LAST close -- see the NOTE on open_count above.
	 * A secondary opener closing early (debug tooling, udev, etc.)
	 * must not tear down the primary (HAL) session's state. */
	if (!atomic_dec_and_test(&rm->open_count))
		return 0;

	/* Ported from dev_release(): the touch IRQ isn't tied to whether
	 * the char device is held open, so without this the pipeline would
	 * keep running full ACTIVE-mode scan/read/signal cycles against no
	 * HAL to receive them once userspace closes the fd. init_finished
	 * gates both rm31080_irq() (queues no further work) and
	 * rm31080_irq_work() (bails immediately if already queued) --
	 * see the flush_workqueue() ordering note on
	 * rm31080_ctrl_enter_manual_mode(). */
	rm->init_finished = false;
	rm31080_ctrl_enter_manual_mode(rm);

	dev_dbg(&rm->spi->dev, "device released (last close): init_finished=false, forced to ACTIVE_MODE\n");

	return 0;
}

/*
 * read(fd, buf, n): userspace pre-populates buf[0] with the register
 * address it wants to read (see bReadSensor()/raydium_spi_read() in the
 * userspace HAL); the kernel echoes that byte's value as the SPI
 * address, ORs in the read bit, and overwrites buf with the result.
 * Ported from dev_read() in rm31080a_ts.c -- including reading buf[0]
 * from the not-yet-copied-in user buffer, which is exactly what
 * downstream does (relying on the previous read()'s leftover content,
 * or whatever the HAL pre-seeded userspace-side).
 */
static ssize_t rm31080_read(struct file *filp, char __user *buf, size_t count, loff_t *pos)
{
	struct rm31080_data *rm = filp->private_data;
	u8 *kbuf;
	u8 addr;
	ssize_t status;

	if (!count)
		return 0;

	if (get_user(addr, (u8 __user *)buf))
		return -EFAULT;

	kbuf = kmalloc(count, GFP_KERNEL);
	if (!kbuf)
		return -ENOMEM;

	if (rm31080_spi_read(rm, addr, kbuf, count)) {
		status = -EFAULT;
	} else {
		status = copy_to_user(buf, kbuf, count) ? -EFAULT : count;
	}

	kfree(kbuf);
	return status;
}

/* write(fd, buf, n): literal passthrough, buf[0] = register address
 * (no read bit), buf[1..] = data. Ported from dev_write(). */
static ssize_t rm31080_write(struct file *filp, const char __user *buf, size_t count, loff_t *pos)
{
	struct rm31080_data *rm = filp->private_data;
	u8 *kbuf;
	ssize_t status;

	if (!count)
		return 0;

	kbuf = kmalloc(count, GFP_KERNEL);
	if (!kbuf)
		return -ENOMEM;

	if (copy_from_user(kbuf, buf, count)) {
		status = -EFAULT;
	} else {
		status = rm31080_spi_write(rm, kbuf, count) ? -EFAULT : count;
	}

	kfree(kbuf);
	return status;
}

/* =========================================================================
 * ioctl dispatch
 * Ported from dev_ioctl() in rm31080a_ts.c. index = upper 16 bits of cmd,
 * matching userspace's (index << 16) | ioctl packing.
 * ========================================================================= */

static u32 rm31080_get_variable(struct rm31080_data *rm, unsigned int index, u8 __user *p)
{
	u8 val;

	switch (index) {
	case RM_VARIABLE_PLATFORM_ID:
		val = (u8)rm->platform_id;
		break;
	case RM_VARIABLE_GPIO_SELECT:
		val = (u8)rm->gpio_select;
		break;
	case RM_VARIABLE_CHECK_SPI_LOCK:
		val = rm->spi_locked | rm->is_suspended;
		/* Downstream's rm_tch_get_spi_lock_status(): decrement resume_cnt
		 * here, at the polling site where userspace checks lock status.
		 * Only decrement if spi_locked is true (the condition downstream
		 * also checks). */
		if (rm->spi_locked && rm->resume_cnt)
			rm->resume_cnt--;
		break;
	default:
		dev_dbg(&rm->spi->dev, "GET_VARIABLE: unhandled index=%u\n", index);
		return -EINVAL;
	}
	dev_dbg(&rm->spi->dev, "GET_VARIABLE: index=%u val=%u\n", index, val);
	return copy_to_user(p, &val, 1) ? RETURN_FAIL : RETURN_OK;
}

static void rm31080_set_variable(struct rm31080_data *rm, unsigned int index, unsigned long arg)
{
	switch (index) {
	case RM_VARIABLE_AUTOSCAN_FLAG:
		/* downstream's switch case ignores arg entirely (userspace
		 * only ever passes 1 anyway) and only fires the transition
		 * out of ACTIVE_MODE -- calling this again while already
		 * PRE_IDLE/IDLE is a silent no-op, exactly like here. */
		mutex_lock(&rm->scan_mode_lock);
		if (rm->scan_mode_state == RM_SCAN_ACTIVE_MODE)
			rm->scan_mode_state = RM_SCAN_PRE_IDLE_MODE;
		mutex_unlock(&rm->scan_mode_lock);
		dev_dbg(&rm->spi->dev,
			"SET_VARIABLE(AUTOSCAN_FLAG): scan_mode_state=%u\n",
			rm->scan_mode_state);
		break;
	case RM_VARIABLE_WATCHDOG_FLAG:
		rm31080_watchdog_configure(rm, arg);
		break;
	case RM_VARIABLE_SET_SPI_UNLOCK:
		/* Downstream: skip unlock if resume_cnt > 1 (another resume
		 * is still pending). Defer to the next polling cycle of
		 * RM_VARIABLE_CHECK_SPI_LOCK, which will decrement and
		 * eventually permit the unlock when resume_cnt reaches 1. */
		if (rm->resume_cnt > 1) {
			dev_dbg(&rm->spi->dev,
				"SET_SPI_UNLOCK: deferring (resume_cnt=%u > 1)\n",
				rm->resume_cnt);
			break;
		}
		rm->spi_locked = false;
		dev_dbg(&rm->spi->dev, "SET_SPI_UNLOCK: spi_locked=false\n");
		break;
	case RM_VARIABLE_REPEAT:
		/* downstream also stores this into a general-purpose
		 * g_st_ts.u8_repeat with no other consumer we've found in
		 * any of the three driver versions checked; only the
		 * noise-scan-specific half (u8_ns_rpt, consulted by
		 * rm31080_ctrl_scan_start()) is tracked here. */
		rm->ns_rpt = (u8)arg;
		dev_dbg(&rm->spi->dev, "SET_VARIABLE(REPEAT): ns_rpt=%u\n", rm->ns_rpt);
		break;
	case RM_VARIABLE_DPW:
		/* Unlike every other SET_VARIABLE case, arg here is a user
		 * pointer to a 9-byte buffer (3 noise-scan columns x 3
		 * bytes), not a packed scalar -- ported from
		 * copy_from_user(&g_st_ts.u8_ns_para[0], (u8 *)arg, 9). */
		mutex_lock(&rm->ns_mode_lock);
		if (copy_from_user(rm->ns_para, (void __user *)(uintptr_t)arg,
				    sizeof(rm->ns_para)))
			dev_warn(&rm->spi->dev, "RM_VARIABLE_DPW: copy_from_user failed\n");
		else
			dev_dbg(&rm->spi->dev,
				"SET_VARIABLE(DPW): ns_para=[%u,%u,%u,%u,%u,%u,%u,%u,%u]\n",
				rm->ns_para[0], rm->ns_para[1], rm->ns_para[2],
				rm->ns_para[3], rm->ns_para[4], rm->ns_para[5],
				rm->ns_para[6], rm->ns_para[7], rm->ns_para[8]);
		mutex_unlock(&rm->ns_mode_lock);
		break;
	case RM_VARIABLE_NS_MODE:
		/*
		 * NOT bounds-checked downstream either (g_st_ts.u8_ns_mode =
		 * (u8)arg; verbatim, no clamp) -- this is a faithfully-ported
		 * gap being closed, not a new one. ns_mode is the wrap
		 * boundary for ns_sel_idx in rm31080_ctrl_scan_start(), which
		 * directly indexes the 9-byte ns_para[] (3 columns x 3 bytes
		 * -- see the NOTE on ns_para above). Any arg > 2 here would
		 * let ns_sel_idx climb past column 2 and read out of bounds
		 * on every following scan_start, via both
		 * rm->ns_para[rm->ns_sel_idx] directly and
		 * rm31080_set_ns_para()'s ns_para[ii * 3 + col]. The HAL is
		 * closed-source and this ioctl is reachable by anything with
		 * access to /dev/touch, so clamp rather than trust it -- a
		 * conforming caller only ever sends 0-2 anyway (matches what
		 * we've observed live: ns_mode=1), so this changes nothing
		 * for valid input.
		 */
		mutex_lock(&rm->ns_mode_lock);
		rm->ns_mode = min_t(u8, (u8)arg, 2);
		mutex_unlock(&rm->ns_mode_lock);
		dev_dbg(&rm->spi->dev, "SET_VARIABLE(NS_MODE): ns_mode=%u (requested %lu)\n",
			rm->ns_mode, arg);
		break;
	case RM_VARIABLE_VERSION:
	case RM_VARIABLE_TEST_VERSION:
	case RM_VARIABLE_SELF_TEST_RESULT:
	case RM_VARIABLE_SCRIBER_FLAG:
	default:
		/* NOTE: these are diagnostic/self-test hooks in downstream
		 * (rm_tch_enter_test_mode(), ...) that don't affect the
		 * basic touch data path. Accepted and otherwise ignored. */
		break;
	case RM_VARIABLE_SET_WAKE_UNLOCK:
		/* Ported: see the NOTE on wakelock_init above. Unlike
		 * downstream's wake_lock_active() guard, __pm_relax() is
		 * safe to call unconditionally. */
		if (rm->wakelock_init)
			__pm_relax(rm->wakelock_init);
		dev_dbg(&rm->spi->dev, "SET_VARIABLE(SET_WAKE_UNLOCK)\n");
		break;
	case RM_VARIABLE_IDLEMODECHECK:
		/*
		 * Verbatim port of downstream's case: a whole-byte overwrite
		 * of the same ctrl.u8_idle_mode_check field that
		 * rm31080_ctrl_enter_auto_mode()/_leave_auto_mode() toggle
		 * bit 0 of (see the NOTE on scan_mode_state). We couldn't
		 * confirm the exact arg value at the one userspace call site
		 * we found (librm31080.so elides it in decompilation), but
		 * downstream applies this unconditionally with no bit-0
		 * preservation and no locking either, so we match that
		 * exactly rather than second-guessing it.
		 */
		rm->ctrl.u8_idle_mode_check = (u8)arg;
		dev_dbg(&rm->spi->dev, "SET_VARIABLE(IDLEMODECHECK): ctrl.u8_idle_mode_check=0x%02x\n",
			rm->ctrl.u8_idle_mode_check);
		break;
	case RM_VARIABLE_TOUCHFILE_STATUS:
		/* Confirmed live: librm31080.so's send_file_info_to_kernel()
		 * reports calibration-file load status here (0 = OK; nonzero
		 * = a HAL-defined error code -- checksum mismatch, bad
		 * version, etc. -- not part of this driver's protocol).
		 * Downstream's only consumer is a sysfs show handler we
		 * don't implement, so this is stored and logged but
		 * otherwise inert. */
		rm->touchfile_check = (u8)arg;
		dev_dbg(&rm->spi->dev, "SET_VARIABLE(TOUCHFILE_STATUS): 0x%02x\n",
			rm->touchfile_check);
		break;
	case RM_VARIABLE_TOUCH_EVENT: {
		/* Confirmed live: librm31080.so's WaterAreaDetection() and
		 * its noise-detection counterpart call this directly (values
		 * matching the RM_TOUCH_EVENT_* constants above exactly).
		 * Unlike TOUCHFILE_STATUS, downstream's consumer here
		 * (rm_tch_generate_event()) is NOT sysfs-gated -- it fires a
		 * uevent unconditionally, so this is a real behavioural port,
		 * not just bookkeeping. */
		char *reason;
		char envp_reason[24];
		char *envp[] = { envp_reason, NULL };

		rm->touch_event = (u8)arg;
		switch (rm->touch_event) {
		case RM_TOUCH_EVENT_STYLUS_DISABLE_BY_WATER:
			reason = "Water";
			break;
		case RM_TOUCH_EVENT_STYLUS_DISABLE_BY_NOISE:
			reason = "Noise";
			break;
		case RM_TOUCH_EVENT_STYLUS_IS_ENABLED:
			reason = "None";
			break;
		default:
			reason = "Others";
			break;
		}
		snprintf(envp_reason, sizeof(envp_reason), "STYLUS_DISABLE=%s", reason);

		dev_dbg(&rm->spi->dev, "SET_VARIABLE(TOUCH_EVENT): 0x%02x -> %s\n",
			rm->touch_event, envp_reason);
		kobject_uevent_env(&rm->miscdev.this_device->kobj, KOBJ_CHANGE, envp);
		break;
	}
	}
}

/*
 * Mirrors one (res_x, res_y) size pair onto both the MT and single-touch
 * (declarative-only, see rm31080_probe()) axes. Called with DT/fallback
 * sizes at probe time and again with whatever RM_IOCTL_SET_PARAMETER
 * supplies at runtime -- both need the same treatment, hence factoring it
 * out rather than duplicating it. Inversion (invert_x/invert_y) doesn't
 * affect the advertised range, only the reported value at each point (see
 * rm31080_report_pointer()), so there's nothing orientation-related to do
 * here beyond the plain set.
 */
static void rm31080_set_input_abs_range(struct rm31080_data *rm, u16 res_x, u16 res_y)
{
	input_set_abs_params(rm->input, ABS_MT_POSITION_X, 0, res_x - 1, 0, 0);
	input_set_abs_params(rm->input, ABS_MT_POSITION_Y, 0, res_y - 1, 0, 0);
	input_set_abs_params(rm->input, ABS_X, 0, res_x - 1, 0, 0);
	input_set_abs_params(rm->input, ABS_Y, 0, res_y - 1, 0, 0);
}

static long rm31080_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct rm31080_data *rm = filp->private_data;
	void __user *argp = (void __user *)(uintptr_t)(arg & MASK_USER_SPACE_POINTER);
	unsigned int index = (cmd >> 16) & 0xFFFF;
	long ret = RETURN_OK;

	switch (cmd & 0xFFFF) {
	case RM_IOCTL_REPORT_POINT: {
		struct rm_touch_event *tp = kmalloc(sizeof(*tp), GFP_KERNEL);

		if (!tp)
			return -ENOMEM;
		if (copy_from_user(tp, argp, sizeof(*tp))) {
			kfree(tp);
			return -EFAULT;
		}
		rm31080_report_pointer(rm, tp);
		kfree(tp);
		break;
	}
	case RM_IOCTL_FINISH_CALC:
		rm->calc_finished = true;
		break;
	case RM_IOCTL_READ_RAW_DATA:
		ret = rm31080_q_read_raw_data(rm, argp, index);
		ret = (ret == RETURN_OK) ? 1 : 0; /* ts.default.so checks nonzero = frame available */
		break;
	case RM_IOCTL_SET_HAL_PID:
		if (rm->hal_pid)
			put_pid(rm->hal_pid);
		rm->hal_pid = get_pid(find_get_pid((pid_t)arg));
		dev_dbg(&rm->spi->dev, "SET_HAL_PID: pid=%d %s\n", (pid_t)arg,
			rm->hal_pid ? "acquired" : "lookup failed");
		break;
	case RM_IOCTL_WATCH_DOG:
		rm31080_watchdog_request(rm);
		break;
	case RM_IOCTL_GET_VARIABLE:
		ret = rm31080_get_variable(rm, index, argp);
		break;
	case RM_IOCTL_INIT_START:
		rm->init_finished = false;
		rm31080_ctrl_enter_manual_mode(rm);
		dev_dbg(&rm->spi->dev, "INIT_START: init_finished=false, forced to ACTIVE_MODE\n");
		break;
	case RM_IOCTL_INIT_END:
		rm->init_finished = true;
		rm->calc_finished = true;
		if (rm->resume_cnt)
			rm->resume_cnt--;
		if (rm->wakelock_init)
			__pm_relax(rm->wakelock_init);
		dev_dbg(&rm->spi->dev, "INIT_END: init_finished=true, calc_finished=true, resume_cnt=%u\n",
			rm->resume_cnt);
		break;
	case RM_IOCTL_SCRIBER_CTRL:
		/* NOTE: scriber (palm-rejection style) mode flag; stored but
		 * not currently consulted anywhere in this port. */
		break;
	case RM_IOCTL_SET_PARAMETER:
		if (copy_from_user(&rm->ctrl, argp, sizeof(rm->ctrl)))
			return -EFAULT;
		if (rm->ctrl.u16_resolution_x && rm->ctrl.u16_resolution_y)
			rm31080_set_input_abs_range(rm, rm->ctrl.u16_resolution_x,
						     rm->ctrl.u16_resolution_y);
		dev_dbg(&rm->spi->dev,
			"SET_PARAMETER: data_length=%u resolution=%ux%u timer_trigger_scale=%u idle_mode_check=%u watch_dog_normal_cnt=%u\n",
			rm->ctrl.u16_data_length, rm->ctrl.u16_resolution_x,
			rm->ctrl.u16_resolution_y, rm->ctrl.u8_timer_trigger_scale,
			rm->ctrl.u8_idle_mode_check, rm->ctrl.u8_watch_dog_normal_cnt);
		break;
	case RM_IOCTL_SET_BASELINE: {
		size_t len = rm->ctrl.u16_data_length ? rm->ctrl.u16_data_length : RM_RAW_DATA_LENGTH;

		if (!rm->baseline)
			return -ENOMEM;
		if (copy_from_user(rm->baseline, argp, len))
			return -EFAULT;
		rm->baseline_pending = true;
		break;
	}
	case RM_IOCTL_SET_VARIABLE:
		rm31080_set_variable(rm, index, arg);
		break;
	case RM_IOCTL_SET_KRL_TBL: {
		u8 hdr[KRL_TBL_FIELD_POS_CASE_NUM];
		u16 len;

		if (index >= KRL_TBL_COUNT)
			return -EINVAL;

		/* Peek just the 2-byte length header first -- userspace's
		 * source pointer here points into the middle of a packed,
		 * variable-length buffer (all 16 tables concatenated); a
		 * blind copy of the full KRL_TBL_MAX_LEN from a table that
		 * doesn't start near the front of that buffer runs off its
		 * end. Ported from rm_set_kernel_tbl(). */
		if (copy_from_user(hdr, argp, sizeof(hdr)))
			return -EFAULT;

		len = hdr[KRL_TBL_FIELD_POS_LEN_H];
		len <<= 8;
		len |= hdr[KRL_TBL_FIELD_POS_LEN_L];

		if (len < sizeof(hdr) || len > KRL_TBL_MAX_LEN)
			return -EINVAL;

		memset(rm->krl_tbl[index], 0, KRL_TBL_MAX_LEN);
		if (copy_from_user(rm->krl_tbl[index], argp, len))
			return -EFAULT;
		dev_dbg(&rm->spi->dev, "SET_KRL_TBL: index=%u len=%u\n", index, len);
		break;
	}
	case RM_IOCTL_GET_SCAN_MODE: {
		u8 val = rm->ctrl.u8_idle_mode_check;

		ret = copy_to_user(argp, &val, 1) ? RETURN_FAIL : RETURN_OK;
		break;
	}
	case RM_IOCTL_INIT_SERVICE:
		rm->init_service = true;
		dev_dbg(&rm->spi->dev, "INIT_SERVICE: init_service=true\n");
		break;
	case RM_IOCTL_SET_CLK:
		if (rm->clk) {
			if (arg)
				ret = clk_prepare_enable(rm->clk);
			else
				clk_disable_unprepare(rm->clk);
		}
		dev_dbg(&rm->spi->dev, "SET_CLK: arg=%lu clk_present=%d ret=%ld\n",
			arg, !!rm->clk, ret);
		break;
	default:
		return -EINVAL;
	}

	return ret ? -EIO : 0;
}

static const struct file_operations rm31080_fops = {
	.owner = THIS_MODULE,
	.open = rm31080_open,
	.release = rm31080_release,
	.read = rm31080_read,
	.write = rm31080_write,
	.unlocked_ioctl = rm31080_ioctl,
	.compat_ioctl = rm31080_ioctl,
};

/* =========================================================================
 * input_dev open/close
 *
 * Ported from rm_tch_input_open()/rm_tch_input_close() in rm31080a_ts.c.
 * The Linux input core calls .open() the moment the first reader (Xorg's
 * evdev backend, evtest, libinput, ...) opens this device's
 * /dev/input/eventN node, and .close() once the last reader closes it.
 * This -- not anything tied to INIT_SERVICE/INIT_END or the KRL tables --
 * is what downstream actually uses to gate the hardware IRQ: no consumer
 * has the device open, no reason to be scanning. Previously missing from
 * this port entirely, which left the IRQ masked for the driver's whole
 * lifetime except as an incidental side effect of whatever CONFIG_IRQ
 * opcodes happen to appear in an executed KRL table.
 * ========================================================================= */

static int rm31080_input_open(struct input_dev *input)
{
	struct rm31080_data *rm = input_get_drvdata(input);

	dev_dbg(&rm->spi->dev, "input device opened, enabling irq %d\n", rm->irq);
	enable_irq(rm->irq);
	return 0;
}

static void rm31080_input_close(struct input_dev *input)
{
	struct rm31080_data *rm = input_get_drvdata(input);

	dev_dbg(&rm->spi->dev, "input device closed, disabling irq %d\n", rm->irq);
	disable_irq(rm->irq);
}

/* =========================================================================
 * Probe / remove / PM
 * ========================================================================= */

static int rm31080_probe(struct spi_device *spi)
{
	struct device *dev = &spi->dev;
	struct rm31080_data *rm;
	int i, ret;

	rm = devm_kzalloc(dev, sizeof(*rm), GFP_KERNEL);
	if (!rm)
		return -ENOMEM;

	rm->spi = spi;
	spi_set_drvdata(spi, rm);
	mutex_init(&rm->krl_lock);
	mutex_init(&rm->q_lock);
	mutex_init(&rm->ns_mode_lock);
	mutex_init(&rm->scan_mode_lock);
	rm->ns_last_rpt = 1;	/* matches downstream's static u8Rpt = 1 */
	INIT_WORK(&rm->irq_work, rm31080_irq_work);
	INIT_DELAYED_WORK(&rm->timer_work, rm31080_timer_work);

	for (i = 0; i < KRL_TBL_COUNT; i++) {
		rm->krl_tbl[i] = devm_kzalloc(dev, KRL_TBL_MAX_LEN, GFP_KERNEL);
		if (!rm->krl_tbl[i])
			return -ENOMEM;
	}

	rm->queue = devm_kzalloc(dev, (size_t)QUEUE_COUNT * RM_RAW_DATA_LENGTH, GFP_KERNEL);
	if (!rm->queue)
		return -ENOMEM;

	rm->baseline = devm_kzalloc(dev, RM_RAW_DATA_LENGTH, GFP_KERNEL);
	if (!rm->baseline)
		return -ENOMEM;

	if (of_property_read_u32(dev->of_node, "raydium,platform-id", &rm->platform_id)) {
		dev_err(dev, "missing required raydium,platform-id property\n");
		return -EINVAL;
	}
	of_property_read_u32(dev->of_node, "raydium,gpio-select", &rm->gpio_select);

	/* Request already-asserted (logical 1 = physical low, given the DT
	 * node's GPIO_ACTIVE_LOW flag): downstream's board-default pinmux
	 * table already leaves this pin driven low (asserted) from very
	 * early boot, before the touch driver even loads, and rm_tch_spi_probe()
	 * explicitly re-asserts before its timed pulse. Starting anywhere
	 * else risks a shorter/undefined reset pulse on the very first
	 * probe. */
	rm->gpio_reset = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(rm->gpio_reset))
		return dev_err_probe(dev, PTR_ERR(rm->gpio_reset), "failed to get reset gpio\n");

	rm->avdd = devm_regulator_get(dev, "avdd");
	if (IS_ERR(rm->avdd))
		return dev_err_probe(dev, PTR_ERR(rm->avdd), "failed to get avdd-supply\n");
	rm->dvdd = devm_regulator_get(dev, "dvdd");
	if (IS_ERR(rm->dvdd))
		return dev_err_probe(dev, PTR_ERR(rm->dvdd), "failed to get dvdd-supply\n");

	/* Optional discrete-GPIO alternative to avdd-supply/dvdd-supply --
	 * see the NOTE on gpio_avdd/gpio_dvdd above. NULL (not a probe
	 * failure) when absent, which is the normal case on boards like
	 * ours that power both rails via regulators instead. No initial
	 * state is forced here: unlike gpio_reset, which this driver drives
	 * unconditionally as part of every probe, these are only ever
	 * touched on demand by a KRL table's CONFIG_3V3/1V8 GPIO sub-cmd on
	 * boards that have one, which sets its own on/off state explicitly
	 * before relying on it. */
	rm->gpio_avdd = devm_gpiod_get_optional(dev, "avdd", GPIOD_ASIS);
	if (IS_ERR(rm->gpio_avdd))
		return dev_err_probe(dev, PTR_ERR(rm->gpio_avdd), "failed to get avdd gpio\n");
	rm->gpio_dvdd = devm_gpiod_get_optional(dev, "dvdd", GPIOD_ASIS);
	if (IS_ERR(rm->gpio_dvdd))
		return dev_err_probe(dev, PTR_ERR(rm->gpio_dvdd), "failed to get dvdd gpio\n");

	/* devm_clk_get_optional_enabled(): optional (matches downstream's
	 * defensive "if (ts && ts->clk)" checks around every other use of
	 * this clock -- this board's DTS always defines it, but other
	 * RM31080 boards may not) AND auto-prepares+enables now, with
	 * devm automatically disabling+unpreparing on remove(). Plain
	 * devm_clk_get() only manages the handle, not the enable state --
	 * the prior code called clk_prepare_enable() in probe with no
	 * matching disable anywhere in remove(), leaking the enable
	 * refcount on every unbind. */
	rm->clk = devm_clk_get_optional_enabled(dev, NULL);
	if (IS_ERR(rm->clk))
		return dev_err_probe(dev, PTR_ERR(rm->clk), "failed to get touch clock\n");

	/* Power/clock/reset sequence ported from rm_tch_spi_probe(): 1.8V
	 * (dvdd) enabled first, settle, THEN 3.3V (avdd) -- downstream's own
	 * comments say "Enable 1v8 first" / "Enable 3v3 then", in that
	 * order, with the settle delay between them, not after both. Then
	 * hold reset low 120ms, release, settle 20ms. The reset timing is
	 * the exact same numbers dmesg shows the downstream driver using. */
	ret = regulator_enable(rm->dvdd);
	if (ret)
		return dev_err_probe(dev, ret, "failed to enable dvdd\n");
	usleep_range(5000, 6000);
	ret = regulator_enable(rm->avdd);
	if (ret)
		return dev_err_probe(dev, ret, "failed to enable avdd\n");

	gpiod_set_value_cansleep(rm->gpio_reset, 1); /* assert (already asserted; explicit for clarity/timing) */
	msleep(120);
	gpiod_set_value_cansleep(rm->gpio_reset, 0); /* release */
	msleep(20);

	spi->mode = SPI_MODE_0;
	ret = spi_setup(spi);
	if (ret)
		return dev_err_probe(dev, ret, "spi_setup failed\n");

	/* IRQ: prefer whatever spi->irq the core already resolved from a
	 * standard "interrupts"/"interrupts-extended" property on the DT
	 * node; fall back to an explicit "irq" gpio if the DTS instead wires
	 * this the same way as the reset line. */
	rm->irq = spi->irq;
	if (rm->irq <= 0) {
		rm->gpio_irq = devm_gpiod_get(dev, "irq", GPIOD_IN);
		if (IS_ERR(rm->gpio_irq))
			return dev_err_probe(dev, PTR_ERR(rm->gpio_irq),
					      "no usable interrupt (neither spi->irq nor irq-gpios)\n");
		rm->irq = gpiod_to_irq(rm->gpio_irq);
		if (rm->irq < 0)
			return rm->irq;
	}

	rm->wq = create_singlethread_workqueue("rm31080_ts");
	if (!rm->wq)
		return -ENOMEM;

	/* Match downstream: IRQ is rising-edge, threaded, one-shot
	 * (IRQF_TRIGGER_RISING | IRQF_ONESHOT in rm_tch_spi_probe()), and
	 * starts masked (IRQF_NO_AUTOEN here vs. an explicit
	 * rm_tch_disable_irq() call right after request_threaded_irq()
	 * downstream -- same effect). It only gets unmasked by
	 * rm31080_input_open() below, once something actually opens the
	 * input device -- which is why this MUST be requested before
	 * input_register_device() runs: that call can make the device
	 * visible to userspace (and thus opened, and thus enable_irq()'d)
	 * before this function returns. */
	ret = devm_request_threaded_irq(dev, rm->irq, NULL, rm31080_irq,
					 IRQF_TRIGGER_RISING | IRQF_ONESHOT | IRQF_NO_AUTOEN,
					 dev_name(dev), rm);
	if (ret) {
		dev_err_probe(dev, ret, "failed to request irq\n");
		goto err_wq;
	}

	rm->input = devm_input_allocate_device(dev);
	if (!rm->input) {
		ret = -ENOMEM;
		goto err_wq;
	}
	rm->input->name = "Raydium RM31080 Touchscreen";
	rm->input->id.bustype = BUS_SPI;
	rm->input->open = rm31080_input_open;
	rm->input->close = rm31080_input_close;
	input_set_drvdata(rm->input, rm);
	__set_bit(EV_ABS, rm->input->evbit);
	__set_bit(EV_KEY, rm->input->evbit);
	__set_bit(BTN_TOOL_RUBBER, rm->input->keybit);
	/*
	 * The modern, portable way to tell userspace "this is a direct-touch
	 * touchscreen" -- libinput (under both X11 and, critically, Wayland
	 * compositors that never read Xorg.conf at all) uses this INPUT_PROP
	 * bit directly, rather than the legacy Xorg evdev driver's
	 * MatchIsTouchscreen heuristics (BTN_TOOL_RUBBER presence, see the
	 * NOTE just below) that predate it.
	 */
	__set_bit(INPUT_PROP_DIRECT, rm->input->propbit);
	/* Capability-only, never actually reported: ported from the L4T
	 * fork, which declares these (but never calls input_report_key/abs
	 * on them -- verified, neither it nor android_kernel's rm31080a_ts.c
	 * ever does) purely so classification heuristics that predate full
	 * MT-B awareness -- notably Xorg's evdev driver's
	 * MatchIsTouchscreen -- recognize this as a touchscreen at all. If
	 * nothing ever opens the input device, input_dev->open() never
	 * fires, the IRQ never gets unmasked, and the raw-data queue never
	 * gets populated -- exactly the symptom this was chasing. */
	input_set_capability(rm->input, EV_KEY, BTN_TOUCH);

	/*
	 * NOT downstream: g_st_ts here just has ABS_(MT_)X/Y range hardcoded
	 * to RM_INPUT_RESOLUTION_X/Y (was 4096x4096, an arbitrary square
	 * placeholder unrelated to this board's actual 1200x1920 panel) and
	 * relies entirely on RM_IOCTL_SET_PARAMETER to correct it later, at
	 * whatever point the closed-source HAL happens to call it. That is
	 * exactly the "wrong until a manual Xorg Calibration quirk compensates
	 * for it" symptom this replaces -- if libinput/Wayland enumerate this
	 * device before SET_PARAMETER ever fires, they cache the wrong
	 * initial range and don't revisit it later just because the driver
	 * quietly called input_set_abs_params() again. Read the real range
	 * from DT before registration instead, so it's right from the very
	 * first enumeration; SET_PARAMETER below still applies on top if the
	 * HAL later supplies its own values, same as before.
	 */
	rm->default_res_x = RM_INPUT_RESOLUTION_X;
	rm->default_res_y = RM_INPUT_RESOLUTION_Y;
	{
		u32 val;

		if (!device_property_read_u32(dev, "touchscreen-size-x", &val))
			rm->default_res_x = (u16)val;
		if (!device_property_read_u32(dev, "touchscreen-size-y", &val))
			rm->default_res_y = (u16)val;
	}
	rm->invert_x = device_property_read_bool(dev, "touchscreen-inverted-x");
	rm->invert_y = device_property_read_bool(dev, "touchscreen-inverted-y");
	input_set_capability(rm->input, EV_ABS, ABS_PRESSURE);
	rm31080_set_input_abs_range(rm, rm->default_res_x, rm->default_res_y);
	input_set_abs_params(rm->input, ABS_MT_PRESSURE, 0, 0xFF, 0, 0);
	input_set_abs_params(rm->input, ABS_MT_TOOL_TYPE, 0, MT_TOOL_MAX, 0, 0);
	ret = input_mt_init_slots(rm->input, MAX_SLOT_AMOUNT, INPUT_MT_DIRECT);
	if (ret) {
		dev_err_probe(dev, ret, "input_mt_init_slots failed\n");
		goto err_wq;
	}
	ret = input_register_device(rm->input);
	if (ret) {
		dev_err_probe(dev, ret, "input_register_device failed\n");
		goto err_wq;
	}

	rm->miscdev.minor = MISC_DYNAMIC_MINOR;
	rm->miscdev.name = "touch";
	rm->miscdev.fops = &rm31080_fops;
	ret = misc_register(&rm->miscdev);
	if (ret) {
		dev_err_probe(dev, ret, "misc_register failed\n");
		goto err_wq;
	}

	rm->wakelock_init = wakeup_source_register(dev, "raydium_touch_wakelock");
	if (!rm->wakelock_init)
		dev_warn(dev, "wakeup_source_register failed, continuing without it\n");

	/* Matches add_timer(&ts_timer_triggle) at the end of
	 * rm_tch_spi_probe(): runs unconditionally for the life of the
	 * device, not gated on watchdog_enable (see the long NOTE on the
	 * struct fields above). */
	schedule_delayed_work(&rm->timer_work, HZ);

	g_rm = rm;
	dev_info(dev, "Raydium RM31080 bridge ready (platform_id=0x%02x gpio_select=0x%02x)\n",
		 rm->platform_id, rm->gpio_select);
	return 0;

err_wq:
	destroy_workqueue(rm->wq);
	return ret;
}

static void rm31080_remove(struct spi_device *spi)
{
	struct rm31080_data *rm = spi_get_drvdata(spi);

	cancel_delayed_work_sync(&rm->timer_work);
	flush_workqueue(rm->wq);
	destroy_workqueue(rm->wq);
	misc_deregister(&rm->miscdev);
	if (rm->wakelock_init)
		wakeup_source_unregister(rm->wakelock_init);
	if (rm->hal_pid)
		put_pid(rm->hal_pid);
	/* rm_tch_spi_remove(): regulator_disable() in the same order
	 * downstream does it, avdd (3v3) then dvdd (1v8) -- reverse of the
	 * dvdd-then-avdd enable order in probe. These are plain
	 * devm_regulator_get() handles (not the *_enable variant), because
	 * krl_config_3v3()/krl_config_1v8() above also toggle them at
	 * runtime via KRL bytecode, so devm can't auto-manage the enable
	 * state the way it does for the clock below. Unlike downstream's
	 * kzalloc+goto probe, a failed regulator_enable() in our probe
	 * returns early via dev_err_probe() before remove() could ever be
	 * reached with an invalid handle, so no extra guard is needed here. */
	regulator_disable(rm->avdd);
	regulator_disable(rm->dvdd);
	if (g_rm == rm)
		g_rm = NULL;
}

#define TCH_WAKE_LOCK_TIMEOUT_MS	500	/* downstream: TCH_WAKE_LOCK_TIMEOUT (HZ/2) */

/*
 * Releases every MT slot (Type B) as "not touching" and syncs. Downstream:
 * the INPUT_PROTOCOL_TYPE_B block at the end of rm_ctrl_suspend() -- same
 * loop shape as the no-touch-count-transition case in
 * rm31080_report_pointer() above, but that copy is left alone (rather than
 * factored into a shared helper called from both places) so this
 * suspend-path addition can't perturb the already hardware-verified
 * touch-reporting path.
 */
static void rm31080_release_all_touches(struct rm31080_data *rm)
{
	int i;

	for (i = 0; i < MAX_SLOT_AMOUNT; i++) {
		input_mt_slot(rm->input, i);
		input_mt_report_slot_state(rm->input, MT_TOOL_FINGER, false);
		input_report_key(rm->input, BTN_TOOL_RUBBER, false);
	}
	input_sync(rm->input);
	rm->last_touch_count = 0;
}

/*
 * Below is a full function-by-function port of rm_ctrl_suspend() /
 * rm_ctrl_resume() / rm_tch_init_ts_structure_part() against the real
 * downstream source (previously flagged as "least battle-tested part of
 * the port" -- not just the wakelock piece, but not yet traced either).
 * Deliberately NOT ported, with reasons:
 *   - early_suspend (CONFIG_HAS_EARLYSUSPEND) and the input_dev
 *     enable()/disable() Android-powerHAL hooks (rm_tch_input_enable/
 *     _disable): both alternate entry points into rm_tch_suspend/resume()
 *     alongside the standard dev_pm_ops path. Neither exists on a mainline
 *     non-Android target -- early_suspend was removed from mainline years
 *     ago, and there's no powerHAL here. SIMPLE_DEV_PM_OPS below is our
 *     only entry point, same as downstream's rm_dev_pm_suspend/_resume.
 *   - The IRQ-poster kthread stop/start (ISR_POST_HANDLER==KTHREAD) and
 *     event-queue thread stop (ENABLE_EVENT_QUEUE): different ISR
 *     backends we don't share -- we use a plain threaded IRQ + workqueue,
 *     for which disable_irq()/enable_irq() around the whole sequence is
 *     the direct equivalent of "stop new work from starting".
 *   - g_pu8_burstread_buf / g_worker_queue_is_flush / g_timer_queue_is_flush
 *     resets inside rm_tch_init_ts_structure_part(): internal bookkeeping
 *     for downstream's own buffer/queue-flush tracking, which doesn't map
 *     onto our queue/q_front/q_rear ring-buffer design.
 *   - u8_test_mode_type, b_is_disabled: diagnostics/powerHAL fields with
 *     no functional consumer anywhere else in this port either (same
 *     boundary as the existing RM_IOCTL_SCRIBER_CTRL NOTE).
 */
static int __maybe_unused rm31080_suspend(struct device *dev)
{
	struct rm31080_data *rm = dev_get_drvdata(dev);

	/* rm_ctrl_suspend(): "if (b_is_suspended == true) return;" -- our
	 * single dev_pm_ops entry point doesn't have downstream's triple
	 * entry-path re-entrancy risk, but the guard is cheap and keeps
	 * disable_irq() balanced against a duplicate call. */
	if (rm->is_suspended)
		return 0;

	disable_irq(rm->irq);

	rm->is_suspended = true;
	rm->init_finished = false;	/* b_init_finish = 0 */

	mutex_lock(&rm->scan_mode_lock);
	if (rm->scan_mode_state == RM_SCAN_IDLE_MODE)
		rm31080_cmd_process(rm, 0, rm->krl_tbl[KRL_INDEX_FUNC_PAUSE_AUTO]);
	mutex_unlock(&rm->scan_mode_lock);

	/* Two distinct wait-for-scan-finish calls at two distinct points,
	 * matching rm_ctrl_suspend() exactly: once before touching the
	 * suspend table at all (let anything already in flight finish),
	 * and once between the table's two cases (case 0 stops scanning;
	 * case 1 is presumably the actual power-down step -- don't run it
	 * while a scan might still be draining). */
	rm31080_wait_for_scan_finish_blocking(rm);
	rm31080_cmd_process(rm, 0, rm->krl_tbl[KRL_INDEX_RM_SUSPEND]);
	rm31080_wait_for_scan_finish_blocking(rm);
	rm31080_cmd_process(rm, 1, rm->krl_tbl[KRL_INDEX_RM_SUSPEND]);

	rm31080_release_all_touches(rm);

	/* spi_locked=1 is the LAST thing rm_ctrl_suspend() does -- locked
	 * on the way down, re-locked on the way back up in resume below,
	 * and only released once userspace confirms reinit is done via
	 * RM_VARIABLE_SET_SPI_UNLOCK (gated by resume_cnt, see above). */
	rm->spi_locked = true;

	rm31080_send_signal(rm, RM_SIGNAL_SUSPEND);
	return 0;
}

static int __maybe_unused rm31080_resume(struct device *dev)
{
	struct rm31080_data *rm = dev_get_drvdata(dev);

	/* rm_ctrl_resume(): spi_locked=1 is the FIRST thing set, before any
	 * of the state resets or the resume table run below. */
	rm->spi_locked = true;
	rm->is_suspended = false;
	rm->resume_cnt++;

	/* rm_tch_init_ts_structure_part(), the parts that apply to our
	 * architecture (see the NOTE above the function block for what's
	 * excluded and why). */
	rm->init_finished = false;	/* b_init_finish = 0 */
	rm->calc_finished = false;	/* b_calc_finish = 0 */
	mutex_lock(&rm->scan_mode_lock);
	rm->scan_mode_state = RM_SCAN_ACTIVE_MODE;
	mutex_unlock(&rm->scan_mode_lock);
	rm->read_para = 0;		/* u16_read_para = 0 */
	rm->baseline_pending = false;	/* b_bl_updated = false -- any
					 * SET_BASELINE staged before suspend
					 * and never flushed via idle-mode
					 * entry is stale; HAL redoes its own
					 * init sequence post-resume anyway */
	rm31080_watchdog_reset(rm, 0);	/* rm_ctrl_watchdog_func(0) */

	/* downstream: wake_lock_active()+wake_unlock() followed by
	 * wake_lock_timeout() -- __pm_wakeup_event() is the direct modern
	 * replacement for that exact "start or refresh a bounded wake
	 * event" sequence in one call. Holds off suspend for
	 * TCH_WAKE_LOCK_TIMEOUT_MS to give the reinit sequence below, and
	 * whatever the HAL does after RM_SIGNAL_RESUME, room to complete;
	 * released early by RM_IOCTL_INIT_END or
	 * RM_VARIABLE_SET_WAKE_UNLOCK once that's confirmed done. */
	if (rm->wakelock_init)
		__pm_wakeup_event(rm->wakelock_init, TCH_WAKE_LOCK_TIMEOUT_MS);
	rm31080_cmd_process(rm, 0, rm->krl_tbl[KRL_INDEX_RM_RESUME]);
	enable_irq(rm->irq);
	rm31080_send_signal(rm, RM_SIGNAL_RESUME);
	return 0;
}

static SIMPLE_DEV_PM_OPS(rm31080_pm_ops, rm31080_suspend, rm31080_resume);

static const struct of_device_id rm31080_of_match[] = {
	{ .compatible = "raydium,rm31080" },
	{ }
};
MODULE_DEVICE_TABLE(of, rm31080_of_match);

static const struct spi_device_id rm31080_spi_id[] = {
	{ "rm31080", 0 },
	{ }
};
MODULE_DEVICE_TABLE(spi, rm31080_spi_id);

static struct spi_driver rm31080_spi_driver = {
	.driver = {
		.name = DRV_NAME,
		.of_match_table = rm31080_of_match,
		.pm = &rm31080_pm_ops,
	},
	.probe = rm31080_probe,
	.remove = rm31080_remove,
	.id_table = rm31080_spi_id,
};
module_spi_driver(rm31080_spi_driver);

MODULE_AUTHOR("JordanViknar <jordanviknar@gmail.com>");
MODULE_DESCRIPTION("Raydium RM31080 SPI touchscreen bridge (downstream-protocol-compatible)");
MODULE_LICENSE("GPL");
