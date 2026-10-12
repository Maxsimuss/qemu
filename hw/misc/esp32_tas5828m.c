/*
 * Virtual TI TAS5828M Class-D amplifier for ESP32 board tests.
 *
 * Control port: TAS5828M datasheet SLASEX7 (DIE_ID 0x95; note TI E2E reports
 * silicon reading 0x98, same as TAS5830).
 * DSP data path: TAS5827/TAS5828M/TAS5830 Process Flows SLUUDB4, Process
 * Flow 1 (48 kHz stereo): SDIN -> input mixer -> 12 BQs/channel -> volume.
 *
 * The device is an electrical peer. It never touches the ESP32 I2C or I2S
 * controllers: it listens to the *resolved* GPIO pad nodes (after GPIO matrix,
 * IO MUX, pulls and external drivers) and drives SDA only as open drain
 * through an external-driver slot of the GPIO model.
 *
 * I2C (7-bit address 0x61 by default): first byte
 * after the address phase is the register pointer with auto-increment
 * (sequential write/read). Register 0x00 selects the page in every book;
 * register 0x7F selects the book only on page 0. On nonzero DSP pages,
 * register 0x7F is data. Sequential transfers wrap within the page;
 * multi-page wrap per I2C_PAGE_AUTO_INC is not modeled.
 *
 * Book 0 page 0 (reset values): 0x01 RESET_CTRL (0x00) W auto-clear
 * only; 0x02 DEVICE_CTRL1 (0x00); 0x03 DEVICE_CTRL2 (0x10) DIS_DSP/MUTE/
 * STATE_CTL (00 Deep Sleep, 01 Sleep, 10 Hi-Z, 11 PLAY); 0x04 (0x00);
 * 0x05 (0x44); 0x0F (0x00); 0x28 SIG_CH_CTRL (0x00, 0 = auto);
 * 0x29 (0x00); 0x30 (0x00); 0x31 (0x00); 0x33 SAP_CTRL1 (0x02) I2S only,
 * 16/24/32-bit, shift 0; 0x34 (0x00); 0x35 SAP_CTRL3 (0x11);
 * 0x37 FS_MON / 0x38 BCK_MON / 0x39 CLKDET_STATUS (RO, measured);
 * 0x40 (0x01); 0x41/0x52/0x55-0x5D/0x6D-0x6F undocumented RW storage;
 * 0x46 (0x01); 0x4C DIG_VOL (0x30, 0 dB, 0.5 dB steps, 0xFF mute);
 * 0x4E (0x33); 0x4F (0x30); 0x50 (0x00); 0x51 (0x00); 0x53 (0x00);
 * 0x54 AGAIN (0x00, stored, not applied to WAV); 0x5E PVDD_ADC (RO 0);
 * 0x60-0x66 GPIO (0x00); 0x67 DIE_ID (RO 0x95); 0x68 POWER_STATE (RO,
 * mirrors STATE_CTL); 0x69 (RO 0); 0x6A-0x6C (0x00); 0x70/0x71/0x72/0x73
 * faults (RO 0 except latched clock fault in 0x71 bit 2); 0x74-0x77 (0x00);
 * 0x78 FAULT_CLEAR (W, 0x80 clears clock fault). Unlisted book-0 offsets
 * are reserved (write ignored, read 0, warning on every access).
 *
 * DSP books: 0x8C pages 0x06 (GangEQ 0x5C, BypassEQ 0x60, vol L 0x64 /
 * vol R 0x68 9.23 default 1.0), 0x09 (mixer ltl 0x74, rtl 0x78,
 * ltr 0x7C 9.23) + 0x0A (rtr 0x08 9.23); other 0x8C/0x78 locations are
 * stored RAM and read back verbatim (DRC/AGL/DPEQ/clipper/crossbar are
 * passthrough — explicit gaps). 0xAA pages 0x01-0x05 hold 12 BQs/channel,
 * 5 x 5.27 coefficients each (B0, B1, B2, A1, A2), default B0 = 1.0.
 * BQ layout: L BQ1-4 p1 0x30.., L BQ5-10 p2 0x08.., L BQ11-12 p3 0x08..,
 * R BQ1-4 p3 0x30.., R BQ5-10 p4 0x08.., R BQ11-12 p5 0x08...
 * Book 0x78 page 0x06 regs 0x54-0x5B report per-channel peak-hold level
 * meters (1.31, best effort — exact meter ballistics undocumented).
 *
 * DSP path per frame (float): I2S slots -> [-1,1) -> input mixer (9.23)
 * -> 12 x DF1 BQ per channel (Q5.27 coefficients apply directly:
 * y = B0*x0 + B1*x1 + B2*x2 + A1*y1 + A2*y2; skipped when BypassEQ,
 * right uses left bank when
 * GangEQ) -> channel volume (9.23) -> DIG_VOL -> SAP dpath -> hard clip ->
 * WAV. DPEQ/DRC/AGL/clipper/crossbar are passthrough; proprietary DSP
 * processing is outside this external-device model's scope.
 *
 * I2S (Philips slave, MSB first, BCLK rising edge, WS low = left): slot
 * width from SAP_CTRL1 (16/24/32). FS_MON/BCK_MON/CLKDET_STATUS derive
 * from measured WS/BCLK; a programmed SIG/SAP format disagreeing with
 * measurement latches the clock fault. Frames store to WAV only in PLAY
 * with DSP out of reset.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include <math.h>
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "qemu/error-report.h"
#include "qemu/bswap.h"
#include "qapi/error.h"
#include "hw/qdev-properties.h"
#include "hw/gpio/esp32_gpio.h"
#include "hw/irq.h"
#include "migration/vmstate.h"
#include "sysemu/sysemu.h"

#define TYPE_ESP32_TAS5828M "esp32-tas5828m"
OBJECT_DECLARE_SIMPLE_TYPE(Esp32Tas5828m, ESP32_TAS5828M)

#define R_RESET_CTRL      0x01
#define R_DEVICE_CTRL1    0x02
#define R_DEVICE_CTRL2    0x03
#define R_SIG_CH_CTRL     0x28
#define R_SAP_CTRL1       0x33
#define R_SAP_CTRL2       0x34
#define R_SAP_CTRL3       0x35
#define R_FS_MON          0x37
#define R_BCLK_MON        0x38
#define R_CLKDET_STATUS   0x39
#define R_DIG_VOL         0x4C
#define R_DIE_ID          0x67
#define R_POWER_STATE     0x68
#define R_GLOBAL_FAULT1   0x71
#define R_FAULT_CLEAR     0x78
#define R_SET_PAGE        0x00
#define R_SET_BOOK        0x7F

#define BOOK_CTRL 0x00
#define BOOK_LVL  0x78
#define BOOK_DSP  0x8C
#define BOOK_BQ   0xAA

#define STATE_DEEP_SLEEP 0x00
#define STATE_SLEEP      0x01
#define STATE_HIZ        0x02
#define STATE_PLAY       0x03

#define GF1_CLOCK_FAULT BIT(2)
#define CS_BCLK_VALID    BIT(1)
#define CS_FS_VALID      BIT(0)
#define CS_BCLK_MISSING  BIT(2)
#define CS_PLL_LOCKED    BIT(3)

#define WAV_HEADER_SIZE 44
#define STAGE_SIZE 65536
#define FLUSH_MS 250
#define GAP_BIT_PERIODS 16
#define NBQ 12

enum I2cPhase {
    I2C_IDLE, I2C_ADDRESS, I2C_WRITE, I2C_READ,
    I2C_ACK_SETUP, I2C_ACK_CLOCK, I2C_ACK_END,
    I2C_READ_ACK_SETUP, I2C_READ_ACK, I2C_READ_ACK_END, I2C_IGNORE,
};

enum PadIndex {
    PIN_SDA, PIN_SCL, PIN_BCLK, PIN_WS, PIN_DATA, PIN_COUNT,
};

enum PdnState { PDN_TIED_HIGH, PDN_LOW, PDN_HIGH, PDN_INDETERMINATE };


/* (page, base) of the 12 BQs per channel in book 0xAA. */
static const uint8_t bq_map[2][NBQ][2] = {
    {{1, 0x30}, {1, 0x44}, {1, 0x58}, {1, 0x6C},
     {2, 0x08}, {2, 0x1C}, {2, 0x30}, {2, 0x44},
     {2, 0x58}, {2, 0x6C}, {3, 0x08}, {3, 0x1C}},
    {{3, 0x30}, {3, 0x44}, {3, 0x58}, {3, 0x6C},
     {4, 0x08}, {4, 0x1C}, {4, 0x30}, {4, 0x44},
     {4, 0x58}, {4, 0x6C}, {5, 0x08}, {5, 0x1C}},
};

struct Esp32Tas5828m {
    DeviceState parent;

    Esp32GpioState *gpio;
    char *wav;
    uint32_t pad[PIN_COUNT];
    uint32_t pdn_pad;
    enum PdnState pdn_state;
    qemu_irq pdn_sink;
    uint32_t driver_slot;
    uint32_t address;
    uint64_t max_bytes;
    bool pullups;

    uint8_t ctrl[256];
    uint8_t cur_book, cur_page;
    bool clock_fault;
    /* DSP RAM: book 0x8C pages 0x00-0x0C, book 0xAA pages 0x00-0x07,
     * book 0x78 page 0x06 (level meters live here). */
    uint8_t ram8c[13][256];
    uint8_t ramaa[8][256];
    uint8_t ram78[256];
    char *coefficient_safety;
    uint8_t bq_write_page, bq_write_base, bq_write_start;
    unsigned bq_write_count;
    bool bq_write_active;
    /* DF1 state per channel per BQ: x1, x2, y1, y2. */
    double bq_state[2][NBQ][4];
    uint32_t peak[2];

    uint8_t phase, next, bits, shift, pointer;
    bool ack, expect_pointer, first_data;

    uint8_t level[PIN_COUNT];
    qemu_irq sink[PIN_COUNT];

    uint64_t bclk_rises;
    int64_t last_bclk_ns;
    bool bclk_seen;
    uint32_t stream_edges;
    bool synced, provisional, ws_prev, left_valid, frame_bad, data_bad;
    bool snap_valid, interval_ok, pending_valid;
    bool right_slot;
    unsigned slot_bits;
    uint32_t slot_shift;
    uint32_t left_value, pending_left, pending_right;
    int64_t snap_ns;
    uint64_t snap_bclk;
    uint32_t meas_rate;
    uint32_t meas_bclk_ratio;
    bool meas_valid;

    int fd;
    bool header_failed;
    uint64_t data_bytes;
    uint32_t wav_rate;
    uint8_t *stage;
    uint32_t stage_len;
    QEMUTimer *flush_timer;
    Notifier exit_notifier;
    uint32_t frames;
};

static inline bool level_binary(uint8_t level)
{
    return level == ESP32_PAD_LOW || level == ESP32_PAD_HIGH;
}

static unsigned tas_depth(Esp32Tas5828m *s)
{
    switch (s->ctrl[R_SAP_CTRL1] & 3) {
    case 0: return 16;
    case 2: return 24;
    case 3: return 32;
    default: return 0;
    }
}

static uint32_t tas_rate_hz(Esp32Tas5828m *s)
{
    return s->wav_rate ? s->wav_rate : 48000;
}

static int32_t be32(const uint8_t *p)
{
    uint32_t v = (uint32_t)p[0] << 24 | (uint32_t)p[1] << 16 |
                 (uint32_t)p[2] << 8 | p[3];
    return (int32_t)v;
}

static double f5_27(int32_t v) { return v / 134217728.0; }
static double f9_23(int32_t v) { return v / 8388608.0; }


/* ------------------------------------------------------------- WAV file */

static void wav_header(Esp32Tas5828m *s, uint8_t h[WAV_HEADER_SIZE], bool final)
{
    uint32_t rate = tas_rate_hz(s);
    unsigned depth = tas_depth(s);
    if (!depth) {
        depth = 32;
    }
    uint32_t bytes = depth / 8;
    uint32_t align = bytes * 2;
    uint64_t data = MIN(s->data_bytes, UINT32_MAX - 64);
    uint64_t riff = 36 + data + ((final && (data & 1)) ? 1 : 0);

    memcpy(h, "RIFF", 4);
    stl_le_p(h + 4, riff);
    memcpy(h + 8, "WAVEfmt ", 8);
    stl_le_p(h + 16, 16);
    stw_le_p(h + 20, 1);
    stw_le_p(h + 22, 2);
    stl_le_p(h + 24, rate);
    stl_le_p(h + 28, rate * align);
    stw_le_p(h + 32, align);
    stw_le_p(h + 34, depth);
    memcpy(h + 36, "data", 4);
    stl_le_p(h + 40, data);
}

static bool wav_write_header(Esp32Tas5828m *s, bool final)
{
    uint8_t h[WAV_HEADER_SIZE];

    wav_header(s, h, final);
    if (lseek(s->fd, 0, SEEK_SET) < 0 ||
        qemu_write_full(s->fd, h, sizeof(h)) != sizeof(h) ||
        lseek(s->fd, 0, SEEK_END) < 0) {
        return false;
    }
    return true;
}

static void wav_fail(Esp32Tas5828m *s, const char *what)
{
    if (!s->header_failed) {
        error_report("esp32-tas5828m: %s '%s': %s", what, s->wav,
                     strerror(errno));
    }
    s->header_failed = true;
}

static void wav_flush(Esp32Tas5828m *s)
{
    if (s->fd < 0) {
        return;
    }
    if (s->stage_len &&
        qemu_write_full(s->fd, s->stage, s->stage_len) != s->stage_len) {
        wav_fail(s, "cannot write");
    }
    s->stage_len = 0;
    if (!wav_write_header(s, false)) {
        wav_fail(s, "cannot update header of");
    }
    timer_del(s->flush_timer);
}

static void wav_close(Esp32Tas5828m *s)
{
    if (s->fd < 0) {
        return;
    }
    wav_flush(s);
    if (s->data_bytes & 1) {
        static const uint8_t pad;
        if (qemu_write_full(s->fd, &pad, 1) != 1) {
            wav_fail(s, "cannot pad");
        }
    }
    if (!wav_write_header(s, true)) {
        wav_fail(s, "cannot finalize header of");
    }
    if (close(s->fd) < 0) {
        wav_fail(s, "cannot close");
    }
    s->fd = -1;
}

static void wav_open(Esp32Tas5828m *s)
{
    s->data_bytes = 0;
    s->frames = 0;
    s->stage_len = 0;
    s->header_failed = false;
    if (!s->wav) {
        return;
    }
    s->fd = qemu_open_old(s->wav, O_WRONLY | O_CREAT | O_TRUNC | O_BINARY,
                          0644);
    if (s->fd < 0) {
        wav_fail(s, "cannot open");
        return;
    }
    if (!wav_write_header(s, false)) {
        wav_fail(s, "cannot write header of");
        close(s->fd);
        s->fd = -1;
    }
}

static void flush_timer_cb(void *opaque)
{
    wav_flush(opaque);
}

static void exit_notify(Notifier *n, void *data)
{
    Esp32Tas5828m *s = container_of(n, Esp32Tas5828m, exit_notifier);

    wav_close(s);
}

static void store_sample(Esp32Tas5828m *s, uint32_t value, unsigned bytes)
{
    for (unsigned i = 0; i < bytes; i++) {
        s->stage[s->stage_len++] = value >> (8 * i);
    }
}

static bool store_frame(Esp32Tas5828m *s, uint32_t left, uint32_t right)
{
    unsigned depth = tas_depth(s);
    unsigned bytes, size;

    if (!depth) {
        return true;
    }
    bytes = depth / 8;
    size = bytes * 2;
    if (s->data_bytes + size + WAV_HEADER_SIZE > s->max_bytes) {
        wav_close(s);
        return false;
    }
    s->frames++;
    s->data_bytes += size;
    if (s->fd < 0) {
        return true;
    }
    store_sample(s, left, bytes);
    store_sample(s, right, bytes);
    if (s->stage_len + 8 > STAGE_SIZE) {
        wav_flush(s);
    } else if (!timer_pending(s->flush_timer)) {
        timer_mod(s->flush_timer,
                  qemu_clock_get_ms(QEMU_CLOCK_REALTIME) + FLUSH_MS);
    }
    return true;
}

/* ---------------------------------------------------------- DSP path */

static double bq_coef(Esp32Tas5828m *s, unsigned ch, unsigned bq, unsigned k)
{
    unsigned bank = (s->ram8c[6][0x5C] != 0) ? 0 : ch;
    uint8_t page = bq_map[bank][bq][0];
    uint8_t base = bq_map[bank][bq][1];
    return f5_27(be32(&s->ramaa[page][base + 4 * k]));
}

static double run_bq(Esp32Tas5828m *s, unsigned ch, unsigned bq, double x)
{
    double *st = s->bq_state[ch][bq];
    double y = bq_coef(s, ch, bq, 0) * x +
               bq_coef(s, ch, bq, 1) * st[0] +
               bq_coef(s, ch, bq, 2) * st[1] +
               bq_coef(s, ch, bq, 3) * st[2] +
               bq_coef(s, ch, bq, 4) * st[3];
    st[1] = st[0];
    st[0] = x;
    st[3] = st[2];
    st[2] = y;
    return y;
}

/* Full DSP path for one stereo frame. Returns 0 when muted/dpath-zeroed. */
static void dsp_process(Esp32Tas5828m *s, double inL, double inR,
                        double *outL, double *outR)
{
    double ltl = f9_23(be32(&s->ram8c[9][0x74]));
    double rtl = f9_23(be32(&s->ram8c[9][0x78]));
    double ltr = f9_23(be32(&s->ram8c[9][0x7C]));
    double rtr = f9_23(be32(&s->ram8c[10][0x08]));
    double l = inL * ltl + inR * rtl;
    double r = inL * ltr + inR * rtr;

    if (s->ram8c[6][0x60] == 0) {
        for (unsigned b = 0; b < NBQ; b++) {
            l = run_bq(s, 0, b, l);
        }
        for (unsigned b = 0; b < NBQ; b++) {
            r = run_bq(s, 1, b, r);
        }
    }
    l *= f9_23(be32(&s->ram8c[6][0x64]));
    r *= f9_23(be32(&s->ram8c[6][0x68]));
    uint8_t dv = s->ctrl[R_DIG_VOL];
    if (s->ctrl[R_DEVICE_CTRL2] & BIT(3)) {
        dv = 0xFF;
    }
    double dg = (dv == 0xFF) ? 0.0 : pow(10.0, (0x30 - (int)dv) * 0.5 / 20.0);
    l *= dg;
    r *= dg;
    uint8_t dpath = s->ctrl[R_SAP_CTRL3];
    double sl = l, sr = r;
    switch ((dpath >> 4) & 3) {
    case 0: sl = 0.0; break;
    case 1: sl = l; break;
    case 2: sl = r; break;
    default: break;
    }
    switch (dpath & 3) {
    case 0: sr = 0.0; break;
    case 1: sr = r; break;
    case 2: sr = l; break;
    default: break;
    }
    /* Output clipper (passthrough model with hard clip). */
    *outL = CLAMP(sl, -1.0, 1.0 - 1.0 / 2147483648.0);
    *outR = CLAMP(sr, -1.0, 1.0 - 1.0 / 2147483648.0);
    /* Peak-hold level meters (1.31, best effort). */
    uint32_t pl = *outL < 0 ? (uint32_t)(-*outL * 2147483648.0) :
                              (uint32_t)(*outL * 2147483648.0);
    uint32_t pr = *outR < 0 ? (uint32_t)(-*outR * 2147483648.0) :
                              (uint32_t)(*outR * 2147483648.0);
    if (pl > s->peak[0]) {
        s->peak[0] = pl;
    }
    if (pr > s->peak[1]) {
        s->peak[1] = pr;
    }
}

/* -------------------------------------------------------------- decoding */

static void lose_sync(Esp32Tas5828m *s, bool noisy, const char *why)
{
    if (noisy) {
        qemu_log_mask(LOG_GUEST_ERROR, "esp32-tas5828m: %s\n", why);
    }
    s->synced = false;
    s->provisional = false;
    s->snap_valid = false;
    s->interval_ok = false;
    s->pending_valid = false;
    s->left_valid = false;
    s->frame_bad = false;
    s->data_bad = false;
    s->slot_bits = 0;
    s->slot_shift = 0;
}

static uint32_t rate_code(uint32_t hz)
{
    switch (hz) {
    case 8000: return 0x02;
    case 16000: return 0x04;
    case 32000: return 0x06;
    case 44100: return 0x08;
    case 48000: return 0x09;
    case 88200: return 0x0A;
    case 96000: return 0x0B;
    case 176400: return 0x0C;
    case 192000: return 0x0D;
    default: return 0x00;
    }
}

static void update_measurements(Esp32Tas5828m *s, uint32_t rate_hz,
                                uint32_t bclk_ratio)
{
    s->meas_rate = rate_hz;
    s->meas_bclk_ratio = bclk_ratio;
    s->meas_valid = rate_hz && bclk_ratio >= 32 && bclk_ratio <= 512;
    if (s->meas_valid) {
        s->wav_rate = rate_hz;
    }
    bool mismatch = false;
    uint8_t sig = s->ctrl[R_SIG_CH_CTRL];
    uint8_t bclk_cfg = (sig >> 4) & 0xF;
    uint8_t fs_cfg = sig & 0xF;
    if (bclk_cfg) {
        uint32_t expect = 0;
        switch (bclk_cfg) {
        case 0x3: expect = 32; break;
        case 0x5: expect = 64; break;
        case 0x7: expect = 128; break;
        case 0x9: expect = 256; break;
        case 0xB: expect = 512; break;
        default: expect = 0xFFFFFFFF; break;
        }
        if (expect != bclk_ratio) {
            mismatch = true;
        }
    }
    if (fs_cfg) {
        /* TAS5828M manual codes (0100/0010 reserved, unlike TAS5830). */
        uint32_t expect_hz = 0xFFFFFFFF;
        switch (fs_cfg) {
        case 0x6: expect_hz = 32000; break;
        case 0x8: expect_hz = 44100; break;
        case 0x9: expect_hz = 48000; break;
        case 0xA: expect_hz = 88200; break;
        case 0xB: expect_hz = 96000; break;
        case 0xC: expect_hz = 176400; break;
        case 0xD: expect_hz = 192000; break;
        default: break;
        }
        if (expect_hz != rate_hz) {
            mismatch = true;
        }
    }
    if (((s->ctrl[R_SAP_CTRL1] >> 4) & 3) != 0 || tas_depth(s) == 0 ||
        s->ctrl[R_SAP_CTRL2] != 0) {
        mismatch = true;
    }
    if (s->meas_valid && tas_depth(s) && bclk_ratio != 2 * tas_depth(s)) {
        mismatch = true;
    }
    if (mismatch && s->meas_valid) {
        if (!s->clock_fault) {
            qemu_log_mask(LOG_GUEST_ERROR,
                          "esp32-tas5828m: clock/format mismatch "
                          "rate=%u ratio=%u\n", rate_hz, bclk_ratio);
        }
        s->clock_fault = true;
    }
}

static void ws_fell(Esp32Tas5828m *s, int64_t now)
{
    s->interval_ok = false;
    if (s->snap_valid) {
        uint64_t dt = now - s->snap_ns;
        uint64_t dbclk = s->bclk_rises - s->snap_bclk;
        uint32_t rate_hz = 0;
        if (dt > 0) {
            static const uint32_t rates[] = {8000, 16000, 32000, 44100,
                                             48000, 88200, 96000, 176400,
                                             192000};
            for (unsigned i = 0; i < ARRAY_SIZE(rates); i++) {
                uint64_t expect = 1000000000ull / rates[i];
                uint64_t diff = dt > expect ? dt - expect : expect - dt;
                if (diff * 100 <= expect) {
                    rate_hz = rates[i];
                    break;
                }
            }
        }
        if (rate_hz && dbclk >= 32 && dbclk <= 512) {
            update_measurements(s, rate_hz, dbclk);
            s->interval_ok = !s->clock_fault;
        } else {
            update_measurements(s, 0, dbclk);
        }
    }
    s->snap_valid = true;
    s->snap_ns = now;
    s->snap_bclk = s->bclk_rises;
}

static bool in_play(Esp32Tas5828m *s)
{
    return (s->pdn_state == PDN_TIED_HIGH || s->pdn_state == PDN_HIGH) &&
           (s->ctrl[R_DEVICE_CTRL2] & 3) == STATE_PLAY &&
           !(s->ctrl[R_DEVICE_CTRL2] & BIT(4));
}

static double slot_to_float(uint32_t slot, unsigned depth)
{
    int32_t v = slot << (32 - depth);
    return v / 2147483648.0;
}

static uint32_t float_to_slot(double x, unsigned depth)
{
    double c = CLAMP(x, -1.0, 1.0 - 1.0 / 2147483648.0);
    int64_t q = llround(c * 2147483648.0);
    uint32_t u = (uint32_t)(q >> (32 - depth));
    if (depth < 32) {
        u &= (depth == 32) ? 0xFFFFFFFFu : ((1u << depth) - 1);
    }
    return u;
}

static void frame_complete(Esp32Tas5828m *s, uint32_t right)
{
    bool good = s->left_valid && !s->frame_bad && !s->data_bad;
    uint32_t left = s->left_value;

    s->left_valid = false;
    s->frame_bad = false;
    s->data_bad = false;
    if (!good) {
        s->pending_valid = false;
        return;
    }
    if (s->provisional) {
        s->provisional = false;
        s->pending_valid = true;
        s->pending_left = left;
        s->pending_right = right;
        return;
    }
    if (!s->interval_ok || !in_play(s)) {
        s->pending_valid = false;
        return;
    }
    unsigned depth = tas_depth(s);
    double dl, dr, pl, pr;
    if (s->pending_valid) {
        s->pending_valid = false;
        dl = slot_to_float(s->pending_left, depth);
        dr = slot_to_float(s->pending_right, depth);
        dsp_process(s, dl, dr, &pl, &pr);
        if (!store_frame(s, float_to_slot(pl, depth),
                         float_to_slot(pr, depth))) {
            return;
        }
    }
    dl = slot_to_float(left, depth);
    dr = slot_to_float(right, depth);
    dsp_process(s, dl, dr, &pl, &pr);
    store_frame(s, float_to_slot(pl, depth), float_to_slot(pr, depth));
}

static void slot_complete(Esp32Tas5828m *s)
{
    unsigned depth = tas_depth(s) ? tas_depth(s) : 32;
    if (s->slot_bits != depth) {
        s->frame_bad = true;
    }
    if (!s->right_slot) {
        s->left_value = s->slot_shift;
        s->left_valid = true;
    } else {
        frame_complete(s, s->slot_shift);
    }
}

static void bclk_rise(Esp32Tas5828m *s, int64_t now)
{
    unsigned depth = tas_depth(s) ? tas_depth(s) : 32;
    uint64_t nominal = 1000000000ull / (48000ull * 2 * depth);
    bool ws_high, bit, change;

    if (s->bclk_seen && (uint64_t)(now - s->last_bclk_ns) >
                        GAP_BIT_PERIODS * nominal) {
        lose_sync(s, false, NULL);
        s->stream_edges = 0;
    }
    s->bclk_seen = true;
    s->last_bclk_ns = now;
    s->stream_edges++;
    if (!level_binary(s->level[PIN_WS])) {
        lose_sync(s, false, NULL);
        return;
    }
    ws_high = s->level[PIN_WS] == ESP32_PAD_HIGH;
    change = ws_high != s->ws_prev;
    s->ws_prev = ws_high;
    bit = s->level[PIN_DATA] == ESP32_PAD_HIGH;

    if (!s->synced) {
        if (s->stream_edges == 1 && !ws_high) {
            s->synced = true;
            s->provisional = true;
            s->right_slot = false;
            s->slot_bits = 0;
            s->slot_shift = 0;
        } else if (change && !ws_high) {
            s->synced = true;
            s->provisional = false;
            s->right_slot = false;
            s->slot_bits = 0;
            s->slot_shift = 0;
            return;
        } else {
            return;
        }
        change = false;
    }
    if (!level_binary(s->level[PIN_DATA])) {
        s->data_bad = true;
    }

    if (change) {
        if (s->slot_bits < 32) {
            s->slot_shift = (s->slot_shift << 1) | bit;
            s->slot_bits++;
        }
        slot_complete(s);
        s->right_slot = ws_high;
        s->slot_bits = 0;
        s->slot_shift = 0;
        return;
    }
    if (s->slot_bits < depth) {
        s->slot_shift = (s->slot_shift << 1) | bit;
        s->slot_bits++;
    } else {
        s->frame_bad = true;
    }
}

static uint64_t serial_half_step(uint64_t *phase, uint64_t numerator,
                                 uint64_t denominator)
{
    if (numerator <= UINT64_MAX - *phase) {
        uint64_t accumulated = *phase + numerator;
        uint64_t ns = accumulated / denominator;
        *phase = accumulated % denominator;
        return ns;
    }
    __uint128_t accumulated = (__uint128_t)*phase + numerator;
    uint64_t ns = accumulated / denominator;
    *phase = accumulated % denominator;
    return ns;
}

static void serial_frame(void *opaque, uint64_t data_bits, uint64_t ws_bits,
                         unsigned bit_count, int64_t first_rising_ns,
                         uint64_t half_num, uint64_t half_den,
                         uint64_t remainder, bool bclk_inverted)
{
    Esp32Tas5828m *s = opaque;
    uint64_t phase = remainder;
    int64_t now = first_rising_ns;
    uint64_t before_phase = phase;
    int64_t ws_falling = now - serial_half_step(&before_phase, half_num,
                                                 half_den);

    if (s->pdn_state != PDN_TIED_HIGH && s->pdn_state != PDN_HIGH) {
        return;
    }

    if (bclk_inverted) {
        now += serial_half_step(&phase, half_num, half_den);
    }
    for (unsigned i = 0; i < bit_count; i++) {
        bool ws = (ws_bits >> i) & 1;
        bool data = (data_bits >> i) & 1;
        bool old_ws = s->level[PIN_WS] == ESP32_PAD_HIGH;
        s->level[PIN_WS] = ws ? ESP32_PAD_HIGH : ESP32_PAD_LOW;
        s->level[PIN_DATA] = data ? ESP32_PAD_HIGH : ESP32_PAD_LOW;
        if (old_ws && !ws) {
            ws_fell(s, bclk_inverted ? now : ws_falling);
        }
        s->level[PIN_BCLK] = ESP32_PAD_HIGH;
        s->bclk_rises++;
        bclk_rise(s, now);
        uint64_t first_ns = serial_half_step(&phase, half_num, half_den);
        uint64_t second_ns = serial_half_step(&phase, half_num, half_den);
        ws_falling = now + first_ns;
        now += first_ns + second_ns;
    }
    s->level[PIN_BCLK] = bclk_inverted ? ESP32_PAD_HIGH : ESP32_PAD_LOW;
}

static void pad_event(void *opaque, int n, int value)
{
    Esp32Tas5828m *s = opaque;
    uint8_t previous = s->level[n];
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);

    s->level[n] = value;
    if (s->pdn_state != PDN_TIED_HIGH && s->pdn_state != PDN_HIGH) {
        return;
    }
    switch (n) {
    case PIN_BCLK:
        if (!level_binary(value)) {
            s->stream_edges = 0;
            s->bclk_seen = false;
            if (in_play(s)) {
                lose_sync(s, false, NULL);
            }
        } else if (previous == ESP32_PAD_LOW && value == ESP32_PAD_HIGH) {
            s->bclk_rises++;
            bclk_rise(s, now);
        }
        break;
    case PIN_WS:
        if (!level_binary(value)) {
            lose_sync(s, false, NULL);
        } else if (previous == ESP32_PAD_HIGH && value == ESP32_PAD_LOW) {
            ws_fell(s, now);
        }
        break;
    default:
        break;
    }
}

/* ----------------------------------------------------------- registers */

static bool ctrl_is_ro(uint8_t reg)
{
    switch (reg) {
    case R_FS_MON: case R_BCLK_MON: case R_CLKDET_STATUS:
    case 0x5E: case R_DIE_ID: case R_POWER_STATE: case 0x69:
    case 0x70: case R_GLOBAL_FAULT1: case 0x72: case 0x73:
        return true;
    default:
        return false;
    }
}

static bool ctrl_is_rsv(uint8_t reg)
{
    switch (reg) {
    case 0x01: case 0x02: case 0x03: case 0x04: case 0x05: case 0x0F:
    case 0x28: case 0x29: case 0x30: case 0x31: case 0x33: case 0x34:
    case 0x35: case 0x37: case 0x38: case 0x39: case 0x40: case 0x41:
    case 0x46: case 0x4C: case 0x4E: case 0x4F: case 0x50: case 0x51:
    case 0x52: case 0x53: case 0x54: case 0x55: case 0x56: case 0x57:
    case 0x58: case 0x59: case 0x5A: case 0x5B: case 0x5C: case 0x5D:
    case 0x5E: case 0x60: case 0x61: case 0x62: case 0x63: case 0x64:
    case 0x65: case 0x66: case 0x67: case 0x68: case 0x69: case 0x6A:
    case 0x6B: case 0x6C: case 0x6D: case 0x6E: case 0x6F: case 0x70:
    case 0x71: case 0x72: case 0x73: case 0x74: case 0x75: case 0x76:
    case 0x77: case 0x78:
        return false;
    default:
        return true;
    }
}

static void tas_reset_regs(Esp32Tas5828m *s)
{
    memset(s->ctrl, 0, sizeof(s->ctrl));
    s->ctrl[0x05] = 0x44;
    s->ctrl[R_DEVICE_CTRL2] = 0x10;
    s->ctrl[R_SAP_CTRL1] = 0x02;
    s->ctrl[R_SAP_CTRL3] = 0x11;
    s->ctrl[0x40] = 0x01;
    s->ctrl[0x46] = 0x01;
    s->ctrl[R_DIG_VOL] = 0x30;
    s->ctrl[0x4E] = 0x33;
    s->ctrl[0x4F] = 0x30;
    s->clock_fault = false;
    s->cur_book = 0x00;
    s->cur_page = 0x00;
}

static void dsp_init_defaults(Esp32Tas5828m *s)
{
    memset(s->ram8c, 0, sizeof(s->ram8c));
    memset(s->ramaa, 0, sizeof(s->ramaa));
    memset(s->ram78, 0, sizeof(s->ram78));
    /* Book 0x8C defaults (Table 4-2): vol 1.0, mixer identity. */
    s->ram8c[6][0x64] = 0x00; s->ram8c[6][0x65] = 0x80;
    s->ram8c[6][0x66] = 0x00; s->ram8c[6][0x67] = 0x00;
    s->ram8c[6][0x68] = 0x00; s->ram8c[6][0x69] = 0x80;
    s->ram8c[6][0x6A] = 0x00; s->ram8c[6][0x6B] = 0x00;
    s->ram8c[6][0x6C] = 0x00; s->ram8c[6][0x6D] = 0xE2;
    s->ram8c[6][0x6E] = 0xC4; s->ram8c[6][0x6F] = 0x6B;
    s->ram8c[9][0x74] = 0x00; s->ram8c[9][0x75] = 0x80;
    s->ram8c[9][0x76] = 0x00; s->ram8c[9][0x77] = 0x00;
    s->ram8c[10][0x08] = 0x00; s->ram8c[10][0x09] = 0x80;
    s->ram8c[10][0x0A] = 0x00; s->ram8c[10][0x0B] = 0x00;
    /* Book 0xAA: every BQ identity (B0 = 1.0). */
    for (unsigned ch = 0; ch < 2; ch++) {
        for (unsigned b = 0; b < NBQ; b++) {
            uint8_t page = bq_map[ch][b][0];
            uint8_t base = bq_map[ch][b][1];
            s->ramaa[page][base] = 0x08;
        }
    }
    memset(s->bq_state, 0, sizeof(s->bq_state));
    s->peak[0] = s->peak[1] = 0;
}

/* Stored Q5.27 coefficients are applied directly in the recurrence. Jury's
 * strict second-order conditions for z^2 - A1*z - A2 are evaluated in integer
 * coefficient units after quantization. Equality is marginal; strict
 * inequality determines stability.
 */
static const char *bq_safety(Esp32Tas5828m *s, uint8_t page, uint8_t base)
{
    const int64_t scale = 1ll << 27;
    int64_t a1 = be32(&s->ramaa[page][base + 12]);
    int64_t a2 = be32(&s->ramaa[page][base + 16]);
    int64_t c1 = scale - a1 - a2;
    int64_t c2 = scale + a1 - a2;
    int64_t c3 = scale + a2;

    if (c1 < 0 || c2 < 0 || c3 < 0) {
        return "unstable";
    }
    if (c1 == 0 || c2 == 0 || c3 == 0) {
        return "marginal";
    }
    return "stable";
}

static bool bq_location(uint8_t page, uint8_t reg, uint8_t *base)
{
    for (unsigned ch = 0; ch < ARRAY_SIZE(bq_map); ch++) {
        for (unsigned i = 0; i < NBQ; i++) {
            uint8_t start = bq_map[ch][i][1];

            if (bq_map[ch][i][0] == page && reg >= start &&
                reg < start + 20) {
                *base = start;
                return true;
            }
        }
    }
    return false;
}

static void bq_write_byte(Esp32Tas5828m *s, uint8_t reg)
{
    if (!s->bq_write_active) {
        if (s->cur_book == BOOK_BQ &&
            bq_location(s->cur_page, reg, &s->bq_write_base)) {
            s->bq_write_page = s->cur_page;
            s->bq_write_start = reg;
            s->bq_write_count = 0;
            s->bq_write_active = true;
        } else {
            return;
        }
    }
    if (s->cur_book != BOOK_BQ || s->cur_page != s->bq_write_page ||
        reg != (uint8_t)(s->bq_write_start + s->bq_write_count) ||
        s->bq_write_count >= 20) {
        s->bq_write_count = 21;
        return;
    }
    s->bq_write_count++;
}

static void bq_write_end(Esp32Tas5828m *s)
{
    if (!s->bq_write_active) {
        return;
    }
    if (s->bq_write_count != 20 ||
        s->bq_write_start != s->bq_write_base) {
        g_free(s->coefficient_safety);
        s->coefficient_safety = g_strdup("incomplete");
    } else {
        const char *result = bq_safety(s, s->bq_write_page,
                                      s->bq_write_base);
        g_free(s->coefficient_safety);
        s->coefficient_safety = g_strdup(result);
        if (strcmp(result, "stable")) {
            qemu_log_mask(LOG_GUEST_ERROR,
                          "esp32-tas5828m: coefficient safety: %s "
                          "quantized BQ at %02x/%02x\n",
                          result, s->bq_write_page, s->bq_write_base);
        }
    }
    s->bq_write_active = false;
    s->bq_write_count = 0;
}

static uint8_t *dsp_ptr(Esp32Tas5828m *s, uint8_t book, uint8_t page,
                        uint8_t reg)
{
    if (book == BOOK_DSP && page < 13) {
        return &s->ram8c[page][reg];
    }
    if (book == BOOK_BQ && page < 8) {
        return &s->ramaa[page][reg];
    }
    if (book == BOOK_LVL && page == 0x06) {
        return &s->ram78[reg];
    }
    return NULL;
}

static void reg_write(Esp32Tas5828m *s, uint8_t reg, uint8_t value)
{
    /*
     * Book/page selection vs DSP data: a write whose register pointer is
     * 0x00 selects the page; one whose pointer is 0x7F selects the book.
     * Sequential data bytes that merely pass through those offsets (e.g.
     * a 20-byte BQ write ending at 0x7F) are data. first_data marks the
     * first data byte after the pointer byte.
     */
    bool pointed = s->first_data;
    s->first_data = false;
    if (pointed && reg == R_SET_PAGE) {
        s->cur_page = value;
        return;
    }
    if (pointed && reg == R_SET_BOOK && s->cur_page == 0) {
        s->cur_book = value;
        s->cur_page = 0x00;
        return;
    }
    if (s->cur_book == BOOK_CTRL && s->cur_page == 0x00) {
        if (ctrl_is_rsv(reg)) {
            warn_report("esp32-tas5828m I2C 0x%02x: write to unknown "
                        "register %02x/%02x/%02x = 0x%02x (reserved)",
                        s->address, s->cur_book, s->cur_page, reg, value);
            return;
        }
        if (ctrl_is_ro(reg)) {
            qemu_log_mask(LOG_GUEST_ERROR,
                          "esp32-tas5828m: write to read-only 0x%02x\n", reg);
            return;
        }
        switch (reg) {
        case R_RESET_CTRL: {
            if (value & BIT(4)) {
                /* RST_DIG_CORE clears both the control registers and DSP RAM. */
                tas_reset_regs(s);
                dsp_init_defaults(s);
                memset(s->bq_state, 0, sizeof(s->bq_state));
                memset(s->peak, 0, sizeof(s->peak));
                g_free(s->coefficient_safety);
                s->coefficient_safety = g_strdup("not-checked");
                lose_sync(s, false, NULL);
            } else if (value & BIT(0)) {
                /* RST_REG leaves DSP coefficient/data RAM untouched. */
                tas_reset_regs(s);
            }
            break;
        }
        case R_FAULT_CLEAR:
            if (value & 0x80) {
                s->clock_fault = false;
            }
            break;
        case R_DEVICE_CTRL2: {
            bool was_play = (s->ctrl[R_DEVICE_CTRL2] & 3) == STATE_PLAY;
            bool will_play = (value & 3) == STATE_PLAY;
            s->ctrl[reg] = value;
            if (was_play && !will_play) {
                wav_flush(s);
            } else if (!was_play && will_play && s->data_bytes == 0 &&
                       s->fd >= 0) {
                wav_write_header(s, false);
            }
            break;
        }
        case R_SAP_CTRL1:
        case R_SIG_CH_CTRL:
            s->ctrl[reg] = value;
            if (s->meas_valid) {
                update_measurements(s, s->meas_rate, s->meas_bclk_ratio);
            }
            if (s->data_bytes == 0 && s->fd >= 0) {
                wav_write_header(s, false);
            }
            break;
        default:
            s->ctrl[reg] = value;
            break;
        }
        return;
    }
    /* DSP book/registers: plain RAM with readback. */
    uint8_t *p = dsp_ptr(s, s->cur_book, s->cur_page, reg);
    if (!p) {
        warn_report("esp32-tas5828m I2C 0x%02x: write to unknown "
                    "register %02x/%02x/%02x = 0x%02x (unmapped)",
                    s->address, s->cur_book, s->cur_page, reg, value);
        return;
    }
    *p = value;
}

static uint8_t reg_read(Esp32Tas5828m *s, uint8_t reg)
{
    if (reg == R_SET_PAGE) {
        return s->cur_page;
    }
    if (reg == R_SET_BOOK && s->cur_page == 0) {
        return s->cur_book;
    }
    if (s->cur_book == BOOK_CTRL && s->cur_page == 0x00) {
        if (ctrl_is_rsv(reg)) {
            warn_report("esp32-tas5828m I2C 0x%02x: read from unknown "
                        "register %02x/%02x/%02x (reserved)",
                        s->address, s->cur_book, s->cur_page, reg);
            return 0;
        }
        switch (reg) {
        case R_RESET_CTRL:
        case R_FAULT_CLEAR:
            return 0;
        case R_DIE_ID:
            return 0x95;
        case R_POWER_STATE:
            return s->ctrl[R_DEVICE_CTRL2] & 3;
        case R_FS_MON: {
            uint32_t code = s->meas_valid ? rate_code(s->meas_rate) : 0;
            uint32_t ratio = s->meas_valid ? s->meas_bclk_ratio : 0;
            return ((ratio >> 8) & 3) << 4 | (code & 0xF);
        }
        case R_BCLK_MON:
            return (s->meas_valid ? s->meas_bclk_ratio : 0) & 0xFF;
        case R_CLKDET_STATUS: {
            uint8_t v = 0;
            if (!s->meas_valid) {
                v |= CS_BCLK_MISSING | CS_BCLK_VALID | CS_FS_VALID |
                     CS_PLL_LOCKED;
            } else if (s->clock_fault) {
                v |= CS_BCLK_VALID | CS_FS_VALID;
            }
            return v;
        }
        case 0x70: case 0x72: case 0x73: case 0x69: case 0x5E:
            return 0;
        case R_GLOBAL_FAULT1:
            return s->clock_fault ? GF1_CLOCK_FAULT : 0;
        default:
            return s->ctrl[reg];
        }
    }
    /* Level meters are live. */
    if (s->cur_book == BOOK_LVL && s->cur_page == 0x06 &&
        reg >= 0x54 && reg < 0x5C) {
        unsigned ch = (reg - 0x54) / 4;
        uint32_t v = s->peak[ch];
        return (v >> (8 * (3 - ((reg - 0x54) % 4)))) & 0xFF;
    }
    uint8_t *p = dsp_ptr(s, s->cur_book, s->cur_page, reg);
    if (!p) {
        warn_report("esp32-tas5828m I2C 0x%02x: read from unknown "
                    "register %02x/%02x/%02x (unmapped)",
                    s->address, s->cur_book, s->cur_page, reg);
        return 0;
    }
    return *p;
}

/* ------------------------------------------------------------ I2C target */

static void sda_drive(Esp32Tas5828m *s, bool low)
{
    esp32_gpio_set_external_drive(s->gpio, s->pad[PIN_SDA], s->driver_slot,
                                  low ? ESP32_PAD_LOW : ESP32_PAD_Z);
}

static void tas_reset(DeviceState *dev);

static void pdn_event(void *opaque, int n, int value)
{
    Esp32Tas5828m *s = opaque;
    enum PdnState next = value == ESP32_PAD_HIGH ? PDN_HIGH :
                         value == ESP32_PAD_LOW ? PDN_LOW :
                         PDN_INDETERMINATE;

    if (s->pdn_state == next) {
        return;
    }
    s->pdn_state = next;
    if (next == PDN_LOW) {
        /* TI specifies all internal regulators are off in shutdown. Model the
         * resulting volatile-state loss as power-on defaults on re-enable. */
        wav_flush(s);
        tas_reset(DEVICE(s));
    } else if (next == PDN_HIGH) {
        /* A newly powered device starts from defaults; playback and clock
         * validity must be re-established by normal register/clock traffic. */
        tas_reset(DEVICE(s));
    } else if (next != PDN_HIGH) {
        /* A floating/contended pad is neither enabled nor a definite reset. */
        s->phase = I2C_IGNORE;
        s->pending_valid = false;
        sda_drive(s, false);
    }
}

static void i2c_load(Esp32Tas5828m *s)
{
    s->shift = reg_read(s, s->pointer++);
    s->bits = 0;
    s->phase = I2C_READ;
    sda_drive(s, !(s->shift & 0x80));
}

static void i2c_rise(Esp32Tas5828m *s)
{
    bool sda = s->level[PIN_SDA] == ESP32_PAD_HIGH;

    switch (s->phase) {
    case I2C_ADDRESS:
    case I2C_WRITE:
        s->shift = (s->shift << 1) | sda;
        if (++s->bits == 8) {
            if (s->phase == I2C_ADDRESS) {
                s->ack = (s->shift >> 1) == s->address;
                s->next = s->shift & 1 ? I2C_READ : I2C_WRITE;
                s->expect_pointer = !(s->shift & 1);
            } else {
                s->ack = true;
                if (s->expect_pointer) {
                    s->pointer = s->shift;
                    s->expect_pointer = false;
                    s->first_data = true;
                } else {
                    bq_write_byte(s, s->pointer);
                    reg_write(s, s->pointer++, s->shift);
                }
                s->next = I2C_WRITE;
            }
            if (!s->ack) {
                s->next = I2C_IGNORE;
            }
            s->phase = I2C_ACK_SETUP;
        }
        break;
    case I2C_ACK_CLOCK:
        s->phase = I2C_ACK_END;
        break;
    case I2C_READ:
        if (++s->bits == 8) {
            s->phase = I2C_READ_ACK_SETUP;
        }
        break;
    case I2C_READ_ACK:
        s->phase = sda ? I2C_IGNORE : I2C_READ_ACK_END;
        break;
    default:
        break;
    }
}

static void i2c_fall(Esp32Tas5828m *s)
{
    switch (s->phase) {
    case I2C_ACK_SETUP:
        s->phase = I2C_ACK_CLOCK;
        sda_drive(s, s->ack);
        break;
    case I2C_ACK_END:
        s->phase = s->next;
        s->bits = s->shift = 0;
        sda_drive(s, false);
        if (s->phase == I2C_READ) {
            i2c_load(s);
        }
        break;
    case I2C_READ:
        sda_drive(s, !(s->shift & (0x80 >> s->bits)));
        break;
    case I2C_READ_ACK_SETUP:
        s->phase = I2C_READ_ACK;
        sda_drive(s, false);
        break;
    case I2C_READ_ACK_END:
        i2c_load(s);
        break;
    default:
        break;
    }
}

static void i2c_event(void *opaque, int n, int value)
{
    Esp32Tas5828m *s = opaque;
    bool level, previous;

    if (s->pdn_state != PDN_TIED_HIGH && s->pdn_state != PDN_HIGH) {
        s->level[n] = value;
        s->phase = I2C_IGNORE;
        sda_drive(s, false);
        return;
    }

    if (!level_binary(value)) {
        s->level[n] = value;
        s->phase = I2C_IGNORE;
        return;
    }
    level = value == ESP32_PAD_HIGH;
    previous = s->level[n] == ESP32_PAD_HIGH;
    s->level[n] = value;
    if (n == PIN_SCL) {
        if (level != previous) {
            if (level) {
                i2c_rise(s);
            } else {
                i2c_fall(s);
            }
        }
    } else if (level != previous && s->level[PIN_SCL] == ESP32_PAD_HIGH) {
        /* Repeated START also closes the preceding write segment. */
        bq_write_end(s);
        s->phase = level ? I2C_IDLE : I2C_ADDRESS;
        s->bits = s->shift = 0;
        s->first_data = false;
        sda_drive(s, false);
    }
}

/* ---------------------------------------------------------- QOM plumbing */

static void tas_reset(DeviceState *dev)
{
    Esp32Tas5828m *s = ESP32_TAS5828M(dev);

    tas_reset_regs(s);
    dsp_init_defaults(s);
    s->phase = I2C_IDLE;
    s->bits = s->shift = s->pointer = s->next = 0;
    s->ack = s->expect_pointer = s->first_data = false;
    s->bq_write_active = false;
    s->bq_write_count = 0;
    g_free(s->coefficient_safety);
    s->coefficient_safety = g_strdup("not-checked");
    s->stream_edges = 0;
    s->bclk_seen = false;
    s->meas_rate = s->meas_bclk_ratio = 0;
    s->meas_valid = false;
    s->wav_rate = 48000;
    lose_sync(s, false, NULL);
    if (s->gpio) {
        sda_drive(s, false);
    }
}

static char *tas_get_coefficient_safety(Object *obj, Error **errp)
{
    Esp32Tas5828m *s = ESP32_TAS5828M(obj);

    return g_strdup(s->coefficient_safety ? s->coefficient_safety :
                    "not-checked");
}

static char *tas_get_pdn_state(Object *obj, Error **errp)
{
    Esp32Tas5828m *s = ESP32_TAS5828M(obj);
    static const char *const names[] = {
        [PDN_TIED_HIGH] = "tied-high",
        [PDN_LOW] = "low",
        [PDN_HIGH] = "high",
        [PDN_INDETERMINATE] = "indeterminate",
    };

    return g_strdup(names[s->pdn_state]);
}

static bool tas_get_audio_active(Object *obj, Error **errp)
{
    Esp32Tas5828m *s = ESP32_TAS5828M(obj);

    return in_play(s) && s->interval_ok && s->synced;
}

static void tas_realize(DeviceState *dev, Error **errp)
{
    Esp32Tas5828m *s = ESP32_TAS5828M(dev);

    if (!s->gpio) {
        error_setg(errp, "esp32-tas5828m needs the 'gpio' link");
        return;
    }
    for (unsigned i = 0; i < PIN_COUNT; i++) {
        if (s->pad[i] >= ESP32_GPIO_PADS) {
            error_setg(errp, "esp32-tas5828m: pad %u is out of range",
                       s->pad[i]);
            return;
        }
        for (unsigned j = 0; j < i; j++) {
            if (s->pad[i] == s->pad[j]) {
                error_setg(errp, "esp32-tas5828m: pad %u used for two signals",
                           s->pad[i]);
                return;
            }
        }
    }
    if (s->pdn_pad != UINT32_MAX) {
        if (s->pdn_pad >= ESP32_GPIO_PADS) {
            error_setg(errp, "esp32-tas5828m: PDN pad %u is out of range",
                       s->pdn_pad);
            return;
        }
        for (unsigned i = 0; i < PIN_COUNT; i++) {
            if (s->pdn_pad == s->pad[i]) {
                error_setg(errp, "esp32-tas5828m: PDN pad %u is also used for a signal",
                           s->pdn_pad);
                return;
            }
        }
    }
    if (s->driver_slot >= ESP32_GPIO_EXT_DRIVERS || s->address > 0x7f) {
        error_setg(errp, "esp32-tas5828m needs a valid driver-slot and a "
                   "7-bit address");
        return;
    }
    if (s->max_bytes < WAV_HEADER_SIZE || s->max_bytes > UINT32_MAX - 64) {
        error_setg(errp, "esp32-tas5828m max-bytes must be 44..4294967231");
        return;
    }
    s->fd = -1;
    s->pdn_state = s->pdn_pad == UINT32_MAX ? PDN_TIED_HIGH :
                   PDN_INDETERMINATE;
    object_property_add_str(OBJECT(dev), "coefficient-safety",
                            tas_get_coefficient_safety, NULL);
    object_property_add_str(OBJECT(dev), "pdn-state", tas_get_pdn_state, NULL);
    object_property_add_bool(OBJECT(dev), "audio-active",
                             tas_get_audio_active, NULL);
    s->stage = g_malloc(STAGE_SIZE);
    s->flush_timer = timer_new_ms(QEMU_CLOCK_REALTIME, flush_timer_cb, s);
    s->exit_notifier.notify = exit_notify;
    qemu_add_exit_notifier(&s->exit_notifier);
    for (unsigned i = 0; i < PIN_COUNT; i++) {
        s->level[i] = ESP32_PAD_Z;
        s->sink[i] = qemu_allocate_irq(i == PIN_SDA || i == PIN_SCL ?
                                       i2c_event : pad_event, s, i);
    }
    if (s->pdn_pad != UINT32_MAX) {
        s->pdn_sink = qemu_allocate_irq(pdn_event, s, 0);
    }
    tas_reset(dev);
    for (unsigned i = 0; i < PIN_COUNT; i++) {
        if (i == PIN_BCLK || i == PIN_WS || i == PIN_DATA) {
            esp32_gpio_add_analytic_pad_listener(s->gpio, s->pad[i], s->sink[i]);
        } else {
            esp32_gpio_add_pad_listener(s->gpio, s->pad[i], s->sink[i]);
        }
    }
    if (s->pdn_pad != UINT32_MAX) {
        esp32_gpio_add_pad_listener(s->gpio, s->pdn_pad, s->pdn_sink);
    }
    esp32_gpio_add_serial_frame_observer(s->gpio, s->pad[PIN_BCLK],
                                         s->pad[PIN_WS], s->pad[PIN_DATA],
                                         serial_frame, s);
    if (s->pullups) {
        /* Optional external I2C pull-up resistors. */
        esp32_gpio_set_board_pull(s->gpio, s->pad[PIN_SDA], true);
        esp32_gpio_set_board_pull(s->gpio, s->pad[PIN_SCL], true);
    }
    if (s->wav) {
        wav_open(s);
    }
}

static void tas_unrealize(DeviceState *dev)
{
    Esp32Tas5828m *s = ESP32_TAS5828M(dev);

    wav_close(s);
    qemu_remove_exit_notifier(&s->exit_notifier);
    esp32_gpio_remove_serial_frame_observer(s->gpio, serial_frame, s);
    if (s->pullups) {
        esp32_gpio_set_board_pull(s->gpio, s->pad[PIN_SDA], false);
        esp32_gpio_set_board_pull(s->gpio, s->pad[PIN_SCL], false);
    }
    for (unsigned i = 0; i < PIN_COUNT; i++) {
        esp32_gpio_remove_pad_listener(s->gpio, s->pad[i], s->sink[i]);
        qemu_free_irq(s->sink[i]);
    }
    if (s->pdn_pad != UINT32_MAX) {
        esp32_gpio_remove_pad_listener(s->gpio, s->pdn_pad, s->pdn_sink);
        qemu_free_irq(s->pdn_sink);
    }
    sda_drive(s, false);
    timer_free(s->flush_timer);
    g_free(s->stage);
}

static const VMStateDescription tas_vmstate = {
    .name = TYPE_ESP32_TAS5828M,
    .version_id = 1,
    .minimum_version_id = 1,
    .unmigratable = 1,
};

static const Property tas_properties[] = {
    DEFINE_PROP_LINK("gpio", Esp32Tas5828m, gpio, TYPE_ESP32_GPIO,
                     Esp32GpioState *),
    DEFINE_PROP_STRING("wav", Esp32Tas5828m, wav),
    DEFINE_PROP_UINT32("sda", Esp32Tas5828m, pad[PIN_SDA], UINT32_MAX),
    DEFINE_PROP_UINT32("scl", Esp32Tas5828m, pad[PIN_SCL], UINT32_MAX),
    DEFINE_PROP_UINT32("bclk", Esp32Tas5828m, pad[PIN_BCLK], UINT32_MAX),
    DEFINE_PROP_UINT32("ws", Esp32Tas5828m, pad[PIN_WS], UINT32_MAX),
    DEFINE_PROP_UINT32("data", Esp32Tas5828m, pad[PIN_DATA], UINT32_MAX),
    DEFINE_PROP_UINT32("pdn", Esp32Tas5828m, pdn_pad, UINT32_MAX),
    DEFINE_PROP_UINT32("driver-slot", Esp32Tas5828m, driver_slot, 2),
    DEFINE_PROP_UINT32("address", Esp32Tas5828m, address, 0x61),
    DEFINE_PROP_BOOL("pullups", Esp32Tas5828m, pullups, true),
    DEFINE_PROP_UINT64("max-bytes", Esp32Tas5828m, max_bytes, 1ull << 30),
    DEFINE_PROP_END_OF_LIST(),
};

static void tas_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = tas_realize;
    dc->unrealize = tas_unrealize;
    device_class_set_legacy_reset(dc, tas_reset);
    dc->vmsd = &tas_vmstate;
    dc->desc = "TI TAS5828M I2C-controlled I2S slave Class-D amplifier "
               "with DSP BQ path, recording WAV from resolved ESP32 pads";
    device_class_set_props(dc, tas_properties);
    set_bit(DEVICE_CATEGORY_MISC, dc->categories);
}

static const TypeInfo tas_type = {
    .name = TYPE_ESP32_TAS5828M,
    .parent = TYPE_DEVICE,
    .instance_size = sizeof(Esp32Tas5828m),
    .class_init = tas_class_init,
};

static void tas_register(void)
{
    type_register_static(&tas_type);
}

type_init(tas_register)
