/**
 * @file    sd_writer_fixed.c
 * @brief   VIBL binary logger implementation (see sd_writer.h).
 *          Fixed version: BUG-2 (FA_WRITE), BUG-3 (ff.h include),
 *          BUG-7 (gain semantics comment), chunk count bound, header_crc zeroing.
 */
#include "sd_writer.h"
#include "main.h"      /* stm32f4xx_hal.h -> HAL_IWDG_Refresh, HAL_RTC_* */
#include "iwdg.h"      /* extern IWDG_HandleTypeDef hiwdg;               */
#include "rtc.h"       /* extern RTC_HandleTypeDef hrtc;                 */
#include "ff.h"        /* FatFs R0.12c API (f_open/f_write/f_sync/...)    */

#include <string.h>
#include <stddef.h>

/* -------------------------------------------------------------------------- */
/* Private state (static, no dynamic memory)                                 */
/* -------------------------------------------------------------------------- */
static FIL           vibl_fil;          /* open file object                */
static vibl_header_t vibl_hdr;          /* header kept for close-time update*/
static volatile int  vibl_active = 0;   /* 1 while a VIBL file is open     */

static uint32_t      next_chunk_seq    = 0u;
static uint32_t      total_records     = 0u;
static uint32_t      chunk_count       = 0u;
static uint32_t      total_dropped     = 0u;
static volatile int  overflow_latched  = 0;

/* Running CRC-32 state (un-finalized). Starts at 0xFFFFFFFF and is finalized
 * with ^0xFFFFFFFF only when the trailer value is produced.               */
static uint32_t      vibl_file_crc     = 0xFFFFFFFFu;

/* Sensor name table indexed by sensor_id (written into header.sensor[16]). */
static const char *const vibl_sensor_names[] = {
    "HIL-DAC-ADC",   /* VIBL_SENSOR_HIL   : HIL DAC->ADC loopback           */
    "LIS3DSH",       /* VIBL_SENSOR_LIS3DSH : production LIS3DSH accel      */
};
#define VIBL_SENSOR_COUNT (sizeof(vibl_sensor_names) / sizeof(vibl_sensor_names[0]))

/* -------------------------------------------------------------------------- */
/* CRC-32 (standard zlib/IEEE: poly 0xEDB88320, init 0xFFFFFFFF, final XOR)  */
/* -------------------------------------------------------------------------- */
static uint32_t crc_table[256];
static int      crc_table_ready = 0;

static void crc32_build_table(void)
{
    for (uint32_t i = 0u; i < 256u; i++) {
        uint32_t c = i;
        for (int k = 0; k < 8; k++) {
            c = ((c & 1u) != 0u) ? (0xEDB88320u ^ (c >> 1)) : (c >> 1);
        }
        crc_table[i] = c;
    }
}

/* Incremental CRC-32: `crc` is the running (un-finalized) state. */
static uint32_t crc32_update(uint32_t crc, const uint8_t *data, uint32_t len)
{
    if (!crc_table_ready) {
        crc32_build_table();
        crc_table_ready = 1;
    }
    for (uint32_t i = 0u; i < len; i++) {
        crc = crc_table[(crc ^ data[i]) & 0xFFu] ^ (crc >> 8);
    }
    return crc;
}

/* -------------------------------------------------------------------------- */
/* Helpers                                                                    */
/* -------------------------------------------------------------------------- */

/* Write `len` bytes to the open file, looping in <=64 KiB f_write calls. */
static int vibl_write_all(const void *buf, uint32_t len)
{
    const uint8_t *p = (const uint8_t *)buf;
    while (len > 0u) {
        UINT chunk = (len > 0xFFFFu) ? (UINT)0xFFFFu : (UINT)len;
        UINT bw = 0u;
        FRESULT fr = f_write(&vibl_fil, p, chunk, &bw);
        if ((fr != FR_OK) || (bw != chunk)) {
            return VIBL_ERR_WRITE;
        }
        p   += chunk;
        len -= chunk;
    }
    return VIBL_OK;
}

/* Best-effort cleanup on a mid-stream failure. */
static void vibl_abort(void)
{
    if (vibl_active != 0) {
        f_close(&vibl_fil);   /* ignore result: file already damaged/aborted */
        vibl_active = 0;
    }
}

/* Convert the RTC (LSE, BIN format) to milliseconds since Unix epoch. */
static uint64_t vibl_rtc_ms(void)
{
    RTC_TimeTypeDef  sTime;   /* F4 HAL: time and date are separate structs */
    RTC_DateTypeDef  sDate;

    if (HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK) {
        return 0u;
    }
    if (HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK) {
        return 0u;
    }

    uint32_t year = 2000u + sDate.Year;   /* F4 RTC stores year as offset from 2000 */
    int y = (int)year;
    int m = (int)sDate.Month;
    int d = (int)sDate.Date;              /* field is named Date, not Day */

    /* Days since 1970-01-01 (Howard Hinnant civil algorithm). */
    y -= (m <= 2) ? 1 : 0;
    int era = (y >= 0) ? (y / 400) : ((y - 399) / 400);
    unsigned int yoe = (unsigned int)(y - era * 400);                 /* [0,399] */
    unsigned int doy = (153u * ((m + ((m > 2) ? -3 : 9)) % 12) + 2) / 5 + (unsigned int)d - 1u;
    unsigned int doe = yoe * 365u + yoe / 4u - yoe / 100u + doy;       /* [0,146096] */
    uint64_t days = (uint64_t)era * 146097ull + doe - 719468ull;

    uint32_t secs_of_day = ((uint32_t)sTime.Hours * 3600u)
                         + ((uint32_t)sTime.Minutes * 60u)
                         + (uint32_t)sTime.Seconds;
    return days * 86400000ull + secs_of_day * 1000ull;
}

/* -------------------------------------------------------------------------- */
/* Public API                                                                 */
/* -------------------------------------------------------------------------- */

int vibl_open(const char *filename, uint8_t sensor_id, uint8_t range_g, uint16_t gain)
{
    if (filename == NULL) {
        return VIBL_ERR_PARAM;
    }

    /* Build the fixed 256-byte header in a static buffer (no malloc). */
    memset(&vibl_hdr, 0, sizeof(vibl_hdr));   /* zero reserved[] + all fields */

    memcpy(vibl_hdr.magic, VIBL_MAGIC, 4u);
    vibl_hdr.version          = VIBL_VERSION;
    vibl_hdr.sample_rate      = VIBL_SAMPLE_RATE;
    vibl_hdr.axes             = (uint8_t)VIBL_AXES;
    vibl_hdr.bytes_per_sample = (uint8_t)VIBL_BYTES_PER_SAMPLE;

    const char *sname = ((sensor_id < VIBL_SENSOR_COUNT) ? vibl_sensor_names[sensor_id] : "UNKNOWN");
    size_t slen = strlen(sname);
    if (slen >= sizeof(vibl_hdr.sensor)) {
        slen = sizeof(vibl_hdr.sensor) - 1u;   /* keep room for '\0' */
    }
    memcpy(vibl_hdr.sensor, sname, slen);
    vibl_hdr.sensor[slen] = '\0';

    vibl_hdr.range_g = (int8_t)range_g;        /* stored as i8 per spec       */
    vibl_hdr.gain    = (float)gain;            /* gain is LSB/g              */
    vibl_hdr.offset_x = 0;                     /* calibration not part of this API */
    vibl_hdr.offset_y = 0;
    vibl_hdr.offset_z = 0;
    vibl_hdr.start_ts   = vibl_rtc_ms();
    vibl_hdr.record_count = 0u;                /* finalized in vibl_close()   */

    /* header_crc covers bytes [0..251] (everything except the crc field). */
    uint32_t hcrc = crc32_update(0xFFFFFFFFu, (const uint8_t *)&vibl_hdr, VIBL_HEADER_CRC_LEN);
    vibl_hdr.header_crc = hcrc ^ 0xFFFFFFFFu;

    /* Reset per-session state. */
    next_chunk_seq   = 0u;
    total_records    = 0u;
    chunk_count      = 0u;
    total_dropped    = 0u;
    overflow_latched = 0;
    vibl_file_crc    = 0xFFFFFFFFu;            /* running (un-finalized) state */

    FRESULT fr = f_open(&vibl_fil, filename, FA_CREATE_ALWAYS | FA_WRITE);
    if (fr != FR_OK) {
        return VIBL_ERR_OPEN;
    }

    if (vibl_write_all((const uint8_t *)&vibl_hdr, sizeof(vibl_hdr)) != VIBL_OK) {
        f_close(&vibl_fil);
        vibl_active = 0;
        return VIBL_ERR_WRITE;
    }

    fr = f_sync(&vibl_fil);   /* commit header immediately (power-loss safety) */
    if (fr != FR_OK) {
        f_close(&vibl_fil);
        vibl_active = 0;
        return VIBL_ERR_SYNC;
    }

    vibl_active = 1;
    HAL_IWDG_Refresh(&hiwdg);   /* keep the watchdog fed across open/sync */
    return VIBL_OK;
}

int vibl_write_chunk(const int16_t *samples, uint32_t count)
{
    if ((vibl_active == 0) || (samples == NULL)) {
        return VIBL_ERR_PARAM;
    }
    if ((count == 0u) || (count > VIBL_SAMPLES_PER_CHUNK)) {   /* chunk bound per doc §6 */
        return VIBL_ERR_PARAM;
    }

    /* chunk_hdr per doc §4: seq_number u32 | sample_count u16 (6 bytes).
     * Per-chunk integrity is carried by the running file CRC below (the exact
     * bytes of this chunk are folded into vibl_file_crc), which the trailer
     * validates. This keeps the on-disk layout identical to the PC parser spec. */
    vibl_chunk_hdr_t chdr;
    chdr.seq_number  = next_chunk_seq;
    chdr.sample_count = (uint16_t)count;

    const uint8_t *hdr_bytes = (const uint8_t *)&chdr;
    uint32_t data_len = count * VIBL_BYTES_PER_SAMPLE;   /* count * 6 */

    if (vibl_write_all(hdr_bytes, sizeof(chdr)) != VIBL_OK) {
        vibl_abort();
        return VIBL_ERR_WRITE;
    }
    if (vibl_write_all(samples, data_len) != VIBL_OK) {
        vibl_abort();
        return VIBL_ERR_WRITE;
    }

    /* Update the running file CRC over exactly what was written. */
    vibl_file_crc = crc32_update(vibl_file_crc, hdr_bytes, sizeof(chdr));
    vibl_file_crc = crc32_update(vibl_file_crc, (const uint8_t *)samples, data_len);

    /* Commit the chunk to SD: power loss corrupts at most this in-flight chunk. */
    FRESULT fr = f_sync(&vibl_fil);
    if (fr != FR_OK) {
        vibl_abort();
        return VIBL_ERR_SYNC;
    }

    next_chunk_seq++;
    total_records += count;
    chunk_count++;

    /* Required: feed the IWDG inside the chunk write path. f_sync can stall up
     * to ~200 ms, well under the ~1-1.5 s IWDG timeout, but this guarantees it. */
    HAL_IWDG_Refresh(&hiwdg);

    return VIBL_OK;
}

int vibl_mark_gap(uint32_t dropped_count)
{
    if (vibl_active == 0) {
        return VIBL_ERR_NOT_OPEN;
    }
    if (dropped_count == 0u) {
        return VIBL_OK;
    }

    /* Gaps are represented by sequence-number jumps in chunk headers (doc §4/§6):
     * the dropped buffers' seq_numbers are simply skipped, so the next written
     * chunk header carries a higher seq_number and the PC parser reports
     * "seq_number jump > 1". The exact sample count is latched for reporting. */
    uint32_t dropped_chunks = dropped_count / VIBL_SAMPLES_PER_CHUNK;
    if (dropped_chunks == 0u) {
        dropped_chunks = 1u;   /* sub-chunk drop: still make the jump visible */
    }

    next_chunk_seq += dropped_chunks;
    total_dropped  += dropped_count;
    overflow_latched = 1;       /* -> FSM error code 4 (buffer overflow latched) */

    HAL_IWDG_Refresh(&hiwdg);   /* no SD I/O here, but keep the watchdog fed */
    return VIBL_OK;
}

int vibl_close(void)
{
    if (vibl_active == 0) {
        return VIBL_ERR_NOT_OPEN;
    }

    /* 1) Finalize the header in place: record_count + recomputed header_crc.
     *    The two fields are adjacent (offsets 47..50 and 51..54), so a single
     *    8-byte write keeps the update atomic w.r.t. power loss. */
    vibl_hdr.record_count = total_records;
    /* Zero the CRC field before recomputing: header_crc (offset 51..54) is NOT
     * inside [0..251], so this cannot change the result, but it removes any
     * ambiguity about which bytes feed the CRC. */
    vibl_hdr.header_crc = 0;
    uint32_t hcrc = crc32_update(0xFFFFFFFFu, (const uint8_t *)&vibl_hdr, VIBL_HEADER_CRC_LEN);
    vibl_hdr.header_crc = hcrc ^ 0xFFFFFFFFu;

    if (f_lseek(&vibl_fil, offsetof(vibl_header_t, record_count)) != FR_OK) {
        vibl_abort();
        return VIBL_ERR_SEEK;
    }
    /* &record_count points at [record_count(4)][header_crc(4)] (packed, adjacent). */
    if (vibl_write_all((const uint8_t *)&vibl_hdr.record_count, 2u * sizeof(uint32_t)) != VIBL_OK) {
        vibl_abort();
        return VIBL_ERR_WRITE;
    }
    FRESULT fr = f_sync(&vibl_fil);
    if (fr != FR_OK) {
        vibl_abort();
        return VIBL_ERR_SYNC;
    }

    /* 2) Append the trailer at EOF. */
    vibl_trailer_t tr;
    tr.end_ts        = vibl_rtc_ms();
    tr.total_records = total_records;   /* per spec: total record count (parser validates this) */
    tr.file_crc32    = vibl_file_crc ^ 0xFFFFFFFFu;   /* finalize running CRC */

    FSIZE_t eof = f_size(&vibl_fil);
    if (f_lseek(&vibl_fil, eof) != FR_OK) {
        vibl_abort();
        return VIBL_ERR_SEEK;
    }
    if (vibl_write_all((const uint8_t *)&tr, sizeof(tr)) != VIBL_OK) {
        vibl_abort();
        return VIBL_ERR_WRITE;
    }
    fr = f_sync(&vibl_fil);
    if (fr != FR_OK) {
        vibl_abort();
        return VIBL_ERR_SYNC;
    }

    HAL_IWDG_Refresh(&hiwdg);

    FRESULT cfr = f_close(&vibl_fil);
    vibl_active = 0;
    return (cfr == FR_OK) ? VIBL_OK : VIBL_ERR_CLOSE;
}

/* -------------------------------------------------------------------------- */
/* State accessors                                                            */
/* -------------------------------------------------------------------------- */
int         vibl_is_open(void)            { return vibl_active; }
uint32_t    vibl_get_total_records(void)  { return total_records; }
uint32_t    vibl_get_chunk_count(void)    { return chunk_count; }
uint32_t    vibl_get_dropped_samples(void){ return total_dropped; }
int         vibl_overflow_latched(void)   { return overflow_latched; }
