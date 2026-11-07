/**
 * @file    sd_writer.h
 * @brief   VIBL binary logger for SD Logger V2 (STM32F405RG, FatFs R0.12c).
 *
 * Implements the on-disk format defined in doc/SD_LOGGER_ARCHITECTURE_V2.md §4:
 *   HEADER (fixed 256 B) | CHUNKS ([chunk_hdr][n*record])... | TRAILER
 * All structures are little-endian and packed to 1 byte. No dynamic memory is
 * used; every chunk commit is followed by f_sync() for power-loss safety, and
 * the IWDG is fed inside vibl_write_chunk().
 */
#ifndef SD_WRITER_H
#define SD_WRITER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* -------------------------------------------------------------------------- */
/* Format constants (doc/SD_LOGGER_ARCHITECTURE_V2.md §4)                     */
/* -------------------------------------------------------------------------- */
#define VIBL_MAGIC              "VIBL"
#define VIBL_VERSION            1u
#define VIBL_SAMPLE_RATE        1600u   /* Hz : TIM2 = 84 MHz / (125 * 420)     */
#define VIBL_AXES               3u      /* X, Y, Z                             */
#define VIBL_BYTES_PER_SAMPLE   6u      /* int16 x, y, z                      */
#define VIBL_SAMPLES_PER_CHUNK  2048u   /* N per buffer (doc §6)              */
#define VIBL_HEADER_SIZE        256u    /* fixed header size in bytes         */
#define VIBL_HEADER_CRC_LEN     (VIBL_HEADER_SIZE - 4u) /* CRC over [0..251]  */

/* Sensor name table index -> 16-byte sensor string written into the header. */
#define VIBL_SENSOR_HIL         0u      /* "HIL-DAC-ADC"                      */
#define VIBL_SENSOR_LIS3DSH     1u      /* "LIS3DSH"                          */

/* -------------------------------------------------------------------------- */
/* Status codes                                                               */
/* -------------------------------------------------------------------------- */
typedef enum {
    VIBL_OK           = 0,
    VIBL_ERR_PARAM    = -1,   /* bad argument (NULL / out of range)          */
    VIBL_ERR_NOT_OPEN = -2,   /* called while no file is open                */
    VIBL_ERR_OPEN     = -3,   /* f_open failed                               */
    VIBL_ERR_WRITE    = -4,   /* f_write short / failed                      */
    VIBL_ERR_SYNC     = -5,   /* f_sync failed                               */
    VIBL_ERR_SEEK     = -6,   /* f_lseek failed                              */
    VIBL_ERR_CLOSE    = -7,   /* f_close failed                              */
} vibl_status_t;

/* -------------------------------------------------------------------------- */
/* On-disk binary structures (little-endian, packed to 1 byte)                */
/* -------------------------------------------------------------------------- */

#pragma pack(push, 1)
typedef struct {
    char     magic[4];            /* "VIBL"                                  */
    uint16_t version;             /* = VIBL_VERSION                          */
    uint32_t sample_rate;         /* Hz                                      */
    uint8_t  axes;                /* number of axes (3)                      */
    uint8_t  bytes_per_sample;    /* 6                                       */
    char     sensor[16];          /* e.g. "LIS3DSH" / "HIL-DAC-ADC"           */
    int8_t   range_g;             /* full-scale range in g (signed per spec) */
    float    gain;                /* IEEE-754 f32 gain factor                */
    int16_t  offset_x;            /* calibration offsets (0 if unused)       */
    int16_t  offset_y;
    int16_t  offset_z;
    uint64_t start_ts;            /* RTC time, ms since Unix epoch           */
    uint32_t record_count;        /* total records (finalized in vibl_close) */
    uint32_t header_crc;          /* CRC-32 over bytes [0..251]              */
    uint8_t  reserved[201];       /* padding -> header is exactly 256 bytes  */
} vibl_header_t;
#pragma pack(pop)

_Static_assert(sizeof(vibl_header_t) == VIBL_HEADER_SIZE,
               "vibl_header_t must be exactly 256 bytes");

#pragma pack(push, 1)
typedef struct {
    uint32_t seq_number;          /* chunk sequence (gaps = jumps > 1)       */
    uint16_t sample_count;        /* number of records in this chunk         */
} vibl_chunk_hdr_t;
#pragma pack(pop)

_Static_assert(sizeof(vibl_chunk_hdr_t) == 6u, "vibl_chunk_hdr_t must be 6 bytes");

#pragma pack(push, 1)
typedef struct {
    uint64_t end_ts;              /* RTC time, ms since Unix epoch           */
    uint32_t total_records;       /* records written (clean stop only)       */
    uint32_t file_crc32;          /* CRC-32 over all chunk payload bytes     */
} vibl_trailer_t;
#pragma pack(pop)

_Static_assert(sizeof(vibl_trailer_t) == 16u, "vibl_trailer_t must be 16 bytes");

/* -------------------------------------------------------------------------- */
/* API                                                                        */
/* -------------------------------------------------------------------------- */

/**
 * Open (create/truncate) a VIBL file and write the fixed 256-byte header.
 * The header CRC is computed over bytes [0..251] and stored in header_crc.
 * record_count is written as 0 here and finalized by vibl_close().
 * @note With _USE_LFN=0 (ffconf.h) the filename must be a valid 8.3 name,
 *       e.g. "LOG0001.VIB".
 */
int vibl_open(const char *filename, uint8_t sensor_id, uint8_t range_g, uint16_t gain);

/**
 * Append one chunk: [chunk_hdr][count*record]. The running file CRC is updated
 * with the exact bytes written (chunk header + samples), then f_sync() commits
 * the chunk to SD and the IWDG is fed. Power loss corrupts at most this chunk.
 */
int vibl_write_chunk(const int16_t *samples, uint32_t count);

/**
 * Record a data gap of `dropped_count` samples (e.g. dropped READY buffers).
 * Per doc §4/§6 the gap is represented by a sequence-number jump in the next
 * chunk header: the skipped seq_numbers are simply not written, so the PC
 * parser detects "seq_number jump > 1". The exact sample count is latched for
 * reporting / error code 4.
 */
int vibl_mark_gap(uint32_t dropped_count);

/**
 * Finalize a clean stop: update header.record_count + recompute header_crc,
 * append the trailer (end_ts | total_records | file_crc32), f_sync and close.
 */
int vibl_close(void);

/* Optional state accessors (used by the FSM / error latching). */
int         vibl_is_open(void);
uint32_t    vibl_get_total_records(void);
uint32_t    vibl_get_chunk_count(void);
uint32_t    vibl_get_dropped_samples(void);
int         vibl_overflow_latched(void);

#ifdef __cplusplus
}
#endif

#endif /* SD_WRITER_H */