/*
 * lifu_config.h
 *
 *  Created on: Oct 24, 2025
 *      Author: gvigelet
 */

#ifndef LIFU_CONFIG_H
#define LIFU_CONFIG_H

#include <stdint.h>
#include <stdbool.h>

/*
 * Drop-in persistent config with single-page, append-only journal.
 * - Uses Flash_Read/Write/Erase from flash_eeprom.h
 * - Uses util_crc16/util_hw_crc16 from utils.h
 * - Stores a bounded JSON blob inline (data_len + bytes) per record
 *
 * Default config if flash invalid: hv_settng=0.0, hv_enabled=false, auto_on=false
 */

#ifdef __cplusplus
extern "C" {
#endif

// ---- Project includes you already have ----
#include "common.h"         // declares your runtime LifuConfig
#include "flash_eeprom.h"   // Flash_Read / Flash_Write / Flash_Erase

// ======================== TUNABLE GEOMETRY ========================
//
// Configure one (1) flash page reserved for config.
// Keep CFG_PAGE_BASE page-aligned and set CFG_PAGE_SIZE to actual size.
//
// SLOT_SIZE sets the per-record capacity (header + JSON). 512 B is a good default on STM32F0.
//
// NOTE: This module NEVER writes outside [CFG_PAGE_BASE, CFG_PAGE_BASE + CFG_PAGE_SIZE).
//
#ifndef CFG_PAGE_BASE
#define CFG_PAGE_BASE      (0x0801F000U)   // example: one 2KB page near the end (adjust for your part)
#endif

#ifndef CFG_PAGE_SIZE
#define CFG_PAGE_SIZE      (2048U)         // STM32F072 often uses 2KB pages (confirm for your exact device)
#endif

#ifndef CFG_SLOT_SIZE
#define CFG_SLOT_SIZE      (512U)          // one record slot size (must divide page size cleanly)
#endif

// Compile-time check
#if (CFG_PAGE_SIZE % CFG_SLOT_SIZE) != 0
#error "CFG_SLOT_SIZE must evenly divide CFG_PAGE_SIZE"
#endif

#define CFG_SLOTS_PER_PAGE   (CFG_PAGE_SIZE / CFG_SLOT_SIZE)

// ======================== RECORD FORMAT ===========================
#define LIFU_MAGIC   (0x4C494655UL)  // 'LIFU'
#define LIFU_VER     (0x00010002UL)  // bump when changing on-flash layout

// On-flash header (fixed prefix of each slot). JSON data follows immediately.
typedef struct __attribute__((packed, aligned(4))) {
    uint32_t magic;       // LIFU_MAGIC
    uint32_t version;     // LIFU_VER
    uint32_t seq;         // monotonic sequence for latest-wins
    float    hv_settng;   // persisted scalar
    uint8_t  hv_enabled;  // 0/1
    uint8_t  auto_on;     // 0/1
    uint8_t  _rsvd[2];    // pad/alignment
    uint16_t data_len;    // bytes of JSON following this header in the slot
    uint16_t crc;         // CRC-16-CCITT over [header..(excluding crc)] + JSON bytes
} lifu_cfg_hdr_t;

_Static_assert(sizeof(lifu_cfg_hdr_t) % 4 == 0, "header must be word-aligned");

// JSON capacity bounded by the slot size
#define MAX_JSON_BYTES  ((uint16_t)(CFG_SLOT_SIZE - sizeof(lifu_cfg_hdr_t)))

// ======================== PUBLIC API ==============================

/**
 * Initialize config system:
 *  - scans the single page for the last valid record by seq
 *  - loads it into RAM; if none found => defaults + write seed entry at slot 0
 */
void LIFU_Config_Init(void);

/** Get current runtime config (in RAM). */
const LifuConfig* LIFU_Config_Get(void);

/**
 * Save current runtime config + current JSON buffer:
 *  - appends into next free slot
 *  - if page is full, erases page and writes record into slot 0 (seed)
 * Returns true on success.
 */
bool LIFU_Config_Save(void);

/** Reset runtime config to defaults and persist immediately (append/seed). */
void LIFU_Config_ResetToDefaults(void);

/** JSON setters/getters (RAM buffer; persisted by LIFU_Config_Save). */
bool     LIFU_Config_SetJSON(const uint8_t* data, uint16_t len);  // bounds-checked to MAX_JSON_BYTES
uint16_t LIFU_Config_GetJSON(uint8_t* out, uint16_t maxlen);      // copies out
const uint8_t* LIFU_Config_GetJSONPtr(uint16_t* len);             // direct ptr to internal buffer (read-only)

/** Optional: mark/flush helpers if you want debounced saves (not required). */
void LIFU_Config_MarkDirty(void);
void LIFU_Config_Service(void);   // call periodically if you use the dirty/debounce pattern

#ifdef __cplusplus
}
#endif

#endif // LIFU_CONFIG_H
