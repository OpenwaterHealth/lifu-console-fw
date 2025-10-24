/*
 * lifu_config.c
 *
 *  Created on: Oct 24, 2025
 *      Author: gvigelet
 */
#include "lifu_config.h"
#include <string.h>   // memset, memcpy
#include "utils.h"    // util_crc16 / util_hw_crc16  (CRC helper)     // uses your existing utils  // (citation in notes)

// ======================== INTERNAL STATE ==========================

static LifuConfig g_cfg;                               // runtime struct (from common.h)
static uint8_t    g_json[MAX_JSON_BYTES];              // RAM JSON buffer
static uint16_t   g_json_len = 0;
static uint32_t   g_seq = 0;

// Optional dirty/debounce
static bool       s_dirty = false;
static uint32_t   s_last_change_ms = 0; // requires HAL_GetTick()

// ======================== CRC HELPERS =============================
//
// Use your utils.h helpers (SW or HW). If you prefer the hardware CRC, swap the call below.
//
static inline uint16_t lifu_crc16(const uint8_t* buf, uint32_t size)
{
    return util_crc16(buf, size);   // or util_hw_crc16((uint8_t*)buf, size);
}

// ======================== UTILITIES ===============================

static inline uint32_t page_addr_of_slot(uint32_t slot_index)
{
    return (CFG_PAGE_BASE + (slot_index * CFG_SLOT_SIZE));
}

static void cfg_defaults(void)
{
    memset(&g_cfg, 0, sizeof(g_cfg));
    g_cfg.magic_num = LIFU_MAGIC;
    g_cfg.hv_settng = 0.0f;
    g_cfg.auto_on   = false;

    // If your LifuConfig has hv_enabled, map it here:
    // g_cfg.hv_enabled = false;

    g_json_len = 0;
}

static void runtime_from_hdr_json(const lifu_cfg_hdr_t* h, const uint8_t* json)
{
    memset(&g_cfg, 0, sizeof(g_cfg));
    g_cfg.magic_num = h->magic;
    g_cfg.hv_settng = h->hv_settng;
    g_cfg.auto_on   = (h->auto_on != 0);

    // Map hv_enabled if present in your runtime LifuConfig
    // g_cfg.hv_enabled = (h->hv_enabled != 0);

    if (h->data_len > 0 && h->data_len <= MAX_JSON_BYTES && json) {
        memcpy(g_json, json, h->data_len);
        g_json_len = h->data_len;
    } else {
        g_json_len = 0;
    }
}

static void fill_hdr_from_runtime(lifu_cfg_hdr_t* h, uint32_t next_seq, uint16_t json_len)
{
    memset(h, 0xFF, sizeof(*h));  // keep erased pattern in spare bytes
    h->magic       = LIFU_MAGIC;
    h->version     = LIFU_VER;
    h->seq         = next_seq;
    h->hv_settng   = g_cfg.hv_settng;
    h->hv_enabled  = (uint8_t)(/* g_cfg.hv_enabled ? */ 0 /* replace if you have the field */);
    h->auto_on     = (uint8_t)(g_cfg.auto_on ? 1 : 0);
    h->data_len    = json_len;
    h->crc         = 0;  // will be calculated when assembling the slot buffer
}

static bool hdr_json_valid(const lifu_cfg_hdr_t* h, const uint8_t* json_bytes)
{
    if (h->magic != LIFU_MAGIC)   return false;
    if (h->version != LIFU_VER)   return false;
    if (h->data_len > MAX_JSON_BYTES) return false;

    // Compute CRC over header (excluding crc field) + json bytes
    const uint32_t head_no_crc_len = (uint32_t)offsetof(lifu_cfg_hdr_t, crc);
    uint16_t crc = lifu_crc16((const uint8_t*)h, head_no_crc_len);
    if (h->data_len && json_bytes) {
        // concatenate by computing CRC over a temporary combined buffer (safer & simpler)
        // But to avoid heap, do a second pass:
        // We'll emulate concatenation by building a small stack buffer if reasonable.
        // Since MAX_JSON_BYTES could be up to ~480, stack usage is fine on F0.
        uint8_t tmp[sizeof(lifu_cfg_hdr_t) + MAX_JSON_BYTES];
        memcpy(tmp, h, head_no_crc_len);               // header without crc
        memcpy(tmp + head_no_crc_len, json_bytes, h->data_len);
        crc = lifu_crc16(tmp, head_no_crc_len + h->data_len);
    }
    return (crc == h->crc);
}

// Return last valid slot index and its header/JSON (if provided).
// Stops scanning when the first truly empty slot is encountered (first word erased).
static int find_last_valid(lifu_cfg_hdr_t* out_hdr, uint8_t* out_json)
{
    uint8_t slot_buf[CFG_SLOT_SIZE];
    lifu_cfg_hdr_t* hdr = (lifu_cfg_hdr_t*)slot_buf;

    int last = -1;
    uint32_t best_seq = 0;

    for (uint32_t i = 0; i < CFG_SLOTS_PER_PAGE; i++) {
        uint32_t addr = page_addr_of_slot(i);

        // Read the entire slot for simplicity and CRC correctness
        Flash_Read(addr, (uint32_t*)slot_buf, (uint32_t)(CFG_SLOT_SIZE / 4));

        // If first word is erased, we assume this and subsequent slots are empty (append-only)
        if (*(uint32_t*)slot_buf == 0xFFFFFFFFU) {
            break;
        }

        // Validate the record (header + inline JSON)
        uint16_t len = hdr->data_len;
        if (len <= MAX_JSON_BYTES) {
            const uint8_t* jsonp = slot_buf + sizeof(lifu_cfg_hdr_t);
            if (hdr_json_valid(hdr, jsonp)) {
                if (hdr->seq >= best_seq) {
                    best_seq = hdr->seq;
                    last = (int)i;
                    if (out_hdr)  *out_hdr = *hdr;
                    if (out_json && len) memcpy(out_json, jsonp, len);
                }
            }
        }
    }
    return last;
}

// Find first empty slot (magic erased)
static int find_first_empty(void)
{
    uint32_t word;
    for (uint32_t i = 0; i < CFG_SLOTS_PER_PAGE; i++) {
        uint32_t addr = page_addr_of_slot(i);
        Flash_Read(addr, &word, 1);
        if (word == 0xFFFFFFFFU) return (int)i;
    }
    return -1; // no empty (page full)
}

// Write one slot atomically (header + JSON), bounded to the slot
static bool write_slot(uint32_t slot_index, const lifu_cfg_hdr_t* hdr_in, const uint8_t* json)
{
    uint8_t slot_buf[CFG_SLOT_SIZE];
    memset(slot_buf, 0xFF, sizeof(slot_buf));

    lifu_cfg_hdr_t hdr = *hdr_in;
    hdr.crc = 0;

    // Assemble continuous header(without crc) + json for CRC
    const uint32_t head_no_crc_len = (uint32_t)offsetof(lifu_cfg_hdr_t, crc);
    memcpy(slot_buf, &hdr, sizeof(hdr));
    if (hdr.data_len) {
        memcpy(slot_buf + sizeof(hdr), json, hdr.data_len);
    }

    // Compute CRC across contiguous [header..json], excluding crc field
    uint8_t crc_buf[sizeof(lifu_cfg_hdr_t) + MAX_JSON_BYTES];
    memcpy(crc_buf, &hdr, head_no_crc_len);
    if (hdr.data_len) memcpy(crc_buf + head_no_crc_len, json, hdr.data_len);
    uint16_t crc = lifu_crc16(crc_buf, head_no_crc_len + hdr.data_len);
    ((lifu_cfg_hdr_t*)slot_buf)->crc = crc;

    // Program the entire slot
    uint32_t addr = page_addr_of_slot(slot_index);
    return (HAL_OK == Flash_Write(addr, (uint32_t*)slot_buf, (uint32_t)(CFG_SLOT_SIZE / 4)));
}

// When page is full: erase page, then seed slot 0 with the new record
static bool erase_and_seed(const lifu_cfg_hdr_t* hdr, const uint8_t* json)
{
    if (HAL_OK != Flash_Erase(CFG_PAGE_BASE, CFG_PAGE_BASE + CFG_PAGE_SIZE))
        return false;
    return write_slot(0, hdr, json);
}

// ======================== PUBLIC API ==============================

void LIFU_Config_Init(void)
{
    lifu_cfg_hdr_t last_hdr;
    uint8_t last_json[MAX_JSON_BYTES];
    int idx = find_last_valid(&last_hdr, last_json);

    if (idx < 0) {
        // nothing valid -> defaults + seed slot 0
        cfg_defaults();
        g_seq = 1;

        lifu_cfg_hdr_t hdr;
        fill_hdr_from_runtime(&hdr, g_seq, g_json_len);
        (void)Flash_Erase(CFG_PAGE_BASE, CFG_PAGE_BASE + CFG_PAGE_SIZE);
        (void)write_slot(0, &hdr, g_json_len ? g_json : NULL);
        return;
    }

    // Load the latest valid record
    g_seq = last_hdr.seq;
    runtime_from_hdr_json(&last_hdr, last_hdr.data_len ? last_json : NULL);
}

const LifuConfig* LIFU_Config_Get(void)
{
    return &g_cfg;
}

bool LIFU_Config_Save(void)
{
    if (g_json_len > MAX_JSON_BYTES) return false;

    lifu_cfg_hdr_t hdr;
    fill_hdr_from_runtime(&hdr, g_seq + 1, g_json_len);

    int empty = find_first_empty();
    bool ok = false;

    if (empty >= 0) {
        ok = write_slot((uint32_t)empty, &hdr, g_json_len ? g_json : NULL);
    } else {
        // page full -> erase and seed slot 0 with newest
        ok = erase_and_seed(&hdr, g_json_len ? g_json : NULL);
    }

    if (ok) {
        g_seq++;
    }
    return ok;
}

void LIFU_Config_ResetToDefaults(void)
{
    cfg_defaults();
    (void)LIFU_Config_Save();
}

// ======================== JSON HELPERS ============================

bool LIFU_Config_SetJSON(const uint8_t* data, uint16_t len)
{
    if (len > MAX_JSON_BYTES) return false;
    if (len && data) memcpy(g_json, data, len);
    g_json_len = len;
    return true;
}

uint16_t LIFU_Config_GetJSON(uint8_t* out, uint16_t maxlen)
{
    uint16_t n = (g_json_len < maxlen) ? g_json_len : maxlen;
    if (n && out) memcpy(out, g_json, n);
    return n;
}

const uint8_t* LIFU_Config_GetJSONPtr(uint16_t* len)
{
    if (len) *len = g_json_len;
    return g_json_len ? g_json : NULL;
}

// ======================== OPTIONAL: DEBOUNCED SAVE ================

void LIFU_Config_MarkDirty(void)
{
    s_dirty = true;
    // HAL_GetTick is available on STM32 HAL
    s_last_change_ms = HAL_GetTick();
}

void LIFU_Config_Service(void)
{
    if (!s_dirty) return;
    if ((HAL_GetTick() - s_last_change_ms) >= 250U) {
        (void)LIFU_Config_Save();
        s_dirty = false;
    }
}
