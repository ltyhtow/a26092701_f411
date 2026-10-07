#include "parameter_store.h"

#include <string.h>

#define RECORD_MAGIC UINT32_C(0x31524D50) /* Bytes: PMR1 */
#define RECORD_COMMIT UINT8_C(0xA5)
#define RECORD_BODY_SIZE (PARAMETER_STORE_HEADER_SIZE + PARAMETER_PROFILE_ENCODED_SIZE)

typedef struct {
    uint8_t body[RECORD_BODY_SIZE];
    bool valid;
    bool blank;
    uint32_t sequence;
} record_t;

static uint16_t get_u16(const uint8_t *p)
{
    return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8U));
}

static uint32_t get_u32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8U) |
        ((uint32_t)p[2] << 16U) | ((uint32_t)p[3] << 24U);
}

static void put_u16(uint8_t *p, uint16_t value)
{
    p[0] = (uint8_t)value;
    p[1] = (uint8_t)(value >> 8U);
}

static void put_u32(uint8_t *p, uint32_t value)
{
    p[0] = (uint8_t)value;
    p[1] = (uint8_t)(value >> 8U);
    p[2] = (uint8_t)(value >> 16U);
    p[3] = (uint8_t)(value >> 24U);
}

/* CRC-32/ISO-HDLC: reflected 0xEDB88320, init/xorout 0xFFFFFFFF.
 * Protect metadata bytes 0..11 and the payload; CRC bytes are excluded.
 * This small record checksum is original application code, not a chip driver. */
static uint32_t record_crc(const uint8_t *body)
{
    uint32_t crc = UINT32_C(0xFFFFFFFF);
    for (size_t i = 0; i < RECORD_BODY_SIZE; ++i) {
        if (i >= 12U && i < PARAMETER_STORE_HEADER_SIZE) {
            continue;
        }
        crc ^= body[i];
        for (unsigned bit = 0; bit < 8U; ++bit) {
            crc = (crc >> 1U) ^ ((crc & 1U) != 0U ? UINT32_C(0xEDB88320) : 0U);
        }
    }
    return crc ^ UINT32_C(0xFFFFFFFF);
}

static bool store_ready(const parameter_store_t *store)
{
    return store != NULL && store->read != NULL && store->write != NULL;
}

static parameter_store_result_t finish(parameter_store_t *store,
                                       parameter_store_result_t result)
{
    if (store != NULL) {
        store->last_result = result;
    }
    return result;
}

static bool read_record(parameter_store_t *store, uint8_t slot, record_t *record)
{
    const uint16_t base = (uint16_t)(slot * PARAMETER_STORE_SLOT_SIZE);
    uint8_t commit = 0;
    memset(record, 0, sizeof(*record));
    if (!store->read(base, record->body, (uint16_t)sizeof(record->body), store->context) ||
        !store->read((uint16_t)(base + PARAMETER_STORE_COMMIT_OFFSET),
                     &commit, 1U, store->context)) {
        return false;
    }
    record->blank = commit == UINT8_MAX;
    for (size_t i = 0; i < sizeof(record->body); ++i) {
        record->blank = record->blank && record->body[i] == UINT8_MAX;
    }
    if (commit != RECORD_COMMIT || get_u32(record->body) != RECORD_MAGIC ||
        get_u16(record->body + 4U) != PARAMETER_STORE_SCHEMA ||
        get_u16(record->body + 6U) != PARAMETER_PROFILE_ENCODED_SIZE ||
        get_u32(record->body + 12U) != record_crc(record->body)) {
        return true;
    }
    parameter_profile_t decoded;
    record->valid = parameter_profile_decode(&decoded, &store->board_defaults,
        record->body + PARAMETER_STORE_HEADER_SIZE, PARAMETER_PROFILE_ENCODED_SIZE);
    record->sequence = get_u32(record->body + 8U);
    return true;
}

static bool sequence_newer(uint32_t left, uint32_t right)
{
    const uint32_t difference = left - right;
    return difference != 0U && difference < UINT32_C(0x80000000);
}

static parameter_store_result_t scan(parameter_store_t *store, record_t records[2])
{
    store->has_record = false;
    if (!read_record(store, 0U, &records[0]) || !read_record(store, 1U, &records[1])) {
        return PARAMETER_STORE_IO_ERROR;
    }
    if (!records[0].valid && !records[1].valid) {
        return records[0].blank && records[1].blank ?
            PARAMETER_STORE_EMPTY : PARAMETER_STORE_INVALID_RECORD;
    }
    const uint8_t active = (uint8_t)(!records[0].valid ||
        (records[1].valid && sequence_newer(records[1].sequence, records[0].sequence)));
    store->has_record = true;
    store->active_slot = active;
    store->sequence = records[active].sequence;
    return PARAMETER_STORE_OK;
}

bool parameter_store_init(parameter_store_t *store, parameter_store_read_fn read,
                          parameter_store_write_fn write, void *context,
                          const balance_controller_config_t *board_defaults)
{
    if (store == NULL) {
        return false;
    }
    memset(store, 0, sizeof(*store));
    parameter_profile_t defaults;
    parameter_profile_defaults(&defaults, board_defaults);
    if (read == NULL || write == NULL || !parameter_profile_validate(&defaults)) {
        store->last_result = PARAMETER_STORE_BAD_ARGUMENT;
        return false;
    }
    store->read = read;
    store->write = write;
    store->context = context;
    store->board_defaults = defaults.controller;
    store->last_result = PARAMETER_STORE_EMPTY;
    return true;
}

parameter_store_result_t parameter_store_load(parameter_store_t *store,
                                              parameter_profile_t *profile)
{
    if (!store_ready(store) || profile == NULL) {
        return finish(store, PARAMETER_STORE_BAD_ARGUMENT);
    }
    record_t records[2];
    const parameter_store_result_t result = scan(store, records);
    if (result == PARAMETER_STORE_OK) {
        (void)parameter_profile_decode(profile, &store->board_defaults,
            records[store->active_slot].body + PARAMETER_STORE_HEADER_SIZE,
            PARAMETER_PROFILE_ENCODED_SIZE);
    }
    return finish(store, result);
}

parameter_store_result_t parameter_store_save(parameter_store_t *store,
                                              const parameter_profile_t *profile)
{
    if (!store_ready(store) || profile == NULL) {
        return finish(store, PARAMETER_STORE_BAD_ARGUMENT);
    }
    uint8_t body[RECORD_BODY_SIZE] = {0};
    if (!parameter_profile_encode(profile, body + PARAMETER_STORE_HEADER_SIZE,
                                  PARAMETER_PROFILE_ENCODED_SIZE)) {
        return finish(store, PARAMETER_STORE_INVALID_PROFILE);
    }
    parameter_profile_t decoded;
    if (profile->pwm_ceiling != store->board_defaults.max_pwm ||
        profile->controller.max_pitch_angle != store->board_defaults.max_pitch_angle ||
        profile->controller.velocity_coupling_mode != store->board_defaults.velocity_coupling_mode ||
        !parameter_profile_decode(&decoded, &store->board_defaults,
            body + PARAMETER_STORE_HEADER_SIZE, PARAMETER_PROFILE_ENCODED_SIZE)) {
        return finish(store, PARAMETER_STORE_INVALID_PROFILE);
    }
    record_t records[2];
    const parameter_store_result_t scanned = scan(store, records);
    if (scanned == PARAMETER_STORE_IO_ERROR) {
        return finish(store, scanned);
    }
    if (store->has_record && memcmp(records[store->active_slot].body + PARAMETER_STORE_HEADER_SIZE,
        body + PARAMETER_STORE_HEADER_SIZE, PARAMETER_PROFILE_ENCODED_SIZE) == 0) {
        return finish(store, PARAMETER_STORE_UNCHANGED);
    }
    const uint8_t slot = store->has_record ? (uint8_t)(store->active_slot ^ 1U) : 0U;
    const uint32_t sequence = store->has_record ? store->sequence + 1U : 1U;
    const uint16_t base = (uint16_t)(slot * PARAMETER_STORE_SLOT_SIZE);
    put_u32(body, RECORD_MAGIC);
    put_u16(body + 4U, PARAMETER_STORE_SCHEMA);
    put_u16(body + 6U, PARAMETER_PROFILE_ENCODED_SIZE);
    put_u32(body + 8U, sequence);
    put_u32(body + 12U, record_crc(body));

    uint8_t marker = 0U;
    if (!store->write((uint16_t)(base + PARAMETER_STORE_COMMIT_OFFSET), &marker, 1U, store->context) ||
        !store->read((uint16_t)(base + PARAMETER_STORE_COMMIT_OFFSET), &marker, 1U, store->context)) {
        return finish(store, PARAMETER_STORE_IO_ERROR);
    }
    if (marker != 0U) {
        return finish(store, PARAMETER_STORE_VERIFY_ERROR);
    }
    if (!store->write(base, body, (uint16_t)sizeof(body), store->context)) {
        return finish(store, PARAMETER_STORE_IO_ERROR);
    }
    uint8_t verify[RECORD_BODY_SIZE];
    if (!store->read(base, verify, (uint16_t)sizeof(verify), store->context)) {
        return finish(store, PARAMETER_STORE_IO_ERROR);
    }
    if (memcmp(verify, body, sizeof(body)) != 0) {
        return finish(store, PARAMETER_STORE_VERIFY_ERROR);
    }
    marker = RECORD_COMMIT;
    if (!store->write((uint16_t)(base + PARAMETER_STORE_COMMIT_OFFSET), &marker, 1U, store->context)) {
        return finish(store, PARAMETER_STORE_IO_ERROR);
    }
    record_t committed;
    if (!read_record(store, slot, &committed)) {
        return finish(store, PARAMETER_STORE_IO_ERROR);
    }
    if (!committed.valid || memcmp(committed.body, body, sizeof(body)) != 0) {
        return finish(store, PARAMETER_STORE_VERIFY_ERROR);
    }
    store->has_record = true;
    store->active_slot = slot;
    store->sequence = sequence;
    return finish(store, PARAMETER_STORE_OK);
}
