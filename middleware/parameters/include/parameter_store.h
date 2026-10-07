#ifndef PARAMETER_STORE_H
#define PARAMETER_STORE_H

#include "parameter_profile.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Two independent 256-byte slots fit in the AT24C08N's 1024-byte capacity.
 * Bytes 512..1023 are reserved, untouched by this record format. */
#define PARAMETER_STORE_SLOT_SIZE 256U
#define PARAMETER_STORE_SLOT_COUNT 2U
#define PARAMETER_STORE_CAPACITY (PARAMETER_STORE_SLOT_SIZE * PARAMETER_STORE_SLOT_COUNT)
#define PARAMETER_STORE_SCHEMA 1U
#define PARAMETER_STORE_HEADER_SIZE 16U
#define PARAMETER_STORE_COMMIT_OFFSET (PARAMETER_STORE_SLOT_SIZE - 1U)

typedef bool (*parameter_store_read_fn)(uint16_t address, uint8_t *data,
                                         uint16_t length, void *context);
typedef bool (*parameter_store_write_fn)(uint16_t address, const uint8_t *data,
                                          uint16_t length, void *context);

typedef enum {
    PARAMETER_STORE_OK = 0,
    PARAMETER_STORE_UNCHANGED,
    PARAMETER_STORE_EMPTY,
    PARAMETER_STORE_INVALID_RECORD,
    PARAMETER_STORE_IO_ERROR,
    PARAMETER_STORE_VERIFY_ERROR,
    PARAMETER_STORE_INVALID_PROFILE,
    PARAMETER_STORE_BAD_ARGUMENT
} parameter_store_result_t;

typedef struct {
    parameter_store_read_fn read;
    parameter_store_write_fn write;
    void *context;
    balance_controller_config_t board_defaults;
    bool has_record;
    uint8_t active_slot;
    uint32_t sequence;
    parameter_store_result_t last_result;
} parameter_store_t;

/* Synchronous single-owner API. Run only in a storage worker, never a control
 * callback. Callbacks must finish the physical write cycle before returning.
 * I/O and invalid-record errors are storage status, not motion-fatal faults. */
bool parameter_store_init(parameter_store_t *store, parameter_store_read_fn read,
                          parameter_store_write_fn write, void *context,
                          const balance_controller_config_t *board_defaults);
/* On every non-OK result the caller's RAM profile is untouched. */
parameter_store_result_t parameter_store_load(parameter_store_t *store,
                                              parameter_profile_t *profile);
/* Explicit SAVE only. The previous valid slot is never overwritten. Invalidates
 * the other slot, writes/verifies its body, then commits with the final byte.
 * Unchanged encoded profiles consume no EEPROM write cycles. After any failure
 * a later operation rescans storage rather than trusting a cached active slot. */
parameter_store_result_t parameter_store_save(parameter_store_t *store,
                                              const parameter_profile_t *profile);

#ifdef __cplusplus
}
#endif
#endif
