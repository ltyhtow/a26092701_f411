#include "parameter_store.h"

#include <assert.h>
#include <stdio.h>
#include <string.h>

typedef struct {
    uint8_t bytes[1024];
    int byte_budget;
    unsigned writes;
    unsigned reads;
    unsigned fail_write;
    unsigned fail_read;
    unsigned ignore_write;
    unsigned corrupt_write;
} fake_nvm_t;

static void reset_faults(fake_nvm_t *nvm)
{
    nvm->byte_budget = -1;
    nvm->writes = nvm->reads = 0U;
    nvm->fail_write = nvm->fail_read = 0U;
    nvm->ignore_write = nvm->corrupt_write = 0U;
}

static void blank(fake_nvm_t *nvm)
{
    memset(nvm, 0, sizeof(*nvm));
    memset(nvm->bytes, 0xFF, sizeof(nvm->bytes));
    memset(nvm->bytes + PARAMETER_STORE_CAPACITY, 0x6A,
           sizeof(nvm->bytes) - PARAMETER_STORE_CAPACITY);
    reset_faults(nvm);
}

static bool read_nvm(uint16_t address, uint8_t *data, uint16_t length, void *context)
{
    fake_nvm_t *nvm = context;
    assert((size_t)address + length <= PARAMETER_STORE_CAPACITY);
    if (++nvm->reads == nvm->fail_read) {
        return false;
    }
    memcpy(data, nvm->bytes + address, length);
    return true;
}

static bool write_nvm(uint16_t address, const uint8_t *data, uint16_t length, void *context)
{
    fake_nvm_t *nvm = context;
    assert((size_t)address + length <= PARAMETER_STORE_CAPACITY);
    ++nvm->writes;
    if (nvm->writes == nvm->fail_write) {
        return false;
    }
    if (nvm->writes == nvm->ignore_write) {
        return true;
    }
    for (uint16_t i = 0; i < length; ++i) {
        if (nvm->byte_budget == 0) {
            return false;
        }
        nvm->bytes[address + i] = data[i];
        if (nvm->byte_budget > 0) {
            --nvm->byte_budget;
        }
    }
    if (nvm->writes == nvm->corrupt_write) {
        nvm->bytes[address] ^= 1U;
    }
    return true;
}

static void init_store(parameter_store_t *store, fake_nvm_t *nvm)
{
    balance_controller_config_t board_defaults;
    balance_controller_default_config(&board_defaults);
    board_defaults.max_pwm = 4500.0f;
    assert(parameter_store_init(store, read_nvm, write_nvm, nvm, &board_defaults));
}

static float load_kp(fake_nvm_t *nvm, uint32_t *sequence)
{
    reset_faults(nvm);
    parameter_store_t store;
    init_store(&store, nvm);
    parameter_profile_t result;
    assert(parameter_store_load(&store, &result) == PARAMETER_STORE_OK);
    assert(!result.controller.auto_recovery_enabled);
    if (sequence != NULL) {
        *sequence = store.sequence;
    }
    return result.controller.balance_loop.kp;
}

/* Independently refresh handcrafted records, so schema/range tests exercise
 * their own validation instead of merely failing the CRC guard. */
static void refresh_crc(uint8_t *body)
{
    uint8_t covered[12U + PARAMETER_PROFILE_ENCODED_SIZE];
    memcpy(covered, body, 12U);
    memcpy(covered + 12U, body + PARAMETER_STORE_HEADER_SIZE, PARAMETER_PROFILE_ENCODED_SIZE);
    uint32_t crc = 0xFFFFFFFFU;
    for (size_t i = 0; i < sizeof(covered); ++i) {
        crc ^= covered[i];
        for (unsigned j = 0; j < 8U; ++j) {
            if ((crc & 1U) != 0U) {
                crc = (crc >> 1U) ^ 0xEDB88320U;
            } else {
                crc >>= 1U;
            }
        }
    }
    crc ^= 0xFFFFFFFFU;
    for (unsigned i = 0; i < 4U; ++i) {
        body[12U + i] = (uint8_t)(crc >> (8U * i));
    }
}

static void put_sequence(uint8_t *body, uint32_t sequence)
{
    for (unsigned i = 0; i < 4U; ++i) {
        body[8U + i] = (uint8_t)(sequence >> (8U * i));
    }
    refresh_crc(body);
}

int main(void)
{
    fake_nvm_t nvm;
    blank(&nvm);
    parameter_store_t store;
    init_store(&store, &nvm);
    parameter_profile_t profile;
    parameter_profile_defaults(&profile, &store.board_defaults);
    parameter_profile_t untouched = profile;
    assert(parameter_store_load(&store, &profile) == PARAMETER_STORE_EMPTY);
    assert(memcmp(&profile, &untouched, sizeof(profile)) == 0);
    assert(nvm.writes == 0U);
    assert(parameter_store_save(&store, &profile) == PARAMETER_STORE_OK);
    assert(store.sequence == 1U && store.active_slot == 0U && store.has_record);
    const unsigned initial_writes = nvm.writes;
    assert(parameter_store_save(&store, &profile) == PARAMETER_STORE_UNCHANGED);
    assert(nvm.writes == initial_writes);
    assert(parameter_profile_set(&profile, PARAMETER_KEY_BALANCE_KP, 181.0f) == PARAMETER_OK);
    assert(parameter_store_save(&store, &profile) == PARAMETER_STORE_OK);
    assert(store.sequence == 2U && store.active_slot == 1U);
    const fake_nvm_t baseline = nvm;
    assert(parameter_profile_set(&profile, PARAMETER_KEY_BALANCE_KP, 182.0f) == PARAMETER_OK);

    /* Cut power at every byte of invalidate + 72-byte body + commit. At most
     * one new record commits, and the previous active slot stays byte exact. */
    for (int budget = 0; budget <= 74; ++budget) {
        nvm = baseline;
        reset_faults(&nvm);
        init_store(&store, &nvm);
        nvm.byte_budget = budget;
        const parameter_store_result_t result = parameter_store_save(&store, &profile);
        assert(result == (budget < 74 ? PARAMETER_STORE_IO_ERROR : PARAMETER_STORE_OK));
        assert(memcmp(nvm.bytes + 256U, baseline.bytes + 256U, 256U) == 0);
        assert(memcmp(nvm.bytes + 512U, baseline.bytes + 512U, 512U) == 0);
        assert(load_kp(&nvm, NULL) == (budget < 74 ? 181.0f : 182.0f));
    }
    for (unsigned call = 1; call <= 3U; ++call) {
        nvm = baseline; reset_faults(&nvm); init_store(&store, &nvm);
        nvm.fail_write = call;
        assert(parameter_store_save(&store, &profile) == PARAMETER_STORE_IO_ERROR);
        assert(load_kp(&nvm, NULL) == 181.0f);
    }
    for (unsigned call = 1; call <= 8U; ++call) {
        nvm = baseline; reset_faults(&nvm); init_store(&store, &nvm);
        nvm.fail_read = call;
        assert(parameter_store_save(&store, &profile) == PARAMETER_STORE_IO_ERROR);
        /* A final read failure can occur after a successful physical commit. */
        assert(load_kp(&nvm, NULL) == (call >= 7U ? 182.0f : 181.0f));
    }
    for (unsigned call = 1; call <= 3U; ++call) {
        nvm = baseline; reset_faults(&nvm); init_store(&store, &nvm);
        nvm.ignore_write = call;
        assert(parameter_store_save(&store, &profile) == PARAMETER_STORE_VERIFY_ERROR);
        assert(load_kp(&nvm, NULL) == 181.0f);
        nvm = baseline; reset_faults(&nvm); init_store(&store, &nvm);
        nvm.corrupt_write = call;
        assert(parameter_store_save(&store, &profile) == PARAMETER_STORE_VERIFY_ERROR);
        assert(load_kp(&nvm, NULL) == 181.0f);
    }

    /* An interrupted first save cannot invent a valid configuration. */
    for (int budget = 0; budget < 74; ++budget) {
        blank(&nvm); init_store(&store, &nvm); nvm.byte_budget = budget;
        assert(parameter_store_save(&store, &profile) == PARAMETER_STORE_IO_ERROR);
        reset_faults(&nvm); init_store(&store, &nvm);
        parameter_profile_t result = untouched;
        const parameter_store_result_t loaded = parameter_store_load(&store, &result);
        assert(loaded == PARAMETER_STORE_EMPTY || loaded == PARAMETER_STORE_INVALID_RECORD);
        assert(memcmp(&result, &untouched, sizeof(result)) == 0);
        assert(parameter_store_save(&store, &profile) == PARAMETER_STORE_OK);
        assert(load_kp(&nvm, NULL) == 182.0f);
    }

    /* Corrupt active payload/header/CRC/commit: fallback remains previous record. */
    const unsigned corrupt_offsets[] = {0U, 4U, 6U, 8U, 12U, 16U, 71U, 255U};
    for (size_t i = 0; i < sizeof(corrupt_offsets) / sizeof(corrupt_offsets[0]); ++i) {
        nvm = baseline;
        nvm.bytes[256U + corrupt_offsets[i]] ^= 1U;
        assert(load_kp(&nvm, NULL) == 180.0f);
    }
    for (unsigned bad_kind = 0; bad_kind < 3U; ++bad_kind) {
        nvm = baseline;
        uint8_t *active = nvm.bytes + 256U;
        if (bad_kind == 0U) active[4] = 2U; /* unsupported schema */
        if (bad_kind == 1U) active[6] = 55U; /* wrong payload length */
        if (bad_kind == 2U) { /* max PWM = 4501, CRC still valid */
            active[68] = 0; active[69] = 0; active[70] = 0x95; active[71] = 0x11;
        }
        refresh_crc(active);
        assert(load_kp(&nvm, NULL) == 180.0f);
    }
    nvm = baseline;
    nvm.bytes[255] = 0U; nvm.bytes[511] = 0U;
    reset_faults(&nvm); init_store(&store, &nvm);
    assert(parameter_store_load(&store, &untouched) == PARAMETER_STORE_INVALID_RECORD);

    /* Wrap ordering: sequence 0 is newer than UINT32_MAX. */
    nvm = baseline;
    put_sequence(nvm.bytes, UINT32_MAX - 1U);
    put_sequence(nvm.bytes + 256U, UINT32_MAX);
    uint32_t sequence = 0;
    assert(load_kp(&nvm, &sequence) == 181.0f && sequence == UINT32_MAX);
    init_store(&store, &nvm);
    assert(parameter_store_save(&store, &profile) == PARAMETER_STORE_OK);
    assert(store.sequence == 0U);
    assert(load_kp(&nvm, &sequence) == 182.0f && sequence == 0U);

    /* No I/O for invalid profile and no hidden persistence of fixed policy. */
    reset_faults(&nvm); init_store(&store, &nvm);
    parameter_profile_t invalid = profile;
    invalid.controller.max_pwm = 5000.0f;
    assert(parameter_store_save(&store, &invalid) == PARAMETER_STORE_INVALID_PROFILE);
    invalid = profile; invalid.controller.max_pitch_angle = 40.0f;
    assert(parameter_store_save(&store, &invalid) == PARAMETER_STORE_INVALID_PROFILE);
    assert(nvm.writes == 0U && nvm.reads == 0U);
    assert(parameter_store_load(NULL, &profile) == PARAMETER_STORE_BAD_ARGUMENT);
    assert(parameter_store_save(&store, NULL) == PARAMETER_STORE_BAD_ARGUMENT);
    assert(!parameter_store_init(&store, NULL, write_nvm, &nvm, NULL));
    puts("parameter store: 149 byte-level power cuts, write/read faults, corruption, wear avoidance and sequence wrap passed");
    return 0;
}
