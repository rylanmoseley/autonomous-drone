#pragma once
#include "common_types.h"

#pragma pack(push, 1)

typedef enum {
    LOG_ERR_NONE            = 0,
    LOG_ERR_SD_FULL         = (1 << 0), // Disk free space < 100 MB
    LOG_ERR_WRITE_FAILED    = (1 << 1), // Block write failed / I/O stall
    LOG_ERR_BUFFER_OVERRUN  = (1 << 2)  // Lock-free queue 100% full (dropped packets)
} LoggerErrorFlags;

typedef struct {
    uint8_t  msg_type;           // Always MSG_TYPE_LOGGER_STATUS (5)
    uint64_t timestamp_ns;
    uint32_t error_state;        // Bitmask of LoggerErrorFlags
    float    free_storage_mb;    // Remaining space on micro-SD card
    float    buffer_fill_pct;    // Utilization of memory ring buffer (0.0 to 100.0%)
} LoggerStatusPacket;

#pragma pack(pop)
