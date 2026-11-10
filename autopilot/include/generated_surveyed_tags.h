// ============================================================================
// AUTO-GENERATED FILE - DO NOT EDIT DIRECTLY
// Generated from: surveyed_tags.json (SHA-256: 4eaa31cc6129b950)
// Arena: Competition Arena Alpha | Frame: NED
// Zero-allocation compile-time static lookup table for surveyed fiducial tags
// ============================================================================
#pragma once

#include <cstdint>
#include <cstddef>
#include "common_types.h"

#pragma pack(push, 1)

struct CompiledSurveyedTagEntry {
    uint16_t tag_id;
    Vec3f world_pos;       // World NED coordinates (meters)
    Quatf world_att;       // Precomputed World NED attitude quaternion
    float size_m;          // Marker side length (meters)
    const char* label;     // Informational tag description
};

#pragma pack(pop)

constexpr size_t COMPILED_SURVEYED_TAG_COUNT = 4;
constexpr const char* COMPILED_ARENA_NAME = "Competition Arena Alpha";
constexpr const char* COMPILED_TAG_MAP_HASH = "4eaa31cc6129b950";

// Static ROM table sorted by tag_id for compile-time or branchless binary search
constexpr CompiledSurveyedTagEntry COMPILED_SURVEYED_TAGS[COMPILED_SURVEYED_TAG_COUNT] = {
    { 1, {1.000000f, -0.800000f, 0.400000f}, {1.000000f, 0.000000f, 0.000000f, 0.000000f}, 0.1600f, "Infield Navigation Anchor" },
    { 2, {2.200000f, -2.200000f, 0.800000f}, {0.965926f, 0.000000f, 0.000000f, 0.258819f}, 0.1600f, "West Perimeter Landmark" },
    { 3, {4.200000f, -0.500000f, 1.000000f}, {0.000000f, 0.000000f, 0.000000f, 1.000000f}, 0.1600f, "Far Outfield Landmark" },
    { 4, {1.800000f, 2.200000f, 0.600000f}, {0.965926f, 0.000000f, 0.000000f, -0.258819f}, 0.2000f, "East Perimeter Landmark" },
};

// O(log N) branchless constexpr binary search lookup in ROM
inline const CompiledSurveyedTagEntry* findCompiledSurveyedTag(uint16_t tag_id) {
    size_t low = 0;
    size_t high = COMPILED_SURVEYED_TAG_COUNT;
    while (low < high) {
        size_t mid = low + (high - low) / 2;
        if (COMPILED_SURVEYED_TAGS[mid].tag_id == tag_id) {
            return &COMPILED_SURVEYED_TAGS[mid];
        }
        if (COMPILED_SURVEYED_TAGS[mid].tag_id < tag_id) {
            low = mid + 1;
        } else {
            high = mid;
        }
    }
    return nullptr;
}

// Populate any AutopilotNode instance with all surveyed tags from ROM
template <typename NodeT>
inline void loadCompiledSurveyedTags(NodeT& node) {
    for (size_t i = 0; i < COMPILED_SURVEYED_TAG_COUNT; ++i) {
        const auto& t = COMPILED_SURVEYED_TAGS[i];
        node.registerSurveyedTag(t.tag_id, t.world_pos, t.world_att, t.size_m);
    }
}

