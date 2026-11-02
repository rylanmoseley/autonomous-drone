#!/usr/bin/env python3
"""
generate_tag_map.py
Parses surveyed tag positioning from JSON at compile time and emits
an optimized, constexpr, zero-allocation C++ lookup header.
"""

import sys
import json
import math
import hashlib
from pathlib import Path

def euler_deg_to_quat(roll_deg: float, pitch_deg: float, yaw_deg: float):
    roll = math.radians(roll_deg)
    pitch = math.radians(pitch_deg)
    yaw = math.radians(yaw_deg)

    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return (w, x, y, z)

def generate_header(json_path: Path, output_header_path: Path):
    if not json_path.exists():
        print(f"Error: Surveyed tags JSON file not found at {json_path}", file=sys.stderr)
        sys.exit(1)

    raw_bytes = json_path.read_bytes()
    file_sha256 = hashlib.sha256(raw_bytes).hexdigest()[:16]

    try:
        data = json.loads(raw_bytes.decode("utf-8"))
    except json.JSONDecodeError as e:
        print(f"Error parsing JSON {json_path}: {e}", file=sys.stderr)
        sys.exit(1)

    tags = data.get("tags", [])
    arena_name = data.get("arena_name", "Default Arena")
    coord_frame = data.get("coordinate_frame", "NED")

    seen_ids = set()
    parsed_tags = []

    for idx, tag in enumerate(tags):
        tag_id = tag.get("id")
        if tag_id is None or not isinstance(tag_id, int) or tag_id < 0 or tag_id > 65535:
            print(f"Error: Tag at index {idx} has invalid id: {tag_id}", file=sys.stderr)
            sys.exit(1)
        if tag_id in seen_ids:
            print(f"Error: Duplicate tag id {tag_id} found in {json_path}", file=sys.stderr)
            sys.exit(1)
        seen_ids.add(tag_id)

        pos = tag.get("position_ned", [0.0, 0.0, 0.0])
        if len(pos) != 3:
            print(f"Error: Tag {tag_id} position_ned must have 3 components, got {pos}", file=sys.stderr)
            sys.exit(1)

        rpy = tag.get("attitude_rpy_deg", [0.0, 0.0, 0.0])
        if len(rpy) != 3:
            print(f"Error: Tag {tag_id} attitude_rpy_deg must have 3 components, got {rpy}", file=sys.stderr)
            sys.exit(1)

        size_m = float(tag.get("size_m", 0.16))
        label = str(tag.get("label", f"Tag_{tag_id}"))

        qw, qx, qy, qz = euler_deg_to_quat(float(rpy[0]), float(rpy[1]), float(rpy[2]))

        parsed_tags.append({
            "id": tag_id,
            "x": float(pos[0]),
            "y": float(pos[1]),
            "z": float(pos[2]),
            "roll_deg": float(rpy[0]),
            "pitch_deg": float(rpy[1]),
            "yaw_deg": float(rpy[2]),
            "qw": qw,
            "qx": qx,
            "qy": qy,
            "qz": qz,
            "size_m": size_m,
            "label": label
        })

    # Sort tags by ID ascending for fast O(log N) binary search lookup
    parsed_tags.sort(key=lambda t: t["id"])

    # Generate C++ Header Content
    lines = [
        "// ============================================================================",
        "// AUTO-GENERATED FILE - DO NOT EDIT DIRECTLY",
        f"// Generated from: {json_path.name} (SHA-256: {file_sha256})",
        f"// Arena: {arena_name} | Frame: {coord_frame}",
        "// Zero-allocation compile-time static lookup table for surveyed fiducial tags",
        "// ============================================================================",
        "#pragma once",
        "",
        "#include <cstdint>",
        "#include <cstddef>",
        '#include "common_types.h"',
        "",
        "#pragma pack(push, 1)",
        "",
        "struct CompiledSurveyedTagEntry {",
        "    uint16_t tag_id;",
        "    Vec3f world_pos;       // World NED coordinates (meters)",
        "    Quatf world_att;       // Precomputed World NED attitude quaternion",
        "    float size_m;          // Marker side length (meters)",
        "    const char* label;     // Informational tag description",
        "};",
        "",
        "#pragma pack(pop)",
        "",
        f"constexpr size_t COMPILED_SURVEYED_TAG_COUNT = {len(parsed_tags)};",
        f'constexpr const char* COMPILED_ARENA_NAME = "{arena_name}";',
        f'constexpr const char* COMPILED_TAG_MAP_HASH = "{file_sha256}";',
        "",
        "// Static ROM table sorted by tag_id for compile-time or branchless binary search",
        "constexpr CompiledSurveyedTagEntry COMPILED_SURVEYED_TAGS[COMPILED_SURVEYED_TAG_COUNT] = {"
    ]

    for t in parsed_tags:
        lines.append(
            f"    {{ {t['id']}, "
            f"{{{t['x']:.6f}f, {t['y']:.6f}f, {t['z']:.6f}f}}, "
            f"{{{t['qw']:.6f}f, {t['qx']:.6f}f, {t['qy']:.6f}f, {t['qz']:.6f}f}}, "
            f"{t['size_m']:.4f}f, "
            f'"{t["label"]}" }},'
        )

    lines.extend([
        "};",
        "",
        "// O(log N) branchless constexpr binary search lookup in ROM",
        "inline const CompiledSurveyedTagEntry* findCompiledSurveyedTag(uint16_t tag_id) {",
        "    size_t low = 0;",
        "    size_t high = COMPILED_SURVEYED_TAG_COUNT;",
        "    while (low < high) {",
        "        size_t mid = low + (high - low) / 2;",
        "        if (COMPILED_SURVEYED_TAGS[mid].tag_id == tag_id) {",
        "            return &COMPILED_SURVEYED_TAGS[mid];",
        "        }",
        "        if (COMPILED_SURVEYED_TAGS[mid].tag_id < tag_id) {",
        "            low = mid + 1;",
        "        } else {",
        "            high = mid;",
        "        }",
        "    }",
        "    return nullptr;",
        "}",
        "",
        "// Populate any AutopilotNode instance with all surveyed tags from ROM",
        "template <typename NodeT>",
        "inline void loadCompiledSurveyedTags(NodeT& node) {",
        "    for (size_t i = 0; i < COMPILED_SURVEYED_TAG_COUNT; ++i) {",
        "        const auto& t = COMPILED_SURVEYED_TAGS[i];",
        "        node.registerSurveyedTag(t.tag_id, t.world_pos, t.world_att, t.size_m);",
        "    }",
        "}",
        ""
    ])

    output_header_path.parent.mkdir(parents=True, exist_ok=True)
    output_content = "\n".join(lines) + "\n"

    # Write only if changed to avoid unnecessary re-compilations
    if output_header_path.exists():
        existing = output_header_path.read_text(encoding="utf-8")
        if existing == output_content:
            return

    output_header_path.write_text(output_content, encoding="utf-8")
    print(f"Generated {output_header_path} ({len(parsed_tags)} surveyed tags, SHA: {file_sha256})")

def main():
    if len(sys.argv) < 3:
        print("Usage: generate_tag_map.py <path/to/surveyed_tags.json> <path/to/output_header.h>", file=sys.stderr)
        sys.exit(1)

    json_path = Path(sys.argv[1]).resolve()
    out_path = Path(sys.argv[2]).resolve()
    generate_header(json_path, out_path)

if __name__ == "__main__":
    main()
