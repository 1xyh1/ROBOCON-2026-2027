#pragma once

#include <cstdint>
#include "br_brain/defs/teamside_def.h"

enum class CellType : uint8_t 
{
    WALL,
    FREE,
    UNKNOWN
};

enum class StructureType : uint8_t
{
    FLAT,
    STAIR,
    RAMP
};



enum class FloorType : uint8_t
{
    NONE,
    L0,
    L1,
    L2,
};

struct MapCell
{
    CellType cell_type = CellType::UNKNOWN;

    StructureType structure_type = StructureType::FLAT;

    FloorType floor = FloorType::NONE;

    TeamSide region_type = TeamSide::PUBLIC;

};