#pragma once

#include <cstdint>


//拿着的物块 （待定）
enum class ItemType : uint8_t
{
    Earth,
    Sky,
    Mustika,
};


namespace COMDEF
{

    constexpr const char* GetItemTypeName(ItemType item)
    {
        switch(item)
        {
            case ItemType::Earth : return "Earth";
            case ItemType::Sky : return "Sky";
            case ItemType::Mustika : return "Mustika";
        }
        return "Unknown";
    }

}
