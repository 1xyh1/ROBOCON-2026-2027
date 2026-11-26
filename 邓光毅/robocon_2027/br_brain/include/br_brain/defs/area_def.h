#pragma once

#include <cstdint>
#include "br_brain/defs/belonging_def.h"

//先暂时定为如下
enum class Area : uint8_t
{

    None,
    RetryArea,                              //BR重试区
    TransferArea,                           //交接区
    L1_North_ExclusiveBuildingArea,         //L1层北专属建造区
    L1_South_ExclusiveBuildingArea,

    L1_North_SharedBuildingArea,            //L1层北共享建造区
    L1_South_SharedBuildingArea,

    MustikaArea,                            //圣物柱
    L2_Northwest_SharedBuildingArea,        //L2层西北建造区
    L2_Southwest_SharedBuildingArea,
    L2_Northeast_SharedBuildingArea,
    L2_Southeast_SharedBuildingArea,




};






namespace COMDEF
{

    constexpr Belonging BelongOf(Area area)
        {
            switch(area)
            {
                case Area::None                           : return Belonging::Shared;
                case Area::RetryArea                      : return Belonging::Exclusive;                          
                case Area::TransferArea                   : return Belonging::Exclusive;     
        
                case Area::L1_North_ExclusiveBuildingArea : return Belonging::Exclusive;       
                case Area::L1_South_ExclusiveBuildingArea : return Belonging::Exclusive;
                case Area::L1_North_SharedBuildingArea    : return Belonging::Shared;       
                case Area::L1_South_SharedBuildingArea    : return Belonging::Shared;  
        
                case Area::MustikaArea                    : return Belonging::Shared;                            
                case Area::L2_Northwest_SharedBuildingArea: return Belonging::Shared;          
                case Area::L2_Southwest_SharedBuildingArea: return Belonging::Shared;  
                case Area::L2_Northeast_SharedBuildingArea: return Belonging::Shared;  
                case Area::L2_Southeast_SharedBuildingArea: return Belonging::Shared;  

            }

            return Belonging::Exclusive;

        }

    constexpr const char* GetAreaName(Area area)
        {
            switch(area)
            {

                case Area::None                           : return "None";
                case Area::RetryArea                      : return "RetryArea";                          
                case Area::TransferArea                   : return "TransferArea";     
            
                case Area::L1_North_ExclusiveBuildingArea : return "L1_North_ExclusiveBuildingArea";       
                case Area::L1_South_ExclusiveBuildingArea : return "L1_South_ExclusiveBuildingArea";
                case Area::L1_North_SharedBuildingArea    : return "L1_North_SharedBuildingArea";    
                case Area::L1_South_SharedBuildingArea    : return "L1_South_SharedBuildingArea";
            
                case Area::MustikaArea                    : return "MustikaArea";                         
                case Area::L2_Northwest_SharedBuildingArea: return "L2_Northwest_SharedBuildingArea";       
                case Area::L2_Southwest_SharedBuildingArea: return "L2_Southwest_SharedBuildingArea";
                case Area::L2_Northeast_SharedBuildingArea: return "L2_Northeast_SharedBuildingArea";
                case Area::L2_Southeast_SharedBuildingArea: return "L2_Southeast_SharedBuildingArea";

            }

            return "Unknown";
        }




}




