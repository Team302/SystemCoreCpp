#pragma once

#include <string>
#include "vision/DragonVisionStructs.h"
#include "frc/geometry/Translation3d.h"
#include "frc/geometry/Transform3d.h"

class DragonVisionUtils
{
public:
    static bool CompareVisionData(std::optional<VisionData> data1, std::optional<VisionData> data2);
};