//====================================================================================================================================================
// Copyright 2025 Lake Orion Robotics FIRST Team 302
//
// Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"),
// to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
// and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
// MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
// DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE
// OR OTHER DEALINGS IN THE SOFTWARE.
//====================================================================================================================================================

#pragma once

class ChassisOptionEnums
{
public:
    enum HeadingOption
    {
        MAINTAIN,
        SPECIFIED_ANGLE,
        FACE_GAME_PIECE,
        FACE_REEF_CENTER,
        FACE_REEF_FACE,
        FACE_CORAL_STATION,
        FACE_BARGE,
        IGNORE
    };

    enum DriveStateType
    {
        ROBOT_DRIVE,
        FIELD_DRIVE,
        TRAJECTORY_DRIVE,
        HOLD_DRIVE,
        POLAR_DRIVE,
        DRIVE_TO_CORAL_STATION,
        DRIVE_TO_LEFT_REEF_BRANCH,
        DRIVE_TO_RIGHT_REEF_BRANCH,
        DRIVE_TO_LEFT_CAGE,
        DRIVE_TO_RIGHT_CAGE,
        DRIVE_TO_CENTER_CAGE,
        DRIVE_TO_BARGE,
        DRIVE_TO_ALGAE,
        DRIVE_TO_PROCESSOR,
        STOP_DRIVE
    };

    enum NoMovementOption
    {
        STOP,
        HOLD_POSITION
    };

    enum AutonControllerType
    {
        RAMSETE,
        HOLONOMIC
    };

    enum AutonChassisOptions
    {
        VISION_DRIVE_SPEAKER,
        NO_VISION
    };
    enum AutonAvoidOptions
    {
        PODIUM,
        ROBOT_COLLISION,
        NO_AVOID_OPTION
    };

    enum PathUpdateOption
    {
        NONE
    };

    ChassisOptionEnums() = delete;
    ~ChassisOptionEnums() = delete;
};