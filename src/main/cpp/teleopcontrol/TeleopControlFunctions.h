
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

// C++ Includes

// FRC includes

// Team 302 includes

class TeleopControlFunctions
{
public:
    enum FUNCTION
    {
        READY,
        ROBOT_ORIENTED_DRIVE,
        HOLONOMIC_DRIVE_FORWARD,
        HOLONOMIC_DRIVE_ROTATE,
        HOLONOMIC_DRIVE_STRAFE,
        AUTO_TURN_FORWARD,
        AUTO_TURN_BACKWARD,
        AUTO_ALIGN_LEFT,
        AUTO_ALIGN_RIGHT,
        AUTO_ALIGN_HUMAN_PLAYER_STATION,
        AUTO_ALIGN_BARGE,
        AUTO_ALIGN_PROCESSOR,
        AUTO_ALIGN_ALGAE,
        RESET_POSITION,
        FACE_REEF,
        SLOW_MODE,
        MANUAL_CLIMB_UP,
        MANUAL_CLIMB_DOWN,
        SWITCH_DESIRED_CORAL_SIDE,
        SYSID_MODIFER,

        // tip correction controls
        TIPCORRECTION_TOGGLE,

        MANUAL_LAUNCH_INC,
        MANUAL_LAUNCH_DEC,
        CLIMB_MODE,
        AUTO_CLIMB,
        ELAVATOR,
        ARM,
        L1_SCORING_POSITION,
        L2_SCORING_POSITION,
        L3_SCORING_POSITION,
        L4_SCORING_POSITION,
        SCORE,
        SCORING_MODE,
        HUMAN_PLAYER_STATION,
        ALGAE_INTAKE,
        EXPEL,
        MANUAL_ON,
        MANUAL_OFF,
        MANUAL_IN,
        MANUAL_OUT,
        ALGAE_HIGH,
        ALGAE_LOW,
        GRAB_ALGAE_REEF,
        FORCE_ELEVATOR

    };
};
