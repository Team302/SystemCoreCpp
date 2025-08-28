
#pragma once

#include "frc2/command/button/CommandXboxController.h"
#include <gamepad/DragonXBox.h>

class DragonHybridController
{
public:
    DragonHybridController(int port);

    // Accessors for command-based functionality
    frc2::CommandXboxController *GetCommandController();

    // Accessors for non-command-based functionality
    DragonXBox *GetNonCommandController();

private:
    frc2::CommandXboxController *m_commandController;
    DragonXBox *m_nonCommandController;
};