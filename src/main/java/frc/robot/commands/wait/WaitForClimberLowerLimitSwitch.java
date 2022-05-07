// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.wait;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

public class WaitForClimberLowerLimitSwitch extends CommandBase {
    Climber m_climber;

    public WaitForClimberLowerLimitSwitch(Climber climber) {
        m_climber = climber;
    }

    @Override
    public boolean isFinished() {
        return m_climber.getLimitSwitchDown();
    }
}
