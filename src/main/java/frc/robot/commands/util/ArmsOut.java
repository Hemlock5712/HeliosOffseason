// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.util;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

public class ArmsOut extends CommandBase {

    Climber m_climber;

    public ArmsOut(Climber climber) {
        m_climber = climber;
    }

    @Override
    public void initialize() {
        m_climber.setArmsOut(true);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
