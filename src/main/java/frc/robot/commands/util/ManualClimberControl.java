// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.util;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

public class ManualClimberControl extends CommandBase {

    Climber m_climber;
    DoubleSupplier m_speed;

    public ManualClimberControl(Climber climber, DoubleSupplier speed) {
        m_climber = climber;
        m_speed = speed;

        addRequirements(climber);
    }

    @Override
    public void execute() {
        m_climber.runLift(m_speed.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
        m_climber.runLift(0);
    }
}
