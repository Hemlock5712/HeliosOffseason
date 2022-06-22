// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class DefenseMode extends CommandBase {

  private final Drivetrain m_drivetrain;

  public DefenseMode(Drivetrain drivetrain) {
    m_drivetrain = drivetrain;
    addRequirements(drivetrain);
  }

  @Override
  public void execute() {
    m_drivetrain.defense();
  }

  @Override
  public void end(boolean interrupted) {
    m_drivetrain.stopModules();
  }
}
