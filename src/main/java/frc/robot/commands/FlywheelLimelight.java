// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;

public class FlywheelLimelight extends CommandBase {
  private final Shooter m_shooter;
  private final Limelight m_limelight;

  public FlywheelLimelight(Shooter shooter, Limelight limelight) {
    m_shooter = shooter;
    m_limelight = limelight;
    addRequirements(shooter);
  }

  @Override
  public void execute() {
    double[] outputVals = m_limelight.calcHoodAndRPM();
    m_shooter.setHoodAngle(outputVals[1]);
    m_shooter.runMotor(outputVals[0]);
  }
}
