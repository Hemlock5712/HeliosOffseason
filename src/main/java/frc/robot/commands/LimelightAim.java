// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;

public class LimelightAim extends CommandBase {
  Limelight m_limelight;
  Drivetrain m_drivetrain;

  public LimelightAim(Drivetrain drivetrain, Limelight limelight) {
    m_drivetrain = drivetrain;
    m_limelight = limelight;
    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double angleOffset = m_limelight.getX();
    PIDController drivetrainPID = m_drivetrain.getLimelightPID();
    double output = drivetrainPID.calculate(angleOffset, 0);
    m_drivetrain.drive(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            0,0,
           -output,
            m_drivetrain.getGyroscopeRotation()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
