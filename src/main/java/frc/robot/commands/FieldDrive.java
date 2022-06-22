// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class FieldDrive extends CommandBase {

  private final Drivetrain m_drivetrain;
  private final DoubleSupplier m_forward;
  private final DoubleSupplier m_strafe;
  private final DoubleSupplier m_rotation;

  public FieldDrive(Drivetrain drivetrain, DoubleSupplier forward, DoubleSupplier strafe, DoubleSupplier rotation) {
    m_drivetrain = drivetrain;

    m_forward = forward;
    m_strafe = strafe;
    m_rotation = rotation;

    addRequirements(drivetrain);
  }

  @Override
  public void execute() {

    m_drivetrain.drive(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            -m_strafe.getAsDouble() * Constants.Swerve.MAX_VELOCITY_METERS,
            m_forward.getAsDouble() * Constants.Swerve.MAX_VELOCITY_METERS,
            m_rotation.getAsDouble() * Constants.Swerve.MAX_ANG_ACCEL,
            m_drivetrain.getGyroscopeRotation()));
  }

  @Override
  public void end(boolean interrupted) {
    m_drivetrain.stopModules();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
