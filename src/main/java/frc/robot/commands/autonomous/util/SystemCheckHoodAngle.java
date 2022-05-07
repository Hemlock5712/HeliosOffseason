// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous.util;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

public class SystemCheckHoodAngle extends CommandBase {

  Shooter m_shooter;
  DoubleSupplier m_angle;

  public SystemCheckHoodAngle(Shooter shooter, DoubleSupplier angle) {

    m_shooter = shooter;
    m_angle = angle;

  }

  @Override
  public void execute() {
    m_shooter.setHoodAngle(m_angle.getAsDouble());
  }

  @Override
  public boolean isFinished() {
    return m_shooter.isHoodAtAngle();
  }
}
