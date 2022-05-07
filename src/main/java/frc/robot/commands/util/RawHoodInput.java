// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.util;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

public class RawHoodInput extends CommandBase {

  Shooter m_shooter;
  DoubleSupplier m_speed;

  public RawHoodInput(Shooter shooter, DoubleSupplier speed) {
    m_shooter = shooter;
    m_speed = speed;

    addRequirements(shooter);
  }

  @Override
  public void execute() {
    m_shooter.setHoodSpeed(m_speed.getAsDouble());
  }

  @Override
  public void end(boolean interrupted) {
    m_shooter.setHoodSpeed(0);
  }
}
