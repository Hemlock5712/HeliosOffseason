// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.util;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Turret;

public class TurretAngleSet extends CommandBase {

  Turret m_turret;
  DoubleSupplier m_angle;

  public TurretAngleSet(Turret turret, DoubleSupplier angle) {
    m_turret = turret;
    m_angle = angle;
    addRequirements(turret);
  }

  @Override
  public void execute() {
    // If turret can't rotate past the arms, lock the turret to forward
    double angle = getAngle();
    m_turret.setAngle(angle);
  }

  private double getAngle() {
    return m_turret.canTurretRotatePastArms() ? m_angle.getAsDouble() : 0;
  }

  @Override
  public boolean isFinished() {
    double angle = getAngle();
    return Math.abs(m_turret.getAngle() - angle) < 0.5;
  }
}
