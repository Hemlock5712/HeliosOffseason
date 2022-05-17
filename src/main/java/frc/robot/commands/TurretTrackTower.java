// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Turret;

public class TurretTrackTower extends CommandBase {
  Turret m_turret;

  public TurretTrackTower(Turret turret) {
    m_turret = turret;

    addRequirements(turret);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    double angle = m_turret.getHubAngle();
    if (!m_turret.canTurretRotatePastArms()) {
      angle = 0;
    }
    m_turret.setAngle(angle);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
