// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.util;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Turret;

public class TurretAlignToFront extends CommandBase {

  Turret turret;

  public TurretAlignToFront(Turret turret) {
    this.turret = turret;
    addRequirements(turret);
  }

  @Override
  public void initialize() {
    turret.setAngle(0);
  }

  @Override
  public void execute() {
  }

  @Override
  public void end(boolean interrupted) {
    turret.setActiveTargeting(false);
  }

  @Override
  public boolean isFinished() {
    return turret.isFront();
  }
}
