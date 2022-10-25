// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.util;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Turret;

public class TurretAlignToFrontOrBack extends CommandBase {

  Turret turret;

  public TurretAlignToFrontOrBack(Turret turret) {
    this.turret = turret;
    addRequirements(turret);
  }

  @Override
  public void initialize() {
    if (turret.isFront()) {
      turret.setAngle(0);
    } else {
      turret.setAngle(180);
    }
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
    return turret.isBack();
  }
}
