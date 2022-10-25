// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.wait;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Turret;

public class WaitForTurretHallEffectSensor extends CommandBase {
  private final Turret turret;

  public WaitForTurretHallEffectSensor(Turret turret) {
    this.turret = turret;
  }

  @Override
  public boolean isFinished() {
    return turret.isMagnetTriggered();
  }
}
