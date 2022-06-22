// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.wait;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

public class WaitForShooterSpeed extends CommandBase {

  private Shooter m_shooter;

  /** Creates a new WaitForShooterSpeed. */
  public WaitForShooterSpeed(Shooter shooter) {
    m_shooter = shooter;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(m_shooter.getShooterError()) < 100;
  }
}
