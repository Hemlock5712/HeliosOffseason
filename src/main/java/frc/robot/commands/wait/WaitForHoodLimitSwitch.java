// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.wait;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

public class WaitForHoodLimitSwitch extends CommandBase {
  Shooter m_shooter;

  public WaitForHoodLimitSwitch(Shooter shooter) {
    m_shooter = shooter;
  }

  @Override
  public boolean isFinished() {
    return m_shooter.isHoodFrontSwitchTriggered();
  }
}
