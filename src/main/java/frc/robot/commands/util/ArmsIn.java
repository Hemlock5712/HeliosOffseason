// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.util;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

/**
 * Pull the climber arms in.
 * <b>THIS IS UNSAFE FOR NORMAL USAGE, IT DOESN'T ACCOUNT FOR THE TURRET</b>
 */
public class ArmsIn extends CommandBase {

  Climber m_climber;

  public ArmsIn(Climber climber) {
    m_climber = climber;
  }

  @Override
  public void initialize() {
    m_climber.setArmsOut(false);
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
