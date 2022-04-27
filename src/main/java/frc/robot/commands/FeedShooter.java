// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Magazine;

public class FeedShooter extends CommandBase {

  private final Magazine m_magazine;

  public FeedShooter(Magazine magazine) {
    m_magazine = magazine;
    addRequirements(magazine);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    m_magazine.runUpperMagazine(.8);
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return !m_magazine.ballInUpper();
  }
}
