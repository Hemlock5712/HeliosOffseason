// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Magazine;

public class MagazineForceCargo extends CommandBase {
  Magazine m_magazine;

  public MagazineForceCargo(Magazine magazine) {
    m_magazine = magazine;

    addRequirements(magazine);
  }

  @Override
  public void initialize() {
    m_magazine.stop();
  }

  @Override
  public void execute() {
    // Don't forcibly cram cargo together if there's already one in
    // the lower magazine
    m_magazine.runUpperMagazine(8000);
    m_magazine.runLowerMagazine(5000);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_magazine.stop();
  }

}
