// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Magazine;

public class MagazineAutoBumpEndWhenFull extends CommandBase {
  private Magazine m_Magazine;

  public MagazineAutoBumpEndWhenFull(Magazine magazine) {
    m_Magazine = magazine;
    addRequirements(magazine);
  }

  @Override
  public void initialize() {
    m_Magazine.stop();
  }

  @Override
  public void execute() {
    // Run the lower magazine slowly until a ball hits the sensor
    // As soon as the sensor is tripped, move the ball up to the
    // upper magazine to stage it for shooting.
    if (m_Magazine.ballInUpper()) {
      m_Magazine.runUpperMagazine(0);
      if (m_Magazine.ballInLower()) {
        m_Magazine.runLowerMagazine(0);
      } else {
        m_Magazine.runLowerMagazine(5000);
      }
    } else {
      m_Magazine.runUpperMagazine(2500);
      m_Magazine.runLowerMagazine(7000);
    }
  }

  @Override
  public boolean isFinished() {
    return m_Magazine.isFull();
  }

  @Override
  public void end(boolean interrupted) {
    m_Magazine.stop();
  }
}
