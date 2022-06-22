// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Magazine;

public class IntakeCargo extends CommandBase {
  Intake m_intake;
  Magazine m_magazine;

  public IntakeCargo(Intake intake, Magazine magazine) {
    m_intake = intake;
    m_magazine = magazine;

    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!m_magazine.isFull()) {
      m_intake.setIntakeDown(true);
      m_intake.setIntakeSpeed(-0.8);
    } else {
      m_intake.setIntakeDown(false);
      m_intake.setIntakeSpeed(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.setIntakeDown(false);
    m_intake.setIntakeSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Automatically put intake up and turn off if magazine is full
    return m_magazine.isFull();
  }
}
