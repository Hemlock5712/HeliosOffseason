// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

public class ClimberBelowBar extends CommandBase {

  Climber m_climber;
  private static int setpoint = 220000;

  public ClimberBelowBar(Climber climber) {
    m_climber = climber;
    addRequirements(climber);
  }

  @Override
  public void execute() {
    m_climber.setSetpoint(setpoint);
  }

  @Override
  public boolean isFinished() {
    return Math.abs(m_climber.getMeasurement() - setpoint) < 200 || m_climber.getLimitSwitchUp();
  }
}
