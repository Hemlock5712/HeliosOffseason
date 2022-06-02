// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.util.ArmsOut;
import frc.robot.subsystems.Climber;

public class ClimberArmsOut extends SequentialCommandGroup {

  public ClimberArmsOut(Climber climber) {

    addCommands(new ArmsOut(climber));
  }
}
