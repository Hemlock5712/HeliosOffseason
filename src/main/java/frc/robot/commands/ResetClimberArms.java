// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.util.ManualClimberControl;
import frc.robot.commands.wait.WaitForClimberLowerLimitSwitch;
import frc.robot.subsystems.Climber;

public class ResetClimberArms extends SequentialCommandGroup {

  public ResetClimberArms(Climber climber) {

    addCommands(new ManualClimberControl(climber, () -> -0.2).raceWith(new WaitForClimberLowerLimitSwitch(climber)),
        new InstantCommand(() -> climber.resetEncoders()));
  }
}
