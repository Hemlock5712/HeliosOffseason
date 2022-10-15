// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.util.ArmsOut;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Turret;

public class ClimberArmsOut extends SequentialCommandGroup {

  public ClimberArmsOut(Climber climber, Turret turret) {

    addCommands(
        new ArmsOut(climber),
        new WaitCommand(1.5),
        new InstantCommand(() -> {
          turret.setHaveArmsGoneDownBefore(true);
          turret.setActiveTargeting(true);
        }));
  }
}
