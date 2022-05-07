// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.util.RawHoodInput;
import frc.robot.commands.wait.WaitForHoodLimitSwitch;
import frc.robot.subsystems.Shooter;

public class ResetHoodAngle extends SequentialCommandGroup {

  public ResetHoodAngle(Shooter shooter) {

    addCommands(
        new RawHoodInput(shooter, () -> -0.1).raceWith(new WaitForHoodLimitSwitch(shooter)),
        new InstantCommand(() -> {
          shooter.zeroHoodEncoder();
        }));
  }
}
