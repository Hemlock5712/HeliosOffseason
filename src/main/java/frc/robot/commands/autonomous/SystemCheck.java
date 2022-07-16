// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.*;
import frc.robot.commands.autonomous.util.*;
import frc.robot.commands.wait.WaitForClimberLowerLimitSwitch;
import frc.robot.commands.wait.WaitForClimberUpperLimitSwitch;
import frc.robot.subsystems.*;

public class SystemCheck extends AutoBaseCommand {

  public SystemCheck(Drivetrain drivetrain, Magazine magazine, Shooter shooter, Intake intake, Climber climber,
      Limelight limelight) {

    super(drivetrain, shooter, intake, magazine, climber, limelight);

    addCommands(
        new FieldDrive(drivetrain, () -> 0.5, () -> 0, () -> 0).withTimeout(1),
        new FieldDrive(drivetrain, () -> 0, () -> 0.5, () -> 0).withTimeout(1),
        new FieldDrive(drivetrain, () -> -0.5, () -> 0, () -> 0).withTimeout(1),
        new FieldDrive(drivetrain, () -> 0, () -> -0.5, () -> 0).withTimeout(1),
        new DefenseMode(drivetrain).withTimeout(1),
        new IntakeCargo(intake, magazine).withTimeout(2),
        new WaitCommand(0.5),
        new MagazineSpitCargo(magazine).withTimeout(2),
        new ResetHoodAngle(shooter),
        new WaitCommand(1),
        new PrintCommand("Start shooting"),
        new ManualShoot(shooter, magazine, () -> 3000, () -> -5).withTimeout(1),
        new ManualShoot(shooter, magazine, () -> 6700, () -> -5).withTimeout(1),
        new ManualShoot(shooter, magazine, () -> 7000, () -> -7).withTimeout(1),
        new ManualShoot(shooter, magazine, () -> 7500, () -> -4).withTimeout(1),
        new ManualShoot(shooter, magazine, () -> 8500, () -> -20).withTimeout(1),
        new ManualShoot(shooter, magazine, () -> 0, () -> 0).withTimeout(1),
        new PrintCommand("End shooting"),
        new ParallelDeadlineGroup(new WaitForClimberUpperLimitSwitch(climber), new ClimberUp(climber)),
        new WaitCommand(1),
        new ParallelDeadlineGroup(new WaitForClimberLowerLimitSwitch(climber), new ClimberDown(climber)),
        new ClimberArmsOut(climber),
        new WaitCommand(2),
        new ClimberArmsIn(climber));
  }

  @Override
  protected void generatePaths() {

  }
}
