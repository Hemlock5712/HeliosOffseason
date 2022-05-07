// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.*;
import frc.robot.commands.autonomous.util.*;
import frc.robot.subsystems.*;

public class SystemCheck extends AutoBaseCommand {

  public SystemCheck(Drivetrain drivetrain, Magazine magazine, Shooter shooter, Intake intake, Climber climber,
      Limelight limelight, Turret turret) {

    super(drivetrain, shooter, intake, magazine, climber, limelight, turret);

    addCommands(
        new FieldDrive(drivetrain, () -> 0.5, () -> 0, () -> 0).withTimeout(1),
        new FieldDrive(drivetrain, () -> 0, () -> 0.5, () -> 0).withTimeout(1),
        new FieldDrive(drivetrain, () -> -0.5, () -> 0, () -> 0).withTimeout(1),
        new FieldDrive(drivetrain, () -> 0, () -> -0.5, () -> 0).withTimeout(1),
        new DefenseMode(drivetrain).withTimeout(1),
        new IntakeCargo(intake, magazine).withTimeout(5),
        new WaitCommand(0.5),
        new MagazineSpitCargo(magazine).withTimeout(3),
        new ResetHoodAngle(shooter),
        new WaitCommand(1),
        new SystemCheckHoodAngle(shooter, () -> 1),
        new WaitCommand(0.5),
        new SystemCheckHoodAngle(shooter, () -> 2),
        new WaitCommand(0.5),
        new SystemCheckHoodAngle(shooter, () -> 3),
        new WaitCommand(0.5),
        new SystemCheckHoodAngle(shooter, () -> 4),
        new WaitCommand(0.5),
        new SystemCheckHoodAngle(shooter, () -> 5),
        new WaitCommand(0.5),
        new SystemCheckHoodAngle(shooter, () -> 6),
        new WaitCommand(0.5),
        new SystemCheckHoodAngle(shooter, () -> 7),
        new WaitCommand(0.5),
        new SystemCheckHoodAngle(shooter, () -> 8),
        new WaitCommand(0.5),
        new SystemCheckHoodAngle(shooter, () -> 9),
        new WaitCommand(0.5),
        new SystemCheckHoodAngle(shooter, () -> 0),
        new SystemCheckShooterSpeed(shooter, () -> 0),
        new WaitCommand(0.5),
        new SystemCheckShooterSpeed(shooter, () -> 1000),
        new WaitCommand(0.5),
        new SystemCheckShooterSpeed(shooter, () -> 5000),
        new WaitCommand(0.5),
        new SystemCheckShooterSpeed(shooter, () -> 10000),
        new WaitCommand(0.5),
        new SystemCheckShooterSpeed(shooter, () -> 15000),
        new WaitCommand(0.5),
        new SystemCheckShooterSpeed(shooter, () -> 10000),
        new WaitCommand(0.5),
        new SystemCheckShooterSpeed(shooter, () -> 5000),
        new WaitCommand(0.5),
        new SystemCheckShooterSpeed(shooter, () -> 1000),
        new WaitCommand(0.5),
        new SystemCheckShooterSpeed(shooter, () -> 0),
        new ResetClimberArms(climber),
        new WaitCommand(1),
        new ClimberBelowBar(climber),
        new WaitCommand(1),
        new ClimberAboveBar(climber),
        new WaitCommand(1),
        new ClimberToBottom(climber),
        new ClimberArmsOut(climber, turret),
        // TODO: ADD TURRET AIMING HERE
        new ClimberArmsIn(climber, turret));
  }

  @Override
  protected void generatePaths() {

  }
}
