// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.wait.WaitForCargoInUpperMagazine;
import frc.robot.commands.wait.WaitForShooterSpeed;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Magazine;
import frc.robot.subsystems.Shooter;
  //, double shootOverride
public class LimelightShoot extends ParallelRaceGroup {

  public LimelightShoot(Shooter shooter, Magazine magazine, Limelight limelight) {
    addCommands(
        new FlywheelLimelight(shooter, limelight),
        new SequentialCommandGroup(
            new WaitCommand(0.25),
            new WaitForShooterSpeed(shooter).withTimeout(1.5),
            new WaitForCargoInUpperMagazine(magazine).raceWith(new MagazineAutoBump(magazine)),
            new FeedShooter(magazine)));

  }
}
