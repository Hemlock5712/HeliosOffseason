// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Magazine;
import frc.robot.subsystems.Shooter;

//, double shootOverride
public class LimelightShootForce extends ParallelCommandGroup {

  public LimelightShootForce(Shooter shooter, Magazine magazine, Limelight limelight) {
    addCommands(
        new FlywheelLimelight(shooter, limelight),
        new MagazineForceCargo(magazine)
    );
  }
}
