// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.util.RawShooterInput;
import frc.robot.commands.wait.WaitForCargoInUpperMagazine;
import frc.robot.commands.wait.WaitForShooterSpeed;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Magazine;
import frc.robot.subsystems.Shooter;

//, double shootOverride
public class ManualShootIntake extends ParallelRaceGroup {

    public ManualShootIntake(Shooter shooter, Magazine magazine, Intake intake, DoubleSupplier speed,
            DoubleSupplier angle) {
        addCommands(
                new RawShooterInput(shooter, speed, angle),
                new SequentialCommandGroup(
                        new ParallelRaceGroup(
                                new SequentialCommandGroup(
                                        new WaitCommand(0.3),
                                        new WaitForShooterSpeed(shooter).withTimeout(1),
                                        new WaitForCargoInUpperMagazine(magazine)),
                                new MagazineAutoBump(magazine)),
                        new FeedShooter(magazine).alongWith(new WaitCommand(0.25))));

    }
}
