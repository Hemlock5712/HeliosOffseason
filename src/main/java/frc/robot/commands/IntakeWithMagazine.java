// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Magazine;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class IntakeWithMagazine extends ParallelDeadlineGroup {
  /** Creates a new IntakeWithMagazine. */
  public IntakeWithMagazine(Intake intake, Magazine magazine) {
    super(
        new IntakeCargo(intake, magazine),
        new MagazineAutoBump(magazine));
  }
}
