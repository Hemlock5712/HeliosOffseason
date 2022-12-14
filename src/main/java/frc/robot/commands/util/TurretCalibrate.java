// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.util;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ClimberArmsOut;
import frc.robot.commands.wait.WaitForTurretHallEffectSensor;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Turret;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TurretCalibrate extends SequentialCommandGroup {
  /** Creates a new TurretCalibrate. */
  public TurretCalibrate(Climber climber, Turret turret) {
    // Add the deadline command in the super() call. Add other commands using
    // addCommands().
    super(
        new ClimberArmsOut(climber, turret),
        new ParallelDeadlineGroup(
            new WaitForTurretHallEffectSensor(turret),
            new InstantCommand(() -> turret.setAngle(turret.getTargetAngle() - .01))),
        new InstantCommand(() -> {
          turret.resetTurretAngle(0);
          turret.setAngle(0);
        }));
    // addCommands(new FooCommand(), new BarCommand());
  }
}
