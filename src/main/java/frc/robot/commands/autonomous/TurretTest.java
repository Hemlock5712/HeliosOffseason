// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import frc.robot.commands.ClimberArmsOut;
import frc.robot.commands.ManualTurretControl;
import frc.robot.commands.autonomous.util.AutoBaseCommand;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Magazine;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TurretTest extends AutoBaseCommand {
  /** Creates a new TurretTest. */
  public TurretTest(Drivetrain drivetrain, Shooter shooter, Intake intake, Magazine magazine, Climber climber,
      Turret turret, Limelight limelight) {
    super(drivetrain, shooter, intake, magazine, climber, turret, limelight);
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new ClimberArmsOut(climber, turret),
        new ManualTurretControl(turret, () -> 30).withTimeout(3),
        new ManualTurretControl(turret, () -> -30).withTimeout(3),
        new ManualTurretControl(turret, () -> 180).withTimeout(5),
        new ManualTurretControl(turret, () -> 0).withTimeout(5));
  }

  @Override
  protected void generatePaths() {

  }
}
