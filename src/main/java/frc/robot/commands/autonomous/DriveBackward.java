// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.IntakeCargo;
import frc.robot.commands.LimelightShoot;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Magazine;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DriveBackward extends SequentialCommandGroup {
  public DriveBackward(Shooter shooter, Drivetrain drivetrainSubsystem, Intake intake, Magazine magazine, Limelight limelight) {
  Shooter m_shooter = shooter;
  Drivetrain m_drivetrainSubsystem = drivetrainSubsystem;
 Intake  m_intake = intake;
 Magazine  m_magazine = magazine;

  // m_shooter.zeroHoodEncoder();
  // m_drivetrainSubsystem.zeroGyroscope();

  TrajectoryConfig trajectoryConfig = Constants.Auto.T_CONFIG;

  Trajectory trajectory1 = TrajectoryGenerator.generateTrajectory(
          new Pose2d(0, 0, new Rotation2d(270)),
          List.of(
                  new Translation2d(0, -.2),
                  new Translation2d(0, -.4)),
          new Pose2d(0, -.6, Rotation2d.fromDegrees(270)),
          trajectoryConfig);

  PIDController xController = Constants.Auto.X_PID_CONTROLLER;
  PIDController yController = Constants.Auto.Y_PID_CONTROLLER;
  ProfiledPIDController thetaController = Constants.Auto.ROT_PID_CONTROLLER;
  thetaController.enableContinuousInput(-Math.PI, Math.PI);

  SwerveControllerCommand swerveControllerCommand1 = new SwerveControllerCommand(
          trajectory1,
          m_drivetrainSubsystem::getPose2d,
          m_drivetrainSubsystem.getKinematics(),
          xController,
          yController,
          thetaController,
          m_drivetrainSubsystem::setAllStates,
          m_drivetrainSubsystem);

  // Add your commands in the addCommands() call, e.g.
  // addCommands(new FooCommand(), new BarCommand());
  // addCommands(
  //         new AutoPickUpBall(m_intake, m_magazine, m_shooter, 11500, -4).raceWith(
  //                 new SequentialCommandGroup(
  //                         new InstantCommand(
  //                                 () -> m_drivetrainSubsystem.resetOdometry(trajectory1.getInitialPose())),
  //                         swerveControllerCommand1,
  //                         new InstantCommand(() -> m_drivetrainSubsystem.stopModules()),
  //                         new AutoShootCommand(m_magazine, m_shooter, 11500),
  //                         new WaitCommand(2),
  //                         new AutoShootCommand(m_magazine, m_shooter, 11500),
  //                         new WaitCommand(1),
  //                         new AutoShootCommand(m_magazine, m_shooter, 11500))));

  addCommands(
    new ParallelCommandGroup(
      new IntakeCargo(m_intake, m_magazine),
      new SequentialCommandGroup(
        new InstantCommand(() -> {
          m_drivetrainSubsystem.setGyroscope(270);
          m_drivetrainSubsystem.resetOdometry(trajectory1.getInitialPose());
        }),
        // new PrintCommand("Reset odometry"),
        swerveControllerCommand1,
        new InstantCommand(() -> {
          m_drivetrainSubsystem.stopModules();
        }),
        new LimelightShoot(shooter, magazine, limelight),
        new LimelightShoot(shooter, magazine, limelight)
      )
    )
  );

}
}
