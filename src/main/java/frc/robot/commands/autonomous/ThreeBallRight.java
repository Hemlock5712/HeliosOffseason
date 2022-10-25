// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.IntakeCargo;
import frc.robot.commands.LimelightAim;
import frc.robot.commands.LimelightShoot;
import frc.robot.commands.MagazineAutoBump;
import frc.robot.commands.ManualShoot;
import frc.robot.commands.autonomous.util.AutoBaseCommand;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Magazine;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;

/** Add your docs here. */
public class ThreeBallRight extends AutoBaseCommand {
	PPSwerveControllerCommand first2Cargo;
	PPSwerveControllerCommand lastCargo;

	public ThreeBallRight(Drivetrain drivetrain, Shooter shooter, Intake intake, Magazine magazine, Climber climber,
			Turret turret, Limelight limelight) {
		super(drivetrain, shooter, intake, magazine, climber, turret, limelight);

		addCommands(
				new InstantCommand(() -> {
					drivetrain.setGyroscope(90);
					drivetrain.resetOdometry(new Pose2d(7.66, 1.86, Rotation2d.fromDegrees(90)));
				}),
				new ParallelDeadlineGroup(
						new SequentialCommandGroup(
								first2Cargo,
								new InstantCommand(() -> drivetrain.stopModules()),
								new LimelightAim(drivetrain, limelight)
										.withTimeout(1),
								new InstantCommand(() -> drivetrain.stopModules())),
						new IntakeCargo(intake, magazine),
						new MagazineAutoBump(magazine)),
				new ManualShoot(shooter, magazine, () -> 7000, () -> -7).withTimeout(1),
				new ManualShoot(shooter, magazine, () -> 7000, () -> -7).withTimeout(1),
				new ParallelDeadlineGroup(
						new SequentialCommandGroup(lastCargo, new InstantCommand(() -> drivetrain.stopModules()),
								new LimelightAim(drivetrain, limelight)
										.withTimeout(1),
								new InstantCommand(() -> drivetrain.stopModules())),
						new IntakeCargo(intake, magazine), new MagazineAutoBump(magazine)),
				new ManualShoot(shooter, magazine, () -> 7500, () -> -4.5).withTimeout(1));
	}

	protected void generatePaths() {
		PathPlannerTrajectory m_first2Cargo = PathPlanner.loadPath("Five Ball A", 5, 3);
		PathPlannerTrajectory m_lastCargo = PathPlanner.loadPath("Five Ball B", 5, 3);

		first2Cargo = new PPSwerveControllerCommand(
				m_first2Cargo,
				drivetrain::getPose2d,
				drivetrain.getKinematics(),
				m_translationController,
				m_strafeController,
				m_thetaController,
				drivetrain::setAllStates,
				drivetrain);

		lastCargo = new PPSwerveControllerCommand(
				m_lastCargo,
				drivetrain::getPose2d,
				drivetrain.getKinematics(),
				m_translationController,
				m_strafeController,
				m_thetaController,
				drivetrain::setAllStates,
				drivetrain);

	}
}
