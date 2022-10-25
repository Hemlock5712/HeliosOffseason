package frc.robot.commands.autonomous;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.FlywheelLimelight;
import frc.robot.commands.IntakeCargo;
import frc.robot.commands.LimelightAim;
import frc.robot.commands.LimelightShoot;
import frc.robot.commands.MagazineAutoBump;
import frc.robot.commands.MagazineSpitCargo;
import frc.robot.commands.ManualShoot;
import frc.robot.commands.autonomous.util.AutoBaseCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Magazine;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;

public class TwoBallStealLeft extends AutoBaseCommand {

  PPSwerveControllerCommand driveBackCommand;
  PPSwerveControllerCommand stealCommand;

  public TwoBallStealLeft(Drivetrain drivetrain, Shooter shooter, Intake intake, Magazine magazine, Climber climber,
      Turret turret, Limelight limelight) {
    super(drivetrain, shooter, intake, magazine, climber, turret, limelight);

    addCommands(
        new InstantCommand(() -> {
          drivetrain.setGyroscope(-64.80);
          drivetrain.resetOdometry(new Pose2d(6.84, 5.74, Rotation2d.fromDegrees(-64.80)));
        }),
        new ParallelDeadlineGroup(
            new SequentialCommandGroup(
                driveBackCommand, // Drive to cargo
                new InstantCommand(() -> drivetrain.stopModules()), // Stop
                new LimelightAim(drivetrain, limelight).withTimeout(1.5)),
            new IntakeCargo(intake, magazine),
            new MagazineAutoBump(magazine)), // Grab cargo
        new InstantCommand(() -> drivetrain.stopModules()),
        // new LimelightShoot(shooter, magazine, limelight).withTimeout(1.5), // Shoot
        // cargo
        // new LimelightShoot(shooter, magazine, limelight).withTimeout(1.5), // Shoot
        // second cargo
        new ManualShoot(shooter, magazine, () -> 6800, () -> -3).withTimeout(1),
        new ManualShoot(shooter, magazine, () -> 6700, () -> -5).withTimeout(1),
        new ParallelDeadlineGroup(
            new SequentialCommandGroup(
                stealCommand,
                new InstantCommand(() -> drivetrain.stopModules())),
            new IntakeCargo(intake, magazine),
            new MagazineAutoBump(magazine)),
        new InstantCommand(() -> {
          drivetrain.stopModules();
          shooter.runMotor(0);
          intake.setIntakeDown(false);
        }),
        new MagazineSpitCargo(magazine).withTimeout(1.5));
  }

  protected void generatePaths() {
    PathPlannerTrajectory m_driveBack1 = PathPlanner.loadPath("Left Ball Steal A", 4, 2);

    driveBackCommand = new PPSwerveControllerCommand(
        m_driveBack1,
        drivetrain::getPose2d,
        drivetrain.getKinematics(),
        m_translationController,
        m_strafeController,
        m_thetaController,
        drivetrain::setAllStates,
        drivetrain);
    PathPlannerTrajectory m_driveBack2 = PathPlanner.loadPath("Left Ball Steal B", 4, 2);

    stealCommand = new PPSwerveControllerCommand(
        m_driveBack2,
        drivetrain::getPose2d,
        drivetrain.getKinematics(),
        m_translationController,
        m_strafeController,
        m_thetaController,
        drivetrain::setAllStates,
        drivetrain);
  }
}
