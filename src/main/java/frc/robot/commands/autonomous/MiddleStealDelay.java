package frc.robot.commands.autonomous;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.IntakeCargo;
import frc.robot.commands.LimelightAim;
import frc.robot.commands.MagazineAutoBump;
import frc.robot.commands.MagazineSpitCargo;
import frc.robot.commands.ManualShoot;
import frc.robot.commands.autonomous.util.AutoBaseCommand;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Magazine;
import frc.robot.subsystems.Shooter;

public class MiddleStealDelay extends AutoBaseCommand {

  PPSwerveControllerCommand driveBackCommand;
  PPSwerveControllerCommand stealCommand;

  public MiddleStealDelay(Drivetrain drivetrain, Shooter shooter, Intake intake, Magazine magazine, Climber climber,
      Limelight limelight) {
    super(drivetrain, shooter, intake, magazine, climber, limelight);

    addCommands(
        new InstantCommand(() -> {
          drivetrain.setGyroscope(-22.99);
          drivetrain.resetOdometry(new Pose2d(6.99, 4.47, Rotation2d.fromDegrees(-22.99)));
        }),
        new WaitCommand(6),
        new ParallelDeadlineGroup(
            new SequentialCommandGroup(
                driveBackCommand, // Drive to cargo
                new InstantCommand(() -> drivetrain.stopModules()), // Stop
                new LimelightAim(drivetrain, limelight).withTimeout(1)
            ),
            new IntakeCargo(intake, magazine),
            new MagazineAutoBump(magazine)), // Grab cargo
        new InstantCommand(() -> drivetrain.stopModules()),
        // new LimelightShoot(shooter, magazine, limelight).withTimeout(1.5),
        new ManualShoot(shooter, magazine, () -> 7000, () -> -3).withTimeout(1.2),
        new ParallelDeadlineGroup(
          new SequentialCommandGroup(
            stealCommand,
            new InstantCommand(() -> drivetrain.stopModules()),
            new MagazineSpitCargo(magazine).withTimeout(2)
          )
        ),
        new InstantCommand(() -> {
          shooter.runMotor(0);
          intake.setIntakeDown(false);
        }),
        new MagazineSpitCargo(magazine)
    );
  }

  protected void generatePaths() {
    PathPlannerTrajectory m_driveBack1 = PathPlanner.loadPath("Middle Steal Delayed A", 6, 4);

    driveBackCommand = new PPSwerveControllerCommand(
        m_driveBack1,
        drivetrain::getPose2d,
        drivetrain.getKinematics(),
        m_translationController,
        m_strafeController,
        m_thetaController,
        drivetrain::setAllStates,
        drivetrain);
      PathPlannerTrajectory m_driveBack2 = PathPlanner.loadPath("Middle Steal Delayed B", 4, 2);

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
