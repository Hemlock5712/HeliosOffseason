package frc.robot.commands.autonomous;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.FlywheelLimelight;
import frc.robot.commands.IntakeCargo;
import frc.robot.commands.LimelightAim;
import frc.robot.commands.LimelightShoot;
import frc.robot.commands.MagazineAutoBump;
import frc.robot.commands.autonomous.util.AutoBaseCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Magazine;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Shooter;

public class TwoBallMiddle extends AutoBaseCommand {

  PPSwerveControllerCommand driveBackCommand;

  public TwoBallMiddle(Drivetrain drivetrain, Shooter shooter, Intake intake, Magazine magazine, Climber climber,
      Limelight limelight) {
    super(drivetrain, shooter, intake, magazine, climber, limelight);

    addCommands(
        new InstantCommand(() -> {
          drivetrain.setGyroscope(27.31);
          drivetrain.resetOdometry(new Pose2d(6.56, 2.69, Rotation2d.fromDegrees(27.31)));
        }),
        new ParallelRaceGroup(
            new SequentialCommandGroup(
                driveBackCommand, // Drive to cargo
                new InstantCommand(() -> drivetrain.stopModules()), // Stop
                new LimelightAim(drivetrain, limelight).withTimeout(1.5),
                new LimelightShoot(shooter, magazine, limelight), // Shoot cargo
                new LimelightShoot(shooter, magazine, limelight) // Shoot second cargo
            ),
            new IntakeCargo(intake, magazine)), // Grab cargo
        new InstantCommand(() -> {
          shooter.runMotor(0);
          intake.setIntakeDown(false);
        })
    );
  }

  protected void generatePaths() {
    PathPlannerTrajectory m_driveBack = PathPlanner.loadPath("Middle Two Ball", 4, 2);

    driveBackCommand = new PPSwerveControllerCommand(
        m_driveBack,
        drivetrain::getPose2d,
        drivetrain.getKinematics(),
        m_translationController,
        m_strafeController,
        m_thetaController,
        drivetrain::setAllStates,
        drivetrain);
  }
}
