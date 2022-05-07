package frc.robot.commands.autonomous;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.FlywheelLimelight;
import frc.robot.commands.IntakeCargo;
import frc.robot.commands.LimelightShoot;
import frc.robot.commands.autonomous.util.AutoBaseCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Magazine;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;

public class TwoBallMiddle extends AutoBaseCommand {

  PPSwerveControllerCommand driveBackCommand;

  public TwoBallMiddle(Drivetrain drivetrain, Shooter shooter, Intake intake, Magazine magazine, Climber climber,
      Limelight limelight, Turret turret) {
    super(drivetrain, shooter, intake, magazine, climber, limelight, turret);

    addCommands(
        new ParallelRaceGroup(
            new SequentialCommandGroup(
                driveBackCommand, // Drive to cargo
                new InstantCommand(() -> drivetrain.stopModules())), // Stop
            new FlywheelLimelight(shooter, limelight), // Start revving shooter
            new IntakeCargo(intake, magazine) // Grab cargo
        ),
        new LimelightShoot(shooter, magazine, limelight), // Shoot cargo
        new LimelightShoot(shooter, magazine, limelight)// Shoot second cargo
    );
  }

  protected void generatePaths() {
    PathPlannerTrajectory m_driveBack = PathPlanner.loadPath("Middle Two Ball", 8, 4);

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
