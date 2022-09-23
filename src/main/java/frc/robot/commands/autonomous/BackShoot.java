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
import edu.wpi.first.wpilibj2.command.WaitCommand;
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

public class BackShoot extends AutoBaseCommand {

    PPSwerveControllerCommand driveBackCommand;
    PPSwerveControllerCommand stealCommand;

    public BackShoot(Drivetrain drivetrain, Shooter shooter, Intake intake, Magazine magazine, Climber climber,
            Turret turret, Limelight limelight) {
        super(drivetrain, shooter, intake, magazine, climber, turret, limelight);

        addCommands(
                new InstantCommand(() -> {
                    drivetrain.setGyroscope(-21.10);
                    drivetrain.resetOdometry(new Pose2d(6.10, 4.9, Rotation2d.fromDegrees(-21.10)));
                }),
                new WaitCommand(10),
                new ParallelDeadlineGroup(
                        new SequentialCommandGroup(driveBackCommand, new InstantCommand(() -> drivetrain.stopModules()),
                                new LimelightAim(drivetrain, limelight).withTimeout(1.5)),
                        new MagazineAutoBump(magazine)),
                // new LimelightShoot(shooter, magazine, limelight),
                new ManualShoot(shooter, magazine, () -> 7000, () -> -7),
                new InstantCommand(() -> {
                    shooter.runMotor(0);
                    intake.setIntakeDown(false);
                }));
    }

    protected void generatePaths() {
        PathPlannerTrajectory m_driveBack1 = PathPlanner.loadPath("BackShoot", 6, 4);

        driveBackCommand = new PPSwerveControllerCommand(
                m_driveBack1,
                drivetrain::getPose2d,
                drivetrain.getKinematics(),
                m_translationController,
                m_strafeController,
                m_thetaController,
                drivetrain::setAllStates,
                drivetrain);
    }
}
