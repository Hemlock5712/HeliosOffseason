package frc.robot.commands.autonomous;

import com.pathplanner.lib.PathConstraints;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.commands.IntakeWithMagazine;
import frc.robot.commands.LimelightAim;
import frc.robot.commands.MagazineSpitCargo;
import frc.robot.commands.autonomous.util.AutoBaseCommandPPBeta;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Magazine;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;

public class MiddleOneBallOneSteal extends AutoBaseCommandPPBeta {

    public MiddleOneBallOneSteal(String pathName, Drivetrain drivetrain, Shooter shooter, Intake intake,
            Magazine magazine, Climber climber, Turret turret, Limelight limelight, PathConstraints constraint,
            PathConstraints... constraints) {
        super(pathName, drivetrain, shooter, intake, magazine, climber, turret, limelight, constraint, constraints);

        addCommands(
                new ParallelDeadlineGroup(
                        drivetrain.followTrajectory(trajectories.get(0), true),
                        new IntakeWithMagazine(intake, magazine)),
                new LimelightAim(drivetrain, limelight).withTimeout(0.5),
                shooter.shootFarTarmac().withTimeout(1),
                drivetrain.followTrajectory(trajectories.get(1)),
                new MagazineSpitCargo(magazine).withTimeout(2),
                drivetrain.followTrajectory(trajectories.get(2)));
    }

}
