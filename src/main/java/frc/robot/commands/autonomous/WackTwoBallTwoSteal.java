package frc.robot.commands.autonomous;

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

public class WackTwoBallTwoSteal extends AutoBaseCommandPPBeta {

    public WackTwoBallTwoSteal(String pathName, Drivetrain drivetrain, Shooter shooter, Intake intake,
            Magazine magazine, Climber climber, Turret turret, Limelight limelight) {
        super(pathName, drivetrain, shooter, intake, magazine, climber, turret, limelight);

        addCommands(
                new ParallelDeadlineGroup(
                        drivetrain.followTrajectory(trajectories.get(0), true),
                        new IntakeWithMagazine(intake, magazine)),
                new LimelightAim(drivetrain, limelight).withTimeout(0.5),
                shooter.shootFarTarmac(),
                shooter.shootFarTarmac(),
                new ParallelDeadlineGroup(
                        drivetrain.followTrajectory(trajectories.get(1)),
                        new IntakeWithMagazine(intake, magazine)),
                drivetrain.followTrajectory(trajectories.get(2)),
                // shooter.shootCloseTarmac(),
                new MagazineSpitCargo(magazine).withTimeout(2),
                drivetrain.followTrajectory(trajectories.get(3)));
    }

}
