package frc.robot.commands.autonomous;

import frc.robot.commands.LimelightAim;
import frc.robot.commands.autonomous.util.AutoBaseCommandPPBeta;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Magazine;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;

public class BackShootMiddle extends AutoBaseCommandPPBeta {

    public BackShootMiddle(String pathName, Drivetrain drivetrain, Shooter shooter, Intake intake, Magazine magazine,
            Climber climber, Turret turret, Limelight limelight) {
        super(pathName, drivetrain, shooter, intake, magazine, climber, turret, limelight);

        addCommands(
                drivetrain.followTrajectory(trajectories.get(0), true),
                new LimelightAim(drivetrain, limelight).withTimeout(0.5),
                shooter.shootCloseTarmac().withTimeout(1),
                drivetrain.followTrajectory(trajectories.get(1)));
    }

}
