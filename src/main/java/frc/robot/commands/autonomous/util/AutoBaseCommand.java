package frc.robot.commands.autonomous.util;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Magazine;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;

public abstract class AutoBaseCommand extends SequentialCommandGroup {

  protected Drivetrain drivetrain;
  protected Shooter shooter;
  protected Intake intake;
  protected Magazine magazine;
  protected Climber climber;
  protected Limelight limelight;
  protected Turret turret;

  protected PIDController m_translationController = Constants.Auto.X_PID_CONTROLLER;
  protected PIDController m_strafeController = Constants.Auto.Y_PID_CONTROLLER;
  protected ProfiledPIDController m_thetaController = Constants.Auto.ROT_PID_CONTROLLER;

  private boolean pathHasBeenGenerated = false;

  public AutoBaseCommand(Drivetrain drivetrain, Shooter shooter, Intake intake, Magazine magazine, Climber climber,
      Limelight limelight, Turret turret) {
    this.drivetrain = drivetrain;
    this.shooter = shooter;
    this.intake = intake;
    this.magazine = magazine;
    this.limelight = limelight;
    this.turret = turret;

    generate();
  }

  public boolean hasBeenGenerated() {
    return pathHasBeenGenerated;
  }

  protected abstract void generatePaths();

  public final void generate() {
    if (!pathHasBeenGenerated) {
      generatePaths();
      pathHasBeenGenerated = true;
    }
  }
}
