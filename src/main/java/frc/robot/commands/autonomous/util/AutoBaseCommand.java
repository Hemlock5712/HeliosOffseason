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

/**
 * Provides a base command structure for autonomous commands.
 * Requires all of the components, as well as setting up path
 * generation PID and generate methods.
 */
public abstract class AutoBaseCommand extends SequentialCommandGroup {

  protected Drivetrain drivetrain;
  protected Shooter shooter;
  protected Intake intake;
  protected Magazine magazine;
  protected Climber climber;
  protected Limelight limelight;

  protected PIDController m_translationController = Constants.Auto.X_PID_CONTROLLER;
  protected PIDController m_strafeController = Constants.Auto.Y_PID_CONTROLLER;
  protected ProfiledPIDController m_thetaController = Constants.Auto.ROT_PID_CONTROLLER;

  private boolean pathHasBeenGenerated = false;

  public AutoBaseCommand(Drivetrain drivetrain, Shooter shooter, Intake intake, Magazine magazine, Climber climber,
      Limelight limelight) {
    this.drivetrain = drivetrain;
    this.shooter = shooter;
    this.intake = intake;
    this.magazine = magazine;
    this.limelight = limelight;

    generate();
  }

  /**
   * Whether the paths have been generated
   * @return Has path been generated
   */
  public boolean hasBeenGenerated() {
    return pathHasBeenGenerated;
  }

  /**
   * Load and generate trajectories in this method. It is automatically called
   * upon creation of the command.
   */
  protected abstract void generatePaths();

  /**
   * Generate the paths for the autonomous routine. If they have already
   * been generated, skip as to not cause delays.
   */
  public final void generate() {
    if (!pathHasBeenGenerated) {
      generatePaths();
      pathHasBeenGenerated = true;
    }
  }
}
