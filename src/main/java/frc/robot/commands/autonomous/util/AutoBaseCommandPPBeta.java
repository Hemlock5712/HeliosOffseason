package frc.robot.commands.autonomous.util;

import java.util.ArrayList;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Magazine;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;

/**
 * Provides a base command structure for autonomous commands.
 * Requires all of the components, as well as setting up path
 * generation PID and generate methods.
 */
public abstract class AutoBaseCommandPPBeta extends SequentialCommandGroup {

  protected ArrayList<PathPlannerTrajectory> trajectories;

  private String pathName;
  private PathConstraints constraint;
  private PathConstraints[] constraints;

  protected Drivetrain drivetrain;
  protected Shooter shooter;
  protected Intake intake;
  protected Magazine magazine;
  protected Climber climber;
  protected Limelight limelight;
  protected Turret turret;

  protected PIDController m_translationController = Constants.Auto.X_PID_CONTROLLER;
  protected PIDController m_strafeController = Constants.Auto.Y_PID_CONTROLLER;
  protected PIDController m_thetaController = Constants.Auto.ROT_PID_CONTROLLER;

  private boolean pathHasBeenGenerated = false;

  /**
   * Base autonomous command for ease of generating paths.
   * 
   * @param pathName
   * @param drivetrain
   * @param shooter
   * @param intake
   * @param magazine
   * @param climber
   * @param turret
   * @param limelight
   * @param constraint
   * @param constraints
   */
  public AutoBaseCommandPPBeta(String pathName, Drivetrain drivetrain, Shooter shooter, Intake intake,
      Magazine magazine,
      Climber climber,
      Turret turret, Limelight limelight, PathConstraints constraint, PathConstraints... constraints) {
    this.pathName = pathName;
    this.drivetrain = drivetrain;
    this.shooter = shooter;
    this.intake = intake;
    this.magazine = magazine;
    this.limelight = limelight;
    this.turret = turret;
    this.constraint = constraint;
    this.constraints = constraints;

    generate();
  }

  /**
   * Base autonomous command for ease of generating paths.
   * 
   * If no path constraints are entered, it is implied that
   * all paths will use 4 m/s velocity and 3 m/s^2 acceleration
   * 
   * @param pathName
   * @param drivetrain
   * @param shooter
   * @param intake
   * @param magazine
   * @param climber
   * @param turret
   * @param limelight
   */
  public AutoBaseCommandPPBeta(String pathName, Drivetrain drivetrain, Shooter shooter, Intake intake,
      Magazine magazine,
      Climber climber,
      Turret turret, Limelight limelight) {
    this(pathName, drivetrain, shooter, intake, magazine, climber, turret, limelight, new PathConstraints(4, 3));
  }

  /**
   * Whether the paths have been generated
   * 
   * @return Has path been generated
   */
  public boolean hasBeenGenerated() {
    return pathHasBeenGenerated;
  }

  /**
   * Load and generate trajectories in this method. It is automatically called
   * upon creation of the command.
   */
  protected void generatePaths() {
    trajectories = PathPlanner.loadPathGroup(pathName, constraint, constraints);
  }

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
