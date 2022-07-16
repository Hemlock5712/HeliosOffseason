// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import frc.robot.commands.CalibrateClimber;
import frc.robot.commands.ClimberArmsIn;
import frc.robot.commands.ClimberArmsOut;
import frc.robot.commands.ClimberDown;
import frc.robot.commands.ClimberUp;
import frc.robot.commands.FieldDrive;
import frc.robot.commands.IntakeCargo;
import frc.robot.commands.LimelightAim;
import frc.robot.commands.LimelightShoot;
import frc.robot.commands.MagazineAutoBump;
import frc.robot.commands.MagazineForceCargo;
import frc.robot.commands.MagazineSpitCargo;
import frc.robot.commands.ManualShoot;
import frc.robot.commands.ResetHoodAngle;
import frc.robot.commands.autonomous.BackShoot;
import frc.robot.commands.autonomous.FiveBallRight;
import frc.robot.commands.autonomous.MiddleStealDelay;
import frc.robot.commands.autonomous.SystemCheck;
import frc.robot.commands.autonomous.ThreeBallRight;
import frc.robot.commands.autonomous.TwoBallStealLeft;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Magazine;
import frc.robot.subsystems.Shooter;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  XboxController primary_joystick = new XboxController(0);
  private final Drivetrain drivetrain = new Drivetrain();
  private final Intake intake = new Intake();
  private final Magazine magazine = new Magazine(primary_joystick);
  private final Shooter shooter = new Shooter();
  private final Limelight limelight = new Limelight();
  private final Climber climber = new Climber();

  private final SendableChooser<Command> m_chooser = new SendableChooser<>();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureAutonomous();
    configureButtonBindings();
  }

  public Drivetrain getDrivetrain() {
    return drivetrain;
  }

  private void configureAutonomous() {
    m_chooser.addOption("System Check",
        new SystemCheck(drivetrain, magazine, shooter, intake, climber, limelight));

    // m_chooser.addOption("Five Ball Right",
    //     new FiveBallRight(drivetrain, shooter, intake, magazine, climber, limelight));
    m_chooser.addOption("Left 2 Ball Steal",
        new TwoBallStealLeft(drivetrain, shooter, intake, magazine, climber, limelight));
    m_chooser.addOption("Middle Steal Delay",
        new MiddleStealDelay(drivetrain, shooter, intake, magazine, climber, limelight));
    m_chooser.addOption("Three Ball Right",
        new ThreeBallRight(drivetrain, shooter, intake, magazine, climber, limelight));
    m_chooser.setDefaultOption("Drive Back Shoot",
        new BackShoot(drivetrain, shooter, intake, magazine, climber, limelight));
    SmartDashboard.putData(m_chooser);
  }

  public SendableChooser<Command> getAutonChooser() {
    return m_chooser;
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    // primary_joystick = new XboxController(0);
    XboxController operator_joystick = new XboxController(1);
    XboxController climber_joystick = new XboxController(2);
    // XboxController climber_joystick = new XboxController(2);

    // Default commands
    magazine.setDefaultCommand(new MagazineAutoBump(magazine));
    // rumble.setDefaultCommand(new MagazineAutoBumpRumble(primary_joystick, rumble,
    // magazine));
    drivetrain.setDefaultCommand(new FieldDrive(drivetrain, () -> -modifyAxis(primary_joystick.getLeftX()),
        () -> modifyAxis(primary_joystick.getLeftY()), () -> modifyAxis(primary_joystick.getRightX())));
    shooter.setDefaultCommand(new ResetHoodAngle(shooter));

    /*
     * Primary Driver Commands
     */
    // Calibrate gyroscope
    new Button(primary_joystick::getAButton).whenPressed(
        drivetrain::zeroGyroscope);

    // Intake
    new Button(primary_joystick::getRightBumper).whenHeld(
        new IntakeCargo(intake, magazine));

    // Reverse intake
    new Button(primary_joystick::getXButton).whenHeld(
        new MagazineSpitCargo(magazine));

    new Button(primary_joystick::getYButton).whenHeld(
        new LimelightAim(drivetrain, limelight));

    new Button(primary_joystick::getLeftBumper).whileHeld(
        new MagazineSpitCargo(magazine));
    // new LimelightShoot(shooter, magazine, limelight)).whenReleased(new
    // InstantCommand(() -> {
    // shooter.runMotor(0);
    // }));

    /*
     * Operator Commands
     */
    // Shoot using limelight
    // new Button(operator_joystick::getRightBumper).whileHeld(
    // new LimelightShoot(shooter, magazine, limelight));

    // Close safe zone
    new Button(operator_joystick::getRightStickButton).whileHeld(
        new ManualShoot(shooter, magazine, () -> 7500, () -> -4.5));
        // Close Tarmac
    new Button(operator_joystick::getRightBumper).whileHeld(
      new ManualShoot(shooter, magazine, () -> 6700, () -> -5)
    );
    // Far tarmac
    new Button(operator_joystick::getBButton).whileHeld(
        new ManualShoot(shooter, magazine, () -> 7000, () -> -7));
    // Low Goal
    new Button(operator_joystick::getYButton).whileHeld(
        new ManualShoot(shooter, magazine, () -> 3000, () -> -5));
    // Far safe zone
    new Button(operator_joystick::getXButton).whileHeld(
        new ManualShoot(shooter, magazine, () -> 8500, () -> -20));
    

    new Button(climber_joystick::getYButton).whileHeld(new ClimberUp(climber))
        .whenReleased(new InstantCommand(() -> climber.runLift(0)));

    // Drop climber to bottom
    // new Button(climber_joystick::getAButton).whenPressed(
    // new ClimberToBottom(climber)).whenReleased(new
    // ClimberToBottomOffBar(climber));
    new Button(climber_joystick::getAButton).whileHeld(new ClimberDown(climber))
        .whenReleased(new InstantCommand(() -> climber.runLift(0)));

    // Retract climber arms, also locks turret to forward position
    new Button(climber_joystick::getBButton).whenPressed(
        new ClimberArmsIn(climber));

    // Extend climber arms, allows free rotation of turret
    new Button(climber_joystick::getXButton).whenPressed(
        new ClimberArmsOut(climber));

    new Button(climber_joystick::getStartButton).whenPressed(
        new CalibrateClimber(climber));
  }

  private double deadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      if (value > 0.0) {
        return (value - deadband) / (1.0 - deadband);
      } else {
        return (value + deadband) / (1.0 - deadband);
      }
    } else {
      return 0.0;
    }
  }

  private double modifyAxis(double value) {
    // Deadband
    value = deadband(value, 0.05);

    // Square the axis
    value = Math.copySign(value * value, value);

    return value;
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_chooser.getSelected();
  }
}
