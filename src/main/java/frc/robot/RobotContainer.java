// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.FieldDrive;
import frc.robot.commands.MagazineAutoBump;
import frc.robot.commands.autonomous.TwoBallMiddle;
import frc.robot.commands.autonomous.util.AutoBaseCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Climber;
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

  private final Drivetrain drivetrain = new Drivetrain();
  private final Intake intake = new Intake();
  private final Magazine magazine = new Magazine();
  private final Shooter shooter = new Shooter();
  private final Limelight limelight = new Limelight();
  private final Climber climber = new Climber();

  private final SendableChooser<AutoBaseCommand> m_chooser = new SendableChooser<>();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureAutonomous();
    configureButtonBindings();
  }

  private void configureAutonomous() {
    m_chooser.addOption("2 Ball Middle", new TwoBallMiddle(drivetrain, shooter, intake, magazine, limelight));
    SmartDashboard.putData(m_chooser);
  }

  public SendableChooser<AutoBaseCommand> getAutonChooser() {
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

    XboxController primary_joystick = new XboxController(0);
    XboxController operator_joystick = new XboxController(1);
    XboxController climber_joystick = new XboxController(2);

    // Default commands
    magazine.setDefaultCommand(new MagazineAutoBump(magazine));
    drivetrain.setDefaultCommand(new FieldDrive(drivetrain, () -> modifyAxis(primary_joystick.getLeftY()),
        () -> modifyAxis(primary_joystick.getLeftX()), () -> modifyAxis(primary_joystick.getRightX())));
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
