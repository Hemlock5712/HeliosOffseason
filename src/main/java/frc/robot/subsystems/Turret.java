// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Turret extends SubsystemBase {

  Climber climber;
  Drivetrain drivetrain;
  TalonFX rotationMotor = new TalonFX(Constants.Turret.MOTOR_ID);
  double currentAngle = 0;
  boolean activeTargeting = false;

  public boolean isActiveTargeting() {
    return activeTargeting;
  }

  public void setActiveTargeting(boolean activeTargeting) {
    this.activeTargeting = activeTargeting;
  }

  public Turret(Climber climber, Drivetrain drivetrain) {
    this.climber = climber;
    this.drivetrain = drivetrain;
    rotationMotor.setNeutralMode(NeutralMode.Coast);
    rotationMotor.setSelectedSensorPosition(0);
    // Set PID parameters
    rotationMotor.config_kP(0, Constants.Turret.kP);
    rotationMotor.config_kI(0, Constants.Turret.kI);
    rotationMotor.config_IntegralZone(0, 10000);
    rotationMotor.config_kD(0, Constants.Turret.kD);
    rotationMotor.config_kF(0, Constants.Turret.kF);
    rotationMotor.configClosedLoopPeakOutput(0, .9);

    // Prevent motor from going further than a certain point
    rotationMotor.configForwardSoftLimitThreshold(
        Constants.Turret.TRAVEL_RIGHT_LIMIT.getDegrees() * Constants.Turret.ENCODER_TICKS_PER_ROTATION);
    rotationMotor.configReverseSoftLimitThreshold(
        Constants.Turret.TRAVEL_LEFT_LIMIT.getDegrees() * Constants.Turret.ENCODER_TICKS_PER_ROTATION);
  }

  /**
   * Sets the target angle of the turret
   * Angles below -45deg will be wrapped to the positive version of that angle, and vice versa,
   * preventing the turret from going past 45deg to the left
   * 
   * @param angle in degrees
   */
  public void setAngle(double angle) {
    currentAngle = ((((angle + 45) % 360) - 45) / 360) * Constants.Turret.ENCODER_TICKS_PER_ROTATION * Constants.Turret.GEAR_RATIO;
  }

  public double getTargetAngle() {
    return cvtTicksToDegrees(currentAngle);
  }

  public double getTrueTurretAngle() {
    return cvtTicksToDegrees(rotationMotor.getSelectedSensorPosition());
  }

  public double cvtTicksToDegrees(double ticks) {
    return (ticks * 360) / (Constants.Turret.ENCODER_TICKS_PER_ROTATION * Constants.Turret.GEAR_RATIO);
  }

  public boolean isFront() {
    double angle = getTrueTurretAngle();
    return angle < Constants.Turret.CLIMBER_ARM_RIGHT_ANGLE && angle > Constants.Turret.CLIMBER_ARM_LEFT_ANGLE;
  }

  public boolean isBack() {
    double angle = getTrueTurretAngle();
    return angle < (360 - Constants.Turret.CLIMBER_POLE_DEADZONE_CENTER - Constants.Turret.CLIMBER_POLE_DEADZONE_WIDTH)
        && angle > (Constants.Turret.CLIMBER_POLE_DEADZONE_CENTER + Constants.Turret.CLIMBER_POLE_DEADZONE_WIDTH);
  }

  public boolean canTurretRotatePastClimberArms() {
    // TODO: Make this work the opposite way as well, allowing for reversed shooting
    // with arms up
    return climber.areArmsDown();
  }

  public Translation2d getHubLocation() {
    return Constants.Field.HUB_LOCATION;
  }

  public double getAngleToFakeHub() {
    Rotation2d robotRotation = drivetrain.getPose2d().getRotation();
    Translation2d robotPosition = drivetrain.getPose2d().getTranslation();
    Translation2d hubPosition = fakeHubLocation();
    Rotation2d hubAngleRotation = new Rotation2d(robotPosition.getX() - hubPosition.getX(), robotPosition.getY() - hubPosition.getY());
    return robotRotation.rotateBy(hubAngleRotation).getDegrees();
  }

  public Translation2d fakeHubLocation() {
    Translation2d hubLocation = getHubLocation();
    Translation2d fakeRobot = fakeRobotLocation();
    double time = Constants.Turret.TIME_OF_FLIGHT.get(hubLocation.getDistance(drivetrain.getPose2d().getTranslation()));
    return hubLocation.minus(fakeRobot.times(time));
  }

  private Translation2d fakeRobotLocation() {
    ChassisSpeeds speeds = drivetrain.getFieldRelativeSpeeds();
    Translation2d translation = new Translation2d(Math.sqrt(Math.pow(speeds.vxMetersPerSecond, 2) + Math.pow(speeds.vyMetersPerSecond, 2)), new Rotation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond));
    return translation;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    rotationMotor.set(TalonFXControlMode.Position, currentAngle);
    SmartDashboard.putNumber("Turret/targetAngle", getTargetAngle());
    SmartDashboard.putNumber("Turret/currentAngle", getTrueTurretAngle());
    SmartDashboard.putBoolean("Turret/isBack", isBack());
    SmartDashboard.putBoolean("Turret/isFront", isFront());
  }
}
