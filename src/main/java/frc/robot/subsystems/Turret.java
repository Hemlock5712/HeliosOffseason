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
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Turret extends SubsystemBase implements AutoCloseable {

  Climber climber;
  Drivetrain drivetrain;
  public TalonFX rotationMotor = new TalonFX(Constants.Turret.MOTOR_ID);
  double currentAngle = 0;
  boolean activeTargeting = false;
  boolean limelightTargeting = false;
  final double leftAngle = 90;
  boolean haveArmsGoneDownBefore = false;

  DigitalInput centerMagnet = new DigitalInput(0);

  public boolean isMagnetTriggered() {
    return !centerMagnet.get();
  }

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
    rotationMotor.config_IntegralZone(0, 5000);
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
   * Angles below -90deg will be wrapped to the positive version of that angle,
   * and vice versa,
   * preventing the turret from going past 90deg to the left
   * 
   * @param angle in degrees
   */
  public void setAngle(double angle) {
    if (!activeTargeting) {
      if (haveArmsGoneDownBefore) {
        angle = 180;
      } else {
        angle = 0;
      }
    }
    currentAngle = Math.min(260, Math.max(-80, (trueMod(angle + leftAngle, 360) - leftAngle) / 360))
        * Constants.Turret.ENCODER_TICKS_PER_ROTATION
        * Constants.Turret.GEAR_RATIO;
  }

  public void resetTurretAngle(double angle) {
    rotationMotor.setSelectedSensorPosition(angle);
  }

  private double trueMod(double n, double m) {
    return (n < 0) ? (m - (Math.abs(n) % m)) % m : (n % m);
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
    return angle < Constants.Turret.CLIMBER_ARM_RIGHT_ANGLE && angle > -Constants.Turret.CLIMBER_ARM_LEFT_ANGLE;
  }

  public boolean isBack() {
    double angle = getTrueTurretAngle();
    return angle < (360 - Constants.Turret.CLIMBER_POLE_DEADZONE_CENTER - Constants.Turret.CLIMBER_POLE_DEADZONE_WIDTH)
        && angle > (Constants.Turret.CLIMBER_POLE_DEADZONE_CENTER + Constants.Turret.CLIMBER_POLE_DEADZONE_WIDTH);
  }

  public boolean canTurretRotatePastClimberArms() {
    return climber.areArmsDown();
  }

  public Translation2d getHubLocation() {
    return Constants.Field.HUB_LOCATION;
  }

  public double getAngleToHub() {
    Rotation2d robotRotation = drivetrain.getPose2d().getRotation();
    Translation2d robotPosition = drivetrain.getPose2d().getTranslation();
    Translation2d hubPosition = getHubLocation();
    Rotation2d hubAngleRotation = new Rotation2d(hubPosition.getX() - robotPosition.getX(),
        hubPosition.getY() - robotPosition.getY());
    return robotRotation.rotateBy(hubAngleRotation.times(-1)).getDegrees();
  }

  public double getAngleToFakeHub() {
    Rotation2d robotRotation = drivetrain.getPose2d().getRotation();
    Translation2d robotPosition = drivetrain.getPose2d().getTranslation();
    Translation2d hubPosition = fakeHubLocation();
    Rotation2d hubAngleRotation = new Rotation2d(hubPosition.getX() - robotPosition.getX(),
        hubPosition.getY() - robotPosition.getY());
    return robotRotation.rotateBy(hubAngleRotation.times(-1)).getDegrees();
  }

  public Translation2d fakeHubLocation() {
    Translation2d hubLocation = getHubLocation();
    Translation2d fakeRobot = fakeRobotLocation();
    double time = Constants.Turret.TIME_OF_FLIGHT
        .floorKey(hubLocation.getDistance(drivetrain.getPose2d().getTranslation()));
    return hubLocation.minus(fakeRobot.times(time));
  }

  private Translation2d fakeRobotLocation() {
    ChassisSpeeds speeds = drivetrain.getFieldRelativeSpeeds();
    Translation2d translation = new Translation2d(
        Math.sqrt(Math.pow(speeds.vxMetersPerSecond, 2) + Math.pow(speeds.vyMetersPerSecond, 2)),
        new Rotation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond));
    return translation;
  }

  public void setLimelightTargeting(boolean isUsingLimelight) {
    limelightTargeting = isUsingLimelight;
  }

  public boolean haveArmsGoneDownBefore() {
    return haveArmsGoneDownBefore;
  }

  public void setHaveArmsGoneDownBefore(boolean haveArmsGoneDownBefore) {
    this.haveArmsGoneDownBefore = haveArmsGoneDownBefore;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    rotationMotor.set(TalonFXControlMode.Position, currentAngle);
    SmartDashboard.putNumber("Turret/targetAngle", getTargetAngle());
    SmartDashboard.putNumber("Turret/currentAngle", getTrueTurretAngle());
    SmartDashboard.putBoolean("Turret/isBack", isBack());
    SmartDashboard.putBoolean("Turret/isFront", isFront());
    SmartDashboard.putBoolean("Turret/limelightTargeting", limelightTargeting);
    SmartDashboard.putBoolean("Turret/hallEffect", isMagnetTriggered());
  }

  @Override
  public void close() throws Exception {
    rotationMotor.DestroyObject();
  }
}
