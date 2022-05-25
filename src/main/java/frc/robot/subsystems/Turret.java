// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Turret extends SubsystemBase implements AutoCloseable {

  private final CANSparkMax m_turretMotor = new CANSparkMax(Constants.Turret.MOTOR_ID, MotorType.kBrushless);
  private final RelativeEncoder m_encoder = m_turretMotor.getEncoder();
  private final SparkMaxPIDController m_turretPID = m_turretMotor.getPIDController();

  private boolean canTurretTurnPastArms = false;

  private final Drivetrain drivetrain;

  public Turret(Drivetrain drivetrain) {
    m_turretPID.setP(Constants.Turret.kP);
    m_turretPID.setI(Constants.Turret.kI);
    m_turretPID.setD(Constants.Turret.kD);
    m_turretPID.setFF(Constants.Turret.kF);
    this.drivetrain = drivetrain;
  }

  public void resetEncoder() {
    m_encoder.setPosition(0);
  }

  /**
   * When climber arms are down, turret should be allowed to freely rotate.
   * @param canTurretTurnPastArms Climber arms are down?
   */
  public void setTurretCanRotatePastArms(boolean canTurretTurnPastArms) {
    this.canTurretTurnPastArms = canTurretTurnPastArms;
  }

  /**
   * Get whether or not the turret can freely rotate
   * @return Climber arms down, turret can spin
   */
  public boolean canTurretRotatePastArms() {
    return canTurretTurnPastArms;
  }

  /**
   * Creates a deadzone where the turret won't shoot directly into the climber hooks.
   * Checks whether the turret is currently facing them, +- the deadzone width
   * @return Should shooter shoot?
   */
  public boolean isTurretFacingClimberArms() {
    double angle = getAngle();
    return Constants.Turret.CLIMBER_POLE_DEADZONE_CENTER - Constants.Turret.CLIMBER_POLE_DEADZONE_WIDTH < angle
        && Constants.Turret.CLIMBER_POLE_DEADZONE_CENTER + Constants.Turret.CLIMBER_POLE_DEADZONE_WIDTH > angle
        || -Constants.Turret.CLIMBER_POLE_DEADZONE_CENTER - Constants.Turret.CLIMBER_POLE_DEADZONE_WIDTH < angle
            && -Constants.Turret.CLIMBER_POLE_DEADZONE_CENTER + Constants.Turret.CLIMBER_POLE_DEADZONE_WIDTH > angle;
  }

  /**
   * Sets the angle that the turret should face.
   * @param angle Angle to rotate to
   */
  public void setAngle(double angle) {
    m_turretPID.setReference(angle * Constants.Turret.GEAR_RATIO, ControlType.kPosition);
  }

  /**
   * Gets the current angle the turret is facing
   * @return Current angle
   */
  public double getAngle() {
    return m_encoder.getPosition() / Constants.Turret.GEAR_RATIO;
  }

  /**
   * Performs vector math to essentially set the robot to (0,0), with the hub at an offset from that point.
   * @return Robot centric position of the hub.
   */
  public Translation2d getTranslatedHubLocation() {
    return Constants.Field.HUB_LOCATION.minus(drivetrain.getPose2d().getTranslation());
  }

  /**
   * Approximate the time that the ball will be in flight if shot from the current point.
   * Useful to compute the offsets from where the tower is while moving.
   * @return Total time ball will fly for if shot
   */
  public double getTimeOfFlight() {
    return Constants.Turret.TIME_OF_FLIGHT.compute(getTranslatedHubLocation().getNorm(), (x, y) -> (x + y) / 2);
  }

  /**
   * Compute the raw robot angle to the hub, based on odometry
   * @return Angle to the hub
   */
  public double getHubAngle() {
    Translation2d hubLocation = getTranslatedHubLocation();
    return Math.atan2(hubLocation.getY(), hubLocation.getX());
  }

  /**
   * Compute where the hub theoretically is, as if the robot isn't moving
   * by adding the offsets from the time the ball will take to fly, as well
   * as the current speed of the robot. This basically just gives the
   * location that you want to shoot in order to get the ball in the
   * hub while actively moving around.
   * @return Theoretical Hub location while moving
   */
  public Translation2d getMovingHubLocation() {
    ChassisSpeeds chassisSpeeds = drivetrain.getFieldRelativeSpeeds();
    double timeOfFlight = getTimeOfFlight();
    return getTranslatedHubLocation().minus(new Translation2d(chassisSpeeds.vxMetersPerSecond * timeOfFlight,
        chassisSpeeds.vyMetersPerSecond * timeOfFlight));
  }

  /**
   * Compute the raw robot angle to the hub, with additional math in place
   * to compute where the ball will end up while driving. Automatically
   * builds these offsets into the angle.
   * @return Angle to the hub while moving
   */
  public double getHubAngleMoving() {
    Translation2d hubLocation = getMovingHubLocation();
    return Math.atan2(hubLocation.getY(), hubLocation.getX());
  }

  @Override
  public void periodic() {
    SmartDashboard.putData(this);
  }

  @Override
  public void close() throws Exception {
    m_turretMotor.close();
    drivetrain.close();
  }
}
