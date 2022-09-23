// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Turret extends SubsystemBase {

  Climber climber;
  TalonFX rotationMotor = new TalonFX(Constants.Turret.MOTOR_ID);
  double currentAngle = 0;
  boolean activeTargeting = false;

  public boolean isActiveTargeting() {
    return activeTargeting;
  }

  public void setActiveTargeting(boolean activeTargeting) {
    this.activeTargeting = activeTargeting;
  }

  public Turret(Climber climber) {
    this.climber = climber;
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
   * 
   * @param angle in degrees
   */
  public void setAngle(double angle) {
    currentAngle = (angle / 360) * Constants.Turret.ENCODER_TICKS_PER_ROTATION * Constants.Turret.GEAR_RATIO;
  }

  public boolean isFront() {
    return currentAngle < Constants.Turret.CLIMBER_ARM_RIGHT_ANGLE
        * Constants.Turret.ENCODER_TICKS_PER_ROTATION
        && currentAngle > Constants.Turret.CLIMBER_ARM_LEFT_ANGLE
            * Constants.Turret.ENCODER_TICKS_PER_ROTATION;
  }

  public boolean isBack() {
    return currentAngle < (360 - Constants.Turret.CLIMBER_POLE_DEADZONE_CENTER
        - Constants.Turret.CLIMBER_POLE_DEADZONE_WIDTH)
        * Constants.Turret.ENCODER_TICKS_PER_ROTATION
        && currentAngle > (Constants.Turret.CLIMBER_POLE_DEADZONE_CENTER + Constants.Turret.CLIMBER_POLE_DEADZONE_WIDTH)
            * Constants.Turret.ENCODER_TICKS_PER_ROTATION;
  }

  public boolean canTurretRotatePastClimberArms() {
    // TODO: Make this work the opposite way as well, allowing for reversed shooting
    // with arms up
    return climber.areArmsDown();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    rotationMotor.set(TalonFXControlMode.Position, currentAngle);
    SmartDashboard.putNumber("Target Angle", currentAngle);
    SmartDashboard.putNumber("Current Angle", rotationMotor.getSelectedSensorPosition());
    SmartDashboard.putBoolean("Turret/isBack", isBack());
    SmartDashboard.putBoolean("Turret/isFront", isFront());
  }
}
