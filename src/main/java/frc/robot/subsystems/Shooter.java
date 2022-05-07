// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  TalonFX leftShooter = new TalonFX(Constants.Shooter.LEFT_MOTOR_ID);
  TalonFX rightShooter = new TalonFX(Constants.Shooter.RIGHT_MOTOR_ID);
  CANSparkMax hood = new CANSparkMax(Constants.Shooter.HOOD_MOTOR_ID, MotorType.kBrushless);
  RelativeEncoder hoodEncoder = hood.getEncoder();
  SparkMaxLimitSwitch backLimitSwitch = hood.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
  SparkMaxLimitSwitch frontLimitSwitch = hood.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);

  double hoodAngleTarget = 0;

  public Shooter() {
    leftShooter.setNeutralMode(NeutralMode.Brake);
    rightShooter.setNeutralMode(NeutralMode.Brake);

    hood.setIdleMode(IdleMode.kBrake);
    hood.setInverted(true);

    hood.getPIDController().setP(0.5);
    hood.getPIDController().setIZone(0.2);

    rightShooter.setInverted(true);
    rightShooter.follow(leftShooter, FollowerType.AuxOutput1);

    leftShooter.config_kP(0, Constants.Shooter.kP, 30);
    leftShooter.config_kI(0, Constants.Shooter.kI, 30);
    leftShooter.config_kD(0, Constants.Shooter.kD, 30);
    leftShooter.config_kF(0, Constants.Shooter.kF, 30);
    leftShooter.config_IntegralZone(0, 50);
    rightShooter.config_kP(0, Constants.Shooter.kP, 30);
    rightShooter.config_kI(0, Constants.Shooter.kI, 30);
    rightShooter.config_kD(0, Constants.Shooter.kD, 30);
    rightShooter.config_kF(0, Constants.Shooter.kF, 30);
    rightShooter.config_IntegralZone(0, 50);
  }

  public double getShooterError() {
    return leftShooter.getClosedLoopError();
  }

  public void runMotor(double velocity) {
    leftShooter.set(ControlMode.Velocity, velocity);
  }

  public double getHoodPosition() {
    return hoodEncoder.getPosition();
  }

  public void setHoodAngle(double angle) {
    hood.getPIDController().setReference(angle, ControlType.kPosition);
    hoodAngleTarget = angle;
  }

  public void setHoodSpeed(double speed) {
    hood.set(-0.3);
  }

  public boolean isHoodBackSwitchTriggered() {
    return backLimitSwitch.isPressed();
  }

  public boolean isHoodAtAngle() {
    return Math.abs(hoodEncoder.getPosition() - hoodAngleTarget) < 0.2;
  }

  public void zeroHoodEncoder() {
    hoodEncoder.setPosition(0);
  }
}
