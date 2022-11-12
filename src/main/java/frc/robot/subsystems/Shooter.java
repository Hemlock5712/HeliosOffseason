// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.music.Orchestra;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxLimitSwitch;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.ManualShoot;

public class Shooter extends SubsystemBase implements AutoCloseable {
  TalonFX leftShooter = new TalonFX(Constants.Shooter.LEFT_MOTOR_ID);
  TalonFX rightShooter = new TalonFX(Constants.Shooter.RIGHT_MOTOR_ID);
  CANSparkMax hood = new CANSparkMax(Constants.Shooter.HOOD_MOTOR_ID, MotorType.kBrushless);
  RelativeEncoder hoodEncoder = hood.getEncoder();
  SparkMaxLimitSwitch backLimitSwitch = hood.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
  SparkMaxLimitSwitch frontLimitSwitch = hood.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);

  double hoodAngleTarget = 0;

  Magazine magazine;

  public Shooter(Magazine magazine) {
    this.magazine = magazine;
    leftShooter.setNeutralMode(NeutralMode.Brake);
    rightShooter.setNeutralMode(NeutralMode.Brake);

    hood.setIdleMode(IdleMode.kBrake);

    hood.getPIDController().setP(0.2);
    hood.getPIDController().setIZone(0.2);
    hood.getPIDController().setOutputRange(-.3, .3);

    rightShooter.follow(leftShooter, FollowerType.AuxOutput1);

    leftShooter.setNeutralMode(NeutralMode.Coast);
    rightShooter.setNeutralMode(NeutralMode.Coast);

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

    leftShooter.setInverted(true);
    rightShooter.setInverted(true);

    runMotor(0);
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
    hood.set(speed);
  }

  public boolean isHoodBackSwitchTriggered() {
    return backLimitSwitch.isPressed();
  }

  public boolean isHoodFrontSwitchTriggered() {
    return frontLimitSwitch.isPressed();
  }

  public boolean isHoodAtAngle() {
    return Math.abs(hoodEncoder.getPosition() - hoodAngleTarget) < 0.5;
  }

  public void zeroHoodEncoder() {
    hoodEncoder.setPosition(0);
  }

  public Command shootCloseTarmac() {
    return new ManualShoot(this, magazine, () -> 6400, () -> -5);
  }

  public Command shootFarTarmac() {
    return new ManualShoot(this, magazine, () -> 6900, () -> -7);
  }

  public Command shootCloseSafeZone() {
    return new ManualShoot(this, magazine, () -> 7500, () -> -4.5);
  }

  public Command shootFarSafeZone() {
    return new ManualShoot(this, magazine, () -> 8500, () -> -20);
  }

  public Command shootLowGoal() {
    return new ManualShoot(this, magazine, () -> 3000, () -> -5);
  }

  public Command shootDemoShot() {
    return new ManualShoot(this, magazine, () -> 6500, () -> -7).withTimeout(2);
  }

  public Command shootDemoShotLowHood() {
    return new ManualShoot(this, magazine, () -> 6500, () -> -60).withTimeout(2);
  }

  public Command SHOOT() {
    return new ManualShoot(this, magazine, () -> 14000, () -> -60).withTimeout(5);
  }

  public void playSong(String name) {
    ArrayList<TalonFX> motors = new ArrayList<TalonFX>();
    motors.add(leftShooter);
    motors.add(rightShooter);
    Orchestra o = new Orchestra(motors);
    o.loadMusic(name);
    o.play();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Shooter/HoodAngle", getHoodPosition());
    SmartDashboard.putNumber("Shooter/SpeedError", leftShooter.getClosedLoopError());
    SmartDashboard.putNumber("Shooter/ShooterSpeed", leftShooter.getSelectedSensorVelocity());
    SmartDashboard.putNumber("Shooter/HoodSpeed", hood.get());
    SmartDashboard.putBoolean("Shooter/HoodAtAngle", isHoodAtAngle());
    SmartDashboard.putBoolean("Shooter/FlywheelAtSpeed", Math.abs(getShooterError()) < 200);
    SmartDashboard.putNumber("Shooter/HoodError", hoodEncoder.getPosition() - hoodAngleTarget);
    SmartDashboard.putData(this);
  }

  @Override
  public void close() throws Exception {
    hood.close();
  }
}
