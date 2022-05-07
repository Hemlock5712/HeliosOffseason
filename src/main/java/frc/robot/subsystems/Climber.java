// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants;

public class Climber extends PIDSubsystem {

  TalonFX leftMotor = new TalonFX(Constants.Climber.LEFT_MOTOR_ID);
  TalonFX rightMotor = new TalonFX(Constants.Climber.RIGHT_MOTOR_ID);

  Solenoid climberSolenoid = new Solenoid(PneumaticsModuleType.REVPH, Constants.Climber.SOLENOID_ID);

  boolean isCalibrated = false;
  boolean armsOut = false;

  public Climber() {
    super(new PIDController(Constants.Climber.kP, Constants.Climber.kI, Constants.Climber.kD));
    getController().setTolerance(10);
    leftMotor.setNeutralMode(NeutralMode.Brake);
    rightMotor.setNeutralMode(NeutralMode.Brake);
    leftMotor.setInverted(true);
    leftMotor.follow(rightMotor);
  }

  public void resetEncoders() {
    leftMotor.setSelectedSensorPosition(0);
    rightMotor.setSelectedSensorPosition(0);
  }

  public boolean getLimitSwitchDown() {
    if (!isCalibrated) {
      isCalibrated = true;
    }
    return rightMotor.isRevLimitSwitchClosed() == 0;
  }

  public boolean getLimitSwitchUp() {
    return rightMotor.isFwdLimitSwitchClosed() == 0;
  }

  public double getMeasurement() {
    return rightMotor.getSelectedSensorPosition();
  }

  public boolean atSetpoint() {
    return getController().atSetpoint();
  }

  public void runLift(double speed) {
    rightMotor.set(ControlMode.PercentOutput, speed);
  }

  public void useOutput(double output, double setpoint) {
    runLift(output);
  }

  public void setArmsOut(boolean armsOut) {
    climberSolenoid.set(armsOut);
    this.armsOut = armsOut;
  }

  public boolean areArmsDown() {
    return armsOut;
  }
}
