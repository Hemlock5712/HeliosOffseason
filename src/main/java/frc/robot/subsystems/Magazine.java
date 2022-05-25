// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Magazine extends SubsystemBase implements AutoCloseable {

  public CANSparkMax lowerMagazine = new CANSparkMax(Constants.Magazine.LOWER_MOTOR, MotorType.kBrushless);
  public CANSparkMax upperMagazine = new CANSparkMax(Constants.Magazine.UPPER_MOTOR, MotorType.kBrushless);

  AnalogInput lowerBallSensor = new AnalogInput(Constants.Magazine.LOWER_SENSOR);
  AnalogInput upperBallSensor = new AnalogInput(Constants.Magazine.UPPER_SENSOR);

  public Magazine() {
    lowerMagazine.setIdleMode(IdleMode.kBrake);
    upperMagazine.setIdleMode(IdleMode.kBrake);
    upperMagazine.setInverted(true);
  }

  public void runLowerMagazine(double speed) {
    lowerMagazine.set(speed);
  }

  public void runUpperMagazine(double speed) {
    upperMagazine.set(speed);
  }

  public boolean ballInUpper() {
    return upperBallSensor.getValue() > Constants.Magazine.UPPER_SENSOR_THRESHOLD;
  }

  public boolean ballInLower() {
    return lowerBallSensor.getValue() < Constants.Magazine.LOWER_SENSOR_THRESHOLD;
  }

  public boolean isFull() {
    return ballInLower() && ballInUpper();
  }

  public double getLowerBallSensor() {
    return lowerBallSensor.getValue();
  }

  public double getUpperBallSensor() {
    return upperBallSensor.getValue();
  }

  public void stop() {
    lowerMagazine.set(0);
    upperMagazine.set(0);
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Magazine/BallInUpper", ballInUpper());
    SmartDashboard.putBoolean("Magazine/BallInLower", ballInLower());
    SmartDashboard.putNumber("Magazine/LowerSpeed", lowerMagazine.get());
    SmartDashboard.putNumber("Magazine/UpperSpeed", upperMagazine.get());
    SmartDashboard.putData(this);
  }

  @Override
  public void close() throws Exception {
    lowerMagazine.close();
    upperMagazine.close();
    lowerBallSensor.close();
    upperBallSensor.close();
  }
}
