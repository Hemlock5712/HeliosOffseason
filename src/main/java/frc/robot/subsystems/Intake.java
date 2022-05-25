// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase implements AutoCloseable {

  public CANSparkMax intakeMotor = new CANSparkMax(Constants.Intake.MOTOR_ID, MotorType.kBrushless);
  Solenoid intakeSolenoid = new Solenoid(PneumaticsModuleType.REVPH, Constants.Intake.SOLENOID_ID);

  public Intake() {
    intakeMotor.setIdleMode(IdleMode.kBrake);
  }

  public void setIntakeDown(boolean down) {
    intakeSolenoid.set(down);
  }

  public void setIntakeSpeed(double speed) {
    intakeMotor.set(speed);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Intake/Speed", intakeMotor.get());
    SmartDashboard.putBoolean("Intake/ArmDown", intakeSolenoid.get());
    SmartDashboard.putData(this);
  }

  @Override
  public void close() throws Exception {
    intakeMotor.close();
    intakeSolenoid.close();
  }
}
