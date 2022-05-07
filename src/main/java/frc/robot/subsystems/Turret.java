// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Turret extends SubsystemBase {

  private CANSparkMax m_turretMotor = new CANSparkMax(Constants.Turret.MOTOR_ID, MotorType.kBrushless);
  private RelativeEncoder m_encoder = m_turretMotor.getEncoder();
  private SparkMaxPIDController m_turretPID = m_turretMotor.getPIDController();

  private boolean canTurretTurnPastArms = false;

  public Turret() {
    m_turretPID.setP(Constants.Turret.kP);
    m_turretPID.setI(Constants.Turret.kI);
    m_turretPID.setD(Constants.Turret.kD);
    m_turretPID.setFF(Constants.Turret.kF);

  }

  public void resetEncoder() {
    m_encoder.setPosition(0);
  }

  public void setTurretCanRotatePastArms(boolean canTurretTurnPastArms) {
    this.canTurretTurnPastArms = canTurretTurnPastArms;
  }

  public boolean canTurretRotatePastArms() {
    return canTurretTurnPastArms;
  }

  public void setAngle(double angle) {
    m_turretPID.setReference(angle, ControlType.kPosition);
  }

  public double getAngle() {
    return m_encoder.getPosition();
  }

}
