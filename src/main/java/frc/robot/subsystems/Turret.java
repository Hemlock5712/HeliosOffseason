// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Turret extends SubsystemBase {

  Climber climber;
  TalonFX rotationMotor = new TalonFX(Constants.Turret.MOTOR_ID);

  public Turret(Climber climber) {
    this.climber = climber;
    rotationMotor.setNeutralMode(NeutralMode.Brake);
  }

  public boolean canTurretRotatePastClimberArms() {
    // TODO: Make this work the opposite way as well, allowing for reversed shooting
    // with arms up
    return climber.areArmsDown();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
