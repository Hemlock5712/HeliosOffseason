package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Turret;

public class TurretPassiveAim extends CommandBase {
  Turret m_turret;
  Limelight limelight;
  Drivetrain drivetrain;

  public TurretPassiveAim(Turret turret, Limelight limelight, Drivetrain drivetrain) {
    m_turret = turret;
    this.limelight = limelight;
    this.drivetrain = drivetrain;
    addRequirements(turret);
  }

  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // double rotationSpeed = drivetrain.robotRotationSpeed() * 20;
    if (limelight.hasValidTarget()) {
      m_turret.setLimelightTargeting(true);
      m_turret.setAngle(m_turret.getTrueTurretAngle() + limelight.getX());
    } else {
      m_turret.setLimelightTargeting(false);
      m_turret.setAngle(m_turret.getAngleToFakeHub());
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
