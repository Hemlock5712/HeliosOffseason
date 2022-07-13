package frc.robot.commands.util;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

public class RawShooterInput extends CommandBase {

    Shooter m_shooter;
    DoubleSupplier m_speed;
    DoubleSupplier m_angle;

    public RawShooterInput(Shooter shooter, DoubleSupplier speed, DoubleSupplier angle) {

        m_shooter = shooter;
        m_speed = speed;
        m_angle = angle;

        addRequirements(shooter);

    }

    @Override
    public void execute() {
        m_shooter.runMotor(m_speed.getAsDouble());
        m_shooter.setHoodAngle(m_angle.getAsDouble());
    }

    @Override
    public boolean isFinished() {
        return m_shooter.getShooterError() < 200 && (Math.abs(m_shooter.getHoodPosition() - m_angle.getAsDouble()) < 1);
    }
}
