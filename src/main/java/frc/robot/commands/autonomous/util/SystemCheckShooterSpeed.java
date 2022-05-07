package frc.robot.commands.autonomous.util;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

public class SystemCheckShooterSpeed extends CommandBase {

    Shooter m_shooter;
    DoubleSupplier m_speed;

    public SystemCheckShooterSpeed(Shooter shooter, DoubleSupplier speed) {

        m_shooter = shooter;
        m_speed = speed;

    }

    @Override
    public void execute() {
        m_shooter.runMotor(m_speed.getAsDouble());
    }

    @Override
    public boolean isFinished() {
        return m_shooter.getShooterError() < 200;
    }
}
