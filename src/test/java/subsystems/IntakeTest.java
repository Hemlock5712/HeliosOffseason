package subsystems;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.REVPHSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;
import com.revrobotics.REVPhysicsSim;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import static org.junit.Assert.*;

public class IntakeTest {
    Intake intake;
    REVPHSim simPH;

    @Before
    public void setup() {
        assert HAL.initialize(500, 0);
        intake = new Intake();
        simPH = new REVPHSim();
        // REVPhysicsSim.getInstance().addSparkMax(intake.intakeMotor, DCMotor.getNeo550(1));
        // REVPhysicsSim.getInstance().run();
    }

    @After
    public void shutdown() throws Exception {
        intake.close();
    }

    @Test
    public void intakeDoesGoDown() {
        intake.setIntakeDown(true);
        assertEquals(true, simPH.getSolenoidOutput(Constants.Intake.SOLENOID_ID));
    }
    @Test
    public void intakeDoesGoUp() {
        intake.setIntakeDown(false);
        assertEquals(false, simPH.getSolenoidOutput(Constants.Intake.SOLENOID_ID));
    }

    @Test
    public void motorDoesRun() {
        intake.setIntakeSpeed(0.5);
        assertEquals(0.5, intake.intakeMotor.get(), 0.05);
        intake.setIntakeSpeed(0);
        assertEquals(0, intake.intakeMotor.get(), 0.05);
    }

    @Test
    public void motorIsInBrakeMode() {
        assertEquals(CANSparkMax.IdleMode.kBrake, intake.intakeMotor.getIdleMode());
    }

    @Test
    public void doesPushDashboardVariables() {
        intake.setIntakeSpeed(0.2);
        intake.setIntakeDown(true);
        intake.periodic();
        assertEquals(0.2, SmartDashboard.getNumber("Intake/Speed", 0), 0.1);
        assertEquals(true, SmartDashboard.getBoolean("Intake/ArmDown", false));
    }
}
