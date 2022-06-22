package subsystems;

import static org.junit.Assert.*;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.REVPHSim;
import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
import frc.robot.Constants;
import frc.robot.subsystems.Climber;
import org.junit.*;

public class ClimberTest {

    Climber climber;
    REVPHSim simPH;
    TalonFXSimCollection leftMotorSim;
    TalonFXSimCollection rightMotorSim;

    @Before
    public void setup() {
        assert HAL.initialize(500, 0);
        climber = new Climber();
        simPH = new REVPHSim();
        leftMotorSim = climber.leftMotor.getSimCollection();
        rightMotorSim = climber.rightMotor.getSimCollection();
    }

    @After
    public void shutdown() throws Exception {
        climber.close();
    }

    @Test
    public void armsDoGoOut() {
        climber.setArmsOut(true);
        assertEquals(true, simPH.getSolenoidOutput(Constants.Climber.SOLENOID_ID));
    }

    @Test
    public void armsDoGoIn() {
        climber.setArmsOut(false);
        assertEquals(false, simPH.getSolenoidOutput(Constants.Climber.SOLENOID_ID));
    }

    @Test
    public void armsDoGoUp() {
        climber.setSetpoint(1500);
        assertEquals(1500, climber.getSetpoint(), 0.1);
        assertEquals(1, climber.getController().calculate(0), .1);
    }

    @Test
    public void armsDoGoDown() {
        leftMotorSim.addIntegratedSensorPosition(1500);
        climber.setSetpoint(0);
        assertEquals(-12, leftMotorSim.getMotorOutputLeadVoltage(), 1);
    }
}
