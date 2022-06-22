package subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVPhysicsSim;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.AnalogInputSim;
import edu.wpi.first.wpilibj.simulation.REVPHSim;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Magazine;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import static org.junit.Assert.assertEquals;

public class MagazineTest {
    Magazine magazine;

    AnalogInputSim simLowerSensor;
    AnalogInputSim simUpperSensor;

    @Before
    public void setup() {
        assert HAL.initialize(500, 0);
        magazine = new Magazine();
        // REVPhysicsSim.getInstance().addSparkMax(magazine.lowerMagazine, DCMotor.getNeo550(1));
        // REVPhysicsSim.getInstance().addSparkMax(magazine.upperMagazine, DCMotor.getNeo550(1));
        simLowerSensor = new AnalogInputSim(Constants.Magazine.LOWER_SENSOR);
        simUpperSensor = new AnalogInputSim(Constants.Magazine.UPPER_SENSOR);
    }

    @After
    public void shutdown() throws Exception {
        magazine.close();
    }

    @Test
    public void detectsNoBallsWhenEmpty() {
        simLowerSensor.setVoltage(1.5);
        simUpperSensor.setVoltage(1);
        assertEquals(false, magazine.ballInLower());
        assertEquals(false, magazine.ballInUpper());
    }

    @Test
    public void detectsLowerBallOnly() {
        simLowerSensor.setVoltage(1);
        simUpperSensor.setVoltage(1);
        assertEquals(true, magazine.ballInLower());
        assertEquals(false, magazine.ballInUpper());
        assertEquals(false, magazine.isFull());
    }

    @Test
    public void detectsUpperBallOnly() {
        simLowerSensor.setVoltage(1.5);
        simUpperSensor.setVoltage(1.5);
        assertEquals(false, magazine.ballInLower());
        assertEquals(true, magazine.ballInUpper());
        assertEquals(false, magazine.isFull());
    }

    @Test
    public void detectsBothBalls() {
        simLowerSensor.setVoltage(1);
        simUpperSensor.setVoltage(1.5);
        assertEquals(true, magazine.ballInLower());
        assertEquals(true, magazine.ballInUpper());
        assertEquals(true, magazine.isFull());
    }

    @Test
    public void runsUpperMagazine() {
        magazine.runUpperMagazine(0.5);
        assertEquals(0.5, magazine.upperMagazine.get(), 0.05);
    }

    @Test
    public void runsLowerMagazine() {
        magazine.runLowerMagazine(0.5);
        assertEquals(0.5, magazine.lowerMagazine.get(), 0.05);
    }

}
