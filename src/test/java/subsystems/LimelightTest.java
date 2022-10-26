package subsystems;

import static org.junit.Assert.assertEquals;

import org.junit.Before;
import org.junit.Test;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.subsystems.Limelight;
import frc.robot.util.LimelightState;

public class LimelightTest {

    Limelight limelight;

    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");
    NetworkTableEntry tv = table.getEntry("tv");
    NetworkTableEntry tshort = table.getEntry("tshort");
    NetworkTableEntry tlong = table.getEntry("tlong");
    NetworkTableEntry thor = table.getEntry("thor");
    NetworkTableEntry tvert = table.getEntry("tvert");
    NetworkTableEntry ledMode = table.getEntry("ledMode");

    @Before
    public void setup() {
        limelight = new Limelight();
        tx.setDouble(14);
        ty.setDouble(-3);
        ta.setDouble(32);
        tv.setDouble(1);
        tshort.setDouble(4);
        tlong.setDouble(8);
        thor.setDouble(8);
        tvert.setDouble(4);
        ledMode.setDouble(LimelightState.OFF.ordinal());
    }

    @Test
    public void readsXOffset() {
        assertEquals(14, limelight.getX(), 0.1);
    }

    @Test
    public void readsYOffset() {
        assertEquals(-3, limelight.getY(), 0.1);
    }

    @Test
    public void readsHorizontalWidth() {
        assertEquals(8, limelight.getHorizontalWidth(), 0.1);
    }

    @Test
    public void readsValidTarget() {
        assertEquals(true, limelight.hasValidTarget());
        tv.setDouble(0);
        assertEquals(false, limelight.hasValidTarget());
    }

    @Test
    public void tellsIfAlignmentIsCorrect() {
        assertEquals(false, limelight.alignGood());
        tx.setDouble(0);
        assertEquals(true, limelight.alignGood());
    }

    @Test
    public void setLimelightToBlink() {
        limelight.blink();
        assertEquals(LimelightState.BLINK.ordinal(), ledMode.getDouble(-1), 0.2);
    }

    @Test
    public void setLimelightToOn() {
        limelight.limelightOn();
        assertEquals(LimelightState.ON.ordinal(), ledMode.getDouble(-1), 0.2);
    }

    @Test
    public void setLimelightToOff() {
        limelight.limelightOff();
        assertEquals(LimelightState.OFF.ordinal(), ledMode.getDouble(-1), 0.2);
    }
}
