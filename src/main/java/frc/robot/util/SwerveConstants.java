package frc.robot.util;

public class SwerveConstants {

    public SwerveConstants(int driveMotor, int steerMotor, int encoder, double steerOffset) {
        DRIVE_MOTOR_ID = driveMotor;
        STEER_MOTOR_ID = steerMotor;
        ENCODER_ID = encoder;
        STEER_OFFSET = -Math.toRadians(steerOffset);
    }

    public int DRIVE_MOTOR_ID;
    public int STEER_MOTOR_ID;
    public int ENCODER_ID;
    public double STEER_OFFSET;
}