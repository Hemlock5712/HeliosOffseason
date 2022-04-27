// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.util.SwerveConstants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final class Swerve {
        public static final double TELEOP_SPEED = 1;
        public static final double MAX_VELOCITY_METERS = 6380.0 / 60.0
                * SdsModuleConfigurations.MK4I_L2.getDriveReduction()
                * SdsModuleConfigurations.MK4I_L2.getWheelDiameter() * Math.PI;

        public static final double MAX_VOLTAGE = 12;

        public static final double MAX_ANG_ACCEL = MAX_VELOCITY_METERS
                / Math.hypot(Constants.Drivetrain.TRACKWIDTH_METERS / 2.0, Constants.Drivetrain.WHEELBASE_METERS / 2.0);
    }

    public static final class Drivetrain {
        public static final int PIGEON_ID = 13;
        public static final double WHEELBASE_METERS = 0.749;
        public static final double TRACKWIDTH_METERS = 0.749;
        public static final SwerveConstants FRONT_LEFT = new SwerveConstants(1, 2, 9, 188.7);
        public static final SwerveConstants FRONT_RIGHT = new SwerveConstants(3, 4, 10, 340.83);
        public static final SwerveConstants BACK_LEFT = new SwerveConstants(5, 6, 11, 90);
        public static final SwerveConstants BACK_RIGHT = new SwerveConstants(7, 8, 12, 225);
    }

    public static final class Magazine {
        public static final int LOWER_MOTOR = 16;
        public static final int UPPER_MOTOR = 17;
        public static final int LOWER_SENSOR = 1;
        public static final int UPPER_SENSOR = 0;

        public static final double UPPER_SENSOR_THRESHOLD = 1000;
        public static final double LOWER_SENSOR_THRESHOLD = 1000;
    }

    public static final class Intake {
        public static final int MOTOR_ID = 18;
        public static final int SOLENOID_ID = 0;
    }

    public static final class Shooter {
        public static final int LEFT_MOTOR_ID = 19;
        public static final int RIGHT_MOTOR_ID = 20;
        public static final int HOOD_MOTOR_ID = 21;

        public static final double kP = .25;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kF = 0.0639;
    }

    public static final class Climber {
        public static final int LEFT_MOTOR_ID = 14;
        public static final int RIGHT_MOTOR_ID = 15;
        public static final int SOLENOID_ID = 1;

        public static final double kP = 0.5;
        public static final double kI = 0;
        public static final double kD = 0;
    }

    public static final class Auto {
        private static final double MAX_ANG_VEL_RAD_AUTO = .4 * Math.PI; // .25

        public static final TrapezoidProfile.Constraints ROT_PROFILE = new TrapezoidProfile.Constraints(
                MAX_ANG_VEL_RAD_AUTO, Constants.Swerve.MAX_ANG_ACCEL);

        public static final PIDController X_PID_CONTROLLER = new PIDController(5, 0, 0); // 5
        // y distance PID controller
        public static final PIDController Y_PID_CONTROLLER = new PIDController(5, 0, 0); // 5, 0, .0 0.3, 0.4, 4
        // ROTATION (angle) PID controller
        public static final ProfiledPIDController ROT_PID_CONTROLLER = new ProfiledPIDController(-3, 0, 0, // .85 works
                ROT_PROFILE);

    }

}
