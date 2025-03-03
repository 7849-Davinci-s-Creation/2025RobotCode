package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.generated.TunerConstants;

import static edu.wpi.first.units.Units.*;

/**
 * PLEASE NOTE THAT ANY MEASUREMENTS MUST BE DONE IN IMPERIAL UNITS,
 * Any Constants you are defining you must specify what units they are, (ex:
 * ELEVATOR_MAXHEIGHT_INCHES)
 */
public final class Constants {
    public static final class OperatorConstants {
        public static final int DRIVER_CONTROLLER_PORT = 1;
        public static final double DRIVER_CONTROLLER_DEADBAND = 0.1;
        public static final double SLIGHT_CREEP_NERF = 5;
        public static final double MAJOR_CREEP_NERF = 10;;
        public static final int OPERATOR_CONTROLLER_PORT = 0;
    }

    public static final class DriveTrainConstants {
        /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
        public static final Rotation2d BLUE_ALLIANCE_PERSPECTIVE_ROTATION = Rotation2d.kZero;
        /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
        public static final Rotation2d RED_ALLIANCE_PERSPECTIVE_ROTATION = Rotation2d.k180deg;

        public static final double SIM_LOOP_PERIOD = 0.005; // 5 ms
        public static final double MAX_SPEED = TunerConstants.SPEED_AT_12_VOLTS.in(MetersPerSecond);

        public static final double MAX_ANGULAR_RATE = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

        public static final double DRIVE_KP = 0.16618;
        public static final double DRIVE_KI = 0;
        public static final double DRIVE_KD = 0;
        public static final double DRIVE_KS = 0.086163;
        public static final double DRIVE_KV = 0.11562;
        
        public static final double STEER_KP = 100;
        public static final double STEER_KI = 0;
        public static final double STEER_KD = 0;
        public static final double STEER_KS = 0.077023;
        public static final double STEER_KV = 2.4786;
        public static final double STEER_KA = 0.077097;

        public static final double ROTATION_KP = 0.094511;
        public static final double ROTATION_KI = 0;
        public static final double ROTATION_KD = 0;
        public static final double ROTATION_KS = 0.052885;
        public static final double ROTATION_KV = 0.11388;
        public static final double ROTATION_KA = 0.0049551;
    }

    public static final class ElevatorConstants {
        public static final double SYSID_RAMP_RATE = .25; // default 1
        public static final double SYSID_STEP_VOLTS = 3; // default 7
        public static final double SYSID_TIMEOUT = 10; // default 10

        // FIND ME
        public static final int MOTOR1_CANID = 0;
        public static final int MOTOR2_CANID = 0;

        // CALCULATE THIS
        public static final double ENCODER_CONVERSION_FACTOR = 0;

        // SYS ID FOR THESE
        public static final double FF_S = 0;
        public static final double FF_G = 0;
        public static final double FF_V = 0;

        public static final double PC_P = 0;
        public static final double PC_I = 0;
        public static final double PC_D = 0;

        // CALCULATE THIS
        public static final double MAX_VELOCITY_MPS = 0;
        public static final double MAX_ACCELERATION_MPS2 = 0;

        public static final double ELEVATOR_MAXHEIGHT_INCHES = 55.5;
    }

    public static final class VisionConstants {
        public static final double CAMERA_MOUNT_HEIGHT_INCHES = 18.5;
    }

    public static final class ClimberConstants {
        public static final int MOTOR1_CANID = 55;
        public static final int MOTOR2_CANID = 54;
    }

    public static final class EndEffectorConstants {
        public static final int INTAKEMOTOR1_CANID = 56;
        public static final int INTAKEMOTOR2_CANID = 57;
    }
}
