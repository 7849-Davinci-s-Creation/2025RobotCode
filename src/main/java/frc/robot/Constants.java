package frc.robot;

public final class Constants {
    public static final class OperatorConstants {
        public static final int DRIVER_CONTROLLER_PORT = 1;
        public static final double DRIVER_CONTROLLER_DEADBAND = 0.1;
        public static final double SLIGHT_CREEP_NERF = 5;
        public static final double MAJOR_CREEP_NERF = 8;
    }

    public static final class DriveTrainConstants {
        public static final double CHASSIS_RADIUS = 0.388;
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
        public static final double MAX_VELOCITY = 0;
        public static final double MAX_ACCELERATION = 0;

        // FIND THIS
        public static final double ELEVATOR_MAXHEIGHT = 0;
    }
}
