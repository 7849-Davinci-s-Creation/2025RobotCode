package frc.robot;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.generated.TunerConstants;

import java.util.HashMap;

import static edu.wpi.first.units.Units.*;

/**
 * PLEASE NOTE THAT ANY MEASUREMENTS MUST BE DONE IN IMPERIAL UNITS,
 * Any Constants you are defining you must specify what units they are, (ex:
 * ELEVATOR_MAXHEIGHT_INCHES)
 */
public final class Constants {
    public static final class OperatorConstants {
        public static final int DRIVER_CONTROLLER_PORT = 1;
        public static final double DRIVER_CONTROLLER_DEADBAND = 0.05;
        public static final double SLIGHT_CREEP_NERF_DRIVE = 1;
        public static final double SLIGHT_CREEP_NERF_ROTATE = 1;
        public static final double MAJOR_CREEP_NERF_DRIVE = 0.5;
        public static final double MAJOR_CREEP_NERF_ROTATE = 0.5;
        public static final int OPERATOR_CONTROLLER_PORT = 0;
    }

    public static final class DriveTrainConstants {
        /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
        public static final Rotation2d BLUE_ALLIANCE_PERSPECTIVE_ROTATION = Rotation2d.kZero;
        /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
        public static final Rotation2d RED_ALLIANCE_PERSPECTIVE_ROTATION = Rotation2d.k180deg;

        public static final double SIM_LOOP_PERIOD = 0.005; // 5 ms
        public static final double MAX_SPEED = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);

        public static final double STEMNIGHT_NERF = MetersPerSecond.of(2.5).in(MetersPerSecond);

        public static final double MAX_ANGULAR_RATE = RotationsPerSecond.of(0.5).in(RadiansPerSecond);

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
    }

    public static final class ElevatorConstants {
        public static final double SYSID_RAMP_RATE = .25; // default 1
        public static final double SYSID_STEP_VOLTS = 3; // default 7
        public static final double SYSID_TIMEOUT = 10; // default 10

        public static final int LIMIT_SWITCH_PORT = 0;

        // FIND ME
        public static final int MOTOR1_CANID = 1;
        public static final int MOTOR2_CANID = 51;

        // 2(gear_ratio*(pi*sprocket_pitch_diameter))
        public static final double GEAR_RATIO = 8.45;
        public static final double SPROCKET_PITCH_DIAMETER_METERS = 0.048514;
        public static final double SPROCKET_PITCH_RADIUS = SPROCKET_PITCH_DIAMETER_METERS / 2;
        public static final double SPROCKET_PITCH_CIRCUMFRANCE = Math.PI * SPROCKET_PITCH_DIAMETER_METERS;
        public static final double ENCODER_CONVERSION_FACTOR = 2 * (GEAR_RATIO * SPROCKET_PITCH_CIRCUMFRANCE);

        // SYS ID FOR THESE
        public static final double FF_S = 0.05;
        public static final double FF_G = 0.1;
        public static final double FF_V = 0.05;
        public static final double FF_A = 0;

        public static final double PC_P = 60;
        public static final double PC_I = 0;
        public static final double PC_D = 0;

        // CALCULATE THIS
        public static final double MAX_VELOCITY_MPS = 0.98144;
        public static final double MAX_ACCELERATION_MPS2 = 4;

        public static final double ELEVATOR_MAXHEIGHT_METERS = 1.6782701159578037;
    }

    public static final class VisionConstants {
        public static final double CAMERA_MOUNT_HEIGHT_INCHES = 18.5;

        public static final Matrix<N3, N1> SINGLE_TAG_DEVIATION = VecBuilder.fill(.1, .2, .3);
        public static final Matrix<N3, N1> MULTI_TAG_DEVIATION = VecBuilder.fill(.1, .1, .2);

        public static final String FRONT_LEFT_CAMERA_NAME = "frontleftcamera";
        public static final String FRONT_RIGHT_CAMERA_NAME = "frontrightcamera";
        public static final String BACK_LEFT_CAMERA_NAME = "backleftcamera";
        public static final String BACK_RIGHT_CAMERA_NAME = "backrightcamera";
    }

    public static final class ClimberConstants {
        public static final int MOTOR1_CANID = 55;
    }

    public static final class EndEffectorConstants {
        // FIND THE CAN IDS
        public static final int INTAKEMOTOR1_CANID = 3;
        public static final int INTAKEMOTOR2_CANID = 4;

        // find the pivot motors canID
        public static final int PIVOTMOTOR1_CANID = 40;

        public static final int ALGAEREMOVER_MOTOR_CANID = 54;

        public static final int LIMIT_SWITCH_PORT = 2;

        // SYS ID FOR THESE
        public static final double PIVOT_P = 20;
        public static final double PIVOT_I = 0;
        public static final double PIVOT_D = 0.03;

        // CALCULATE THIS
        public static final double MAX_VELOCITY_MPS = 2;
        public static final double MAX_ACCELERATION_MPS2 = 2.5;

        // SYSID FOR THIS
        public static final double PIVOT_S = 0.01;
        public static final double PIVOT_G = 0.5;
        public static final double PIVOT_V = 0;
        public static final double PIVOT_A = 0;

        // FIND THIS
        public static final double MAX_ANGLE_DEGREES = 135;

        public static final double GEAR_RATIO = 90;

        public static final double SYSID_RAMP_RATE = .25; // default 1
        public static final double SYSID_STEP_VOLTS = 3; // default 7
        public static final double SYSID_TIMEOUT = 5; // default 10
    }

    public static final class LEDConstants {
        public static final int CANDLE_ID = 10;

        public static final int NUMBER_LED = 31;
    }

    public static final class FieldConstants {
        public static final class Node {
            private final Pose2d redPose;
            private final Pose2d bluePose;

            public Node(Pose2d redPose, Pose2d bluePose) {
                this.redPose = redPose;
                this.bluePose = bluePose;
            }
        }

        public static final HashMap<String, Node> NODES = new HashMap<>();

        // FIND ALL OF THESE
        public static final Node NODE1_POSE = new Node(null, null);
        public static final Node NODE2_POSE = new Node(null, null);;
        public static final Node NODE3_POSE = new Node(null, null);;
        public static final Node NODE4_POSE = new Node(null, null);;
        public static final Node NODE5_POSE = new Node(null, null);;
        public static final Node NODE6_POSE = new Node(null, null);;
        public static final Node NODE7_POSE = new Node(null, null);;
        public static final Node NODE8_POSE = new Node(null, null);;
        public static final Node NODE9_POSE = new Node(null, null);;
        public static final Node NODE10_POSE = new Node(null, null);;
        public static final Node NODE11_POSE = new Node(null, null);;
        public static final Node NODE12_POSE = new Node(null, null);;

        public static Pose2d getScoringNodePose(String nodeName, DriverStation.Alliance alliance) {
            final Node wantedNode = NODES.get(nodeName);

            if (alliance.equals(DriverStation.Alliance.Red)) {
                return wantedNode.redPose;
            }

            return wantedNode.bluePose;
        }

        static {
            NODES.put("node1", NODE1_POSE);
            NODES.put("node2", NODE2_POSE);
            NODES.put("node3", NODE3_POSE);
            NODES.put("node4", NODE4_POSE);
            NODES.put("node5", NODE5_POSE);
            NODES.put("node6", NODE6_POSE);
            NODES.put("node7", NODE7_POSE);
            NODES.put("node8", NODE8_POSE);
            NODES.put("node9", NODE9_POSE);
            NODES.put("node10", NODE10_POSE);
            NODES.put("node11", NODE11_POSE);
            NODES.put("node12", NODE12_POSE);
        }

        public static final double RED_LEFT_FEEDERSTATION_DEGREES = 0;
        public static final double RED_RIGHT_FEEDERSTATION_DEGREES = 0;

        public static final double BLUE_LEFT_FEEDERSTATION_DEGREES = 127;
        public static final double BLUE_RIGHT_FEEDERSTATION_DEGREES = -127;

        public static final double LOWER_ALGAE_DISTANCE_METERS = 0.75;

        public static final double L4_ELEVATOR_DISTANCE_METERS = 1.35;

        public static final double L3_ELEVATOR_DISTANCE_METERS = 0.55;

        public static final double L2_ELEVATOR_DISTANCE_METERS = 0.15;

        public static final double L1_ELEVATOR_DISTANCE_METERS = 0;

        public static final double INTAKE_HEIGHT_METERS = 0.29;
    }

    public enum FeederStation {
        LEFT, RIGHT
    }

    public enum CoralLevel {
        L1,L2,L3,L4,LA
    }
}
