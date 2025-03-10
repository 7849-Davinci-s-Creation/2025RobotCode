package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import frc.robot.Constants;

import static edu.wpi.first.units.Units.*;

public final class Elevator extends SubsystemBase implements NiceSubsystem {
        // Motors (plus dumb neo config crap)
        private final SparkMax motor1;

        // Encoders
        private final RelativeEncoder encoder;

        private final SysIdRoutine routine;
        private final MutVoltage appliedVoltage = Volts.mutable(0);
        private final MutDistance elevatorPosition = Inches.mutable(0);
        private final MutLinearVelocity elevatorVelocity = InchesPerSecond.mutable(0);

        // HOW WE WILL BE CONTROLLING THE ELEVATOR
        private final ElevatorFeedforward elevatorFeedforward;
        private final ProfiledPIDController positionController;

        private static Elevator instance;

        DigitalInput elevatorLimitSwitch = new DigitalInput(Constants.ElevatorConstants.LIMIT_SWITCH_PORT);

        public Elevator() {
                motor1 = new SparkMax(Constants.ElevatorConstants.MOTOR1_CANID, MotorType.kBrushless);
                final SparkMax motor2 = new SparkMax(Constants.ElevatorConstants.MOTOR2_CANID, MotorType.kBrushless);

                motor1.clearFaults();
                motor2.clearFaults();

                // DON'T FORGET TO CONFIGURE NEOS BEFORE RUNNING TESTS (find can ids, and
                // calculate conversion factor)
                SparkBaseConfig motor1Config = new SparkMaxConfig().idleMode(IdleMode.kBrake);
                SparkBaseConfig motor2Config = new SparkMaxConfig().idleMode(IdleMode.kBrake)
                                .follow(Constants.ElevatorConstants.MOTOR1_CANID);

                // Config motors
                // UNCOMMENT THIS WHEN VALUE IS CALCULATED
                // motor1Config.encoder.positionConversionFactor(Constants.ElevatorConstants.ENCODER_CONVERSION_FACTOR)
                // .velocityConversionFactor(Constants.ElevatorConstants.ENCODER_CONVERSION_FACTOR);
                // motor2Config.encoder.positionConversionFactor(Constants.ElevatorConstants.ENCODER_CONVERSION_FACTOR)
                // .velocityConversionFactor(Constants.ElevatorConstants.ENCODER_CONVERSION_FACTOR);

                motor1.configure(motor1Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
                motor2.configure(motor2Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

                encoder = motor1.getEncoder();

                // SYSID CRAP
                final Config sysIDConfig = new Config(
                                Volts.of(Constants.ElevatorConstants.SYSID_RAMP_RATE).per(Second),
                                Volts.of(Constants.ElevatorConstants.SYSID_STEP_VOLTS),
                                Seconds.of(Constants.ElevatorConstants.SYSID_TIMEOUT));

                routine = new SysIdRoutine(sysIDConfig, new SysIdRoutine.Mechanism(motor1::setVoltage,
                                (log) -> {
                                        log.motor("elevatorMotor1").voltage(appliedVoltage.mut_replace(
                                                        motor1.get() * RobotController.getBatteryVoltage(), Volts))
                                                        .linearPosition(elevatorPosition.mut_replace(
                                                                        encoder.getPosition(), Inches))
                                                        .linearVelocity(elevatorVelocity.mut_replace(
                                                                        encoder.getVelocity(), InchesPerSecond));
                                },
                                this));

                elevatorFeedforward = new ElevatorFeedforward(Constants.ElevatorConstants.FF_S,
                                Constants.ElevatorConstants.FF_G, Constants.ElevatorConstants.FF_V);

                positionController = new ProfiledPIDController(Constants.ElevatorConstants.PC_P,
                                Constants.ElevatorConstants.PC_I, Constants.ElevatorConstants.PC_D,
                                new TrapezoidProfile.Constraints(
                                                Constants.ElevatorConstants.MAX_VELOCITY_MPS,
                                                Constants.ElevatorConstants.MAX_ACCELERATION_MPS2));
        }

        public static Elevator getInstance() {
                if (instance == null) {
                        instance = new Elevator();
                }

                return instance;
        }

        public void goToSetpoint(double setPoint) {
                double clampedSetpoint = MathUtil.clamp(setPoint, 0,
                                Constants.ElevatorConstants.ELEVATOR_MAXHEIGHT_INCHES);

                double pidResult = positionController.calculate(encoder.getPosition(), clampedSetpoint);
                double ffResult = elevatorFeedforward.calculate(positionController.getSetpoint().velocity);

                // SO WE DO NOT RUN ELEVATOR TOO FAR UP / DOWN
                if (elevatorLimitSwitch.get()) {
                        motor1.set(0);
                } else if (encoder.getPosition() >= Constants.ElevatorConstants.ELEVATOR_MAXHEIGHT_INCHES) {
                        motor1.set(0);
                } else {
                        motor1.set(pidResult + ffResult);
                }
        }

        public Runnable runElevatorUp() {
                return () -> motor1.set(0.3);
        }

        public Runnable runElevatorDown() {
                if (elevatorLimitSwitch.get()) {
                        return () -> motor1.set(0);

                } else {
                        return () -> motor1.set(-0.3);
                }
        }

        public Runnable pleaseStop() {
                return () -> motor1.set(0);
        }

        @Override
        public void periodic() {
                // check if we are at the bottom of elevator and set position to 0 so we
                // don't mess our measurements up
                if (elevatorLimitSwitch.get()) {
                        encoder.setPosition(0);
                }

                SmartDashboard.putNumber("Elevator Position (Inches)", encoder.getPosition());
                SmartDashboard.putNumber("Elevator Velocity (In/S)", encoder.getVelocity());
        }

        @Override
        public void initialize() {

        }

        public Command sysIDQuasistatic(SysIdRoutine.Direction direction) {
                return routine.quasistatic(direction);
        }

        public Command sysIDDynamic(SysIdRoutine.Direction direction) {
                return routine.dynamic(direction);
        }
}
