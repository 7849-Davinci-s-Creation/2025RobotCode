package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import frc.robot.Constants;

public final class EndEffector extends SubsystemBase implements NiceSubsystem {
    private static EndEffector instance;

    private final SparkMax intakeMotor1;
    private final SparkMax intakeMotor2;

    private final Servo servo;

    // private final SparkMax pivotMotor1;
    // private final SparkMax algaeRemoverMotor;

    // private final RelativeEncoder pivotEncoder;

    // private final DigitalInput pivotLimitSwitch;

    // private final ProfiledPIDController pidPivotcontroller;
    // private final ArmFeedforward pivotFeedForward;

    // private final SysIdRoutine routine;
    private final MutVoltage appliedVoltage = Volts.mutable(0);
    private final MutAngle pivotPosition = Degrees.mutable(0);
    private final MutLinearVelocity pivotVelocity = InchesPerSecond.mutable(0);

    private EndEffector() {
        intakeMotor1 = new SparkMax(Constants.EndEffectorConstants.INTAKEMOTOR1_CANID, MotorType.kBrushless);
        intakeMotor2 = new SparkMax(Constants.EndEffectorConstants.INTAKEMOTOR2_CANID,
                MotorType.kBrushless);

        intakeMotor1.clearFaults();
        intakeMotor2.clearFaults();

        final SparkBaseConfig intakeMotor1Config = new SparkMaxConfig().idleMode(IdleMode.kBrake);
        intakeMotor1Config.inverted(true);
        final SparkBaseConfig intakeMotor2Config = new SparkMaxConfig().idleMode(IdleMode.kBrake);

        intakeMotor1.configure(intakeMotor1Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        intakeMotor2.configure(intakeMotor2Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // pivotMotor1 = new SparkMax(Constants.EndEffectorConstants.PIVOTMOTOR1_CANID, MotorType.kBrushless);
        

        // pivotMotor1.clearFaults();
        

        final SparkBaseConfig pivotMotor1Config = new SparkMaxConfig().idleMode(IdleMode.kBrake).inverted(false);
    

        // THE NEW REV API IS ONE OF THE WORST THINGS I HAVE EVER WORKED WITH, THIS DOES
        // NOT WORK, YOU CANNOT HAVE ONE
        // MOTOR INVERTED , FOLLOWING MEANS EVERYTHING IS THE SAME :))))))))))))))))))))
        // SO SMART REV THANKS YOU TOTALLY NEEDED
        // TO CHANGE THIS API.
        // pivotMotor2Config.follow(Constants.EndEffectorConstants.PIVOTMOTOR1_CANID);

        // pivotMotor1.configure(pivotMotor1Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        // pivotEncoder = pivotMotor1.getEncoder();

        // pivotLimitSwitch = new DigitalInput(Constants.EndEffectorConstants.LIMIT_SWITCH_PORT);

        // pidPivotcontroller = new ProfiledPIDController(Constants.EndEffectorConstants.PIVOT_P,
        //         Constants.EndEffectorConstants.PIVOT_I, Constants.EndEffectorConstants.PIVOT_D,
        //         new Constraints(Constants.EndEffectorConstants.MAX_VELOCITY_MPS,
        //                 Constants.EndEffectorConstants.MAX_ACCELERATION_MPS2));

        // pivotFeedForward = new ArmFeedforward(Constants.EndEffectorConstants.PIVOT_S,
        //         Constants.EndEffectorConstants.PIVOT_G, Constants.EndEffectorConstants.PIVOT_V,
        //         Constants.EndEffectorConstants.PIVOT_A);

        // SYS ID CRAP
        final Config sysIDConfig = new Config(
                Volts.of(Constants.EndEffectorConstants.SYSID_RAMP_RATE).per(Second),
                Volts.of(Constants.EndEffectorConstants.SYSID_STEP_VOLTS),
                Seconds.of(Constants.EndEffectorConstants.SYSID_TIMEOUT));

        // routine = new SysIdRoutine(sysIDConfig, new SysIdRoutine.Mechanism(this::runMotorsForSysID,
        //         (log) -> log.motor("pivotMotor1").voltage(appliedVoltage.mut_replace(
        //                 pivotMotor1.getAppliedOutput() * pivotMotor1.getBusVoltage(), Volts))
        //                 .angularPosition(pivotPosition.mut_replace(
        //                         pivotEncoder.getPosition(), Degrees))
        //                 .linearVelocity(pivotVelocity.mut_replace(
        //                         pivotEncoder.getVelocity(), InchesPerSecond)),
        //         this));


        // algae remover prototype stuff
        servo = new Servo(1);
    }

    public static EndEffector getInstance() {
        if (instance == null) {
            instance = new EndEffector();
        }

        return instance;
    }

    // public void runMotorsForSysID(Voltage voltage) {
    //     pivotMotor1.setVoltage(-voltage.in(Volts));
    // }

    public Runnable intake() {
        return () -> {
            intakeMotor1.set(0.5);
            intakeMotor2.set(0.5);
        };
    }

    public Runnable outake() {
        return () -> {
            intakeMotor1.set(-0.15);
            intakeMotor2.set(-0.15);
        };
    }

    public Runnable stopIntake() {
        return () -> {
            intakeMotor1.set(0);
            intakeMotor2.set(0);
        };
    }

    public Runnable stopAlgaeAndIntake() {
        return () -> stopIntake().run();
    }

    public Runnable runServo() {
        return () -> servo.set(1);
    }

    // public Runnable stopPivot() {
    //     return () -> {
    //         pivotMotor1.set(0);
            
    //     };
    // }

    // public Runnable stopAll() {
    //     return () -> {
    //         stopIntake().run();
    //         stopPivot().run();
    //     };
    // }

    // public void zeroEndEffector() {
    //     pivot(0);
    // }

    // public Runnable runPivotMotorsUp() {
    //     return () -> pivotMotor1.set(-0.2);
    // }

    // public Runnable runPivotMotorsDown() {
    //     return () -> pivotMotor1.set(0.20);
    // }

    // public Runnable runPivotMotorsDown(double speed) {
    //     return () -> pivotMotor1.set(speed);
    // }

    public Angle convertAngleToSensorUnits(Angle measurment) {
        return Rotations.of(measurment.in(Rotations) * Constants.EndEffectorConstants.GEAR_RATIO);
    }

    // public void pivot(double angle) {
    //     double clampedAngle = MathUtil.clamp(angle, 0, Constants.EndEffectorConstants.MAX_ANGLE_DEGREES);
 
    //     double pidControllerResult = pidPivotcontroller.calculate(Math.toRadians(getDegrees()), Math.toRadians(clampedAngle));
    //     double ffResult = pivotFeedForward
    //             .calculate(pidPivotcontroller.getSetpoint().position, pidPivotcontroller.getSetpoint().velocity);

    //     DriverStation.reportWarning(String.valueOf( -(pidControllerResult + ffResult)), false);
        
    //     pivotMotor1.setVoltage(pidControllerResult + ffResult);
    // }

    // public Command setGoal(double angle) {
    //     return run(() -> pivot(angle));
    // }

    // public Runnable zeroPivotEncoder() {
    //     return () -> pivotEncoder.setPosition(0);
    // }

    // private double getDegrees() {
    //     return Math.abs((pivotEncoder.getPosition() * 360) / Constants.EndEffectorConstants.GEAR_RATIO);
    // }

    // public Command sysIDQuasistatic(SysIdRoutine.Direction direction) {
    //     return routine.quasistatic(direction);
    // }

    // public Command sysIDDynamic(SysIdRoutine.Direction direction) {
    //     return routine.dynamic(direction);
    // }

    // public boolean getLimitSwitch() {
    //     return pivotLimitSwitch.get();
    // }

    @Override
    public void periodic() {
        // check if we are at the bottom (in case of power cycle)
        // if (pivotLimitSwitch.get()) {
        //     pivotEncoder.setPosition(0);
        // }

        // SmartDashboard.putNumber("EndEffector Angle (Degrees)", getDegrees());
        // SmartDashboard.putNumber("EndEffector Velocity (RPM)", pivotEncoder.getVelocity());
        // SmartDashboard.putBoolean("End Effector Limit Switch", pivotLimitSwitch.get());
    }

    @Override
    public void initialize() {

    }
}
