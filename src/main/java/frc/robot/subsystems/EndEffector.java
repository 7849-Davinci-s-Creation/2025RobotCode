package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SmartMotionConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import frc.robot.Constants;

public final class EndEffector extends SubsystemBase implements NiceSubsystem {
    private static EndEffector instance;

    private final SparkMax intakeMotor1;
    private final SparkMax intakeMotor2;
    private final SparkMax pivotMotor1;

    private final RelativeEncoder pivotEncoder;

    private final ProfiledPIDController pidPivotcontroller;
    private final ArmFeedforward pivotFeedForward;

    private final SysIdRoutine routine;
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

        pivotMotor1 = new SparkMax(Constants.EndEffectorConstants.PIVOTMOTOR1_CANID, MotorType.kBrushless);
        SparkMax pivotMotor2 = new SparkMax(Constants.EndEffectorConstants.PIVOTMOTOR2_CANID, MotorType.kBrushless);

        pivotMotor1.clearFaults();
        pivotMotor2.clearFaults();

        final SparkBaseConfig pivotMotor1Config = new SparkMaxConfig().idleMode(IdleMode.kBrake);
        final SparkBaseConfig pivotMotor2Config = new SparkMaxConfig().idleMode(IdleMode.kBrake);
        pivotMotor2Config.follow(Constants.EndEffectorConstants.PIVOTMOTOR1_CANID);

        pivotMotor1.configure(pivotMotor1Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        pivotMotor2.configure(pivotMotor2Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        pivotEncoder = pivotMotor1.getEncoder();

        pidPivotcontroller = new ProfiledPIDController(Constants.EndEffectorConstants.PIVOT_P,
                Constants.EndEffectorConstants.PIVOT_I, Constants.EndEffectorConstants.PIVOT_D,
                new TrapezoidProfile.Constraints(Constants.EndEffectorConstants.MAX_VELOCITY_MPS,
                        Constants.EndEffectorConstants.MAX_ACCELERATION_MPS2));

        pivotFeedForward = new ArmFeedforward(Constants.EndEffectorConstants.PIVOT_S,
                Constants.EndEffectorConstants.PIVOT_G, Constants.EndEffectorConstants.PIVOT_V,
                Constants.EndEffectorConstants.PIVOT_A);

        // SYS ID CRAP
        final Config sysIDConfig = new Config(
                Volts.of(Constants.EndEffectorConstants.SYSID_RAMP_RATE).per(Second),
                Volts.of(Constants.EndEffectorConstants.SYSID_STEP_VOLTS),
                Seconds.of(Constants.EndEffectorConstants.SYSID_TIMEOUT));

        routine = new SysIdRoutine(sysIDConfig, new SysIdRoutine.Mechanism(pivotMotor1::setVoltage,
                (log) -> {
                    log.motor("pivotMotor1").voltage(appliedVoltage.mut_replace(
                            pivotMotor1.get() * RobotController.getBatteryVoltage(), Volts))
                            .angularPosition(pivotPosition.mut_replace(
                                    pivotEncoder.getPosition(), Degrees))
                            .linearVelocity(pivotVelocity.mut_replace(
                                    pivotEncoder.getVelocity(), InchesPerSecond));
                },
                this));
    }

    public static EndEffector getInstance() {
        if (instance == null) {
            instance = new EndEffector();
        }

        return instance;
    }

    public Runnable intake() {
        return () -> {
            intakeMotor1.set(0.5);
            intakeMotor2.set(0.5);
        };
    }

    public Runnable outake() {
        return () -> {
            intakeMotor1.set(-0.3);
            intakeMotor2.set(-0.3);
        };
    }

    public Runnable stopIntake() {
        return () -> {
            intakeMotor1.set(0);
            intakeMotor2.set(0);
        };
    }

    public Runnable stopPivot() {
        return () -> {
            pivotMotor1.set(0);
        };
    }

    public Runnable stopAll() {
        return () -> {
            stopIntake();
            stopPivot();
        };
    }

    public Runnable runPivotMotorsDown() {
        return () -> pivotMotor1.set(-1);
    }

    public Runnable runPivotMotorsUp() {
        return () -> pivotMotor1.set(1);
    }

    public void pivot(double angle) {
        double clampedAngle = MathUtil.clamp(angle, 0, Constants.EndEffectorConstants.MAX_ANGLE);

        pivotMotor1.setVoltage(pidPivotcontroller.calculate(getDegrees(), clampedAngle) + pivotFeedForward
                .calculate(getDegrees() / (Math.PI / 180), pidPivotcontroller.getSetpoint().velocity));

    }

    private double getDegrees() {
        return (pivotEncoder.getPosition() * 360) * Constants.EndEffectorConstants.GEAR_RATIO;
    }

    public Command sysIDQuasistatic(SysIdRoutine.Direction direction) {
        return routine.quasistatic(direction);
    }

    public Command sysIDDynamic(SysIdRoutine.Direction direction) {
        return routine.dynamic(direction);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("EndEffector Angle (Degrees)", getDegrees());
        SmartDashboard.putNumber("EndEffector Velocity (RPM)", pivotEncoder.getVelocity());
    }

    @Override
    public void initialize() {

    }
}
