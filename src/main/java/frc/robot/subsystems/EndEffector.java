package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public final class EndEffector extends SubsystemBase implements NiceSubsystem {
    private static EndEffector instance;

    private final SparkMax intakeMotor1;
    private final SparkMax pivotMotor1;

    private final RelativeEncoder pivotEncoder;

    private final ProfiledPIDController pidPivotcontroller;
    private final ArmFeedforward pivotFeedForward;

    private EndEffector() {
        intakeMotor1 = new SparkMax(Constants.EndEffectorConstants.INTAKEMOTOR1_CANID, MotorType.kBrushless);
        final SparkMax intakeMotor2 = new SparkMax(Constants.EndEffectorConstants.INTAKEMOTOR2_CANID,
                MotorType.kBrushless);

        intakeMotor1.clearFaults();
        intakeMotor2.clearFaults();

        final SparkBaseConfig intakeMotor1Config = new SparkMaxConfig().idleMode(IdleMode.kBrake);
        final SparkBaseConfig intakeMotor2Config = new SparkMaxConfig().idleMode(IdleMode.kBrake);
        intakeMotor2Config.follow(Constants.EndEffectorConstants.INTAKEMOTOR1_CANID);

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

    }

    public static EndEffector getInstance() {
        if (instance == null) {
            instance = new EndEffector();
        }

        return instance;
    }

    public Runnable intake() {
        return () -> intakeMotor1.set(0.5);
    }

    public Runnable outTake() {
        return () -> intakeMotor1.set(-0.5);
    }

    public Runnable stop() {
        return () -> intakeMotor1.set(0);
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
        return (pivotEncoder.getPosition() * 360) * 100;
    }

    @Override
    public void periodic() {

    }

    @Override
    public void initialize() {

    }
}
