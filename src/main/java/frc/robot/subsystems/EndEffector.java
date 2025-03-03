package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public final class EndEffector extends SubsystemBase implements NiceSubsystem {
    private static EndEffector instance;

    private final SparkMax intakeMotor1;
    private final SparkMax intakeMotor2;
    private final SparkBaseConfig intakeMotor1Config;
    private final SparkBaseConfig intakeMotor2Config;

    private EndEffector() {
        intakeMotor1 = new SparkMax(Constants.EndEffectorConstants.INTAKEMOTOR1_CANID, MotorType.kBrushless);
        intakeMotor2 = new SparkMax(Constants.EndEffectorConstants.INTAKEMOTOR2_CANID, MotorType.kBrushless);

        intakeMotor1.clearFaults();
        intakeMotor2.clearFaults();

        intakeMotor1Config = new SparkMaxConfig().idleMode(IdleMode.kBrake);
        intakeMotor2Config = new SparkMaxConfig().idleMode(IdleMode.kBrake);
        intakeMotor2Config.follow(Constants.EndEffectorConstants.INTAKEMOTOR1_CANID);

        intakeMotor1.configure(intakeMotor1Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        intakeMotor2.configure(intakeMotor2Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
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

    public Runnable outTake() {
        return () -> {
            intakeMotor1.set(-0.5);
            intakeMotor2.set(-0.5);
        };
    }

    public Runnable stop() {
        return () -> {
            intakeMotor1.set(0);
            intakeMotor2.set(0);
        };
    }

    @Override
    public void periodic() {

    }

    @Override
    public void initialize() {

    }
}
