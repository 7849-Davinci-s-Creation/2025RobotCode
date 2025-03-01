package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public final class Climber extends SubsystemBase implements NiceSubsystem {
    private static Climber instance;

    private final SparkMax motor1;
    private final SparkMax motor2;
    private final SparkBaseConfig motor1Config;
    private final SparkBaseConfig motor2Config;

    private Climber() {
        motor1 = new SparkMax(Constants.ClimberConstants.MOTOR1_CANID, MotorType.kBrushless);
        motor2 = new SparkMax(Constants.ClimberConstants.MOTOR2_CANID, MotorType.kBrushless);

        motor1Config = new SparkMaxConfig().idleMode(IdleMode.kBrake);
        motor2Config = new SparkMaxConfig().idleMode(IdleMode.kBrake);
        motor2Config.follow(Constants.ClimberConstants.MOTOR1_CANID);

        motor1.configure(motor1Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        motor2.configure(motor2Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public static Climber getInstance() {
        if (instance == null) {
            instance = new Climber();
        }

        return instance;
    }

    public void climb() {
        // slow climb for now
        motor1.set(0.10);
    }

    @Override
    public void periodic() {

    }

    @Override
    public void initialize() {

    }
}
