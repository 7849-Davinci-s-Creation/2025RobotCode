package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public final class EndEffector extends SubsystemBase implements NiceSubsystem {
    private static EndEffector instance;

    public static EndEffector getInstance() {
        if (instance == null) {
            instance = new EndEffector();
        }

        return instance;
    }

    @Override
    public void periodic() {

    }

    @Override
    public void initialize() {

    }
}
