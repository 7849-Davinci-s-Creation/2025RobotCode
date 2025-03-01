package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public final class Climber extends SubsystemBase implements NiceSubsystem {
    private static Climber instance;

    private Climber() {
    }

    public static Climber getInstance() {
        if (instance == null) {
            instance = new Climber();
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
