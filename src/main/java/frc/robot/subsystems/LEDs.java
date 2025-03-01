package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public final class LEDs extends SubsystemBase implements NiceSubsystem {
    private static LEDs instance;

    public static LEDs getInstance() {
        if (instance == null) {
            instance = new LEDs();
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
