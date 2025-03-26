package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.EndEffector;
import lib.Commands.TimedCommand;

public class Outtake extends TimedCommand {
    private final EndEffector endEffector;

    public Outtake(EndEffector endEffector, double seconds, Subsystem... requirments) {
        super(seconds, requirments);

        this.endEffector = endEffector;
    }

    @Override
    public void init() {

    }

    @Override
    public void exec() {
        endEffector.outake().run();
    }

    @Override
    public void end() {
        endEffector.stopIntake().run();
    }
}
