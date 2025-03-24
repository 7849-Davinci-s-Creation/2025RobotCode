package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.EndEffector;
import lib.Commands.TimedCommand;

public class Outtake extends Command {
    private final EndEffector endEffector;

    private double startTime;

    public Outtake(EndEffector endEffector) {
        this.endEffector = endEffector;

        addRequirements(endEffector);
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        endEffector.outake();

    }

    @Override
    public void end(boolean interuppted) {
        
    }
    
    @Override
    public boolean isFinished() {
        return false;
    }
}
