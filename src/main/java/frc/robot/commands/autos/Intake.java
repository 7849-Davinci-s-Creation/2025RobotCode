package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.EndEffector;

public class Intake extends Command {
    private final EndEffector endEffector;
    private final Elevator elevator;

    public Intake(EndEffector endEffector, Elevator elevator, double seconds) {
        this.endEffector = endEffector;
        this.elevator = elevator;

        addRequirements(elevator, endEffector);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        endEffector.intake();
        elevator.setGoal(Constants.FieldConstants.INTAKE_HEIGHT_METERS);
    }

    @Override
    public void end(boolean interupted) {
        elevator.pleaseStop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    
}
