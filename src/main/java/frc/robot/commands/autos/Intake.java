package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.EndEffector;
import lib.Commands.TimedCommand;

public class Intake extends TimedCommand {
    private final EndEffector endEffector;
    private final Elevator elevator;

    public Intake(EndEffector endEffector, Elevator elevator, double seconds, Subsystem... requirements) {
        super(seconds, requirements);

        this.endEffector = endEffector;
        this.elevator = elevator;
    }

    @Override
    public void init() {

    }

    @Override
    public void exec() {
        endEffector.intake().run();
        elevator.goToSetpoint(Constants.FieldConstants.INTAKE_HEIGHT_METERS);
    }

    @Override
    public void end() {
        endEffector.stopIntake().run();
    }
    
}
