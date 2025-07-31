package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.EndEffector;

public class ZeroAlgaeRemover extends Command {
    private final EndEffector endEffector;

    public ZeroAlgaeRemover(EndEffector endEffector) {
        this.endEffector = endEffector;

        addRequirements(endEffector);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        //elevator.runElevatorDown(0.30).run();

        endEffector.zeroAlgaeRemover().run();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return endEffector.getAlgaeRemoverPosition() <= 0;
    }

}

