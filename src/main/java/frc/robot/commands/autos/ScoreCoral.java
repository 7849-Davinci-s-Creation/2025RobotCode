package frc.robot.commands.autos;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.CoralLevel;
import frc.robot.subsystems.Elevator;

public class ScoreCoral extends Command {
    private final CoralLevel coralLevel;
    private final Elevator elevator;

    public ScoreCoral(CoralLevel coralLevel, Elevator elevator) {
        this.coralLevel = coralLevel;
        this.elevator = elevator;

        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        Runnable runnable;

        switch (coralLevel) {
            case L1 -> {
                runnable = () -> elevator.setGoal(Constants.FieldConstants.L1_ELEVATOR_DISTANCE_METERS);
                break;
            }

            case L2 -> {
                runnable = () -> elevator.setGoal(Constants.FieldConstants.L2_ELEVATOR_DISTANCE_METERS);
                break;
            }

            case L3 -> {
                runnable = () -> elevator.setGoal(Constants.FieldConstants.L3_ELEVATOR_DISTANCE_METERS);
            }

            case L4 -> {
                runnable = () -> elevator.setGoal(Constants.FieldConstants.L4_ELEVATOR_DISTANCE_METERS);
            }

            default -> runnable = () -> {
                DriverStation.reportError("ERROR WHEN TRYING TO SCORE CORAL IN AUTO", true);
            };
        }

        runnable.run();
    }

    @Override
    public void end(boolean interuppted) {

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
