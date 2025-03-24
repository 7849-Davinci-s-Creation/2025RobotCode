package frc.robot.commands.autos;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.Constants.CoralLevel;
import frc.robot.subsystems.Elevator;
import lib.Commands.TimedCommand;

public class ScoreCoral extends TimedCommand {
    private final CoralLevel coralLevel;
    private final Elevator elevator;

    public ScoreCoral(CoralLevel coralLevel, Elevator elevator, double seconds, Subsystem... requirments) {
        super(seconds, requirments);

        this.coralLevel = coralLevel;
        this.elevator = elevator;
    }

    @Override
    public void init() {

    }

    @Override
    public void exec() {
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
    public void end() {

    }
}
