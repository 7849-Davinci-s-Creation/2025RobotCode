package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public final class AlignToNode extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final DriverStation.Alliance alliance;
    private final String scoringNode;

    public AlignToNode(CommandSwerveDrivetrain drivetrain, DriverStation.Alliance alliance, String scoringNode) {
        this.drivetrain = drivetrain;
        this.alliance = alliance;
        this.scoringNode = scoringNode;

        addRequirements(drivetrain);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        Pose2d target = Constants.NodeConstants.getScoringNodePose(scoringNode, alliance);

        drivetrain.pathfind(target);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
