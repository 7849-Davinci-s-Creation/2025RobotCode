// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.ZeroElevator;
import frc.robot.commands.ZeroEndEffector;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.EndEffector;
import frc.robot.subsystems.Vision;
import lib.RobotMethods;

public final class RobotContainer implements RobotMethods {
        // Subsystems
        private final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
        private final Climber climber = Climber.getInstance();
        private final EndEffector endEffector = EndEffector.getInstance();
        private final Vision vision = Vision.getInstance();
        private final Elevator elevator = Elevator.getInstance();

        // Controllers
        private final CommandXboxController driverController = new CommandXboxController(
                        Constants.OperatorConstants.DRIVER_CONTROLLER_PORT);

        private final CommandXboxController operatorController = new CommandXboxController(
                        Constants.OperatorConstants.OPERATOR_CONTROLLER_PORT);

        // Everything else
        private final SendableChooser<Command> autoChooser;

        public RobotContainer() {
                // Initialize subsystems
                drivetrain.initialize();
                climber.initialize();
                vision.initialize();
                elevator.initialize();

                // ---- ALL ROBOT SUBSYSTEMS SHOULD BE INITIALIZED BEFORE DOING ANYTHING ELSE
                // IF THEY HAVE NOT THEN YOU ARE DOING SOMETHING COMPLETELY WRONG !! ----

                // the pathplanner auto builder must have been initialized before you call
                // buildAutoChooser();
                autoChooser = AutoBuilder.buildAutoChooser();
                SmartDashboard.putData(autoChooser);

                configureDefault();
                configureBindings();
        }

        private void configureDefault() {
                // Note that X is defined as forward according to WPILib convention,
                // and Y is defined as to the left according to WPILib convention.
                drivetrain.setDefaultCommand(
                                // Drivetrain will execute this command periodically
                                drivetrain.applyRequest(() -> drivetrain.getDrive()
                                                .withVelocityX(drivetrain.calculateVelocity(driverController.getLeftY(),
                                                                Constants.DriveTrainConstants.MAX_SPEED)) // Drive
                                                // forward
                                                // with
                                                // negative Y
                                                // (forward)
                                                .withVelocityY(drivetrain.calculateVelocity(driverController.getLeftX(),
                                                                Constants.DriveTrainConstants.MAX_SPEED)) // Drive
                                                // left
                                                // with
                                                // negative
                                                // X
                                                // (left)
                                                .withRotationalRate(drivetrain.calculateVelocity(
                                                                driverController.getRightX(),
                                                                Constants.DriveTrainConstants.MAX_ANGULAR_RATE)) // Drive
                                // counterclockwise
                                // with
                                // negative X (left)
                                ));
        }

        private void configureBindings() {
                // THIS IS STUPID UGLY, but behavior breaks otherwise, so we keep :/
                driverController.leftTrigger().whileTrue(
                                drivetrain.applyRequest(() -> drivetrain.getDrive()
                                                .withVelocityX(drivetrain.calculateVelocity(driverController.getLeftY(),
                                                                Constants.OperatorConstants.SLIGHT_CREEP_NERF_DRIVE)) // Drive
                                                // forward
                                                // with
                                                // negative Y
                                                // (forward)
                                                .withVelocityY(drivetrain.calculateVelocity(driverController.getLeftX(),
                                                                Constants.OperatorConstants.SLIGHT_CREEP_NERF_DRIVE)) // Drive
                                                // left
                                                // with
                                                // negative
                                                // X
                                                // (left)
                                                .withRotationalRate(drivetrain.calculateVelocity(
                                                                driverController.getRightX(),
                                                                Constants.OperatorConstants.SLIGHT_CREEP_NERF_ROTATE)) // Drive
                                // counterclockwise
                                // with
                                // negative X (left)
                                ));

                driverController.rightTrigger().whileTrue(
                                drivetrain.applyRequest(() -> drivetrain.getDrive()
                                                .withVelocityX(drivetrain.calculateVelocity(driverController.getLeftY(),
                                                                Constants.OperatorConstants.MAJOR_CREEP_NERF_DRIVE)) // Drive
                                                // forward
                                                // with
                                                // negative Y
                                                // (forward)
                                                .withVelocityY(drivetrain.calculateVelocity(driverController.getLeftX(),
                                                                Constants.OperatorConstants.MAJOR_CREEP_NERF_DRIVE)) // Drive
                                                // left
                                                // with
                                                // negative
                                                // X
                                                // (left)
                                                .withRotationalRate(drivetrain.calculateVelocity(
                                                                driverController.getRightX(),
                                                                Constants.OperatorConstants.MAJOR_CREEP_NERF_ROTATE) // Drive
                                                // counterclockwise
                                                // with
                                                // negative X (left)
                                                )));

                // // Run SysId routines when holding back/start and X/Y.
                // // Note that each routine should be run exactly once in a single log.
                driverController.leftBumper().and(driverController.y())
                                .whileTrue(drivetrain.sysIdDynamic(SysIdRoutine.Direction.kForward));
                driverController.leftBumper().and(driverController.x())
                                .whileTrue(drivetrain.sysIdDynamic(SysIdRoutine.Direction.kReverse));
                driverController.rightBumper().and(driverController.y())
                                .whileTrue(drivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
                driverController.rightBumper().and(driverController.x())
                                .whileTrue(drivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));

                // reset the field-centric heading on left bumper press
                driverController.back().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

                driverController.a().whileTrue(drivetrain.applyRequest(drivetrain::getBrake));

                driverController.b().whileTrue(drivetrain.applyRequest(
                                () -> drivetrain.getPoint().withModuleDirection(
                                                new Rotation2d(-driverController.getLeftY(),
                                                                -driverController.getLeftX()))));

                driverController.leftBumper().whileTrue(
                                Commands.runOnce(
                                                () -> drivetrain.setControl(drivetrain.driveWithFeederStationAngle(
                                                                DriverStation.getAlliance().get(),
                                                                Constants.FeederStation.LEFT))));

                driverController.rightBumper().whileTrue(
                                Commands.runOnce(
                                                () -> drivetrain.setControl(drivetrain.driveWithFeederStationAngle(
                                                                DriverStation.getAlliance().get(),
                                                                Constants.FeederStation.RIGHT))));

                // operator controller

                operatorController.leftTrigger().and(operatorController.rightTrigger())
                                .whileTrue(Commands.runOnce(climber.climb()))
                                .onFalse(Commands.runOnce(climber.stop()));

                operatorController.rightTrigger().whileTrue(Commands.runOnce(climber.lowerClimber()))
                                .onFalse(Commands.runOnce(climber.stop()));

                // END EFFECTOR
                operatorController.leftBumper().whileTrue(
                                new ParallelCommandGroup(
                                                Commands.runOnce(endEffector.intake()),
                                                endEffector.setGoal(Constants.FieldConstants.INTAKE_ANGLE_DEGREES)))
                                .onFalse(
                                                // zero end effector
                                                new ParallelCommandGroup(
                                                                Commands.runOnce(endEffector.stopAlgaeAndIntake()),
                                                                Commands.runOnce(endEffector.stopPivot()),
                                                                new ZeroEndEffector(endEffector)));

                operatorController.rightBumper().whileTrue(Commands.runOnce(endEffector.outake()))
                                .onFalse(Commands.runOnce(endEffector.stopAlgaeAndIntake()));

                operatorController.povLeft().whileTrue(Commands.runOnce(endEffector.runPivotMotorsUp()))
                                .onFalse(Commands.runOnce(endEffector.stopPivot()));
                operatorController.povRight().whileTrue(Commands.runOnce(endEffector.runPivotMotorsDown()))
                                .onFalse(Commands.runOnce(endEffector.stopPivot()));

                // scoring positions
                // L4
                operatorController.y().whileTrue(scoreCoral(Constants.CoralLevel.L4)).onFalse(zeroMechanisms());

                // L3
                operatorController.b().whileTrue(scoreCoral(Constants.CoralLevel.L3)).onFalse(zeroMechanisms());

                // l2
                operatorController.x().whileTrue(scoreCoral(Constants.CoralLevel.L2)).onFalse(zeroMechanisms());

                // l1
                operatorController.a().whileTrue(scoreCoral(Constants.CoralLevel.L1)).onFalse(zeroMechanisms());

                // ELEVATOR
                operatorController.povUp().whileTrue(Commands.runOnce(elevator.runElevatorUp()))
                                .onFalse(Commands.runOnce(elevator.pleaseStop()));
                operatorController.povDown().whileTrue(Commands.runOnce(elevator.runElevatorDown()))
                                .onFalse(Commands.runOnce(elevator.pleaseStop()));

                // zero end effector / elevator's encoder
                operatorController.back().onTrue(Commands.runOnce(elevator.zeroEncoder()));
                operatorController.start().onTrue(Commands.runOnce(endEffector.zeroPivotEncoder()));
        }

        public Command scoreCoral(Constants.CoralLevel coralLevel) {
                return switch (coralLevel) {
                        case L1 -> new ParallelCommandGroup(
                                        endEffector.setGoal(
                                                        Constants.FieldConstants.L1_ENDEFFECTOR_ANGLE_DEGREES),
                                        elevator.setGoal(Constants.FieldConstants.L1_ELEVATOR_DISTANCE_METERS));

                        case L2 -> new ParallelCommandGroup(
                                        endEffector.setGoal(
                                                        Constants.FieldConstants.L2_ENDEFFECTOR_ANGLE_DEGREES),
                                        elevator.setGoal(Constants.FieldConstants.L2_ELEVATOR_DISTANCE_METERS));

                        case L3 -> new ParallelCommandGroup(
                                        endEffector.setGoal(
                                                        Constants.FieldConstants.L3_ENDEFFECTOR_ANGLE_DEGREES),
                                        elevator.setGoal(Constants.FieldConstants.L3_ELEVATOR_DISTANCE_METERS));

                        case L4 -> new ParallelCommandGroup(
                                        endEffector.setGoal(
                                                        Constants.FieldConstants.L4_ENDEFFECTOR_ANGLE_DEGREES),
                                        elevator.setGoal(Constants.FieldConstants.L4_ELEVATOR_DISTANCE_METERS));
                };
        }

        public Command zeroMechanisms() {
                return new ParallelCommandGroup(
                                Commands.run(endEffector.stopPivot()),
                                Commands.run(elevator.pleaseStop()),
                                new ZeroElevator(elevator),
                                new ZeroEndEffector(endEffector));
        }

        public Command getAutonomousCommand() {
                return autoChooser.getSelected();
        }

        @Override
        public void robotPeriodic() {

        }

        @Override
        public void disabledInit() {

        }

        @Override
        public void disabledPeriodic() {

        }

        @Override
        public void disabledExit() {

        }

        @Override
        public void autonomousInit() {
        }

        @Override
        public void autonomousPeriodic() {

        }

        @Override
        public void autonomousExit() {

        }

        @Override
        public void teleopInit() {
        }

        @Override
        public void teleopPeriodic() {

        }

        @Override
        public void teleopExit() {

        }

        @Override
        public void testInit() {

        }

        @Override
        public void testPeriodic() {

        }

        @Override
        public void testExit() {

        }

        @Override
        public void simulationPeriodic() {

        }
}
