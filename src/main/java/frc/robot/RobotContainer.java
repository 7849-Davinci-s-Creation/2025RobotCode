// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import com.ctre.phoenix.led.RainbowAnimation;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.CoralLevel;
import frc.robot.commands.ZeroElevator;
import frc.robot.commands.autos.Intake;
import frc.robot.commands.autos.Outtake;
import frc.robot.commands.autos.ScoreCoral;
import frc.robot.commands.autos.Timer;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.EndEffector;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Vision;
import lib.RobotMethods;

public final class RobotContainer implements RobotMethods {
        // Subsystems
        private final CommandSwerveDrivetrain drivetrain;
        private final EndEffector endEffector;
        private final Elevator elevator;
        private final LED led;
        // private final Vision frontRightVision;
        // private final Vision backRightVision;
        // private final Vision backLeftVision;

        // Controllers
        private final CommandXboxController driverController = new CommandXboxController(
                        Constants.OperatorConstants.DRIVER_CONTROLLER_PORT);

        private final CommandPS4Controller operatorController = new CommandPS4Controller(
                        Constants.OperatorConstants.OPERATOR_CONTROLLER_PORT);

        // Everything else
        private final SendableChooser<Command> autoChooser;

        private final edu.wpi.first.wpilibj.Timer timer = new edu.wpi.first.wpilibj.Timer();

        public RobotContainer() {
                timer.start();

                // need to instantiate subsystems in constructor so pathplanner has
                // subsystems to reference when we register named commands.
                drivetrain = TunerConstants.createDrivetrain();
                endEffector = EndEffector.getInstance();
                elevator = Elevator.getInstance();
                led = LED.getInstance();

                // frontRightVision = new Vision(Constants.VisionConstants.FRONT_RIGHT_CAMERA_NAME,
                //                 Constants.VisionConstants.FRONTRIGHT_CAMERA_POSITION, drivetrain);

                // backRightVision = new Vision(Constants.VisionConstants.BACK_RIGHT_CAMERA_NAME,
                //                 Constants.VisionConstants.BACKRIGHT_CAMERA_POSITION, drivetrain);

                // backLeftVision = new Vision(Constants.VisionConstants.BACK_LEFT_CAMERA_NAME,
                //                 Constants.VisionConstants.BACKLEFT_CAMERA_POSITION, drivetrain);

                // Initialize subsystems
                drivetrain.initialize();
                elevator.initialize();
                led.initialize();
                // frontRightVision.initialize();

                // ---- ALL ROBOT SUBSYSTEMS SHOULD BE INITIALIZED AND INSTANTIATED BEFORE DOING
                // ANYTHING ELSE
                // IF THEY HAVE NOT THEN YOU ARE DOING SOMETHING COMPLETELY WRONG !! ----

                // the pathplanner auto builder must have been initialized before you call
                // buildAutoChooser();
                configureDefault();
                configureBindings();
                registerNamedCommands();

                // port forward photon vision dashboards
                PortForwarder.add(5800, "orangepi1.local", 5800);
                PortForwarder.add(5800, "orangepi2.local", 5801);

                autoChooser = AutoBuilder.buildAutoChooser();

                SmartDashboard.putData(autoChooser);

                led.doAnimate(new RainbowAnimation(0.75, 0.75, Constants.LEDConstants.NUMBER_LED));
        }

        private void configureDefault() {
                // Note that X is defined as forward according to WPILib convention,
                // and Y is defined as to the left according to WPILib convention.
                drivetrain.setDefaultCommand(
                                // Drivetrain will execute this command periodically
                                drivetrain.applyRequest(() -> drivetrain.getDrive()
                                                .withVelocityX(drivetrain.calculateVelocity(driverController.getLeftY(),
                                                                Constants.DriveTrainConstants.STEMNIGHT_NERF)) // Drive
                                                // forward
                                                // with
                                                // negative Y
                                                // (forward)
                                                .withVelocityY(drivetrain.calculateVelocity(driverController.getLeftX(),
                                                                Constants.DriveTrainConstants.STEMNIGHT_NERF)) // Drive
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

                // Mega super pose estimator
                // (needs to be seperated to individual cameras to update same drivetrain pose)

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

                // reset the field-centric heading on back press
                driverController.back().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

                driverController.a().whileTrue(drivetrain.applyRequest(drivetrain::getBrake));

                driverController.b().whileTrue(drivetrain.applyRequest(
                                () -> drivetrain.getPoint().withModuleDirection(
                                                new Rotation2d(-driverController.getLeftY(),
                                                                -driverController.getLeftX()))));

                driverController.leftBumper().whileTrue(
                                Commands.run(
                                                () -> drivetrain.setControl(drivetrain.driveWithFeederStationAngle(
                                                                DriverStation.getAlliance().get(),
                                                                Constants.FeederStation.LEFT))));

                driverController.rightBumper().whileTrue(
                                Commands.run(
                                                () -> drivetrain.setControl(drivetrain.driveWithFeederStationAngle(
                                                                DriverStation.getAlliance().get(),
                                                                Constants.FeederStation.RIGHT))));

                // operator controller

                operatorController.L2()
                                .whileTrue(Commands.runOnce(endEffector.intake()))
                                .onFalse(Commands.runOnce(endEffector.stopAlgaeAndIntake()));

                operatorController.R2().whileTrue(Commands.runOnce(endEffector.stopAlgaeAndIntake()))
                                .onFalse(Commands.runOnce(endEffector.stopAlgaeAndIntake()));

                // END EFFECTOR
                operatorController.L1().whileTrue(
                                new ParallelCommandGroup(
                                                Commands.runOnce(endEffector.intake()),
                                                elevator.setGoal(Constants.FieldConstants.INTAKE_HEIGHT_METERS)))
                                .onFalse(
                                                // zero end effector
                                                new ParallelCommandGroup(
                                                                Commands.runOnce(endEffector.stopAlgaeAndIntake()),
                                                                new ZeroElevator(elevator)));

                operatorController.R1().whileTrue(Commands.runOnce(endEffector.outake()))
                                .onFalse(Commands.runOnce(endEffector.stopAlgaeAndIntake()));

                // scoring positions
                // L4
                operatorController.triangle().whileTrue(elevator.goToLevel(Constants.CoralLevel.L4))
                                .onFalse(zeroMechanisms());

                // L3
                operatorController.circle().whileTrue(elevator.goToLevel(Constants.CoralLevel.L3))
                                .onFalse(zeroMechanisms());

                // l2
                operatorController.square().whileTrue(elevator.goToLevel(Constants.CoralLevel.L2))
                                .onFalse(zeroMechanisms());

                // l1
                operatorController.cross().whileTrue(
                                elevator.goToLevel(Constants.CoralLevel.L1)).onFalse(
                                                Commands.run(endEffector.stopIntake()));

                // ELEVATOR
                operatorController.povUp().whileTrue(Commands.runOnce(elevator.runElevatorUp()))
                                .onFalse(Commands.runOnce(elevator.pleaseStop()));
                operatorController.povDown().whileTrue(Commands.runOnce(elevator.runElevatorDown()))
                                .onFalse(Commands.runOnce(elevator.pleaseStop()));

                // zero end effector / elevator's encoder
                operatorController.PS().onTrue(Commands.runOnce(elevator.zeroEncoder()));

                operatorController.povLeft().whileTrue(Commands.run(endEffector.runAlgaeRemoverUp()))
                                .onFalse(Commands.run(endEffector.stopAlgaeRemover()));

                operatorController.povRight().whileTrue(Commands.run(endEffector.runAlgaeRemoverDown()))
                                .onFalse(Commands.run(endEffector.stopAlgaeRemover()));

                operatorController.L3().whileTrue(elevator.goToLevel(CoralLevel.L4))
                                .onFalse(zeroMechanisms());

                operatorController.L3().whileTrue(elevator.goToLevel(CoralLevel.HA)).onFalse(zeroMechanisms());

                operatorController.R3().whileTrue(elevator.goToLevel(CoralLevel.LA))
                                .onFalse(zeroMechanisms());
        }

        public void registerNamedCommands() {
                NamedCommands.registerCommand("intake", new SequentialCommandGroup(
                                new Intake(endEffector, elevator, 1.5, endEffector, elevator),
                                new ParallelRaceGroup(zeroMechanisms(),
                                                new Timer(1))));

                NamedCommands.registerCommand("outtake", new Outtake(endEffector, 0.5, endEffector));

                NamedCommands.registerCommand("scorel1",
                                new ScoreCoral(Constants.CoralLevel.L1, elevator, 0.65, elevator));
                NamedCommands.registerCommand("scorel2",
                                new ScoreCoral(Constants.CoralLevel.L2, elevator, 0.65, elevator));
                NamedCommands.registerCommand("scorel3",
                                new ScoreCoral(Constants.CoralLevel.L3, elevator, 0.65, elevator));
                NamedCommands.registerCommand("scorel4",
                                new ScoreCoral(Constants.CoralLevel.L4, elevator, 0.65, elevator));
                NamedCommands.registerCommand("zeroele", zeroMechanisms());
        }

        public Command zeroMechanisms() {
                return new ZeroElevator(elevator);
        }

        public Command getAutonomousCommand() {
                return autoChooser.getSelected();
        }

        @Override
        public void robotPeriodic() {
                // periodically call garbage collector
                if (timer.advanceIfElapsed(5)) {
                        System.gc();
                }
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
