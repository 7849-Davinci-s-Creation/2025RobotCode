// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import lib.RobotMethods;

public final class RobotContainer implements RobotMethods {
        // Subsystems
        private final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
        private final Climber climber = Climber.getInstance();

        // Controllers
        private final CommandXboxController driverController = new CommandXboxController(
                        Constants.OperatorConstants.DRIVER_CONTROLLER_PORT);

        private final CommandJoystick operatorController = new CommandJoystick(
                        Constants.OperatorConstants.OPERATOR_CONTROLLER_PORT);

        // Everything else
        private final SendableChooser<Command> autoChooser;

        public RobotContainer() {
                // Initialize subsystems
                drivetrain.initialize();
                climber.initialize();

                // ---- ALL ROBOT SUBSYSTEMS SHOULD BE INITIALIZED BEFORE DOING ANYTHING ELSE
                // IF THEY HAVE NOT THEN YOU ARE DOING SOMETHING COMPLETELY WRONG !! ----

                // the pathplanner auto builder must have been initialized before you all
                //
                // buildAutoChooser();
                autoChooser = AutoBuilder.buildAutoChooser();
                SmartDashboard.putData(autoChooser);

                configureBindings();
        }

        private void configureBindings() {
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

                // THIS IS STUPID UGLY, but behavior breaks otherwise, so we keep :/
                driverController.leftTrigger().whileTrue(
                                drivetrain.applyRequest(() -> drivetrain.getDrive()
                                                .withVelocityX(drivetrain.calculateVelocity(driverController.getLeftY(),
                                                                Constants.DriveTrainConstants.MAX_SPEED)
                                                                / Constants.OperatorConstants.SLIGHT_CREEP_NERF) // Drive
                                                // forward
                                                // with
                                                // negative Y
                                                // (forward)
                                                .withVelocityY(drivetrain.calculateVelocity(driverController.getLeftX(),
                                                                Constants.DriveTrainConstants.MAX_SPEED)
                                                                / Constants.OperatorConstants.SLIGHT_CREEP_NERF) // Drive
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

                driverController.rightTrigger().whileTrue(
                                drivetrain.applyRequest(() -> drivetrain.getDrive()
                                                .withVelocityX(drivetrain.calculateVelocity(driverController.getLeftY(),
                                                                Constants.DriveTrainConstants.MAX_SPEED)
                                                                / Constants.OperatorConstants.MAJOR_CREEP_NERF) // Drive
                                                // forward
                                                // with
                                                // negative Y
                                                // (forward)
                                                .withVelocityY(drivetrain.calculateVelocity(driverController.getLeftX(),
                                                                Constants.DriveTrainConstants.MAX_SPEED)
                                                                / Constants.OperatorConstants.MAJOR_CREEP_NERF) // Drive
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

                operatorController.button(1).whileTrue(Commands.runOnce(climber.climb()))
                                .onFalse(Commands.runOnce(climber.stop()));
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
