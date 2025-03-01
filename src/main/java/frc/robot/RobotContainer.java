// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import lib.RobotMethods;

public final class RobotContainer implements RobotMethods {
        // Subsystems
        private final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

        // Controllers
        private final CommandXboxController joystick = new CommandXboxController(
                        Constants.OperatorConstants.DRIVER_CONTROLLER_PORT);

        // Everything else
        private final SendableChooser<Command> autoChooser;

        public RobotContainer() {
                drivetrain.initialize();

                // the pathplanner auto builder must have been initialized before you all buildAutoChooser();
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
                                                .withVelocityX(drivetrain.calculateVelocity(joystick.getLeftY(),
                                                                Constants.DriveTrainConstants.MAX_SPEED)) // Drive
                                                // forward
                                                // with
                                                // negative Y
                                                // (forward)
                                                .withVelocityY(drivetrain.calculateVelocity(joystick.getLeftX(),
                                                                Constants.DriveTrainConstants.MAX_SPEED)) // Drive
                                                // left
                                                // with
                                                // negative
                                                // X
                                                // (left)
                                                .withRotationalRate(drivetrain.calculateVelocity(joystick.getRightX(),
                                                                Constants.DriveTrainConstants.MAX_ANGULAR_RATE)) // Drive
                                // counterclockwise
                                // with
                                // negative X (left)
                                ));

                // THIS IS STUPID UGLY, but behavior breaks otherwise, so we keep :/
                joystick.leftTrigger().whileTrue(
                                drivetrain.applyRequest(() -> drivetrain.getDrive()
                                                .withVelocityX(drivetrain.calculateVelocity(joystick.getLeftY(),
                                                                Constants.DriveTrainConstants.MAX_SPEED)
                                                                / Constants.OperatorConstants.SLIGHT_CREEP_NERF) // Drive
                                                // forward
                                                // with
                                                // negative Y
                                                // (forward)
                                                .withVelocityY(drivetrain.calculateVelocity(joystick.getLeftX(),
                                                                Constants.DriveTrainConstants.MAX_SPEED)
                                                                / Constants.OperatorConstants.SLIGHT_CREEP_NERF) // Drive
                                                // left
                                                // with
                                                // negative
                                                // X
                                                // (left)
                                                .withRotationalRate(drivetrain.calculateVelocity(joystick.getRightX(),
                                                                Constants.DriveTrainConstants.MAX_ANGULAR_RATE)) // Drive
                                // counterclockwise
                                // with
                                // negative X (left)
                                ));

                joystick.rightTrigger().whileTrue(
                                drivetrain.applyRequest(() -> drivetrain.getDrive()
                                                .withVelocityX(drivetrain.calculateVelocity(joystick.getLeftX(),
                                                                Constants.DriveTrainConstants.MAX_SPEED)
                                                                / Constants.OperatorConstants.MAJOR_CREEP_NERF) // Drive
                                                // forward
                                                // with
                                                // negative Y
                                                // (forward)
                                                .withVelocityY(drivetrain.calculateVelocity(joystick.getLeftX(),
                                                                Constants.DriveTrainConstants.MAX_SPEED)
                                                                / Constants.OperatorConstants.MAJOR_CREEP_NERF) // Drive
                                                // left
                                                // with
                                                // negative
                                                // X
                                                // (left)
                                                .withRotationalRate(
                                                                drivetrain.calculateVelocity(joystick.getRightX(),
                                                                                Constants.DriveTrainConstants.MAX_ANGULAR_RATE)
                                                                                / Constants.OperatorConstants.MAJOR_CREEP_NERF) // Drive
                                // counterclockwise
                                // with
                                // negative X (left)
                                ));

                // // Run SysId routines when holding back/start and X/Y.
                // // Note that each routine should be run exactly once in a single log.
                joystick.leftBumper().and(joystick.y())
                                .whileTrue(drivetrain.sysIdDynamic(SysIdRoutine.Direction.kForward));
                joystick.leftBumper().and(joystick.x())
                                .whileTrue(drivetrain.sysIdDynamic(SysIdRoutine.Direction.kReverse));
                joystick.rightBumper().and(joystick.y())
                                .whileTrue(drivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
                joystick.rightBumper().and(joystick.x())
                                .whileTrue(drivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));

                // reset the field-centric heading on left bumper press
                joystick.back().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

                joystick.a().whileTrue(drivetrain.applyRequest(drivetrain::getBrake));
                joystick.b().whileTrue(drivetrain.applyRequest(
                                () -> drivetrain.getPoint().withModuleDirection(
                                                new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));
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
