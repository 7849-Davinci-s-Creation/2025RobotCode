// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import lib.OperatorControllerUtil;
import lib.RobotMethods;
import pabeles.concurrency.IntOperatorTask;

public class RobotContainer implements RobotMethods {
        // Subsystems
        public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

        // Controllers
        private final CommandXboxController joystick = new CommandXboxController(
                        Constants.OperatorConstants.DRIVER_CONTROLLER_PORT);

        // Everything else

        /* Setting up bindings for necessary control of the swerve drive platform */
        private final double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired
                                                                                            // top
                                                                                            // speed
        private final double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per
                                                                                                // second
        // max angular velocity

        private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
                        .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
                        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive
                                                                                 // motors

        private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
        private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

        private final SendableChooser<Command> autoChooser;

        public RobotContainer() {
                Telemetry logger = new Telemetry(MaxSpeed);
                drivetrain.registerTelemetry(logger::telemeterize);

                drivetrain.configPathPlanner();

                autoChooser = AutoBuilder.buildAutoChooser();
                SmartDashboard.putData(autoChooser);

                configureBindings();
        }

        private void configureBindings() {
                // Note that X is defined as forward according to WPILib convention,
                // and Y is defined as to the left according to WPILib convention.
                drivetrain.setDefaultCommand(
                                // Drivetrain will execute this command periodically
                                drivetrain.applyRequest(() -> drive
                                                .withVelocityX(drivetrain.calculateVelocity(joystick.getLeftY(),
                                                                MaxSpeed)) // Drive
                                                // forward
                                                // with
                                                // negative Y
                                                // (forward)
                                                .withVelocityY(drivetrain.calculateVelocity(joystick.getLeftX(),
                                                                MaxSpeed)) // Drive
                                                // left
                                                // with
                                                // negative
                                                // X
                                                // (left)
                                                .withRotationalRate(drivetrain.calculateVelocity(joystick.getRightX(),
                                                                MaxAngularRate)) // Drive
                                // counterclockwise
                                // with
                                // negative X (left)
                                ));

                joystick.leftTrigger().whileTrue(
                                drivetrain.applyRequest(() -> drive
                                                .withVelocityX(drivetrain.calculateVelocity(joystick.getLeftY(),
                                                                MaxSpeed)
                                                                / Constants.OperatorConstants.SLIGHT_CREEP_NERF) // Drive
                                                // forward
                                                // with
                                                // negative Y
                                                // (forward)
                                                .withVelocityY(drivetrain.calculateVelocity(joystick.getLeftX(),
                                                                MaxSpeed)
                                                                / Constants.OperatorConstants.SLIGHT_CREEP_NERF) // Drive
                                                // left
                                                // with
                                                // negative
                                                // X
                                                // (left)
                                                .withRotationalRate(drivetrain.calculateVelocity(joystick.getRightX(),
                                                                MaxAngularRate)) // Drive
                                // counterclockwise
                                // with
                                // negative X (left)
                                ));

                joystick.rightTrigger().whileTrue(
                                drivetrain.applyRequest(() -> drive
                                                .withVelocityX(drivetrain.calculateVelocity(joystick.getLeftX(),
                                                                MaxSpeed)
                                                                / Constants.OperatorConstants.MAJOR_CREEP_NERF) // Drive
                                                // forward
                                                // with
                                                // negative Y
                                                // (forward)
                                                .withVelocityY(drivetrain.calculateVelocity(joystick.getLeftX(),
                                                                MaxSpeed)
                                                                / Constants.OperatorConstants.MAJOR_CREEP_NERF) // Drive
                                                // left
                                                // with
                                                // negative
                                                // X
                                                // (left)
                                                .withRotationalRate(
                                                                drivetrain.calculateVelocity(joystick.getRightX(),
                                                                                MaxAngularRate)
                                                                                / Constants.OperatorConstants.MAJOR_CREEP_NERF) // Drive
                                // counterclockwise
                                // with
                                // negative X (left)
                                ));

                // IF WE NEED TO USE SYSID ROUTINES THEY ARE HERE
                // // Run SysId routines when holding back/start and X/Y.
                // // Note that each routine should be run exactly once in a single log.
                // joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
                // joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
                // joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
                // joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

                // reset the field-centric heading on left bumper press
                joystick.back().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

                joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
                joystick.b().whileTrue(drivetrain.applyRequest(
                                () -> point.withModuleDirection(
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
