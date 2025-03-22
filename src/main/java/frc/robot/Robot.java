// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public final class Robot extends TimedRobot {
  private Command autonomousCommand;

  private final RobotContainer robotContainer;

  public Robot() {
    robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    robotContainer.robotPeriodic();
  }

  @Override
  public void disabledInit() {
    robotContainer.disabledInit();
  }

  @Override
  public void disabledPeriodic() {
    robotContainer.disabledPeriodic();
  }

  @Override
  public void disabledExit() {
    robotContainer.disabledExit();
  }

  @Override
  public void autonomousInit() {
    autonomousCommand = robotContainer.getAutonomousCommand();

    if (autonomousCommand != null) {
      autonomousCommand.schedule();
    }

    robotContainer.autonomousInit();
  }

  @Override
  public void autonomousPeriodic() {
    robotContainer.autonomousPeriodic();
  }

  @Override
  public void autonomousExit() {
    robotContainer.autonomousExit();
  }

  @Override
  public void teleopInit() {
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }

    robotContainer.teleopInit();
  }

  @Override
  public void teleopPeriodic() {
    robotContainer.teleopPeriodic();
  }

  @Override
  public void teleopExit() {
    robotContainer.teleopExit();
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();

    robotContainer.testInit();
  }

  @Override
  public void testPeriodic() {
    robotContainer.testPeriodic();
  }

  @Override
  public void testExit() {
    robotContainer.testExit();
  }

  @Override
  public void simulationPeriodic() {
    robotContainer.simulationPeriodic();
  }
}
