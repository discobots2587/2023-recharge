// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
// import frc.robot.commands.IntakeHold;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    RobotContainer.intake.intakeEncZero();
    // RobotContainer.pdh.setSwitchableChannel(false);
    // PortForwarder.add(8888, "limelight.local", 5800);
    // PortForwarder.add(8889, "limelight.local", 5801);
  }
  //githubtest

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    // RobotContainer.photonEstimator.run();
    SmartDashboard.putNumber("Front Left", RobotContainer.drivetrain.getleftFront().getAbsoluteEncoderAngle() * (180 / Math.PI));   //Front right
    SmartDashboard.putNumber("Front Right", RobotContainer.drivetrain.getrightFront().getAbsoluteEncoderAngle() * (180 / Math.PI)); //Front left
    SmartDashboard.putNumber("Back Right", RobotContainer.drivetrain.getrightBack().getAbsoluteEncoderAngle() * (180 / Math.PI));    //Back right
    SmartDashboard.putNumber("Back Left", RobotContainer.drivetrain.getleftBack().getAbsoluteEncoderAngle() * (180 / Math.PI));    //Back left
    SmartDashboard.putNumber("Heading", RobotContainer.drivetrain.getHeading());
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    RobotContainer.pdh.setSwitchableChannel(false);
  }

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
    RobotContainer.pdh.setSwitchableChannel(true);
    RobotContainer.intake.intakeEncZero();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit()
  {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    RobotContainer.drivetrain.setAllMode(true);
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    RobotContainer.drivetrain.resetAllEncoders();
    // RobotContainer.shooter.shooterOff();
    RobotContainer.pdh.setSwitchableChannel(true);
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
