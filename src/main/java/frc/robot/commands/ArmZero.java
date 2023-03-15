// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class ArmZero extends CommandBase {
  /** Creates a new ArmZero. */
  public ArmZero() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize()
  {
    SmartDashboard.putNumber("Speed", 0.1);
    RobotContainer.arm.armRotateOpenEnded(SmartDashboard.getNumber("Speed", 0.1));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute()
  {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.arm.armHold();
    Timer.delay(3);//Stops the bounce on the arm.
    RobotContainer.arm.armEncZero();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return RobotContainer.arm.getHomeSwitch();
  }
}
