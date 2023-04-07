// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
// import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Arm;
import frc.robot.Constants;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import frc.robot.Robot;

public class ArmMove extends CommandBase {
  /** Creates a new ArmHold. */
  double targetHigh;
  double targetMid;
  double targetStow;
  BooleanSupplier UpSup;
  BooleanSupplier midSup;
  BooleanSupplier StowSup;

  Arm arm;

  public ArmMove(Arm armSup, BooleanSupplier UpButton, BooleanSupplier MidButton, BooleanSupplier StowButton) {
    // Use addRequirements() here to declare subsystem dependencies.
    arm = armSup;
    UpSup = UpButton;
    midSup = MidButton;
    StowSup = StowButton;
    addRequirements(RobotContainer.arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize()
  {
    targetHigh = Constants.ArmConstants.ENCODER_ROT_UP;
    targetMid = Constants.ArmConstants.ENCODER_ROT_MID;
    targetStow = -1;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute()
  {
    if (UpSup.getAsBoolean()) {arm.armRotateTo(targetHigh); }
    else if (midSup.getAsBoolean()) {arm.armRotateTo(targetMid); }
    else if (StowSup.getAsBoolean()) {arm.armRotateTo(targetStow); }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted)
  {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished()
  {
    return false;
  }
}
