// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
// import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.IntakeConstants;
// import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
// import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import frc.robot.Robot;

public class IntakeMove extends CommandBase
{
  /** Creates a new ArmHold. */
  double targetDown;
  double targetStow;
  BooleanSupplier downSup;
  BooleanSupplier stowSup;

  Intake intake;

  public IntakeMove(Intake intakeSup, BooleanSupplier DownButton, BooleanSupplier StowButton) {
    // Use addRequirements() here to declare subsystem dependencies.
    intake = intakeSup;
    downSup = DownButton;
    stowSup = StowButton;
    addRequirements(RobotContainer.intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize()
  {
    targetDown = IntakeConstants.ENCODER_ROT_DOWN;
    targetStow = IntakeConstants.ENCODER_ROT_STOW;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute()
  {
    if (downSup.getAsBoolean()) {intake.intakeRotateTo(targetDown); SmartDashboard.putString("Intake Position", "Down");}
    else if (stowSup.getAsBoolean()) {intake.intakeRotateTo(targetStow); SmartDashboard.putString("Intake Position", "Stow");}
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
