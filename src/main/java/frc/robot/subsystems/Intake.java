// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
  private CANSparkMax driver;
  private VictorSPX wheels;
  private DigitalInput intSw;
  private RelativeEncoder intakeEncoder;

  private SparkMaxPIDController intakeController;

  /** Creates a new IntakeRollers. */
  public Intake() {
    driver = new CANSparkMax(60, MotorType.kBrushless);
    driver.setIdleMode(IdleMode.kBrake);
    driver.setSmartCurrentLimit(15, 25);
    intakeEncoder = driver.getEncoder();
    

    wheels = new VictorSPX(61);
    wheels.setNeutralMode(NeutralMode.Brake);
    
    
    intSw = new DigitalInput(IntakeConstants.INTAKE_LIM_SWITCH_PORT);

    intakeController = driver.getPIDController();

    intakeController.setP(IntakeConstants.INTAKE_kP);
    intakeController.setI(IntakeConstants.INTAKE_kI);
    intakeController.setD(IntakeConstants.INTAKE_kD);
    driver.burnFlash();
  }

  public void groundPickUp()
  {
    wheels.set(ControlMode.PercentOutput, -25); 
    // return false;
  }
  
  public void groundOuttake()
  {
    wheels.set(ControlMode.PercentOutput, 100);
    // return false;
  }

  public void groundIntakeStop()
  {
    wheels.set(ControlMode.PercentOutput, 0);
  }
  
  public void intakeHold(){
    driver.set(0);
  }

  public void intakeEncZero()
  {
    intakeEncoder.setPosition(0);
  }

  public void intakeRotateOpenEnded(double speed)
  {
    driver.set(speed);
  }

  public void intakeRotateTo(double rotations)
  {
    // double rotations = degrees/360.0;
    intakeController.setReference(rotations, CANSparkMax.ControlType.kPosition);
  };

  public boolean getHomeSwitch()
  {
    return !intSw.get();
  }

  @Override
  public void periodic()
  {
    // This method will be called once per scheduler run
    //SmartDashboard.putNumber("Arm Adjust", armAdjust);
    SmartDashboard.putNumber("Intake Encoder", intakeEncoder.getPosition());
    SmartDashboard.putBoolean("Intake Homed", getHomeSwitch());
  }
}