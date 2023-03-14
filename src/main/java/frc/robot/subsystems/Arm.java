// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANSparkMax;
// import com.revrobotics.EncoderType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxRelativeEncoder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;

public class Arm extends SubsystemBase {
  private CANSparkMax armDrive;
  private VictorSPX intake;
  private RelativeEncoder armEncoder;
  private DigitalInput homeSwitch;
  
  private SparkMaxPIDController armController;


  /** Creates a new Arm. */
  public Arm() {
    armDrive = new CANSparkMax(ArmConstants.ARM_DRIVER_ID, MotorType.kBrushless);
    armDrive.setIdleMode(IdleMode.kBrake);
    armEncoder = armDrive.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);
    armController = armDrive.getPIDController();
    
    homeSwitch = new DigitalInput(0);

    intake = new VictorSPX(ArmConstants.ARM_INTAKE_ID);
    
    armController.setP(ArmConstants.ARM_kP);
    armController.setI(ArmConstants.ARM_kI);
    armController.setD(ArmConstants.ARM_kD);
  }

  public void pickUp()
  {
    intake.set(ControlMode.PercentOutput, -25);
    // return false;
  }
  
  public void outtake()
  {
    intake.set(ControlMode.PercentOutput, 100);
    // return false;
  }

  public void intakeStop()
  {
    intake.set(ControlMode.PercentOutput, 0);
  }
  
  public void armHold(){
    armDrive.set(0);
  }

  public void armEncZero()
  {
    armEncoder.setPosition(0);
  }

  public void armRotateOpenEnded(double speed)
  {
    armDrive.set(speed);
  }

  public void armRotateTo(double degrees)
  {

  };

  public boolean getHomeSwitch()
  {
    return homeSwitch.get();
  }

  @Override
  public void periodic()
  {
    // This method will be called once per scheduler run
    //SmartDashboard.putNumber("Arm Adjust", armAdjust);
    SmartDashboard.putNumber("Arm Encoder", armEncoder.getPosition());
    SmartDashboard.putBoolean("Arm Homed", getHomeSwitch());
  }
}
