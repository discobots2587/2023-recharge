// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
 
// import java.time.Instant;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.SwerveConstants;

public final class Autos {
  private static PIDController frontController = new PIDController(SwerveConstants.AUTO_kP_FRONT, 0, 0);
  private static PIDController sideController = new PIDController(SwerveConstants.AUTO_kP_SIDE, 0, 0);
  private static PIDController turnController = new PIDController(SwerveConstants.AUTO_kP_TURN, 0, 0);

  public static CommandBase SimpleTest() {
    PathPlannerTrajectory trajectory = PathPlanner.loadPath("SimpleTest", SwerveConstants.AUTO_DRIVE_MAX_SPEED / 2.25, SwerveConstants.AUTO_DRIVE_MAX_ACCELERATION);

    PPSwerveControllerCommand driveOnCS = makeSwerveControllerCommand(trajectory);

    return new SequentialCommandGroup(
      new InstantCommand(() -> RobotContainer.intake.groundOuttake()),
      new InstantCommand(() -> RobotContainer.drivetrain.resetOdometry(getInitialPose(trajectory))), 
      new InstantCommand(() -> RobotContainer.drivetrain.setAllMode(true)),
      driveOnCS,
      new AutoBalance(),
      new InstantCommand(() -> RobotContainer.drivetrain.stopModules())
    );
  }

  public static CommandBase Drive() {
    PathPlannerTrajectory trajectory = PathPlanner.loadPath("MobilityBonus", SwerveConstants.AUTO_DRIVE_MAX_SPEED / 2.25, SwerveConstants.AUTO_DRIVE_MAX_ACCELERATION);

    PPSwerveControllerCommand move = makeSwerveControllerCommand(trajectory);

    return new SequentialCommandGroup(
      // new InstantCommand(() -> RobotContainer.intake.groundOuttake()),
      new InstantCommand(() -> RobotContainer.drivetrain.resetOdometry(getInitialPose(trajectory))), 
      new InstantCommand(() -> RobotContainer.drivetrain.setAllMode(true)),
      move, 
      new InstantCommand(() -> RobotContainer.drivetrain.stopModules())
    );
  }

  public static CommandBase Balance() {
    PathPlannerTrajectory trajectory = PathPlanner.loadPath("Balance", SwerveConstants.AUTO_DRIVE_MAX_SPEED / 2.25, SwerveConstants.AUTO_DRIVE_MAX_ACCELERATION);

    PPSwerveControllerCommand move = makeSwerveControllerCommand(trajectory);

    return new SequentialCommandGroup(
      new InstantCommand(() -> RobotContainer.drivetrain.resetOdometry(getInitialPose(trajectory))), 
      new InstantCommand(() -> RobotContainer.drivetrain.setAllMode(true)),
      move,
      new AutoBalance(),
      new InstantCommand(() -> RobotContainer.drivetrain.stopModules())
    );
  }
  
  public static CommandBase cubeAndDrive() {
    PathPlannerTrajectory trajectory = PathPlanner.loadPath("cubeAndDrive", SwerveConstants.AUTO_DRIVE_MAX_SPEED / 2.25, SwerveConstants.AUTO_DRIVE_MAX_ACCELERATION);

    PPSwerveControllerCommand move = makeSwerveControllerCommand(trajectory);

    return new SequentialCommandGroup(
      new InstantCommand(() -> RobotContainer.drivetrain.resetOdometry(getInitialPose(trajectory))), 
      new InstantCommand(() -> RobotContainer.drivetrain.setAllMode(true)),
      new InstantCommand(() -> RobotContainer.intake.groundOuttake()), 
      new WaitCommand(2),
      new InstantCommand(() -> RobotContainer.intake.groundIntakeStop()), 
      move, 
      new InstantCommand(() -> RobotContainer.drivetrain.stopModules())
    );
  }

  public static CommandBase highNodeAndDrive() {
    PathPlannerTrajectory trajectory = PathPlanner.loadPath("highNodeAndDrive", SwerveConstants.AUTO_DRIVE_MAX_SPEED / 2.25, SwerveConstants.AUTO_DRIVE_MAX_ACCELERATION);

    PPSwerveControllerCommand move = makeSwerveControllerCommand(trajectory);

    return new SequentialCommandGroup(
      new InstantCommand(() -> RobotContainer.drivetrain.resetOdometry(getInitialPose(trajectory))), 
      new InstantCommand(() -> RobotContainer.drivetrain.setAllMode(true)),
      new InstantCommand(() -> RobotContainer.arm.armRotateTo(Constants.ArmConstants.ENCODER_ROT_UP)), 
      new WaitCommand(1.5),
      new InstantCommand(() -> RobotContainer.arm.pickUp()),
      new WaitCommand(1.5), 
      new InstantCommand(() -> RobotContainer.arm.intakeStop()),
      new InstantCommand(() -> RobotContainer.arm.armRotateTo(0)), 
      new WaitCommand(1.5),
      move,
      new InstantCommand(() -> RobotContainer.drivetrain.stopModules())
    );
  }

  public static CommandBase highNodeAndBalance() {
    PathPlannerTrajectory trajectory = PathPlanner.loadPath("highNodeAndBalance", SwerveConstants.AUTO_DRIVE_MAX_SPEED / 2.25, SwerveConstants.AUTO_DRIVE_MAX_ACCELERATION);

    PPSwerveControllerCommand move = makeSwerveControllerCommand(trajectory);

    return new SequentialCommandGroup(
      new InstantCommand(() -> RobotContainer.drivetrain.resetOdometry(getInitialPose(trajectory))), 
      new InstantCommand(() -> RobotContainer.drivetrain.setAllMode(true)),
      new InstantCommand(() -> RobotContainer.arm.armRotateTo(Constants.ArmConstants.ENCODER_ROT_UP)), 
      new WaitCommand(1.5),
      new InstantCommand(() -> RobotContainer.arm.pickUp()),
      new WaitCommand(1), 
      new InstantCommand(() -> RobotContainer.arm.intakeStop()),
      new InstantCommand(() -> RobotContainer.arm.armRotateTo(0)), 
      new WaitCommand(1.5),
      move,
      new AutoBalance(),
      new InstantCommand(() -> RobotContainer.drivetrain.stopModules())
    );
  }

  public static CommandBase cubeHighNodeAndStop() {
    return new SequentialCommandGroup(
      new InstantCommand(() -> RobotContainer.drivetrain.setAllMode(true)),
      new InstantCommand(() -> RobotContainer.arm.pickUp()), 
      new WaitCommand(1),
      new InstantCommand(() -> RobotContainer.arm.intakeStop()), 
      new InstantCommand(() -> RobotContainer.drivetrain.stopModules())
    );
  }

  public static CommandBase driveBack() {
    PathPlannerTrajectory trajectory = PathPlanner.loadPath("DriveBack", SwerveConstants.AUTO_DRIVE_MAX_SPEED, SwerveConstants.AUTO_DRIVE_MAX_ACCELERATION);

    PPSwerveControllerCommand driveBack = makeSwerveControllerCommand(trajectory);

    return new SequentialCommandGroup(
      new InstantCommand(() -> RobotContainer.drivetrain.resetOdometry(getInitialPose(trajectory))),
      new InstantCommand(() -> RobotContainer.drivetrain.setAllMode(true)),
      driveBack,
      new InstantCommand(() -> RobotContainer.drivetrain.stopModules())
    );
  }

  public static CommandBase testAuto(){
    PathPlannerTrajectory trajectory = PathPlanner.loadPath("highNodeAndBalance", SwerveConstants.AUTO_DRIVE_MAX_SPEED / 2.25, SwerveConstants.AUTO_DRIVE_MAX_ACCELERATION);

    PPSwerveControllerCommand move = makeSwerveControllerCommand(trajectory);

    return new SequentialCommandGroup(
      new InstantCommand(() -> RobotContainer.drivetrain.resetOdometry(getInitialPose(trajectory))), 
      new InstantCommand(() -> RobotContainer.drivetrain.setAllMode(true)),
      move,
      new AutoBalance(),
      new InstantCommand(() -> RobotContainer.drivetrain.stopModules())
    );
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }

  private static PPSwerveControllerCommand makeSwerveControllerCommand(PathPlannerTrajectory traj){
    turnController.enableContinuousInput(-Math.PI, Math.PI);
    return new PPSwerveControllerCommand(
      traj, 
      RobotContainer.drivetrain::getPose, 
      SwerveConstants.DRIVE_KINEMATICS,
      frontController, 
      sideController,
      turnController, 
      RobotContainer.drivetrain::setModuleStates,
      true,
      RobotContainer.drivetrain);
  }

  private static Pose2d getInitialPose(PathPlannerTrajectory traj){
    PathPlannerState initialState = DriverStation.getAlliance().equals(Alliance.Red) ? 
    PathPlannerTrajectory.transformStateForAlliance(traj.getInitialState(), Alliance.Red) :
    traj.getInitialState();

    return new Pose2d(initialState.poseMeters.getTranslation(), initialState.holonomicRotation);
  }
}
