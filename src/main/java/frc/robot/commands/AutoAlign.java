package frc.robot.commands;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.PoseEstimatorSubsystem;

public class AutoAlign extends CommandBase{

    public AutoAlign() {

    }

    @Override
    public void initialize() {

    }

  // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

    }

  // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    
    }

  // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }


    // public static CommandBase centerAlign() {
    //     PathPlannerTrajectory trajectory = PathPlanner.generatePath(
    //         new PathConstraints(1, 1), 
    //         new PathPoint(new Translation2d(RobotContainer.poseEstimator.getPoseX(), RobotContainer.poseEstimator.getPoseY()), Rotation2d.fromDegrees(RobotContainer.poseEstimator.getPoseRotation())),

    //     );
    // }
}
