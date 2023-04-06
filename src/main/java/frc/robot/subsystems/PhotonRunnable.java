package frc.robot.subsystems;

import static frc.robot.Constants.VisionConstants.APRILTAG_AMBIGUITY_THRESHOLD;
import static frc.robot.Constants.VisionConstants.APRILTAG_CAMERA_TO_ROBOT;
import static frc.robot.Constants.VisionConstants.FIELD_LENGTH_METERS;
import static frc.robot.Constants.VisionConstants.FIELD_WIDTH_METERS;

import java.io.IOException;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotState;
import frc.robot.RobotContainer;

import frc.robot.Constants.kAutoAlign;

/**
 * Runnable that gets AprilTag data from PhotonVision.
 */
public class PhotonRunnable implements Runnable {

  private final PhotonPoseEstimator photonPoseEstimator;
  private final PhotonCamera photonCamera;
  private final AtomicReference<EstimatedRobotPose> atomicEstimatedRobotPose = new AtomicReference<EstimatedRobotPose>();

  public PhotonRunnable() {
    this.photonCamera = new PhotonCamera("Arducam_OV9281_USB_Camera");
    PhotonPoseEstimator photonPoseEstimator = null;
    try {
      var layout = AprilTagFields.k2023ChargedUp.loadAprilTagLayoutField();
      // PV estimates will always be blue, they'll get flipped by robot thread
      layout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);
      if (photonCamera != null) {
        photonPoseEstimator = new PhotonPoseEstimator(
            layout, PoseStrategy.MULTI_TAG_PNP, photonCamera, APRILTAG_CAMERA_TO_ROBOT.inverse());
      }
    } catch(IOException e) {
      DriverStation.reportError("Failed to load AprilTagFieldLayout", e.getStackTrace());
      photonPoseEstimator = null;
    }
    this.photonPoseEstimator = photonPoseEstimator;
  }

  @Override
  public void run() {      
    // Get AprilTag data
    if (photonPoseEstimator != null && photonCamera != null && !RobotState.isAutonomous()) {
      var photonResults = photonCamera.getLatestResult();
      if (photonResults.hasTargets() 
          && (photonResults.targets.size() > 1 || photonResults.targets.get(0).getPoseAmbiguity() < APRILTAG_AMBIGUITY_THRESHOLD)) {
        photonPoseEstimator.update(photonResults).ifPresent(estimatedRobotPose -> {
          var estimatedPose = estimatedRobotPose.estimatedPose;
          // Make sure the measurement is on the field
          if (estimatedPose.getX() > 0.0 && estimatedPose.getX() <= FIELD_LENGTH_METERS
              && estimatedPose.getY() > 0.0 && estimatedPose.getY() <= FIELD_WIDTH_METERS) {
            atomicEstimatedRobotPose.set(estimatedRobotPose);
          }
        });
      }
    }  
  }

  public void alignLeft() {
    PhotonPipelineResult cameraResult = photonCamera.getLatestResult();

    if(cameraResult.hasTargets()) {
      List<PhotonTrackedTarget> targets = cameraResult.getTargets();
      int id = targets.get(0).getFiducialId();

      if(id == 1) {
        RobotContainer.drivetrain.moveToPose(() -> kAutoAlign.SCORING_POSES[kAutoAlign.left_1], kAutoAlign.DEFAULT_OFFSET);
      } else if(id == 2) {
        RobotContainer.drivetrain.moveToPose(() -> kAutoAlign.SCORING_POSES[kAutoAlign.left_2], kAutoAlign.DEFAULT_OFFSET);
      } else if(id == 3) {
        RobotContainer.drivetrain.moveToPose(() -> kAutoAlign.SCORING_POSES[kAutoAlign.left_3], kAutoAlign.DEFAULT_OFFSET);
      } else if(id == 6) {
        RobotContainer.drivetrain.moveToPose(() -> kAutoAlign.SCORING_POSES[kAutoAlign.left_6], kAutoAlign.DEFAULT_OFFSET);
      } else if(id == 7) {
        RobotContainer.drivetrain.moveToPose(() -> kAutoAlign.SCORING_POSES[kAutoAlign.left_7], kAutoAlign.DEFAULT_OFFSET);
      } else if(id == 8) {
        RobotContainer.drivetrain.moveToPose(() -> kAutoAlign.SCORING_POSES[kAutoAlign.left_8], kAutoAlign.DEFAULT_OFFSET);
      }
    }    
  }

  public void alignCenter() {
    PhotonPipelineResult cameraResult = photonCamera.getLatestResult();

    if(cameraResult.hasTargets()) {
      List<PhotonTrackedTarget> targets = cameraResult.getTargets();
      int id = targets.get(0).getFiducialId();

      if(id == 1) {
        RobotContainer.drivetrain.moveToPose(() -> kAutoAlign.SCORING_POSES[kAutoAlign.center_1], kAutoAlign.DEFAULT_OFFSET);
      } else if(id == 2) {
        RobotContainer.drivetrain.moveToPose(() -> kAutoAlign.SCORING_POSES[kAutoAlign.center_2], kAutoAlign.DEFAULT_OFFSET);
      } else if(id == 3) {
        RobotContainer.drivetrain.moveToPose(() -> kAutoAlign.SCORING_POSES[kAutoAlign.center_3], kAutoAlign.DEFAULT_OFFSET);
      } else if(id == 6) {
        RobotContainer.drivetrain.moveToPose(() -> kAutoAlign.SCORING_POSES[kAutoAlign.center_6], kAutoAlign.DEFAULT_OFFSET);
      } else if(id == 7) {
        RobotContainer.drivetrain.moveToPose(() -> kAutoAlign.SCORING_POSES[kAutoAlign.center_7], kAutoAlign.DEFAULT_OFFSET);
      } else if(id == 8) {
        RobotContainer.drivetrain.moveToPose(() -> kAutoAlign.SCORING_POSES[kAutoAlign.center_8], kAutoAlign.DEFAULT_OFFSET);
      }
    }    
  }

  public void alignRight() {
    PhotonPipelineResult cameraResult = photonCamera.getLatestResult();

    if(cameraResult.hasTargets()) {
      List<PhotonTrackedTarget> targets = cameraResult.getTargets();
      int id = targets.get(0).getFiducialId();

      if(id == 1) {
        RobotContainer.drivetrain.moveToPose(() -> kAutoAlign.SCORING_POSES[kAutoAlign.right_1], kAutoAlign.DEFAULT_OFFSET);
      } else if(id == 2) {
        RobotContainer.drivetrain.moveToPose(() -> kAutoAlign.SCORING_POSES[kAutoAlign.right_2], kAutoAlign.DEFAULT_OFFSET);
      } else if(id == 3) {
        RobotContainer.drivetrain.moveToPose(() -> kAutoAlign.SCORING_POSES[kAutoAlign.right_3], kAutoAlign.DEFAULT_OFFSET);
      } else if(id == 6) {
        RobotContainer.drivetrain.moveToPose(() -> kAutoAlign.SCORING_POSES[kAutoAlign.right_6], kAutoAlign.DEFAULT_OFFSET);
      } else if(id == 7) {
        RobotContainer.drivetrain.moveToPose(() -> kAutoAlign.SCORING_POSES[kAutoAlign.right_7], kAutoAlign.DEFAULT_OFFSET);
      } else if(id == 8) {
        RobotContainer.drivetrain.moveToPose(() -> kAutoAlign.SCORING_POSES[kAutoAlign.right_8], kAutoAlign.DEFAULT_OFFSET);
      }
    }    
  }



  /**
   * Gets the latest robot pose. Calling this will only return the pose once. If it returns a non-null value, it is a
   * new estimate that hasn't been returned before.
   * This pose will always be for the BLUE alliance. It must be flipped if the current alliance is RED.
   * @return latest estimated pose
   */
  public EstimatedRobotPose grabLatestEstimatedPose() {
    return atomicEstimatedRobotPose.getAndSet(null);
  }

}
