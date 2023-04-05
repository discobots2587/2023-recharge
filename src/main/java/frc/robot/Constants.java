// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;


/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final int PDH_ID = 55;
  public static final PowerDistribution PDH = new PowerDistribution(Constants.PDH_ID, ModuleType.kRev);

  public static class IOConstants {
    public static final int DRIVER_CONTROLLER_PORT = 1;
    public static final int OP_CONTROLLER_PORT = 0;
  }

  public static final class SwerveConstants{
    //Drivetrain motor/encoder IDs
    public static final int LEFT_FRONT_DRIVE_ID = 11;
    public static final int RIGHT_FRONT_DRIVE_ID = 1;
    public static final int LEFT_BACK_DRIVE_ID = 20;
    public static final int RIGHT_BACK_DRIVE_ID = 31;
    
    public static final int LEFT_FRONT_TURN_ID = 10;
    public static final int RIGHT_FRONT_TURN_ID = 2;
    public static final int LEFT_BACK_TURN_ID = 21;
    public static final int RIGHT_BACK_TURN_ID = 30;
    
    public static final int RIGHT_FRONT_CANCODER_ID_threncID = 0;
    public static final int LEFT_FRONT_CANCODER_ID_threncID = 1;
    public static final int RIGHT_BACK_CANCODER_ID_threncID = 2;
    public static final int LEFT_BACK_CANCODER_ID_threncID = 3;

    //public static final SerialPort PIGEON_ID = SerialPort();

    //Drivetrain characteristics
    public static final double LEFT_FRONT_OFFSET = 184.32; //183.96
    public static final double RIGHT_FRONT_OFFSET = 299.88; //299.88
    public static final double LEFT_BACK_OFFSET = 255.6; //254.52
    public static final double RIGHT_BACK_OFFSET = 277.56; //277.92

    public static final double WHEEL_DIAMETER = Units.inchesToMeters(4);
    public static final double DRIVE_MOTOR_GEAR_RATIO = 6.75;
    public static final double TURN_MOTOR_GEAR_RATIO = 150.0/7;
    public static final double DRIVE_MOTOR_PCONVERSION = WHEEL_DIAMETER * Math.PI / DRIVE_MOTOR_GEAR_RATIO;
    public static final double TURN_MOTOR_PCONVERSION = 2 * Math.PI / TURN_MOTOR_GEAR_RATIO;
    public static final double DRIVE_MOTOR_VCONVERSION = DRIVE_MOTOR_PCONVERSION / 60.0;
    public static final double TURN_MOTOR_VCONVERSION = TURN_MOTOR_PCONVERSION / 60.0;
    
    public static final double KP_TURNING = 0.75;

    public static final double DRIVETRAIN_MAX_SPEED = 4.6;
    public static final double DRIVETRAIN_MAX_ANGULAR_SPEED = 3.5 * Math.PI;

    //Teleop constraints
    public static final double TELE_DRIVE_MAX_SPEED = DRIVETRAIN_MAX_SPEED * 0.5;
    public static final double TELE_DRIVE_FAST_SPEED = DRIVETRAIN_MAX_SPEED * 0.75;
    public static final double TELE_DRIVE_MAX_ANGULAR_SPEED = DRIVETRAIN_MAX_ANGULAR_SPEED /1.75;
    public static final double TELE_DRIVE_MAX_ACCELERATION = 3;
    public static final double TELE_DRIVE_MAX_ANGULAR_ACCELERATION = 3;
    public static final double deadbandValue = 0.1;

    //Auton constraints
    public static final double AUTO_DRIVE_MAX_SPEED = DRIVETRAIN_MAX_SPEED / 1.5;
    public static final double AUTO_DRIVE_MAX_ANGULAR_SPEED = DRIVETRAIN_MAX_ANGULAR_SPEED / 2.0;
    public static final double AUTO_DRIVE_MAX_ACCELERATION = 3;
    public static final double AUTO_DRIVE_MAX_ANGULAR_ACCELERATION = Math.PI;

    public static final double AUTO_kP_FRONT = 0.4; //0.4
    public static final double AUTO_kP_SIDE = 0.4;
    public static final double AUTO_kP_TURN = 0; //2.4

    //Swerve Kinematics
    public static final double TRACK_WIDTH = Units.inchesToMeters(16.75);
    public static final double WHEEL_BASE = Units.inchesToMeters(24.75);
    public static final SwerveDriveKinematics DRIVE_KINEMATICS = new SwerveDriveKinematics(
        new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2),
        new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2),
        new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2),
        new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2)
    );

    public static final double kS_PERCENT = 0.035;
    public static final double kP_PERCENT = 0.009;
  }

  // public static final class IntakeConstants{
  //   public static final int INTAKE_DRIVER_ID = 50;
  //   public static final int INTAKE_PIVOT_ID = 51;

  //   public static final double PIVOT_kP = 0.15;
  //   public static final double PIVOT_kI = 0;
  //   public static final double PIVOT_kD = 0;
  //   public static final double PIVOT_MIN_OUTPUT = -0.25;
  //   public static final double PIVOT_MAX_OUTPUT = 0.25;

  //   public static final double DEPLOYED_ROT = 10.2;
  // }

  public static final class ArmConstants{
    public static final int ARM_DRIVER_ID = 50;
    public static final int ARM_INTAKE_ID = 51;
    public static final int ARM_LIM_SWITCH_PORT = 0;

    public static final double ARM_kP = 0.05; //0.25
    public static final double ARM_kI = 0; //0.0001
    public static final double ARM_kD = 0.0;

    public static final double ENCODER_ROT_UP = -56;
    public static final double ENCODER_ROT_MID = -30;
    // public static final int ARM_PIVOT_ID = 24;
  }
  public static final class IntakeConstants{
    public static final int INTAKE_DRIVER_ID = 60;
    public static final int INTAKE_INTAKE_ID = 61;
    public static final int INTAKE_LIM_SWITCH_PORT = 1;

    public static final double INTAKE_kP = 0.05;
    public static final double INTAKE_kI = 0.0;
    public static final double INTAKE_kD = 0.0;

    public static final double ENCODER_ROT_DOWN = -14.0; // TUNE THIS
    public static final double ENCODER_ROT_STOW = 0; // TUNE THIS
    // public static final int ARM_PIVOT_ID = 24;
  }
  public static class VisionConstants {
    // VisionConstants from FRC-7028 PhotonVision code

    /** Physical location of the apriltag camera on the robot, relative to the center of the robot. */
    // center of robot 10" forward of center, 1" left of center and 12" up from center of robot
    public static final Translation3d camera1Translation = new Translation3d(Units.inchesToMeters(10.0), 
    Units.inchesToMeters(1.0), Units.inchesToMeters(12.));// Cam mounted facing forward
    public static final Rotation3d camera1Rotation = new Rotation3d(0,0,0); // pointing forward
    public static final Transform3d APRILTAG_CAMERA_TO_ROBOT =
              new Transform3d(
                      camera1Translation,
                      camera1Rotation); // transform camera1 from center of robot
  

    public static final double FIELD_LENGTH_METERS = 16.54175;
    public static final double FIELD_WIDTH_METERS = 8.0137;

    // Pose on the opposite side of the field. Use with `relativeTo` to flip a pose to the opposite alliance
    public static final Pose2d FLIPPING_POSE = new Pose2d(
        new Translation2d(FIELD_LENGTH_METERS, FIELD_WIDTH_METERS),
        new Rotation2d(Math.PI));

    /** Minimum target ambiguity. Targets with higher ambiguity will be discarded */
    public static final double APRILTAG_AMBIGUITY_THRESHOLD = 0.2;
  }

  public static class kAutoAlign {

    public static final Rotation2d DEFAULT_ROTATION = Rotation2d.fromDegrees(0);  //EITHER 0 OR 180 (NOT TESTED)

    //PID Tolerance
    public static final double X_TOLLERENCE = 0.01;
    public static final double Y_TOLLERENCE = 0.02;
    public static final double THETA_TOLLERENCE = 0.02;

    //PID Values (NEEDS TUNING)
    public static final double X_P = 9.0; //
    public static final double X_I = 0.0; //
    public static final double X_D = 0.0; //

    public static final double Y_P = 9.0; //
    public static final double Y_I = 0.0; //
    public static final double Y_D = 0.0; //
            
    public static final double THETA_P = 9.0; //
    public static final double THETA_I = 0.0; //
    public static final double THETA_D = 0.0; //

    public static final double ERROR = 0.1;

    //Offset Value
    public static final Translation2d DEFAULT_OFFSET = new Translation2d(1.0, 0);

    //Scoring Poses
    public static final Pose2d[] SCORING_POSES = {

        /*
         * [ID in array] - [April tag ID]_[left/center/right]
         * 
         * 0  - 1_left
         * 1  - 1_center
         * 2  - 1_right
         * 
         * 3  - 2_left
         * 4  - 2_center
         * 5  - 2_right
         * 
         * 6  - 3_left
         * 7  - 3_center
         * 8  - 3_right
         * 
         * 9  - 6_left
         * 10 - 6_center
         * 11 - 6_right
         * 
         * 12 - 7_left
         * 13 - 7_center
         * 14 - 7_right
         * 
         * 15 - 8_left
         * 16 - 8_center
         * 17 - 8_right
        */

        //Tag 1
        new Pose2d(15.3, 0.4, Rotation2d.fromDegrees(180)),         //Left of tag
        new Pose2d(15.3, 1.071626, Rotation2d.fromDegrees(180)),    //In front of tag
        new Pose2d(15.3, 1.64, Rotation2d.fromDegrees(180)),        //Right of tag

        //Tag 2
        new Pose2d(15.3, 2.1, Rotation2d.fromDegrees(180)),         //Left of tag
        new Pose2d(15.3, 2.748026, Rotation2d.fromDegrees(180)),    //In front of tag
        new Pose2d(15.3, 3.3, Rotation2d.fromDegrees(180)),         //Right of tag

        //Tag 3
        new Pose2d(15.3, 3.857, Rotation2d.fromDegrees(180)),       //Left of tag
        new Pose2d(15.3, 4.424426, Rotation2d.fromDegrees(180)),    //In front of tag
        new Pose2d(15.3, 5.0, Rotation2d.fromDegrees(180)),         //Right of tag

        //Tag 6
        new Pose2d(1.2, 0.4, Rotation2d.fromDegrees(0)),            //Left of tag
        new Pose2d(1.2, 1.071626, Rotation2d.fromDegrees(0)),       //In front of tag
        new Pose2d(1.2, 1.64, Rotation2d.fromDegrees(0)),           //Right of tag

        //Tag 7
        new Pose2d(1.2, 2.1, Rotation2d.fromDegrees(0)),            //Left of tag
        new Pose2d(1.2, 2.748026, Rotation2d.fromDegrees(0)),       //In front of tag
        new Pose2d(1.2, 3.3, Rotation2d.fromDegrees(0)),            //Right of tag

        //Tag 8
        new Pose2d(1.2, 3.857, Rotation2d.fromDegrees(0)),          //Left of tag
        new Pose2d(1.2, 4.424426, Rotation2d.fromDegrees(0)),       //In front of tag
        new Pose2d(1.2, 5.0, Rotation2d.fromDegrees(0))             //Right of tag
    };
  }

}
