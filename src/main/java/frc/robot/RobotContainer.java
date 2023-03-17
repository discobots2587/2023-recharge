// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.lib.drivers.EForwardableConnections;
import frc.lib.drivers.Launchpad;
import frc.lib.drivers.LaunchpadButton;
import frc.robot.Constants.IOConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.SwerveDrive;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public static final Drivetrain drivetrain = Drivetrain.getInstance();
  public static final Arm arm = Arm.getInstance();

  public static final PowerDistribution pdh = new PowerDistribution(Constants.PDH_ID, ModuleType.kRev);

  private SendableChooser<String> autoChooser = new SendableChooser<>();

  public static final XboxController driverController = new XboxController(IOConstants.DRIVER_CONTROLLER_PORT);
  private final JoystickButton resetHeading_Start = new JoystickButton(driverController, XboxController.Button.kA.value);
  private final JoystickButton armScore_B = new JoystickButton(driverController, XboxController.Button.kB.value);
  private final JoystickButton subsDriveMode_Y = new JoystickButton(driverController, XboxController.Button.kY.value);
  // private final JoystickButton gridDriveMode_A = new JoystickButton(driverController, XboxController.Button.kA.value);
  
  public static final Launchpad opController = new Launchpad();
  private final LaunchpadButton armHigh_1_0 = new LaunchpadButton(opController, 1, 0);
  private final LaunchpadButton armMid_2_0 = new LaunchpadButton(opController, 2, 0);
  private final LaunchpadButton armLow_3_0 = new LaunchpadButton(opController, 3, 0);
  private final LaunchpadButton armZero_4_0 = new LaunchpadButton(opController, 4, 0);
  private final LaunchpadButton armSubs_2_1 = new LaunchpadButton(opController, 2, 1);

  private final LaunchpadButton armAdjustUp_1_3 = new LaunchpadButton(opController, 1, 3);
  private final LaunchpadButton armAdjustDown_2_3 = new LaunchpadButton(opController, 2, 3);

  public static final XboxController backupOpController = new XboxController(IOConstants.OP_CONTROLLER_PORT);
  private final JoystickButton armUp_Y = new JoystickButton(backupOpController, XboxController.Button.kY.value);
  private final JoystickButton armDown_A = new JoystickButton(backupOpController, XboxController.Button.kA.value);
  private final JoystickButton armSubs_X = new JoystickButton(backupOpController, XboxController.Button.kX.value);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    portForwarding();
    configureBindings();
    drivetrain.setDefaultCommand(new SwerveDrive());

    SmartDashboard.putData("Auton Chooser", autoChooser);
    autoChooser.setDefaultOption("SimpleTest", "SimpleTest");
    autoChooser.addOption("2CubeNC", "2CubeNC");
    autoChooser.addOption("2CubeNC_Bal", "2CubeNC_Bal");
    autoChooser.addOption("3CubeNC_Bal", "3CubeNC_Bal");
    autoChooser.addOption("4CubeNC", "4CubeNC");
    // autoChooser.addOption("1Cone1CubeNC_Bal", "1Cone1CubeNC_Bal");
    // autoChooser.addOption("1Cone1CubeC_Bal", "1Cone1CubeC_Bal");
    autoChooser.addOption("Nothing", "TestAuto");
    autoChooser.addOption("DriveBack", "DriveBack");
    autoChooser.addOption("SimpleTest", "SimpleTest");
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    resetHeading_Start.onTrue(new InstantCommand(drivetrain::zeroHeading, drivetrain));
    // shoot_RB.whileTrue(new Shoot(0.75)).onFalse(new InstantCommand(() -> transport.feederStop())
    //   .andThen(new InstantCommand(() -> shooter.shooterOff())));
    armScore_B.whileTrue(new RunCommand(() -> arm.intakeOut())).onFalse(new InstantCommand(() -> arm.intakeIn()));
    // gridDriveMode_A.whileTrue(new RunCommand(() -> drivetrain.setGridMode())).onFalse(new InstantCommand(() -> drivetrain.setNormalMode()));
    subsDriveMode_Y.whileTrue(new RunCommand(() -> drivetrain.setSubsMode())).onFalse(new InstantCommand(() -> drivetrain.setNormalMode()));


    armHigh_1_0.onTrue(new InstantCommand(() -> arm.setHighMode()));
    armMid_2_0.onTrue(new InstantCommand(() -> arm.setMidMode()));
    armLow_3_0.onTrue(new InstantCommand(() -> arm.setLowMode()));
    armZero_4_0.onTrue(new InstantCommand(() -> arm.setZeroMode()));
    armSubs_2_1.onTrue(new InstantCommand(() -> arm.setSubsMode()));

    armAdjustUp_1_3.onTrue(new InstantCommand(() -> arm.armAdjustUp()));
    armAdjustDown_2_3.onTrue(new InstantCommand(() -> arm.armAdjustDown()));

    armUp_Y.onTrue(new InstantCommand(() -> arm.armUp()));
    armDown_A.onTrue(new InstantCommand(() -> arm.armDown()));
    armSubs_X.onTrue(new InstantCommand(() -> arm.setSubsMode()));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    if(autoChooser.getSelected().equals("SimpleTest")) {
      drivetrain.resetAllEncoders();
      drivetrain.setHeading(0);
      return Autos.SimpleTest();
    } else{
      drivetrain.resetAllEncoders();
      drivetrain.setHeading(0);
      return Autos.driveBack();
    }
  }

  private void portForwarding(){
    EForwardableConnections.addPortForwarding(EForwardableConnections.LIMELIGHT_ARM_CAMERA_FEED);
    EForwardableConnections.addPortForwarding(EForwardableConnections.LIMELIGHT_ARM_WEB_VIEW);
  }

  
  // public void loadGridButtons(){
  //   for(int r = 0; r < 3; r++){
  //     for(int c = 0; c < 9; c++){
  //       gridButtons[r][c] = new LaunchpadButton(opController, r, c);
  //     }
  //   }
  // }
}
