// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.IOConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.SwerveDrive;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.XboxController;
// import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.ArmMove;
// import frc.robot.commands.ArmZero;
import frc.robot.commands.IntakeMove;
// import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
// import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
// import edu.wpi.first.wpilibj2.command.RunCommand;
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
  //public static final Arm arm = Arm.getInstance();
  public static final Intake intake = new Intake();
  public static final Arm arm = new Arm();
  public static final PowerDistribution pdh = Constants.PDH;
  private SendableChooser<String> autoChooser = new SendableChooser<>();
  public final ArmMove armMove;
  // public final IntakeMove intakeMove;


  public static final XboxController driverController = new XboxController(IOConstants.DRIVER_CONTROLLER_PORT);
  public static final XboxController OpController = new XboxController(IOConstants.OP_CONTROLLER_PORT);

  private final JoystickButton resetHeading_Start = new JoystickButton(driverController, XboxController.Button.kA.value);
  // private final JoystickButton subsDriveMode_Y = new JoystickButton(driverController, XboxController.Button.kY.value);
  // private final JoystickButton gridDriveMode_A = new JoystickButton(driverController, XboxController.Button.kA.value);

  private final JoystickButton Intake_ON_LB = new JoystickButton(OpController, XboxController.Button.kLeftBumper.value);
  private final JoystickButton Intake_OFF_RB = new JoystickButton(OpController, XboxController.Button.kRightBumper.value);

  // private final JoystickButton ZERO_ARM = new JoystickButton(driverController, XboxController.Button.kA.value);
  private final JoystickButton ARM_UP = new JoystickButton(OpController, XboxController.Button.kX.value);
  private final JoystickButton ARM_Mid = new JoystickButton(OpController, XboxController.Button.kA.value);
  private final JoystickButton ARM_STOW = new JoystickButton(OpController, XboxController.Button.kB.value);
  
  private final JoystickButton INTAKE_DOWN = new JoystickButton(OpController, XboxController.Button.kY.value);

  private final JoystickButton resetIntakeEncoder = new JoystickButton(driverController, XboxController.Button.kY.value);
  private final JoystickButton resetArmEncoder = new JoystickButton(driverController, XboxController.Button.kY.value);

  private final JoystickButton lockWheels = new JoystickButton(driverController, XboxController.Button.kB.value);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer()
  {
    armMove = new ArmMove(arm, () -> ARM_UP.getAsBoolean(), () -> ARM_Mid.getAsBoolean(), () -> ARM_STOW.getAsBoolean()); 
    // intakeMove = new IntakeMove(intake, () -> INTAKE_DOWN.getAsBoolean(), () -> INTAKE_STOW.getAsBoolean());
    
    // arm.armEncZero();
    // intake.intakeEncZero();
    // Configure the trigger bindings
    configureBindings();
    drivetrain.setDefaultCommand(new SwerveDrive());
    arm.setDefaultCommand(armMove);

    SmartDashboard.putData("Auton Chooser", autoChooser);
    autoChooser.setDefaultOption("Drive", "Drive");                   //Default auton
    autoChooser.addOption("DriveBack", "DriveBack");
    autoChooser.addOption("SimpleTest", "SimpleTest");                //Drives straight (testing only)
    autoChooser.addOption("Drive", "Drive");                          //Drives out of community (3 points // loading side zone)
    autoChooser.addOption("Balance(NEEDS TUNING)", "Balance");        //Drives and balances on charge station (12 points // charging station zone)
    autoChooser.addOption("Cube+Drive(UNTESTED)", "Cube+Drive");      //Outtakes game piece into hybrid node and drives out of community (6 points // loading side zone)
    autoChooser.addOption("highNodeAndDrive", "highNodeAndDrive");    //()
    autoChooser.addOption("highNodeAndBalance", "highNodeAndBalance");//()
    autoChooser.addOption("TEST", "TEST");                            //()
  }
    // intake.setDefaultCommand(new IntakeHold());
    // arm.setDefaultCommand(armMove);
    // intake.setDefaultCommand(intakeMove);
                                                                                                                              

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
    // gridDriveMode_A.whileTrue(new RunCommand(() -> drivetrain.setGridMode())).onFalse(new InstantCommand(() -> drivetrain.setNormalMode()));
    // subsDriveMode_Y.whileTrue(new RunCommand(() -> drivetrain.setSubsMode())).onFalse(new InstantCommand(() -> drivetrain.setNormalMode()));

    //Intaking and outtaking
    Intake_ON_LB.onTrue(new InstantCommand(() -> arm.pickUp())); //intake cones, outtake cubes
    Intake_ON_LB.onTrue(new InstantCommand(() -> intake.groundPickUp())); 
    Intake_ON_LB.onFalse(new InstantCommand(() -> arm.intakeStop()));
    Intake_ON_LB.onFalse(new InstantCommand(() -> intake.groundIntakeStop()));

    Intake_OFF_RB.onTrue(new InstantCommand(() -> arm.outtake())); // intake cubes, outtake cones
    Intake_OFF_RB.onTrue(new InstantCommand(() -> intake.groundOuttake()));
    Intake_OFF_RB.onFalse(new InstantCommand(() -> arm.intakeStop()));
    Intake_OFF_RB.onFalse(new InstantCommand(() -> intake.groundIntakeStop()));
    
    //Intake up and down Neo Control
    INTAKE_DOWN.onTrue(new IntakeMove(intake, () -> true, ()-> false));
    INTAKE_DOWN.onFalse(new IntakeMove(intake, () -> false, ()-> true));
    
    //Zero encoders
    resetIntakeEncoder.onTrue(new InstantCommand(() -> intake.intakeEncZero()));
    resetArmEncoder.onTrue(new InstantCommand(() -> arm.armEncZero()));

    //Lock wheels
    lockWheels.onTrue(new InstantCommand(() -> drivetrain.formX()));
    lockWheels.onTrue(new InstantCommand(() -> drivetrain.stopModules()));

    //Arm Zero
    // ZERO_ARM.onTrue(new ArmZero());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    if(autoChooser.getSelected().equals("Drive")) {
      drivetrain.resetAllEncoders();
      drivetrain.setHeading(0);
      return Autos.Drive();
    } else if(autoChooser.getSelected().equals("SimpleTest")) {
      drivetrain.resetAllEncoders();
      drivetrain.setHeading(0);
      return Autos.SimpleTest();
    } else if(autoChooser.getSelected().equals("Balance")) {
      drivetrain.resetAllEncoders();
      drivetrain.setHeading(0);
      return Autos.Balance();
    } else if(autoChooser.getSelected().equals("Cube+Drive")) {
      drivetrain.resetAllEncoders();
      drivetrain.setHeading(0);
      return Autos.cubeAndDrive();
    } else if(autoChooser.getSelected().equals("highNodeAndDrive")) {
      drivetrain.resetAllEncoders();
      drivetrain.setHeading(0);
      return Autos.highNodeAndDrive();
    } else if(autoChooser.getSelected().equals("highNodeAndBalance")) {
      drivetrain.resetAllEncoders();
      drivetrain.setHeading(0);
      return Autos.highNodeAndBalance();
    } else if(autoChooser.getSelected().equals("TEST")) {
      drivetrain.resetAllEncoders();
      drivetrain.setHeading(0);
      return Autos.testAuto();
    } else {
      drivetrain.resetAllEncoders();
      drivetrain.setHeading(0);
      return Autos.driveBack();
    }
  }

  
  // public void loadGridButtons(){
  //   for(int r = 0; r < 3; r++){
  //     for(int c = 0; c < 9; c++){
  //       gridButtons[r][c] = new LaunchpadButton(opController, r, c);
  //     }
  //   }
  // }
}
