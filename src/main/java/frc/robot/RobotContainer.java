// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.IOConstants;
import frc.robot.commands.ArmMove;
import frc.robot.commands.ArmZero;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.IntakeRollers;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.XboxController;
// import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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
  public static final IntakeRollers intakeRollers = IntakeRollers.getInstance();
  public static final Arm  arm = new Arm();
  public static final PowerDistribution pdh = Constants.PDH;
  public final ArmMove armMove;


  public static final XboxController driverController = new XboxController(IOConstants.DRIVER_CONTROLLER_PORT);
  private final JoystickButton Intake_ON_LB = new JoystickButton(driverController, XboxController.Button.kLeftBumper.value);
  private final JoystickButton Intake_OFF_RB = new JoystickButton(driverController, XboxController.Button.kRightBumper.value);
  private final JoystickButton ZERO_ARM = new JoystickButton(driverController, XboxController.Button.kA.value);
  private final JoystickButton ARM_UP = new JoystickButton(driverController, XboxController.Button.kX.value);
  private final JoystickButton ARM_Mid = new JoystickButton(driverController, XboxController.Button.kY.value);
  private final JoystickButton ARM_STOW = new JoystickButton(driverController, XboxController.Button.kB.value);
   
  // public static final Launchpad opController = new Launchpad();
  // // private final LaunchpadButton[][] gridButtons = new LaunchpadButton[3][9];
  // private final LaunchpadButton armHigh_1_0 = new LaunchpadButton(opController, 1, 0);
  // private final LaunchpadButton armZero_4_0 = new LaunchpadButton(opController, 4, 0);
  // private final LaunchpadButton intakeToggle_1_5 = new LaunchpadButton(opController, 1, 5);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer()
  {
    armMove = new ArmMove(arm, () -> ARM_UP.getAsBoolean(), () -> ARM_Mid.getAsBoolean(), () -> ARM_STOW.getAsBoolean());
    // Configure the trigger bindings
    configureBindings();
    // intake.setDefaultCommand(new IntakeHold());
    arm.setDefaultCommand(armMove);
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
    Intake_ON_LB.onTrue(new InstantCommand(() -> arm.pickUp()));
    Intake_ON_LB.onFalse(new InstantCommand(() -> arm.intakeStop()));
    Intake_OFF_RB.onTrue(new InstantCommand(() -> arm.outtake()));
    Intake_OFF_RB.onFalse(new InstantCommand(() -> arm.intakeStop()));
    ZERO_ARM.onTrue(new ArmZero());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
  }
}
