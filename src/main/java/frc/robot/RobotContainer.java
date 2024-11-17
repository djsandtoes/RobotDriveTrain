// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
//import frc.robot.Constants.AutoConstants;
//import frc.robot.Constants.AutoTypes;
//import frc.robot.commands.AutonomousCommand;
import frc.robot.commands.SwerveGamepadDriveCommand;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.ServoSubsystem;

import java.io.Serial;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final XboxController xboxController = new XboxController(OperatorConstants.XBOX_CONTROLLER_PORT);
  private final CommandXboxController commandXboxController = new CommandXboxController(OperatorConstants.XBOX_CONTROLLER_PORT);
 
  // The robot's subsystems and commands are defined here...
  private final DriveTrainSubsystem driveTrain = new DriveTrainSubsystem();
  private final ServoSubsystem servoSS = new ServoSubsystem();

  // Dashboard Choosers
  private final SendableChooser<Boolean> fieldRelativeChooser = new SendableChooser<>();
  private final SendableChooser<Command> AutoChooser = AutoBuilder.buildAutoChooser();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    NamedCommands.registerCommand("RaiseFlag", servoSS.SetServo(90));
    NamedCommands.registerCommand("LowerFlag", servoSS.SetServo(0));

    // Configure the trigger bindings
    configureBindings();

    fieldRelativeChooser.setDefaultOption("Field Relative", true);
    fieldRelativeChooser.addOption("Robot Relative", false);
    SmartDashboard.putData(fieldRelativeChooser);
    SmartDashboard.putData(AutoChooser);

    driveTrain.setDefaultCommand(new SwerveGamepadDriveCommand(driveTrain, commandXboxController::getLeftX,
        commandXboxController::getLeftY, commandXboxController::getRightX, fieldRelativeChooser::getSelected));
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
  
    // Define subsystem commands that get triggered here
    commandXboxController.a().onTrue(NamedCommands.getCommand("RaiseFlag"));
    commandXboxController.b().onTrue(NamedCommands.getCommand("LowerFlag"));

    ////////////////////////////////////////////////
    //  SHOOTAKE
    ////////////////////////////////////////////////
    //    ACTIVE default (Safety=N/A, Alternate=OFF)
    //      ^ INTAKE IN
    //      v SHOOTER OUT
    /*
    commandLaunchpad.intakeIn().and(commandLaunchpad.miscBlue().negate())
                               .onTrue(new SequentialCommandGroup(new ArmToPositionCommand(Arm, ArmPosition.INTAKE),
                                                                  new IntakeMoveInCommand(inTake, Arm::getArmPosition, ledController, xboxController, 0, false), 
                                                                  new ArmToPositionCommand(Arm, ArmPosition.SPEAKER_SHOOTER)));
    commandLaunchpad.shooterOut().and(commandLaunchpad.miscBlue().negate())
                                 .whileTrue(new ParallelCommandGroup(new ShooterMoveOutCommand(Shooter, Arm::getArmPosition, ledController, xboxController, 0), 
                                                                     new IntakeMoveInCommand(inTake, Arm::getArmPosition, ledController, xboxController, IntakeConstants.INTAKE_MOVE_IN_SHOOT_DELAY_IN_MS, true))
                                            .andThen(new ArmToPositionCommand(Arm, ArmPosition.INTAKE)));
    */

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return AutoChooser.getSelected();
    //return null;  
  }

  public Command getTeleopInitCommand() {
    // return new ArmToReverseLimitCommand(Arm);
    // FIXME: For now will not do anything when we start Teleop mode
    return null;  
  }

}
