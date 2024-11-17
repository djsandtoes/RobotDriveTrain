// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.DriveTrainSubsystem;

public class SwerveGamepadDriveCommand extends Command {

  private final DriveTrainSubsystem swerveDriveTrain;
  private final DoubleSupplier xSpeedSupplier, ySpeedSupplier, rotateSpeedSupplier;
  private final BooleanSupplier fieldOrientedDrive;

  /** Creates a new SwerveControllerDrive. */
  public SwerveGamepadDriveCommand(DriveTrainSubsystem swerveDriveTrain, DoubleSupplier ySpeedSupplier,
      DoubleSupplier xSpeedSupplier, DoubleSupplier rotateSpeedSupplier, BooleanSupplier fieldOrientedDrive) {
    this.swerveDriveTrain = swerveDriveTrain;
    this.ySpeedSupplier = ySpeedSupplier;
    this.xSpeedSupplier = xSpeedSupplier;
    this.rotateSpeedSupplier = rotateSpeedSupplier;
    this.fieldOrientedDrive = fieldOrientedDrive;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.swerveDriveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  // This is already done in the subsystem
  // swerveDriveTrain.zeroHeading();
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //SmartDashboard.putNumber("Gyro value", swerveDriveTrain.getHeading());
    double xSpeed = xSpeedSupplier.getAsDouble();
    double ySpeed = ySpeedSupplier.getAsDouble();
    double rotateSpeed = rotateSpeedSupplier.getAsDouble();

    swerveDriveTrain.drive(
                -MathUtil.applyDeadband(xSpeed, OperatorConstants.XBOX_DEADBAND),
                -MathUtil.applyDeadband(ySpeed, OperatorConstants.XBOX_DEADBAND),
                -MathUtil.applyDeadband(rotateSpeed, OperatorConstants.XBOX_DEADBAND),
                fieldOrientedDrive.getAsBoolean(), true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerveDriveTrain.drive(0, 0, 0, interrupted, interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
