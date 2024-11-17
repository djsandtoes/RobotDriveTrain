// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Servo;

public class ServoSubsystem extends SubsystemBase {
  Servo myServo;

  public ServoSubsystem() {
    myServo = new Servo(1);
    SmartDashboard.putNumber("Servo Angle",0);
    }

  public void SetServoAngle (double angle) {
    myServo.setAngle(angle);
  }
  public double GetServoAngle () {
    return myServo.getAngle();
  }

  public Command SetServo(double angle) {
    return runOnce(() -> SetServoAngle(angle));
  }
  
  @Override
  public void periodic() {
       SmartDashboard.putNumber("Servo Angle",0);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
