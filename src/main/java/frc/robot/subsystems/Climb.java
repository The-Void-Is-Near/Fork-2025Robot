// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climb extends SubsystemBase {
  /** Creates a new TeleopDeepClimb. */
  public static final TalonFX climbMotor = new TalonFX(Constants.climb.motorID);
  public double theta;


  public void rotate(double theta){
    double position = climbMotor.getPosition().getValueAsDouble(); //in Rotations
    double inDegrees = position * 360; //to Degrees
    double newPosition = (inDegrees+theta)/360; //to Rotations
    climbMotor.setPosition(newPosition);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
