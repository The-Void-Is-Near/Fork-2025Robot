// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.ctre.phoenix6.hardware.TalonFX;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Intake {
  /** Creates a new Intake. */
  TalonFX intakeM;
  public Intake() {
    intakeM = new TalonFX(16);//MOTOR ID
    // Use addRequirements() here to declare subsystem dependencies.
  }
  public void spin(double PercentOut){
    intakeM.setVoltage(-PercentOut);
    
  }
}
