// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.reefPosition;
import frc.robot.subsystems.Elevator;

public class TeleopElevator extends Command {
  Elevator climb;
  reefPosition reefPos;
  /** Creates a new TeleopClimb. */
  public TeleopElevator(Elevator climb, reefPosition reefPos) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(climb);
    this.climb = climb;
    this.reefPos = reefPos;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    switch (reefPos) {
      case L1:
      climb.setPosition(Units.Inches.of(0.0));
      SmartDashboard.putNumber("Extension", 0);
      reefPos = reefPosition.L2;
        break;
      case L2:
      climb.setPosition(Units.Inches.of(10.0));
      SmartDashboard.putNumber("Extension", 1);
      reefPos = reefPosition.L3;
        break;
      case L3:
      climb.setPosition(Units.Inches.of(30.0));
      SmartDashboard.putNumber("Extension", 2);
      reefPos = reefPosition.L4;
        break;
      case L4:
      climb.setPosition(Units.Inches.of(60.0));
      SmartDashboard.putNumber("Extension", 3);
      reefPos = reefPosition.L5;
        break;
      case L5:
      climb.setPosition(Units.Inches.of(0.0));
      SmartDashboard.putNumber("Extension",4);
          reefPos = reefPosition.L1;
        break;
      }
    }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
