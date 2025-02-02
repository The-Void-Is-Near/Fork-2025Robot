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
  Elevator elevator;
  reefPosition nextReefPos; // Target position
  reefPosition currentReefPos = reefPosition.L1; // Current position
  boolean end;
  boolean invert;

  /** Creates a new TeleopClimb. */
  public TeleopElevator(Elevator elevator, boolean invert) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elevator);
    this.elevator = elevator;
    this.invert = invert;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    end = false;
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putBoolean("Invert", invert);
    switch (currentReefPos) {
      case L1:
        if (!invert) { // Moving up
          nextReefPos = reefPosition.L2;
        } else { // At the lowest position, can't move down
          end = true; // End command gracefully
          return;
        }
        break;
    
      case L2:
        nextReefPos = invert ? reefPosition.L1 : reefPosition.L3;
        break;
    
      case L3:
        nextReefPos = invert ? reefPosition.L2 : reefPosition.L4;
        break;
    
      case L4:
        if (!invert) { // At the highest position, can't move up
          end = true;
          return;
        } else {
          nextReefPos = reefPosition.L3;
        }
        break;
    }

    // if (nextReefPos != reefPosition.NONE) {
      elevator.setPosition(Units.Inches.of(nextReefPos == reefPosition.L1 ? 0.0 : 
                                          nextReefPos == reefPosition.L2 ? 20.0 : 
                                          nextReefPos == reefPosition.L3 ? 40.0 : 
                                          nextReefPos == reefPosition.L4 ? 60.0 : 0.0));
      SmartDashboard.putNumber("Extension", nextReefPos.ordinal());
      currentReefPos = nextReefPos;
      SmartDashboard.putString("ExtensionPos", currentReefPos.toString());
      end = true;
    // }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    nextReefPos = currentReefPos;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return end;
  }
}
