// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

public class TeleopElevator extends Command {
  DoubleSupplier climbPositionSup;
  Elevator climb;
  /** Creates a new TeleopClimb. */
  public TeleopElevator(Elevator climb, DoubleSupplier climbPositionSup) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(climb);
    this.climb = climb;
    this.climbPositionSup = climbPositionSup;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    double climbPositionVal = MathUtil.applyDeadband(climbPositionSup.getAsDouble(), 0.2);
    climb.setPosition(null);
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
