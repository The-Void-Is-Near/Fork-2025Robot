// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.reefPosition;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;

public class AutoElevator extends Command {
    Elevator elevator;
    Intake intake;
    reefPosition reefPosition;
    Command outtake = new SequentialCommandGroup(
            new InstantCommand(() -> intake.setPosition(Units.Inches.of(2.5))));

    /** Creates a new TeleopElevator. */
    public AutoElevator(Elevator elevator, Intake intake, reefPosition reefPosition) {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(elevator, intake);
        this.intake = intake;
        this.elevator = elevator;
        this.reefPosition = reefPosition;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        elevator.setReefPosition(reefPosition);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return elevator.isAtSetpoint();
    }
}