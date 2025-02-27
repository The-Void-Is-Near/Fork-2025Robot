
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.constIntake;
import frc.robot.subsystems.Intake;

public class TeleopOuttake extends Command {
  /** Creates a new TeleopOuttake. */
  Intake intake;

  public TeleopOuttake(Intake intake) {
    this.intake = intake;
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    intake.setVoltage(constIntake.OUTTAKE_VOLTAGE);
  }

  @Override
  public void end(boolean interrupted) {
    intake.setVoltage(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
