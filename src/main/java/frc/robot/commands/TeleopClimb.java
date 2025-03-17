
package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climb;

public class TeleopClimb extends Command {
  Climb robotClimb;

  @Override
  public void execute() {
    // deepClimbM.Rotate(theta);
    robotClimb.rotate(0.0);

  }

  @Override
  public void initialize() {}
  @Override
  public void end(boolean interrupted) {}
  @Override
  public boolean isFinished() {
    return false;
  }
}
