

package frc.robot.commands;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj2.command.Command;

public class TeleopIntake extends Command {
  /** Creates a new TeleopIntake. */
  Intake intake;
  double voltageOut;
  public TeleopIntake(Intake intake, double voltageOut) {
    this.intake = intake;
    this.voltageOut = voltageOut;
  }

  @Override
  public void execute() {
    intake.Spin(voltageOut);
  }
/* */
  @Override
  public void initialize() {}
  @Override
  public void end(boolean interrupted) {}
  @Override
  public boolean isFinished() {
    return false;
  }
}
