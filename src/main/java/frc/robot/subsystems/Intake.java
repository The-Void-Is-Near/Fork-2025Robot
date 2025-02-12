

package frc.robot.subsystems;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase{
  TalonFX intakeM;

  public Intake() {
    intakeM = new TalonFX(Constants.IntakeVals.motorID);
  }
  public void Spin(double voltageOut){
    intakeM.setVoltage(voltageOut);
  }
}
