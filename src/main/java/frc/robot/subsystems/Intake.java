
package frc.robot.subsystems;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFXS;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

@Logged
public class Intake extends SubsystemBase {
  TalonFXS intakeM;

  @NotLogged
  MotionMagicVoltage motionRequest;

  public Intake() {
    intakeM = new TalonFXS(Constants.constIntake.MOTOR_ID, Constants.CAN_BUS_NAME);
    intakeM.getConfigurator().apply(Constants.constIntake.INTAKE_CONFIG);
  }

  public void setPosition(Distance inches) {
    intakeM.setControl(motionRequest.withPosition(Units.Inches.of(intakeM.getPosition().getValueAsDouble()).plus(inches).in(Units.Inches)));
  }

  public void setVoltage(double voltageOut) {
    intakeM.setVoltage(voltageOut);
  }
}
