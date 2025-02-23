package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Swerve;


public class TeleopSwerve extends Command {
    private Swerve s_Swerve;
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    private BooleanSupplier robotCentricSup;
    double elevatorMultiplier;
    Elevator elevator;
    public TeleopSwerve(Swerve s_Swerve, Elevator elevator, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup, BooleanSupplier robotCentricSup) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);
        this.elevator = elevator;
        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.robotCentricSup = robotCentricSup;
    }

    @Override
    public void execute() {
        elevatorMultiplier = MathUtil.clamp(1 - MathUtil.applyDeadband(elevator.getElevatorPosition().magnitude()/30, Constants.constElevator.MULTIPLIER_DEADZONE), 0.1, 1);
        /* Get Values, Deadband*/
        double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.STICK_DEADBAND);
        double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.STICK_DEADBAND);
        double rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.STICK_DEADBAND);
        SmartDashboard.putNumber("Translation", translationVal);
        SmartDashboard.putNumber("Strafe", strafeVal);
        SmartDashboard.putNumber("Rotation", rotationVal);
        SmartDashboard.putNumber("Elevator/Elevator Multiplier", elevatorMultiplier);

        /* Drive */
        s_Swerve.drive(
            new Translation2d(translationVal, strafeVal).times(Constants.Swerve.MAX_SPEED).times(elevatorMultiplier),
            rotationVal * Constants.Swerve.MAX_ANGULAR_VELOCITY * elevatorMultiplier,
            !robotCentricSup.getAsBoolean(),
            true
        );
    }
}