package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
// import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import frc.lib.util.COTSTalonFXSwerveConstants;
import frc.lib.util.SwerveModuleConstants;

public final class Constants {
    public static final String canBusName = "*";
    public static final double stickDeadband = 0.1;

    public static final class Swerve {
        public static final int pigeonID = 1;

        public static final COTSTalonFXSwerveConstants chosenModule = COTSTalonFXSwerveConstants.SDS.MK4i
                .KrakenX60(COTSTalonFXSwerveConstants.SDS.MK4i.driveRatios.L1);

        /* Drivetrain Constants */
        public static final double trackWidth = Units.Inches.of(21.75).in(Meters);
        public static final double wheelBase = Units.Inches.of(21.75).in(Meters);
        public static final double wheelCircumference = chosenModule.wheelCircumference;

        /*
         * Swerve Kinematics
         * No need to ever change this unless you are not doing a traditional
         * rectangular/square 4 module swerve
         */
        public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
                new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        /* Module Gear Ratios */
        public static final double driveGearRatio = chosenModule.driveGearRatio;
        public static final double angleGearRatio = chosenModule.angleGearRatio;

        /* Motor Inverts */
        public static final InvertedValue angleMotorInvert = chosenModule.angleMotorInvert;
        public static final InvertedValue driveMotorInvert = chosenModule.driveMotorInvert;

        /* Angle Encoder Invert */
        public static final SensorDirectionValue cancoderInvert = chosenModule.cancoderInvert;

        /* Swerve Current Limiting */
        public static final int angleCurrentLimit = 25;
        public static final int angleCurrentThreshold = 40;
        public static final double angleCurrentThresholdTime = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveCurrentLimit = 40;
        public static final int driveCurrentThreshold = 60;
        public static final double driveCurrentThresholdTime = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        /*
         * These values are used by the drive falcon to ramp in open loop and closed
         * loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc
         */
        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        /* Angle Motor PID Values */
        public static final double angleKP = chosenModule.angleKP;
        public static final double angleKI = chosenModule.angleKI;
        public static final double angleKD = chosenModule.angleKD;

        /* Drive Motor PID Values */
        public static final double driveKP = 0.12;
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0;

        /* Drive Motor Characterization Values From SYSID */
        public static final double driveKS = 0.32;
        public static final double driveKV = 1.51;
        public static final double driveKA = 0.27;

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double maxSpeed = 4.5;
        /** Radians per Second */
        public static final double maxAngularVelocity = 10.0;

        /* Neutral Modes */
        public static final NeutralModeValue angleNeutralMode = NeutralModeValue.Coast;
        public static final NeutralModeValue driveNeutralMode = NeutralModeValue.Brake;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 {
            public static final int driveMotorID = 1;
            public static final int angleMotorID = 5;
            public static final int canCoderID = 1;
            public static final Rotation2d angleOffset = Rotation2d.fromRotations(0.429443);
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    canCoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 {
            public static final int driveMotorID = 2;
            public static final int angleMotorID = 6;
            public static final int canCoderID = 2;
            public static final Rotation2d angleOffset = Rotation2d.fromRotations(-0.458496);
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    canCoderID, angleOffset);
        }

        /* Back Left Module - Module 2 */
        public static final class Mod2 {
            public static final int driveMotorID = 4;
            public static final int angleMotorID = 8;
            public static final int canCoderID = 4;
            public static final Rotation2d angleOffset = Rotation2d.fromRotations(0.323975);
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 {
            public static final int driveMotorID = 3;
            public static final int angleMotorID = 7;
            public static final int canCoderID = 3;
            public static final Rotation2d angleOffset = Rotation2d.fromRotations(-0.027344);
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    canCoderID, angleOffset);
        }
    }

    public static class constElevator {
        public static TalonFXConfiguration ELEVATOR_CONFIG = new TalonFXConfiguration();
        static {
            ELEVATOR_CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Brake;
            ELEVATOR_CONFIG.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

            ELEVATOR_CONFIG.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
            ELEVATOR_CONFIG.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Units.Inches.of(66).in(Units.Inches);
            ELEVATOR_CONFIG.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
            ELEVATOR_CONFIG.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Units.Inches.of(0)
                    .in(Units.Inches);

            ELEVATOR_CONFIG.Slot0.GravityType = GravityTypeValue.Elevator_Static;
            // Elevator motors will provide feedback in INCHES the carriage has moved
            ELEVATOR_CONFIG.Feedback.SensorToMechanismRatio = 0.4545;

            ELEVATOR_CONFIG.Slot0.kG = 0.3; // Volts to overcome gravity
            ELEVATOR_CONFIG.Slot0.kS = 0.4; // Volts to overcome static friction
            ELEVATOR_CONFIG.Slot0.kV = 0.001; // Volts for a velocity target of 1 rps
            ELEVATOR_CONFIG.Slot0.kA = 0.001; // Volts for an acceleration of 1 rps/s
            ELEVATOR_CONFIG.Slot0.kP = 0.3;
            ELEVATOR_CONFIG.Slot0.kI = 0.0;
            ELEVATOR_CONFIG.Slot0.kD = 0.0;

            ELEVATOR_CONFIG.MotionMagic.MotionMagicCruiseVelocity = 350;
            ELEVATOR_CONFIG.MotionMagic.MotionMagicAcceleration = 2500;
        }
            public static final Distance DEADZONE_DISTANCE = Units.Inches.of(1);
    }

    public static final class Limelight {
        // public static final double limelightHeight = Units.inchesToMeters(20.0);
        // public static final double limelightAngle = Units.degreesToRadians(30.0);
        public static final String limelightName = "limelight-front";
    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 1;

        /* Constraint for the motion profilied robot angle controller */
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }
}
