package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
// import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import frc.lib.util.COTSTalonFXSwerveConstants;
import frc.lib.util.SwerveModuleConstants;

public final class Constants {
    public static final String CAN_BUS_NAME = "*";
    public static final double STICK_DEADBAND = 0.1;

    public static final class Swerve {
        public static final int PIGEON_ID = 0;

        public static final COTSTalonFXSwerveConstants CHOSEN_MODULE = COTSTalonFXSwerveConstants.SDS.MK4n
                .KrakenX60(COTSTalonFXSwerveConstants.SDS.MK4i.driveRatios.L3);

        /* Drivetrain Constants */
        public static final double TRACK_WIDTH = Units.Inches.of(21.75).in(Meters);
        public static final double WHEEL_BASE = Units.Inches.of(21.75).in(Meters);
        public static final double WHEEL_CIRCUMFRENCE = CHOSEN_MODULE.wheelCircumference;

        /*
         * Swerve Kinematics
         * No need to ever change this unless you are not doing a traditional
         * rectangular/square 4 module swerve
         */
        public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
                new Translation2d(WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
                new Translation2d(WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0),
                new Translation2d(-WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
                new Translation2d(-WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0));

        /* Module Gear Ratios */
        public static final double DRIVE_GEAR_RATIO = CHOSEN_MODULE.driveGearRatio;
        public static final double ANGLE_GEAR_RATIO = CHOSEN_MODULE.angleGearRatio;

        /* Motor Inverts */
        public static final InvertedValue ANGLE_MOTOR_INVERT = CHOSEN_MODULE.angleMotorInvert;
        public static final InvertedValue DRIVE_MOTOR_INVERT = CHOSEN_MODULE.driveMotorInvert;

        /* Angle Encoder Invert */
        public static final SensorDirectionValue CANCODER_INVERT = CHOSEN_MODULE.cancoderInvert;

        /* Swerve Current Limiting */
        public static final int ANGLE_CURRENT_LIMIT = 25;
        public static final int ANGLE_CURRENT_THRESHOLD = 40;
        public static final double ANGLE_CURRENT_THRESHOLD_TIME = 0.1;
        public static final boolean ANGLE_ENABLE_CURRENT_LIMIT = true;

        public static final int DRIVE_CURRENT_LIMIT = 40;
        public static final int DRIVE_CURRENT_THRESHOLD = 60;
        public static final double DRIVE_CURRENT_THRESHOLD_TIME = 0.1;
        public static final boolean DRIVE_ENABLE_CURRENT_LIMIT = true;

        /*
         * These values are used by the drive falcon to ramp in open loop and closed
         * loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc
         */
        public static final double OPEN_LOOP_RAMP = 0.25;
        public static final double CLOSED_LOOP_RAMP = 0.0;

        /* Angle Motor PID Values */
        public static final double ANGLE_KP = CHOSEN_MODULE.angleKP;
        public static final double ANGLE_KI = CHOSEN_MODULE.angleKI;
        public static final double ANGLE_KD = CHOSEN_MODULE.angleKD;

        /* Drive Motor PID Values */
        public static final double DRIVE_KP = 0.12;
        public static final double DRIVE_KI = 0.0;
        public static final double DRIVE_KD = 0.0;
        public static final double DRIVE_KF = 0.0;

        /* Drive Motor Characterization Values From SYSID */
        public static final double DRIVE_KS = 0.32;
        public static final double DRIVE_KV = 1.51;
        public static final double DRIVE_KA = 0.27;

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double MAX_SPEED = 4.5;
        /** Radians per Second */
        public static final double MAX_ANGULAR_VELOCITY = 10.0;

        /* Neutral Modes */
        public static final NeutralModeValue ANGLE_NEUTRAL_MODE = NeutralModeValue.Coast;
        public static final NeutralModeValue DRIVE_NEUTRAL_MODE = NeutralModeValue.Brake;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 {
            public static final int DRIVE_MOTOR_ID = 7;
            public static final int ANGLE_MOTOR_ID = 9;
            public static final int CAN_CODER_ID = 8;
            public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromRotations(0.100342);
            public static final SwerveModuleConstants CONSTANTS = new SwerveModuleConstants(DRIVE_MOTOR_ID,
                    ANGLE_MOTOR_ID,
                    CAN_CODER_ID, ANGLE_OFFSET);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 {
            public static final int DRIVE_MOTOR_ID = 4;
            public static final int ANGLE_MOTOR_ID = 6;
            public static final int CAN_CODER_ID = 5;
            public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromRotations(-0.081543);
            public static final SwerveModuleConstants CONSTANTS = new SwerveModuleConstants(DRIVE_MOTOR_ID,
                    ANGLE_MOTOR_ID,
                    CAN_CODER_ID, ANGLE_OFFSET);
        }

        /* Back Left Module - Module 2 */
        public static final class Mod2 {
            public static final int DRIVE_MOTOR_ID = 10;
            public static final int ANGLE_MOTOR_ID = 12;
            public static final int CAN_CODER_ID = 11;
            public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromRotations(-0.374268);
            public static final SwerveModuleConstants CONSTANTS = new SwerveModuleConstants(DRIVE_MOTOR_ID,
                    ANGLE_MOTOR_ID,
                    CAN_CODER_ID, ANGLE_OFFSET);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 {
            public static final int DRIVE_MOTOR_ID = 1;
            public static final int ANGLE_MOTOR_ID = 3;
            public static final int CAN_CODER_ID = 2;
            public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromRotations(-0.068115);
            public static final SwerveModuleConstants CONSTANTS = new SwerveModuleConstants(DRIVE_MOTOR_ID,
                    ANGLE_MOTOR_ID,
                    CAN_CODER_ID, ANGLE_OFFSET);
        }
    }

    /* Intake Constants */
    public static final class constIntake {
        public static TalonFXSConfiguration INTAKE_CONFIG = new TalonFXSConfiguration();
        static {
            INTAKE_CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Brake;
            INTAKE_CONFIG.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
            INTAKE_CONFIG.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;

            INTAKE_CONFIG.Slot0.GravityType = GravityTypeValue.Elevator_Static;
            INTAKE_CONFIG.Slot0.kS = 0.6; // Volts to overcome static friction
            INTAKE_CONFIG.Slot0.kG = 0;
            INTAKE_CONFIG.Slot0.kV = 0.001; // Volts for a velocity target of 1 rps
            INTAKE_CONFIG.Slot0.kA = 0.001; // Volts for an acceleration of 1 rps/s
            INTAKE_CONFIG.Slot0.kP = 0.95;
            INTAKE_CONFIG.Slot0.kI = 0.01;
            INTAKE_CONFIG.Slot0.kD = 0.0095;

            // Inches the outside of the wheel has moved
            INTAKE_CONFIG.ExternalFeedback.SensorToMechanismRatio = 0.31847;
            INTAKE_CONFIG.MotionMagic.MotionMagicCruiseVelocity = 400;
            INTAKE_CONFIG.MotionMagic.MotionMagicAcceleration = 1500;
        }
        public static final int MOTOR_ID = 17;
        public static final double OUTTAKE_VOLTAGE = 3;
        public static final double INTAKE_VOLTAGE = -3;
    }

    public static class constElevator {
        public static TalonFXConfiguration ELEVATOR_CONFIG = new TalonFXConfiguration();
        static {
            ELEVATOR_CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Brake;
            ELEVATOR_CONFIG.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

            ELEVATOR_CONFIG.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
            ELEVATOR_CONFIG.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Units.Inches.of(72).in(Units.Inches);
            ELEVATOR_CONFIG.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
            ELEVATOR_CONFIG.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Units.Inches.of(0)
                    .in(Units.Inches);

            ELEVATOR_CONFIG.Slot0.GravityType = GravityTypeValue.Elevator_Static;
            // Elevator motors will provide feedback in INCHES the carriage has moved
            ELEVATOR_CONFIG.Feedback.SensorToMechanismRatio = 0.343; // 17 inches to ground

            ELEVATOR_CONFIG.Slot0.kG = 0.75; // Volts to overcome gravity
            ELEVATOR_CONFIG.Slot0.kS = 0.5; // Volts to overcome static friction
            ELEVATOR_CONFIG.Slot0.kV = 0.001; // Volts for a velocity target of 1 rps
            ELEVATOR_CONFIG.Slot0.kA = 0.001; // Volts for an acceleration of 1 rps/s
            ELEVATOR_CONFIG.Slot0.kP = 1.6;
            ELEVATOR_CONFIG.Slot0.kI = 0.001;
            ELEVATOR_CONFIG.Slot0.kD = 0.1;

            ELEVATOR_CONFIG.MotionMagic.MotionMagicCruiseVelocity = 400;
            ELEVATOR_CONFIG.MotionMagic.MotionMagicAcceleration = 500;
        }
        public static TalonFXConfiguration COAST_MODE_CONFIGURATION = new TalonFXConfiguration();
        static {
            COAST_MODE_CONFIGURATION.MotorOutput.NeutralMode = NeutralModeValue.Coast;
            COAST_MODE_CONFIGURATION.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        }
        public static final Distance DEADZONE_DISTANCE = Units.Inches.of(1);
        public static final double MULTIPLIER_DEADZONE = 0.1;
        public static final int LEFT_MOTOR_FOLLOWER_ID = 13;
        public static final int RIGHT_MOTOR_LEADER_ID = 14;

        public static final AngularVelocity MANUAL_ZEROING_START_VELOCITY = Units.RotationsPerSecond.of(5);
        public static final AngularVelocity MANUAL_ZEROING_DELTA_VELOCITY = Units.RotationsPerSecond.of(5);

        /**
         * The voltage supplied to the motor in order to zero
         */
        public static final Voltage ZEROING_VOLTAGE = Units.Volts.of(-1);
        /**
         * The value that the motor reports when it is at it's zeroed position. This
         * may not necessarily be 0 due to mechanical slop
         */
        public static final Distance ZEROED_POS = Units.Meters.of(0);

        /**
         * The velocity that the motor goes at once it has zeroed (and can no longer
         * continue in that direction)
         */
        public static final AngularVelocity ZEROED_VELOCITY = Units.RotationsPerSecond.of(0.2);

        /**
         * The elapsed time required to consider the motor as zeroed
         */
        public static final Time ZEROED_TIME = Units.Seconds.of(1);
        public static final Time ZEROING_TIMEOUT = Units.Seconds.of(3);
    }

    public enum reefPosition {
        NONE,
        L1,
        L2,
        L3,
        L4
    }

    public static final class Limelight {
        // public static final double limelightHeight = Units.inchesToMeters(20.0);
        // public static final double limelightAngle = Units.degreesToRadians(30.0);
        public static final String LIMELIGHT_FRONT_NAME = "limelight-front";
    }

    public static final class AutoConstants {
        public static final double KMAX_SPEED_METERS_PER_SECOND = 3;
        public static final double KMAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 3;
        public static final double KMAX_ANGULAR_SPEED_RADIANS_PER_SECOND = Math.PI;
        public static final double KMAX_ANGULAR_SPEED_RADIANS_PER_SECOND_SQUARED = Math.PI;

        public static final double KPX_CONTROLLER = 1;
        public static final double KPY_CONTROLLER = 1;
        public static final double KP_THETA_CONTROLLER = 1;

        public static final double MASS = 115;
        // TODO: Calcuate the real value
        public static final double MOI = 6.8;
        public static final double WHEEL_COF = 1.0;
        public static final DCMotor DRIVE_MOTOR = DCMotor.getKrakenX60(1);
        public static final ModuleConfig MODULE_CONFIG = new ModuleConfig(Constants.Swerve.WHEEL_CIRCUMFRENCE / 2,
                Constants.Swerve.MAX_SPEED, WHEEL_COF,
                DRIVE_MOTOR,
                Constants.Swerve.DRIVE_CURRENT_LIMIT, 1);

        public static final Translation2d[] MODULE_OFFSETS = {
                new Translation2d(Constants.Swerve.WHEEL_BASE / 2.0, Constants.Swerve.TRACK_WIDTH / 2.0),
                new Translation2d(Constants.Swerve.WHEEL_BASE / 2.0, -Constants.Swerve.TRACK_WIDTH / 2.0),
                new Translation2d(-Constants.Swerve.WHEEL_BASE / 2.0, Constants.Swerve.TRACK_WIDTH / 2.0),
                new Translation2d(-Constants.Swerve.WHEEL_BASE / 2.0, -Constants.Swerve.TRACK_WIDTH / 2.0) };

        public static final RobotConfig ROBOT_CONFIG = new RobotConfig(MASS, MOI, MODULE_CONFIG, MODULE_OFFSETS);

        /* Constraint for the motion profilied robot angle controller */
        public static final TrapezoidProfile.Constraints KTHETA_CONTROLLER_CONSTRAINTS = new TrapezoidProfile.Constraints(
                KMAX_ANGULAR_SPEED_RADIANS_PER_SECOND, KMAX_ANGULAR_SPEED_RADIANS_PER_SECOND_SQUARED);
    }
}
