// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Swerve;

public class TeleopLimelightDrive extends Command {
  Swerve swerve;
  Limelight limelight;
  boolean amp;
  ChassisSpeeds relativeSpeed;
  boolean gyro;
  int invert;
  boolean align;
  Translation2d translation;
  Translation2d translationCoordinates;
  Translation2d finalTranslation;
  /** Creates a new TeleopLimelightDrive. */
  public TeleopLimelightDrive(Swerve swerve, Limelight limelight, boolean align) {
    this.swerve = swerve;
    this.align = align;
    this.limelight = limelight;
    addRequirements(swerve, limelight);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    LimelightHelpers.setLEDMode_ForceOn("limelight-front");
    // translationCoordinates = swerve.getLimelightPose();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
        Translation2d translationVector = new Translation2d(0, 0.5);
        Rotation2d rotationVector = new Rotation2d(0);
        Pose2d finalVector = new Pose2d(translationVector, rotationVector);
        double translationAllowedX = 1;
        double translationAllowedY = 0;
        double translationCoordinatesX;
        double translationCoordinatesY;

    limelight.limelightTagMode(true);
    swerve.getLimelightPose();
    if (align == true | swerve.mt2Pose !=null) {
    // translationCoordinates = swerve.mt2Pose.minus(finalVector);
    translationCoordinatesX = swerve.mt2Pose.getX()-(translationAllowedX);
    translationCoordinatesY = swerve.mt2Pose.getY()-(translationAllowedY);
    translationCoordinates = new Translation2d(translationCoordinatesX, translationCoordinatesY);
    finalTranslation = new Translation2d(translationCoordinates.getX(), translationVector.getY());
    } else if (align == false | swerve.mt2Pose !=null) {
    translationCoordinatesX = swerve.mt2Pose.getX()+(translationAllowedX);
    translationCoordinatesY = swerve.mt2Pose.getY()+(translationAllowedY);
    translationCoordinates = new Translation2d(translationCoordinatesX, translationCoordinatesY);
    finalTranslation = new Translation2d(translationCoordinates.getX(), translationVector.getY());

        double strafeProportional;
        @SuppressWarnings("unused")
        boolean strafeDisqualifier;
        @SuppressWarnings("unused")
        Translation2d translationDifference;
        if (align == true) {
          // strafeVal = limelight.limelight_strafe_proportional() - 1;
          strafeProportional = limelight.limelight_strafe_proportional();
          strafeDisqualifier = true;
          SmartDashboard.putNumber("strafeProportional", strafeProportional);
          SmartDashboard.putBoolean("strafeDisqualifier", true);
          SmartDashboard.putBoolean("align", align);
        } else {
          // strafeVal = limelight.limelight_strafe_proportional() + 1;
          strafeProportional = limelight.limelight_strafe_proportional();
          strafeDisqualifier = false;
          SmartDashboard.putNumber("strafeProportional", strafeProportional);
          SmartDashboard.putBoolean("strafeDisqualifier", false);
          SmartDashboard.putBoolean("align", align);
        }
        /* Drive */
        swerve.drive(finalTranslation, swerve.mt2Pose.getRotation().getDegrees(), true, true);
      }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    limelight.limelightTagMode(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
