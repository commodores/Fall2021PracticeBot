// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class Drivetrain extends SubsystemBase {

  private final Translation2d m_frontLeftLocation;
  private final Translation2d m_frontRightLocation;
  private final Translation2d m_backLeftLocation;
  private final Translation2d m_backRightLocation;

  private final SwerveModule m_frontLeft;
  private final SwerveModule m_frontRight;
  private final SwerveModule m_backLeft;
  private final SwerveModule m_backRight;

  private final AHRS m_gyro;

  private final Field2d m_field;

  private final SwerveDriveKinematics m_kinematics;
  private final SwerveDriveOdometry m_odometry;

  /** Creates a new Drivetrain. */
  public Drivetrain() {
    
    m_frontLeftLocation = new Translation2d(0.381, 0.381);
    m_frontRightLocation = new Translation2d(0.381, -0.381);
    m_backLeftLocation = new Translation2d(-0.381, 0.381);
    m_backRightLocation = new Translation2d(-0.381, -0.381);

    m_frontLeft = new SwerveModule(1, 2, 9,  DriveConstants.kFrontLeftOffset);
    m_frontRight = new SwerveModule(3, 4, 10, DriveConstants.kFrontRightOffset);
    m_backLeft = new SwerveModule(5, 6, 11, DriveConstants.kBackLeftOffset);
    m_backRight = new SwerveModule(7, 8, 12, DriveConstants.kBackRightOffset);

    m_gyro = new AHRS(SPI.Port.kMXP);

    m_field = new Field2d();

    m_kinematics =
        new SwerveDriveKinematics(
            m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);
                                      
    m_odometry =
        new SwerveDriveOdometry(m_kinematics, m_gyro.getRotation2d());

    m_gyro.reset();
    SmartDashboard.putData("Field", m_field);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  @SuppressWarnings("ParameterName")
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    var swerveModuleStates =
        m_kinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, m_gyro.getRotation2d())
                : new ChassisSpeeds(xSpeed, ySpeed, rot));
    SwerveDriveKinematics.normalizeWheelSpeeds(swerveModuleStates, DriveConstants.kMaxSpeed);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_backLeft.setDesiredState(swerveModuleStates[2]);
    m_backRight.setDesiredState(swerveModuleStates[3]);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_odometry.update(
        m_gyro.getRotation2d(),
        m_frontLeft.getState(),
        m_frontRight.getState(),
        m_backLeft.getState(),
        m_backRight.getState());
        m_field.setRobotPose(m_odometry.getPoseMeters());
  }
}
