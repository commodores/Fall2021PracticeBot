// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
//import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class SwerveModule extends SubsystemBase {
  
  private static final double kModuleMaxAngularVelocity = DriveConstants.kMaxAngularSpeed;
  private static final double kModuleMaxAngularAcceleration = 2 * Math.PI; // radians per second squared

  private final WPI_TalonFX m_driveMotor;
  private final CANSparkMax m_turningMotor;

  private final Rotation2d m_turningEncoderOffset;

  private final CANCoder m_turningEncoder;

  private final PIDController m_drivePIDController;

  private final ProfiledPIDController m_turningPIDController;

  // Gains are for example purposes only - must be determined for your own robot!
  private final SimpleMotorFeedforward m_driveFeedforward;
  private final SimpleMotorFeedforward m_turnFeedforward;

  /** Creates a new SwerveModule. */
  public SwerveModule(int driveMotorChannel, int turningMotorChannel, int turningEncoder, Rotation2d offset, boolean driveReverse, boolean turnReverse) {
    m_drivePIDController = new PIDController(1, 0, 0);

    m_turningPIDController =
      new ProfiledPIDController(
          1.7,
          0,
          0,
          new TrapezoidProfile.Constraints(
              kModuleMaxAngularVelocity, kModuleMaxAngularAcceleration));

    m_driveFeedforward = new SimpleMotorFeedforward(1, 3);
    m_turnFeedforward = new SimpleMotorFeedforward(1, 0.5);

    m_driveMotor = new WPI_TalonFX(driveMotorChannel);
    m_turningMotor = new CANSparkMax(turningMotorChannel, MotorType.kBrushless);

    m_driveMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

    m_turningEncoderOffset = offset;
    
    m_turningEncoder = new CANCoder(turningEncoder);

    m_turningEncoder.setPositionToAbsolute();
    m_turningEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
    m_turningEncoder.configMagnetOffset(m_turningEncoderOffset.getDegrees());

    // Limit the PID Controller's input range between -pi and pi and set the input
    // to be continuous.
    m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(getDriveMotorEncoder(), new Rotation2d(getTurningMotorEncoder()));
  }

  private double getDriveMotorEncoder() {
    return m_driveMotor.getSelectedSensorVelocity() * 10 / DriveConstants.kDriveEncoderResolution * Math.PI
        * DriveConstants.kWheelDiameter / 5.25;
  }

  private double getTurningMotorEncoder() {
    return Math.toRadians(m_turningEncoder.getAbsolutePosition());
  }
  

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Optimize the reference state to avoid spinning further than 90 degrees
    SwerveModuleState state =
        SwerveModuleState.optimize(desiredState, new Rotation2d(getTurningMotorEncoder()));

    // Calculate the drive output from the drive PID controller.
    final double driveOutput =
        m_drivePIDController.calculate(getDriveMotorEncoder(), state.speedMetersPerSecond);

    final double driveFeedforward = m_driveFeedforward.calculate(state.speedMetersPerSecond);

    // Calculate the turning motor output from the turning PID controller.
    final double turnOutput =
        m_turningPIDController.calculate(getTurningMotorEncoder(), state.angle.getRadians());

    final double turnFeedforward =
        m_turnFeedforward.calculate(m_turningPIDController.getSetpoint().velocity);

    m_driveMotor.setVoltage(driveOutput + driveFeedforward);
    m_turningMotor.setVoltage(turnOutput + turnFeedforward);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
