// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;

import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final class DriveConstants {
        
        //Robot Characteristics
        public static final double kRobotLength = 0.6096;
        public static final double kRobotWidth = 0.6096;
        public static final double kWheelDiameter = 0.0762;

        public static final int kDriveEncoderResolution = 2048;
        public static final double kDriveGearRatio = 5.25;

        //Power Management for Drivetrain
        public static StatorCurrentLimitConfiguration kTalonCurrentConfig = new StatorCurrentLimitConfiguration(true, 60, 40, 1.0);

        public static int kVoltageCompensation = 12;

        public static int kRevContinuosCurrentLimit = 15;
        public static int kRevPeakCurrentLimit = 30;
        
        //CAN IDS for Motors and Encoders
        public static final int kFrontLeftDriveChannel = 10;
        public static final int kFrontLeftTurnChannel = 11;
        public static final int kFrontLeftEncoderChannel = 12;

        public static final int kFrontRightDriveChannel = 13;
        public static final int kFrontRightTurnChannel = 14;
        public static final int kFrontRightEncoderChannel = 15;

        public static final int kBackLeftDriveChannel = 16;
        public static final int kBackLeftTurnChannel = 17;
        public static final int kBackLeftEncoderChannel = 18;

        public static final int kBackRightDriveChannel = 19;
        public static final int kBackRightTurnChannel = 20;
        public static final int kBackRightEncoderChannel = 21;

        //Robot Charachterization Information
        public static final double ksVolts = 1;
        public static final double kvVoltSecondsPerMeter = 0.8;
        public static final double kaVoltSecondsSquaredPerMeter = 0.15;

        public static final double kMaxSpeedMetersPerSecond = 3;

        //Cancoder offsets (angle to magnetic north)
        //these were moved to the cancoder configuration
        //these numbers are just for reference
        public static final Rotation2d kFrontLeftOffset = Rotation2d.fromDegrees(-8.88); //8.4
        public static final Rotation2d kFrontRightOffset = Rotation2d.fromDegrees(-126.470); //132
        public static final Rotation2d kBackLeftOffset = Rotation2d.fromDegrees(0); //0
        public static final Rotation2d kBackRightOffset = Rotation2d.fromDegrees(-42.80); //45
    }

    public static final class ModuleConstants {
        public static final double kMaxModuleAngularSpeedRadiansPerSecond = 2 * Math.PI;
        public static final double kMaxModuleAngularAccelerationRadiansPerSecondSquared = 2 * Math.PI;
    
        public static final double kPModuleTurningController = 1;
    
        public static final double kPModuleDriveController = 1;

        public static final SimpleMotorFeedforward m_driveFeedforward = new SimpleMotorFeedforward(1,3);
        public static final SimpleMotorFeedforward m_turnFeedforward = new SimpleMotorFeedforward(1,.5);
      }

      public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
    
        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 1;
    
        // Constraint for the motion profilied robot angle controller
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
      }




}
