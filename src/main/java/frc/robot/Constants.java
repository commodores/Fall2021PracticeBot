// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.geometry.Rotation2d;

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
        public static final double kMaxSpeed = 3.0; // 3 meters per second
        public static final double kMaxAngularSpeed = Math.PI; // 1/2 rotation per second

        public static final double kRobotLength = 0.6096;
        public static final double kRobotWidth = 0.6096;
        public static final double kWheelDiameter = 0.0762;

        public static final int kDriveEncoderResolution = 2048;

        public static final Rotation2d kFrontLeftOffset = Rotation2d.fromDegrees(8.3); //8.4
        public static final Rotation2d kFrontRightOffset = Rotation2d.fromDegrees(135); //132
        public static final Rotation2d kBackLeftOffset = Rotation2d.fromDegrees(0); //0
        public static final Rotation2d kBackRightOffset = Rotation2d.fromDegrees(45); //45

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
    }




}
