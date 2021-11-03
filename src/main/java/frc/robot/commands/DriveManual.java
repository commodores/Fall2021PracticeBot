// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj.GenericHID;

public class DriveManual extends CommandBase {
  /** Creates a new DriveManual. */
  private final Drivetrain m_swerve;
  private boolean fieldRelative;
  private final SlewRateLimiter m_xspeedLimiter;
  private final SlewRateLimiter m_yspeedLimiter;
  private final SlewRateLimiter m_rotLimiter;

  public DriveManual(Drivetrain drivetrain, Boolean getFieldRelative) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_swerve = drivetrain;
    addRequirements(drivetrain);

    fieldRelative = getFieldRelative;

    m_xspeedLimiter = new SlewRateLimiter(1);
    m_yspeedLimiter = new SlewRateLimiter(1);
    m_rotLimiter = new SlewRateLimiter(1);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.
    final var xSpeed =
        -m_xspeedLimiter.calculate(RobotContainer.m_controller.getY(GenericHID.Hand.kLeft))
            * DriveConstants.kMaxSpeed;

    // Get the y speed or sideways/strafe speed. We are inverting this because
    // we want a positive value when we pull to the left. Xbox controllers
    // return positive values when you pull to the right by default.
    final var ySpeed =
        -m_yspeedLimiter.calculate(RobotContainer.m_controller.getX(GenericHID.Hand.kLeft))
            * DriveConstants.kMaxSpeed;

    // Get the rate of angular rotation. We are inverting this because we want a
    // positive value when we pull to the left (remember, CCW is positive in
    // mathematics). Xbox controllers return positive values when you pull to
    // the right by default.
    final var rot =
        -m_rotLimiter.calculate(RobotContainer.m_controller.getX(GenericHID.Hand.kRight))
            * DriveConstants.kMaxAngularSpeed;

    m_swerve.drive(xSpeed, ySpeed, rot, fieldRelative);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
