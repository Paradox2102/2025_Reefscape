// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.PositionTrackerPose;
import frc.robot.subsystems.DriveSubsystem;

public class DriveToPosition extends Command {
  /** Creates a new AutoTranslate. */
  DriveSubsystem m_subsystem;
  double m_xPos;
  double m_yPos;
  Rotation2d m_rotation = new Rotation2d();
  PositionTrackerPose m_tracker;

  double m_currentX = 0;
  double m_currentY = 0;
  double m_currentRot = 0;

  double xVelocity = 0;
  double yVelocity = 0; 

  private static final double k_p = 0.8;
  private static final double k_i = .5;//.02;
  private static final double k_d = 0;//.02;
  private static final double k_deadzoneMeters = .01;

  PIDController m_xPID = new PIDController(k_p, k_i, k_d);
  PIDController m_yPID = new PIDController(k_p, k_i, k_d);
  double m_rot = 0;

  public DriveToPosition(DriveSubsystem driveSubsystem) {
    m_xPID.setIZone(.15);
    m_yPID.setIZone(.15);
    m_subsystem = driveSubsystem;
    m_tracker = m_subsystem.getTracker();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_xPos = m_subsystem.getReefPosition().targetPose().getX();
    m_yPos = m_subsystem.getReefPosition().targetPose().getY();
    m_xPID.setSetpoint(m_xPos);
    m_yPID.setSetpoint(m_yPos);
    m_rotation = m_subsystem.getReefPosition().targetPose().getRotation();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d currentPose = m_subsystem.getTracker().getPose2d();
    m_currentX = currentPose.getX();
    m_currentY = currentPose.getY();
    m_currentRot = currentPose.getRotation().getDegrees();

    xVelocity = Constants.States.m_alliance == Alliance.Blue ? m_xPID.calculate(m_currentX) : - m_xPID.calculate(m_currentX);
    yVelocity = m_yPID.calculate(m_currentY);
    double rotVelocity = m_subsystem.orientPID(() -> m_rotation.getDegrees()).get().getDegrees();

    // xVelocity += k_f * Math.signum(xVelocity);
    // yVelocity += k_f * Math.signum(yVelocity);
    rotVelocity += Constants.DriveConstants.k_rotateF * Math.signum(rotVelocity);
    
    xVelocity = Math.abs(m_currentX - m_xPos) < k_deadzoneMeters ? 0 : xVelocity;
    yVelocity = Math.abs(m_currentY - m_yPos) < k_deadzoneMeters ? 0 : yVelocity;

    yVelocity *= Constants.States.m_alliance == Alliance.Blue ? -1 : 1;

    xVelocity = MathUtil.clamp(xVelocity, -0.2, 0.2);
    yVelocity = MathUtil.clamp(yVelocity, -0.2, 0.2);
    // rotVelocity = MathUtil.clamp(rotVelocity, -0.5, 0.5);

    // xVelocity = MathUtil.applyDeadband(xVelocity, .07);
    // yVelocity = MathUtil.applyDeadband(yVelocity, .07);

    m_subsystem.drive(xVelocity, -yVelocity, rotVelocity, true, true);
    SmartDashboard.putNumber("x dist", m_xPos - m_currentX);
    SmartDashboard.putNumber("y dist", m_yPos - m_currentY);
    SmartDashboard.putNumber("rot dist", m_rotation.getDegrees() - m_currentRot);
    SmartDashboard.putNumber("x", m_currentX);
    SmartDashboard.putNumber("y", m_currentY);
    SmartDashboard.putNumber("rot", m_currentRot);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("DriveToPosition End");
    m_subsystem.drive(0, 0, 0, true, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return 
      (
        Math.abs(m_xPos - m_currentX) < k_deadzoneMeters
        && 
        Math.abs(m_yPos - m_currentY) < k_deadzoneMeters
        &&
        Math.abs(m_rotation.getDegrees() - m_currentRot) < Constants.DriveConstants.k_rotateDeadzone
      );
  }
}
