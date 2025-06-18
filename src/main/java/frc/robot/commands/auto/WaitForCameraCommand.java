// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class WaitForCameraCommand extends Command {
  private static final double k_tolerance = 0.5;
  private double m_distance = 1;
  private DriveSubsystem m_subsystem;
  private Timer m_timer = new Timer();
  /** Creates a new WaitForCameraCommand. */
  public WaitForCameraCommand(DriveSubsystem subsystem) {
    m_subsystem = subsystem;
    addRequirements(m_subsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_subsystem.drive(0, 0, 0, true, false);
    m_timer.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double x = Math.abs(m_subsystem.getCameraPose().getX() - m_subsystem.getTracker().getPose2d().getX());
    double y = Math.abs(m_subsystem.getCameraPose().getY() - m_subsystem.getTracker().getPose2d().getY());
    m_distance = Math.sqrt((x*x)+(y*y));
    SmartDashboard.putNumber("Camera Vs Odometry Distance", m_distance);
    if (m_distance < k_tolerance){
      m_timer.start();
    } else {
      m_timer.stop();
      m_timer.reset();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_timer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_timer.get() > 0.5;
  }
}
