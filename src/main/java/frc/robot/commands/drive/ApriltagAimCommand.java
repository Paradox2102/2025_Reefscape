// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import java.util.List;
import java.util.function.DoubleSupplier;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.DriveSubsystem;

public class ApriltagAimCommand extends Command {
  private DriveSubsystem m_subsystem;
  private PhotonCamera m_camera;
  private PhotonPipelineResult m_result;
  private double m_pos = 0;
  private double m_horizDist = 0;
  private static final double k_ldist = 0.132;
  private static final double k_rdist = 0.487;
  DoubleSupplier m_joystickLeftY = () -> 0;

  /** Creates a new ApriltagAimCommand. */
  public ApriltagAimCommand(PhotonCamera camera, DriveSubsystem subsystem, boolean left) {
    m_camera = camera;
    m_subsystem =  subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    if(left){
      m_pos = k_ldist;
    } else {
      m_pos = k_rdist;
    }
    addRequirements(m_subsystem);
  }

  public ApriltagAimCommand(PhotonCamera camera, DriveSubsystem subsystem, boolean left, DoubleSupplier joystickLeftY) {
    this(camera, subsystem, left);
    m_joystickLeftY = joystickLeftY;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_result = m_camera.getLatestResult();
    double x = 0;
    m_horizDist = 0;
    if (m_result.hasTargets()) {
      PhotonTrackedTarget target = m_result.getBestTarget();
      List<TargetCorner> corners = target.getDetectedCorners();
      for (int i = 0; i < 4; i++) {
        x += corners.get(i).x;
      }
      x /= 4;
      //find horizontal distance, where -0.5 is all the way to the left and 0.5 is all the way to the right
      m_horizDist = (x/1280)-m_pos;
    }
    SmartDashboard.putNumber("tag x", x);
    SmartDashboard.putNumber("tag x dist", m_horizDist);
    double move = -Math.sqrt(Math.pow(m_joystickLeftY.getAsDouble(), 2));
    m_subsystem.drive(move, m_horizDist*2, 0, false, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    ;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_horizDist < 0.05;
  }
}
