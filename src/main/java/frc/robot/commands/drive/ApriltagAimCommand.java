// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class ApriltagAimCommand extends Command {
  private DriveSubsystem m_subsystem;
  private PhotonCamera m_camera;
  private PhotonPipelineResult m_result;
  private double m_dist = 0;
  private static final double k_ldist = 0;
  private static final double k_rdist = 0;
  /** Creates a new ApriltagAimCommand. */
  public ApriltagAimCommand(PhotonCamera camera, DriveSubsystem subsystem, boolean left) {
    m_camera = camera;
    m_subsystem =  subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    if(left){
      m_dist = k_ldist;
    } else {
      m_dist = k_rdist;
    }
    addRequirements(m_subsystem);
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
    double horizDist = 0;
    if (m_result.hasTargets()) {
      PhotonTrackedTarget target = m_result.getBestTarget();
      List<TargetCorner> corners = target.getDetectedCorners();
      for (int i = 0; i < 4; i++) {
        x += corners.get(i).x;
      }
      x /= 4;
      //find horizontal distance, where -0.5 is all the way to the left and 0.5 is all the way to the right
      horizDist = (x/1280)-m_dist; //todo: find what the x actually means for photonvision, and convert to -0.5-0.5 number
    }
    SmartDashboard.putNumber("tag x", x);
    SmartDashboard.putNumber("tag x dist", horizDist);
    //m_subsystem.drive(0, horizDist/2, 0, false, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
