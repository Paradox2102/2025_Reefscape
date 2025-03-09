// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import java.util.List;
import java.util.function.BooleanSupplier;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class ApriltagAimCommand extends Command {
  private DriveSubsystem m_subsystem;
  private PhotonCamera m_camera;
  private PhotonPipelineResult m_result;
  private double m_pos = 0;
  private double m_horizDist = 0;
  private static final double k_ldist = 0.677;
  private static final double k_rdist = 0.15;
  BooleanSupplier m_left;

  /** Creates a new ApriltagAimCommand. */
  public ApriltagAimCommand(PhotonCamera camera, DriveSubsystem drive) {
    m_camera = camera;
    m_subsystem = drive;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(m_subsystem.getReefPosition().isLeft()){
      m_pos = k_ldist;
    } else {
      m_pos = k_rdist;
    }
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
    m_subsystem.drive(0, m_horizDist/2, 0, false, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_horizDist < 0.05;
  }
}
