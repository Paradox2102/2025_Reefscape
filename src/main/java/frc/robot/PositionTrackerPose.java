package frc.robot;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N3;
import frc.robot.subsystems.DriveSubsystem;

// import frc.lib.CSVWriter;
// import frc.lib.CSVWriter.Field;

public class PositionTrackerPose {
  private SwerveDrivePoseEstimator m_poseEstimator;
  private DriveSubsystem m_driveSubsystem;
  private static final AprilTagFieldLayout k_apriltags = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField); //TODO: Change to k2025Reefscape when added
  private PhotonCamera m_camera1; //TODO: add camera names, transforms, and more once we figure out camera layouts
  private PhotonCamera m_camera2;
  private PhotonPoseEstimator m_photon1;
  private double m_timestamp1 = 0;
  private double m_timestamp2 = 0;
  private PhotonPoseEstimator m_photon2;
  public static final Vector<N3> k_visionSD6mm = VecBuilder.fill(0.01, 0.01, 0.5); // Default vision standerd devations
  public static final Vector<N3> k_odometrySD = VecBuilder.fill(0.1, 0.1, 0.1); // Default odometry standard

  public PositionTrackerPose(double x, double y,
                             DriveSubsystem driveSubsystem, PhotonCamera camera1, PhotonCamera camera2) {
    super();
    m_driveSubsystem = driveSubsystem;
    // Let's have arrays of cameras and estimators, please. Much easier to update when we change the number of cameras. Same with their names and transforms. -Gavin
    m_camera1 = camera1;
    m_camera2 = camera2;
    m_photon1 = new PhotonPoseEstimator(k_apriltags, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, new Transform3d());
    m_photon1.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    m_photon2 = new PhotonPoseEstimator(k_apriltags, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, new Transform3d());

    m_poseEstimator = new SwerveDrivePoseEstimator(
        m_driveSubsystem.getSwerve(), m_driveSubsystem.getGyroRotation2d(),
        m_driveSubsystem.getModulePosition(),
        new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
        k_odometrySD, k_visionSD6mm);

    // m_posServer = new PositionServer();
    // m_posServer.start();
  }

  public Pose2d getPose2d() { return m_poseEstimator.getEstimatedPosition(); }

  public void setXYAngle(double x, double y, double angleInDegrees) {
    m_poseEstimator.resetPosition(
        m_driveSubsystem.getGyro().getRotation2d(),
        m_driveSubsystem.getModulePosition(),
        // FIXME: It would be much better to have this method take a Pose2d. -Gavin
        new Pose2d(x, y, Rotation2d.fromDegrees(angleInDegrees)));
    System.out.println("pose2d " + getPose2d());
  }

  public static class PositionContainer {
    public double x, y;

    public PositionContainer(double x, double y) {
      this.x = x;
      this.y = y;
    }
  }

  //private boolean m_front = true;

  public void update() {
    m_photon1.setReferencePose(getPose2d());
    m_photon2.setReferencePose(getPose2d());
    List<PhotonPipelineResult> camera1Result = m_camera1.getAllUnreadResults();
    List<PhotonPipelineResult> camera2Result = m_camera2.getAllUnreadResults();

    try {
      if(camera1Result.get(0).hasTargets() && camera1Result.get(0).getTimestampSeconds() != m_timestamp1){
        m_poseEstimator.addVisionMeasurement(
          m_photon1.update(camera1Result.get(0)).get().estimatedPose.toPose2d(), 
          camera1Result.get(0).getTimestampSeconds());
      }
      if(camera2Result.get(0).hasTargets() && camera2Result.get(0).getTimestampSeconds() != m_timestamp2){
        m_poseEstimator.addVisionMeasurement(
          m_photon2.update(camera2Result.get(0)).get().estimatedPose.toPose2d(), 
          camera2Result.get(0).getTimestampSeconds());
      }
    } catch (Exception e) {}
  }
}