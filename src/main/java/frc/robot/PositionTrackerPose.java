package frc.robot;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.DriveSubsystem;

// import frc.lib.CSVWriter;
// import frc.lib.CSVWriter.Field;

public class PositionTrackerPose {
  private SwerveDrivePoseEstimator m_poseEstimator;
  private DriveSubsystem m_driveSubsystem;
  public static final AprilTagFieldLayout k_apriltags = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
  private PhotonCamera m_cameraFL;
  private PhotonCamera m_cameraFR;
  private PhotonCamera m_cameraBR;
  private PhotonCamera[] m_cameras = {m_cameraFL, m_cameraFR, m_cameraBR};
  private PhotonPoseEstimator m_photonFL;
  private PhotonPoseEstimator m_photonFR;
  private PhotonPoseEstimator m_photonBR;
  private ArrayList<PhotonPoseEstimator> m_estimators = new ArrayList<>();
  public static final Vector<N3> k_visionSD6mm = VecBuilder.fill(0.9, 0.9, 0.9); // Default vision standerd devations
  public static final Vector<N3> k_odometrySD = VecBuilder.fill(0.1, 0.1, 0.1); // Default odometry standard
  private final Field2d m_testField = new Field2d();

  public PositionTrackerPose(double x, double y,
      DriveSubsystem driveSubsystem, PhotonCamera[] cameras) {
    super();
    m_driveSubsystem = driveSubsystem;
    // Let's have arrays of cameras and estimators, please. Much easier to update
    // when we change the number of cameras. Same with their names and transforms.
    // -Gavin
    for(int i = 0; i < cameras.length; i++){
      m_cameras[i] = cameras[i];
    }
    m_photonFL = new PhotonPoseEstimator(k_apriltags, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, new Transform3d(
        new Translation3d(0.267, 0.267, 0.223), new Rotation3d(0, Math.toRadians(0), Math.toRadians(0))));
    m_estimators.add(m_photonFL);
    m_photonFR = new PhotonPoseEstimator(k_apriltags, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, new Transform3d(
      new Translation3d(0.267, -0.267, 0.223), new Rotation3d(Math.toRadians(0), Math.toRadians(0), Math.toRadians(0))));
    m_estimators.add(m_photonFR);
    m_photonBR = new PhotonPoseEstimator(k_apriltags, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, new Transform3d(
        new Translation3d(-0.267, -0.267, 0.223), new Rotation3d(0, Math.toRadians(-20), Math.toRadians(225))));
    m_estimators.add(m_photonBR);
    for(int i = 0; i < m_estimators.size(); i++){
      m_estimators.get(i).setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    }

    m_poseEstimator = new SwerveDrivePoseEstimator(
        m_driveSubsystem.getSwerve(), m_driveSubsystem.getGyroRotation2d(),
        m_driveSubsystem.getModulePosition(),
        new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
        k_odometrySD, k_visionSD6mm);

    // m_posServer = new PositionServer();
    // m_posServer.start();
    SmartDashboard.putData("photon pose", m_testField);
  }

  public Pose2d getPose2d() {
    return m_poseEstimator.getEstimatedPosition();
  }

  public void setPose(Pose2d pose) {
    m_poseEstimator.resetPosition(
        m_driveSubsystem.getGyro().getRotation2d(),
        m_driveSubsystem.getModulePosition(),
        new Pose2d(pose.getX(), pose.getY(), pose.getRotation()));
    System.out.println("pose2d " + getPose2d());
  }

  public List<Optional<EstimatedRobotPose>> getEstimatedGlobalPose() {
    List<Optional<EstimatedRobotPose>> visionEsts = new ArrayList<>(0);
    for(int i = 0; i < m_cameras.length; i++){
      Optional<EstimatedRobotPose> est = Optional.empty();
      for (var change : m_cameras[i].getAllUnreadResults()) {
        est = m_estimators.get(i).update(change);
      }
      visionEsts.add(est);
    }
    return visionEsts;
  }

  public void displayRobotPosWithCamera() {
    List<Optional<EstimatedRobotPose>> results = getEstimatedGlobalPose();
    for (var est : results) {
      if (est.isPresent()) {
        SmartDashboard.putNumber("photon x", est.get().estimatedPose.toPose2d().getX());
        SmartDashboard.putNumber("photon y", est.get().estimatedPose.toPose2d().getY());
        SmartDashboard.putNumber("photon angle", est.get().estimatedPose.toPose2d().getRotation().getDegrees());
        m_testField.setRobotPose(est.get().estimatedPose.toPose2d());
      }
    }
  }

  public static class PositionContainer {
    public double x, y;

    public PositionContainer(double x, double y) {
      this.x = x;
      this.y = y;
    }
  }

  // private boolean m_front = true;

  public void update() {
    for(int i = 0; i < m_estimators.size(); i++){
      m_estimators.get(i).setReferencePose(getPose2d());
    }

    Rotation2d gyroRotation = m_driveSubsystem.getGyroRotation2d();
    SwerveModulePosition[] modules = m_driveSubsystem.getModulePosition();
    m_poseEstimator.update(gyroRotation, modules);

    var visionEst = getEstimatedGlobalPose();
    for (var result : visionEst) {
      result.ifPresent(
          est -> {
            m_poseEstimator.addVisionMeasurement(est.estimatedPose.toPose2d(), est.timestampSeconds);
          }
        );
    }
  }
}