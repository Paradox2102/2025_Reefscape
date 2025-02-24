package frc.robot;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
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
  private static final AprilTagFieldLayout k_apriltags = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
  private PhotonCamera m_cameraFL;
  private PhotonCamera m_cameraFR;
  private PhotonCamera m_cameraBL;
  private PhotonCamera m_cameraBR;
  private PhotonPoseEstimator m_photonFL;
  private PhotonPoseEstimator m_photonFR;
  private PhotonPoseEstimator m_photonBL;
  private PhotonPoseEstimator m_photonBR;
  public static final Vector<N3> k_visionSD6mm = VecBuilder.fill(0.01, 0.01, 0.5); // Default vision standerd devations
  public static final Vector<N3> k_odometrySD = VecBuilder.fill(0.1, 0.1, 0.1); // Default odometry standard
  private final Field2d m_testField = new Field2d();

  public PositionTrackerPose(double x, double y,
      DriveSubsystem driveSubsystem, PhotonCamera cameraFL, PhotonCamera cameraFR, PhotonCamera cameraBL, PhotonCamera cameraBR) {
    super();
    m_driveSubsystem = driveSubsystem;
    // Let's have arrays of cameras and estimators, please. Much easier to update
    // when we change the number of cameras. Same with their names and transforms.
    // -Gavin
    m_cameraFL = cameraFL;
    // m_cameraFR = cameraFR;
    m_cameraBR = cameraBR;
    m_cameraBL = cameraBL;
    m_photonFL = new PhotonPoseEstimator(k_apriltags, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, new Transform3d(
        new Translation3d(0.267, 0.267, 0.223), new Rotation3d(0, Math.toRadians(0), Math.toRadians(0))));
    m_photonFL.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    // m_photonFR = new PhotonPoseEstimator(k_apriltags, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, new Transform3d(
    //   new Translation3d(-0.267, 0.267, 0.223), new Rotation3d(Math.toRadians(270), Math.toRadians(0), Math.toRadians(0))));
    // m_photonFR.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    m_photonBL = new PhotonPoseEstimator(k_apriltags, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, new Transform3d(
        new Translation3d(-0.267, 0.267, 0.223), new Rotation3d(0, Math.toRadians(-20), Math.toRadians(135))));
    m_photonBL.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    m_photonBR = new PhotonPoseEstimator(k_apriltags, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, new Transform3d(
        new Translation3d(-0.267, -0.267, 0.223), new Rotation3d(0, Math.toRadians(-20), Math.toRadians(225))));
    m_photonBR.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

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

  // FIXME: delete next 2 functions
  // public List<PhotonPipelineResult> getFLCameraUnreadResults() {
  // return m_cameraFL.getAllUnreadResults();
  // }

  /* TESTING CLASSES! DO NOT USE */
  // public Optional<EstimatedRobotPose> getPhoton1Pose() {
  // Optional<EstimatedRobotPose> visionEst = Optional.empty();
  // for (var change : m_camera1.getAllUnreadResults()) {
  // visionEst = m_photonFL.update(change);
  // }
  // return visionEst;
  // }

  // private double m_lastTimestamp = 0;

  // private Optional<EstimatedRobotPose> photon2Update(PhotonPipelineResult
  // result){
  // if(Math.abs(m_lastTimestamp - result.getTimestampSeconds()) < 1e-6) {
  // System.out.println("timestamp is the same");
  // }
  // return m_photon2.update(result);
  // }

  // public Optional<EstimatedRobotPose> getPhoton2Pose() {
  // Optional<EstimatedRobotPose> visionEst = Optional.empty();
  // List<PhotonPipelineResult> results = m_camera2.getAllUnreadResults();
  // if(results.size() > 0){
  // //System.out.println(results.get(0).hasTargets());
  // //System.out.println(results.get(0).getTimestampSeconds());
  // if(results.get(0).hasTargets()){
  // //System.out.println(m_photon2.update(results.get(0)).isPresent());
  // visionEst = photon2Update(results.get(0));
  // }
  // }
  // // for (var change : results) {
  // // visionEst = m_photon2.update(change);
  // // }
  // return visionEst;
  // }

  public List<Optional<EstimatedRobotPose>> getEstimatedGlobalPose() {
    Optional<EstimatedRobotPose> visionEstFL = Optional.empty();
    // Optional<EstimatedRobotPose> visionEstFR = Optional.empty();
    Optional<EstimatedRobotPose> visionEstBL = Optional.empty();
    Optional<EstimatedRobotPose> visionEstBR = Optional.empty();
    List<Optional<EstimatedRobotPose>> visionEsts = new ArrayList<>(0);
    for (var change : m_cameraFL.getAllUnreadResults()) {
      visionEstFL = m_photonFL.update(change);
    }
    visionEsts.add(visionEstFL);
    // for (var change : m_cameraFR.getAllUnreadResults()) {
    //   visionEstFR = m_photonFR.update(change);
    // }
    // visionEsts.add(visionEstFR);
    for (var change : m_cameraBL.getAllUnreadResults()) {
      visionEstBL = m_photonBL.update(change);
    }
    visionEsts.add(visionEstBL);
    for (var change : m_cameraBR.getAllUnreadResults()) {
      visionEstBR = m_photonBR.update(change);
    }
    visionEsts.add(visionEstBR);
    return visionEsts;
  }

  public void displayRobotPosWithCamera() {
    List<Optional<EstimatedRobotPose>> results = getEstimatedGlobalPose();
    // System.out.println(results.isPresent());
    for (var est : results) {
      if (est.isPresent()) {
        SmartDashboard.putNumber("photon2 x", est.get().estimatedPose.toPose2d().getX());
        SmartDashboard.putNumber("photon2 y", est.get().estimatedPose.toPose2d().getY());
        SmartDashboard.putNumber("photon2 angle", est.get().estimatedPose.toPose2d().getRotation().getDegrees());
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
    m_photonFL.setReferencePose(getPose2d());
    m_photonBR.setReferencePose(getPose2d());

    Rotation2d gyroRotation = m_driveSubsystem.getGyroRotation2d();
    SwerveModulePosition[] modules = m_driveSubsystem.getModulePosition();
    m_poseEstimator.update(gyroRotation, modules);

    var visionEst = getEstimatedGlobalPose();
    for (var result : visionEst) {
      result.ifPresent(
          est -> {
            m_poseEstimator.addVisionMeasurement(est.estimatedPose.toPose2d(), est.timestampSeconds);
          });
      //m_testField.setRobotPose(result.get().estimatedPose.toPose2d());
    }

    // try {
    // if(camera1Result.get(0).hasTargets() &&
    // camera1Result.get(0).getTimestampSeconds() != m_timestamp1){
    // m_poseEstimator.addVisionMeasurement(
    // m_photonFL.update(camera1Result.get(0)).get().estimatedPose.toPose2d(),
    // camera1Result.get(0).getTimestampSeconds());
    // }
    // if(camera2Result.get(0).hasTargets() &&
    // camera2Result.get(0).getTimestampSeconds() != m_timestamp2){
    // m_poseEstimator.addVisionMeasurement(
    // m_photon2.update(camera2Result.get(0)).get().estimatedPose.toPose2d(),
    // camera2Result.get(0).getTimestampSeconds());
    // }
    // } catch (Exception e) {}
  }
}