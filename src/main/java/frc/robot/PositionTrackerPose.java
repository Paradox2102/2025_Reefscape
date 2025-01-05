package frc.robot;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.DriveSubsystem;

// import frc.lib.CSVWriter;
// import frc.lib.CSVWriter.Field;

public class PositionTrackerPose {
  private SwerveDrivePoseEstimator m_poseEstimator;
  private DriveSubsystem m_driveSubsystem;
  public static final Vector<N3> k_visionSD6mm = VecBuilder.fill(0.01, 0.01, 0.5); // Default vision standerd devations
  public static final Vector<N3> k_odometrySD = VecBuilder.fill(0.1, 0.1, 0.1); // Default odometry standard

  public PositionTrackerPose(double x, double y,
                             DriveSubsystem driveSubsystem) {
    super();
    m_driveSubsystem = driveSubsystem;

    // For the extended constructor, the default values are:
    // VecBuilder.fill(0.02, 0.02, 0.01) - SD of internal state
    // VecBuilder.fill(0.1, 0.1, 0.1)) - SD of vision pose measurment
    // Based on the experiments performed on 2023-03-17 Friday, the vison pose
    // estimates (in metres, metres, radians) should be between:
    // env vs cam: 0.038299922 0.03770631627 0.01169028658
    // est vs cam: 0.01801886372 0.01776299463 0.007439846418
    // As these are significantly lower than the defaults, using them would make
    // us trust the camera more. Slicing the data by distance, I see a stong
    // linear relationship between distance and SD for all of X, Y, and angle,
    // but at the maximum distance the errors are still less than the default.
    // -Gavin

    m_poseEstimator = new SwerveDrivePoseEstimator(
        m_driveSubsystem.getSwerve(), m_driveSubsystem.getGyroRotation2d(),
        m_driveSubsystem.getModulePosition(),
        new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
        k_odometrySD, k_visionSD6mm);

    // m_posServer = new PositionServer();
    // m_posServer.start();
  }

  public Pose2d getPose2d() { return m_poseEstimator.getEstimatedPosition(); }

  // BUG: The third parameter to resetPosition should use the estimated position
  // angle, not the gyro angle.  This function should take a Pose2d (estimated
  // position) and a Rotation2d (gyro angle) and use them to reset the position.
  // Alternatively, it could take just a Pose2d because we already have access
  // to the drive subsystem. These wrapper types are supposed to make this kind
  // of bug less likely, but constantly converting into and out of them defeats
  // the purpose.  -Gavin
  public void setXYAngle(double x, double y, double angleInDegrees) {
    m_poseEstimator.resetPosition(
        Rotation2d.fromDegrees(-m_driveSubsystem.getGyro().getYaw().getValueAsDouble()),
        m_driveSubsystem.getModulePosition(),
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

  private boolean m_front = true;

  public void update() {
    //photon stuff to go here!!!!
  }
}