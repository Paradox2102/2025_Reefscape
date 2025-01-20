// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;

// import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
// import com.pathplanner.lib.util.PIDConstants;
// import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.ParadoxField;
import frc.robot.PositionTrackerPose;
import frc.utils.SwerveUtils;
import java.util.function.BooleanSupplier;

public class DriveSubsystem extends SubsystemBase {

  private FieldPosition m_reefPosition = FieldPosition.ONE;
  private FieldPosition m_source = FieldPosition.SOURCE_RED;

  private enum FieldPosition {
    // blue alliance
    ONE(new Pose2d(new Translation2d(5.81, 3.86), Rotation2d.fromDegrees(180))),
    TWO(new Pose2d(new Translation2d(5.27, 2.98), Rotation2d.fromDegrees(120))),
    THREE(new Pose2d(new Translation2d(5.01, 2.82), Rotation2d.fromDegrees(120))),
    FOUR(new Pose2d(new Translation2d(3.96, 2.82), new Rotation2d(60))),
    FIVE(new Pose2d(new Translation2d(3.69, 2.98), Rotation2d.fromDegrees(60))),
    SIX(new Pose2d(new Translation2d(3.17, 3.86), Rotation2d.fromDegrees(0))),
    SEVEN(new Pose2d(new Translation2d(3.17, 4.17), Rotation2d.fromDegrees(0))),
    EIGHT(new Pose2d(new Translation2d(3.69, 5.09), Rotation2d.fromDegrees(-60))),
    NINE(new Pose2d(new Translation2d(3.96, 5.23), Rotation2d.fromDegrees(-60))),
    TEN(new Pose2d(new Translation2d(5.01, 5.23), Rotation2d.fromDegrees(-120))),
    ELEVEN(new Pose2d(new Translation2d(5.27, 5.09), Rotation2d.fromDegrees(-120))),
    TWELVE(new Pose2d(new Translation2d(5.81, 4.17), Rotation2d.fromDegrees(180))),
    SOURCE_RED(new Pose2d(new Translation2d(1.7, 7.38), Rotation2d.fromDegrees(127.5))),
    SOURCE_BLUE(new Pose2d(new Translation2d(1.7, .65), new Rotation2d(-127.5)));

    private Pose2d m_targetPos;

    FieldPosition(Pose2d pose) {
      m_targetPos = pose;
    }

    public Pose2d targetPose() {
      return m_targetPos;
    }
  }

  private final Field2d m_field = new Field2d();
  // Create MaxSwerveModules
  private final MaxSwerveModule m_frontLeft = new MaxSwerveModule(Constants.DriveConstants.k_FLDriveMotor,
      Constants.DriveConstants.k_FLTurningMotor,
      Constants.DriveConstants.k_FLOffset);

  private final MaxSwerveModule m_frontRight = new MaxSwerveModule(Constants.DriveConstants.k_FRDriveMotor,
      Constants.DriveConstants.k_FRTurningMotor,
      Constants.DriveConstants.k_FROffset);

  private final MaxSwerveModule m_backLeft = new MaxSwerveModule(Constants.DriveConstants.k_BLDriveMotor,
      Constants.DriveConstants.k_BLTurningMotor,
      Constants.DriveConstants.k_BLOffset);

  private final MaxSwerveModule m_backRight = new MaxSwerveModule(Constants.DriveConstants.k_BRDriveMotor,
      Constants.DriveConstants.k_BRTurningMotor,
      Constants.DriveConstants.k_BROffset);

  // The gyro sensor
  private final Pigeon2 m_gyro = new Pigeon2(0);

  private PIDController m_orientPID = new PIDController(
      Constants.DriveConstants.k_rotateP, Constants.DriveConstants.k_rotateI,
      Constants.DriveConstants.k_rotateD);

  // Slew rate filter variables for controlling lateral acceleration
  private double m_currentRotation = 0.0;
  private double m_currentTranslationDir = 0.0;
  private double m_currentTranslationMag = 0.0;

  private SlewRateLimiter m_magLimiter = new SlewRateLimiter(Constants.DriveConstants.k_magnitudeSlewRate);
  private SlewRateLimiter m_rotLimiter = new SlewRateLimiter(Constants.DriveConstants.k_rotationalSlewRate);
  private double m_prevTime = 0;

  private Pose2d m_futurePos = new Pose2d();

  private final SwerveDriveKinematics m_swerve = new SwerveDriveKinematics(
      new Translation2d(.298, .298), new Translation2d(.298, -.298),
      new Translation2d(-.298, .298), new Translation2d(-.298, -.298));

  PositionTrackerPose m_tracker;

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    m_gyro.reset();
    SmartDashboard.putData("Field", m_field);

    m_orientPID.enableContinuousInput(-180, 180);
    m_orientPID.setIZone(Constants.DriveConstants.k_rotateIZone);

    // AutoBuilder.configure(
    //     this::getPose, // Robot pose supplier
    //     this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
    //     this::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
    //     (speeds, feedforwards) -> driveWithChassisSpeedRobotRelative(speeds), // Method that will drive the robot given ROBOT RELATIVE
    //                                                           // ChassisSpeeds. Also optionally outputs individual
    //                                                           // module feedforwards
    //     new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic
    //                                     // drive trains
    //         new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
    //         new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
    //     ),
    //     Constants.DriveConstants.k_pathConfig, // The robot configuration
    //     () -> {
    //       // Boolean supplier that controls when the path will be mirrored for the red
    //       // alliance
    //       // This will flip the path being followed to the red side of the field.
    //       // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

    //       var alliance = DriverStation.getAlliance();
    //       if (alliance.isPresent()) {
    //         return alliance.get() == DriverStation.Alliance.Red;
    //       }
    //       return false;
    //     },
    //     this // Reference to this subsystem to set requirements
    // );

    // PathPlannerLogging.setLogActivePathCallback((poses) -> m_field.getObject("path").setPoses(poses));
  }

  public void setReefPosition(FieldPosition position) {
    m_reefPosition = position;
  }

  public void setSource(FieldPosition source) {
    m_source = source;
  }

  public Pose2d getReefPosition() {
    return m_reefPosition.targetPose();
  }

  public Pose2d getSourcePosition() {
    return m_source.targetPose();
  }

  public SwerveDriveKinematics getSwerve() {
    return m_swerve;
  }

  public void setTracker(PositionTrackerPose tracker) {
    m_tracker = tracker;
  }

  public SwerveModulePosition[] getModulePosition() {
    return new SwerveModulePosition[] {
        m_frontLeft.getPosition(), m_frontRight.getPosition(),
        m_backLeft.getPosition(), m_backRight.getPosition() };
  }

  // Any interface that takes or receives an angle should be using Rotation2d.
  // This protects us from confusing degrees and radians. - Gavin
  public double orientPID(double setpoint) {
    double heading = getHeadingInDegrees();
    double rot = m_orientPID.calculate(heading, setpoint);

    // rot += (Constants.DriveConstants.k_rotateF * Math.signum(rot));
    // return Math.abs(heading) < Constants.DriveConstants.k_rotateDeadzone ? 0
    // : rot;
    rot = Math.abs(rot) > Constants.DriveConstants.k_maxRotInput
        ? Constants.DriveConstants.k_maxRotInput * Math.signum(rot)
        : rot;
    return rot;
  }

  public Pose2d getEstimatedFuturePos() {
    return m_futurePos;
  }

  public static boolean m_setGyroZero = true;
  public static double m_gyroZero = 0;

  @Override
  public void periodic() {

    // SmartDashboard.putNumber("Rotate Error",
    // getRotationDistanceFromTargetError());
    // // Update the odometry in the periodic block
    SmartDashboard.putNumber("Turn FR Radians",
        m_frontRight.getPosition().angle.getRadians()); /// Math.PI);
    SmartDashboard.putNumber(
        "Turn FL Radians",
        m_frontLeft.getPosition().angle.getRadians()); // - (Math.PI / 2)) / Math.PI);
    SmartDashboard.putNumber(
        "Turn BR Radians",
        m_backRight.getPosition().angle.getRadians()); // + (Math.PI / 2)) / Math.PI);
    SmartDashboard.putNumber(
        "Turn BL Radians", m_backLeft.getPosition().angle.getRadians()); // + (Math.PI)) / Math.PI);
    // SmartDashboard.putNumber("Pose Est X",
    // (m_tracker.getPose2dFRC().getTranslation().getX()));
    // SmartDashboard.putNumber("Pose Est Y",
    // (m_tracker.getPose2dFRC().getTranslation().getY()));
    // SmartDashboard.putNumber("Pose Est Rot",
    // (m_tracker.getPose2dFRC().getRotation().getDegrees()));
    // SmartDashboard.putNumber("Pigeon2", m_gyro.getYaw().getValueAsDouble());
    // SmartDashboard.putNumber("Gyro Rotation2D",
    // getGyroRotation2d().getDegrees());
    // Dashboard.putNumber("Tracker
    // Rotation2D", m_tracker.getPose2d().getRotation().getDegrees());
    SmartDashboard.putNumber("Gyro Angle", ParadoxField.normalizeAngle(-m_gyro.getYaw().getValueAsDouble()));
    // spin robot very fast for many rotations eventually back to the original
    // angle, look at the gyro angle and see the offset

    // SmartDashboard.putNumber("Speaker X",
    // getSpeakerLocationMeters().m_xMeters); SmartDashboard.putNumber("Speaker
    // Y", getSpeakerLocationMeters().m_yMeters);
    // SmartDashboard.putBoolean("Face Speaker",
    // Constants.States.m_faceSpeaker); SmartDashboard.putBoolean("Shoot
    // Front/Back", Constants.States.m_shootIntakeSide);
    // SmartDashboard.putBoolean("Aim On", Constants.States.m_autoRotateAim);
    // SmartDashboard.putNumber("FL Encoder Diff",
    // m_frontLeft.getMagEncoderPosRadians() - m_frontLeft.getMotorPosRadians());
    // SmartDashboard.putNumber("FR Encoder Diff",
    // m_frontRight.getMagEncoderPosRadians() - m_frontRight.getMotorPosRadians());
    // SmartDashboard.putNumber("BL Encoder Diff",
    // m_backLeft.getMagEncoderPosRadians() - m_backLeft.getMotorPosRadians());
    // SmartDashboard.putNumber("BR Encoder Diff",
    // m_backRight.getMagEncoderPosRadians() - m_backRight.getMotorPosRadians());

    // For efficiency. we could pass in the module states here, to avoid calling it
    // twice. Maybe also currentPos. - Gavin

    // Estimate future position of robot
    // *****************************************
    Pose2d currentPos = m_tracker.getPose2d();

    double yaw = ParadoxField.normalizeAngle(m_gyro.getYaw().getValueAsDouble());
    if (m_setGyroZero) {
      m_gyroZero = ParadoxField.normalizeAngle(currentPos.getRotation().getDegrees() - yaw);
      m_setGyroZero = false;
    }
    SmartDashboard.putNumber("Gyro offset",
        ParadoxField.normalizeAngle(currentPos.getRotation().getDegrees() - yaw - m_gyroZero));
    SmartDashboard.putNumber("Gyro Zero", m_gyroZero);
    SmartDashboard.putNumber("Gyro", yaw);
    SmartDashboard.putNumber("Gyro Diff", ParadoxField.normalizeAngle(currentPos.getRotation().getDegrees() - yaw));
    SmartDashboard.putNumber("Gyro Est Yaw", ParadoxField.normalizeAngle(currentPos.getRotation().getDegrees()));

    double currentY = currentPos.getY();
    double currentX = currentPos.getX();
    Rotation2d currentAngle = currentPos.getRotation();

    ChassisSpeeds chassisSpeed = ChassisSpeeds.fromRobotRelativeSpeeds(
        m_swerve.toChassisSpeeds(getModuleStates()), currentAngle);
    double yVelocity = chassisSpeed.vyMetersPerSecond;
    double xVelocity = chassisSpeed.vxMetersPerSecond;
    double angularVelocity = chassisSpeed.omegaRadiansPerSecond;

    double futureY = currentY + yVelocity * Constants.DriveConstants.k_lookAheadTimeSeconds;
    double futureX = currentX + xVelocity * Constants.DriveConstants.k_lookAheadTimeSeconds;
    double futureAngle = ParadoxField.normalizeAngle(
        currentAngle.getDegrees() +
            Math.toDegrees(angularVelocity) *
                Constants.DriveConstants.k_lookAheadTimeSeconds);

    m_futurePos = new Pose2d(futureX, futureY, Rotation2d.fromDegrees(futureAngle));
    // *********************************************************

    // m_field.setRobotPose(m_tracker.getPose2dFRC().getTranslation().getX(),
    // m_tracker.getPose2dFRC().getTranslation().getY(),
    // m_tracker.getPose2dFRC().getRotation());
    m_tracker.update();
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_tracker.getPose2d();
  }

  public Field2d getField() {
    return m_field;
  }

  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = {
        m_frontLeft.getState(), m_frontRight.getState(), m_backLeft.getState(),
        m_backRight.getState() };
    return states;
  }

  public ChassisSpeeds getChassisSpeeds() {
    return m_swerve.toChassisSpeeds(getModuleStates());
  }

  public void driveWithChassisSpeedRobotRelative(ChassisSpeeds chassisSpeeds) {
    var swerveModuleStates = m_swerve.toSwerveModuleStates(chassisSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, Constants.DriveConstants.k_maxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_backLeft.setDesiredState(swerveModuleStates[2]);
    m_backRight.setDesiredState(swerveModuleStates[3]);
    // Pose2d pose = m_tracker.getPose2dFRC();
    // Logger.log("Robot Pose",0,String.format(",%f,%f,%f", pose.getX(),
    // pose.getY(), pose.getRotation().getDegrees()));
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_tracker.setXYAngle(pose.getX(), pose.getY(), pose.getRotation().getDegrees());
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to
   *                      the
   *                      field.
   * @param rateLimit     Whether to enable rate limiting for smoother control.
   */
  public void drive(double xSpeed, double ySpeed, double rot,
      boolean fieldRelative, boolean rateLimit) {

    double xSpeedCommanded;
    double ySpeedCommanded;

    if (rateLimit) {
      // Convert XY to polar for rate limiting
      double inputTranslationDir = Math.atan2(ySpeed, xSpeed);
      double inputTranslationMag = Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2));

      // Calculate the direction slew rate based on an estimate of the lateral
      // acceleration
      double directionSlewRate;
      if (m_currentTranslationMag != 0.0) {
        directionSlewRate = Math.abs(Constants.DriveConstants.k_directionSlewRate /
            m_currentTranslationMag);
      } else {
        directionSlewRate = 500.0; // some high number that means the slew rate
                                   // is effectively instantaneous
      }

      double currentTime = WPIUtilJNI.now() * 1e-6;
      double elapsedTime = currentTime - m_prevTime;
      double angleDif = SwerveUtils.AngleDifference(inputTranslationDir,
          m_currentTranslationDir);
      if (angleDif < 0.45 * Math.PI) {
        m_currentTranslationDir = SwerveUtils.StepTowardsCircular(
            m_currentTranslationDir, inputTranslationDir,
            directionSlewRate * elapsedTime);
        m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
      } else if (angleDif > 0.85 * Math.PI) {
        if (m_currentTranslationMag > 1e-4) { // some small number to avoid floating-point errors with
                                              // equality checking
          // keep currentTranslationDir unchanged
          m_currentTranslationMag = m_magLimiter.calculate(0.0);
        } else {
          m_currentTranslationDir = SwerveUtils.WrapAngle(m_currentTranslationDir + Math.PI);
          m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
        }
      } else {
        m_currentTranslationDir = SwerveUtils.StepTowardsCircular(
            m_currentTranslationDir, inputTranslationDir,
            directionSlewRate * elapsedTime);
        m_currentTranslationMag = m_magLimiter.calculate(0.0);
      }
      m_prevTime = currentTime;

      xSpeedCommanded = m_currentTranslationMag * Math.cos(m_currentTranslationDir);
      ySpeedCommanded = m_currentTranslationMag * Math.sin(m_currentTranslationDir);
      m_currentRotation = m_rotLimiter.calculate(rot);

    } else {
      xSpeedCommanded = xSpeed;
      ySpeedCommanded = ySpeed;
      m_currentRotation = rot;
    }

    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = xSpeedCommanded * Constants.DriveConstants.k_maxSpeedMetersPerSecond;
    double ySpeedDelivered = ySpeedCommanded * Constants.DriveConstants.k_maxSpeedMetersPerSecond;
    double rotDelivered = m_currentRotation
        * Constants.DriveConstants.k_maxAngularSpeed;

    BooleanSupplier allianceRed = () -> {
      var alliance = DriverStation.getAlliance();
      if (alliance.isPresent()) {
        return alliance.get() == DriverStation.Alliance.Red;
      }
      return false;
    };

    var swerveModuleStates = m_swerve.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(
                xSpeedDelivered, ySpeedDelivered, rotDelivered,
                Rotation2d.fromDegrees(getPose().getRotation().getDegrees() +
                    (allianceRed.getAsBoolean() ? 180 : 0)))
            : new ChassisSpeeds(-xSpeedDelivered, -ySpeedDelivered,
                rotDelivered),
        new Translation2d(0, 0));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, Constants.DriveConstants.k_maxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_backLeft.setDesiredState(swerveModuleStates[2]);
    m_backRight.setDesiredState(swerveModuleStates[3]);
  }

  public Rotation2d getGyroRotation2d() {
    return m_gyro.getRotation2d();
  }

  public double getRotationRateDegreesPerSecond() {
    // getRate returns degrees per second clockwise, so we negate it to get
    // degrees per second counterclockwise.
    return m_gyro.getAngularVelocityZWorld().getValueAsDouble();
  }

  public Pigeon2 getGyro() {
    return m_gyro;
  }

  public PositionTrackerPose getTracker() {
    return m_tracker;
  }

  public void setBrakeMode(boolean brake) {
    m_frontLeft.setBrakeMode(brake);
    m_frontRight.setBrakeMode(brake);
    m_backLeft.setBrakeMode(brake);
    m_backRight.setBrakeMode(brake);
  }

  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  public void setX() {
    m_frontLeft.setDesiredState(
        new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(
        new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_backLeft.setDesiredState(
        new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_backRight.setDesiredState(
        new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  public void spinAllModules() {
    m_frontLeft.spin();
    m_frontRight.spin();
    m_backLeft.spin();
    m_backRight.spin();
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, Constants.DriveConstants.k_maxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_backLeft.setDesiredState(desiredStates[2]);
    m_backRight.setDesiredState(desiredStates[3]);
  }

  public void setModuleStatesWithSpeed(SwerveModuleState[] desiredStates,
      double speed) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, Constants.DriveConstants.k_maxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_backLeft.setDesiredState(desiredStates[2]);
    m_backRight.setDesiredState(desiredStates[3]);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_backLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_backRight.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  public void setHeading(double angle) {
    m_tracker.setXYAngle(m_tracker.getPose2d().getX(),
        m_tracker.getPose2d().getY(), angle);
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeadingInDegrees() {
    double angle = m_tracker.getPose2d().getRotation().getDegrees();
    return ParadoxField.normalizeAngle(angle);
  }
}
