// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.PathPlannerLogging;

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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.ParadoxField;
import frc.robot.PositionTrackerPose;
import frc.utils.SwerveUtils;

import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.function.BooleanSupplier;

public class DriveSubsystem extends SubsystemBase {

  private FieldPosition m_reefPosition = FieldPosition.SEVEN;
  private FieldPosition m_source = FieldPosition.SOURCE_RIGHT;
  public static RobotConfig k_pathConfig;{
  try {
    k_pathConfig = RobotConfig.fromGUISettings();
  } catch (Exception e) {}}

  public enum FieldPosition {
    // blue alliance
    ONE(new Pose2d(new Translation2d(5.81, 3.86), Rotation2d.fromDegrees(0)), new Pose2d(new Translation2d(11.75, 4.19), Rotation2d.fromDegrees(180)), true, "1"),
    TWO(new Pose2d(new Translation2d(5.27, 2.98), Rotation2d.fromDegrees(-60)), new Pose2d(new Translation2d(12.26, 5.1), Rotation2d.fromDegrees(120)), false, "2"),
    THREE(new Pose2d(new Translation2d(5.01, 2.82), Rotation2d.fromDegrees(-60)), new Pose2d(new Translation2d(12.57, 5.25), Rotation2d.fromDegrees(120)), true, "3"),
    FOUR(new Pose2d(new Translation2d(3.96, 2.82), Rotation2d.fromDegrees(-120)), new Pose2d(new Translation2d(13.55, 5.25), Rotation2d.fromDegrees(60)), false, "4"),
    FIVE(new Pose2d(new Translation2d(3.69, 2.98), Rotation2d.fromDegrees(-120)), new Pose2d(new Translation2d(13.84, 5.1), Rotation2d.fromDegrees(60)), true, "5"),
    SIX(new Pose2d(new Translation2d(3.17, 3.86), Rotation2d.fromDegrees(180)), new Pose2d(new Translation2d(14.37, 4.19), Rotation2d.fromDegrees(0)), false, "6"),
    SEVEN(new Pose2d(new Translation2d(3.17, 4.17), Rotation2d.fromDegrees(0)), new Pose2d(new Translation2d(14.37, 3.85), Rotation2d.fromDegrees(0)), true, "7"),
    EIGHT(new Pose2d(new Translation2d(3.69, 5.09), Rotation2d.fromDegrees(120)), new Pose2d(new Translation2d(13.84, 2.98), Rotation2d.fromDegrees(-60)), false, "8"),
    NINE(new Pose2d(new Translation2d(3.96, 5.23), Rotation2d.fromDegrees(120)), new Pose2d(new Translation2d(13.55, 2.81), Rotation2d.fromDegrees(-60)), true, "9"),
    TEN(new Pose2d(new Translation2d(5.01, 5.23), Rotation2d.fromDegrees(-120)), new Pose2d(new Translation2d(12.57, 2.81), Rotation2d.fromDegrees(-120)), false, "10"),
    ELEVEN(new Pose2d(new Translation2d(5.27, 5.09), Rotation2d.fromDegrees(60)), new Pose2d(new Translation2d(12.26, 2.98), Rotation2d.fromDegrees(-120)), true, "11"),
    TWELVE(new Pose2d(new Translation2d(5.81, 4.17), Rotation2d.fromDegrees(0)), new Pose2d(new Translation2d(11.75, 3.85), Rotation2d.fromDegrees(180)), false, "12"),
    SOURCE_RIGHT(new Pose2d(new Translation2d(1.7, .65), Rotation2d.fromDegrees(127.5)), new Pose2d(new Translation2d(16.3, 7.1), Rotation2d.fromDegrees(-127.5)), false, "Right"),
    SOURCE_LEFT(new Pose2d(new Translation2d(1.7, 7.38), Rotation2d.fromDegrees(-127.5)), new Pose2d(new Translation2d(16.3, 0.95), Rotation2d.fromDegrees(127.5)),false, "Left");

    private Pose2d m_bluePose;
    private Pose2d m_redPose;
    private String m_name;
    private boolean m_left;

    FieldPosition(Pose2d bluePose, Pose2d redPose, boolean left, String name) {
      m_bluePose = bluePose;
      m_redPose = redPose;
      m_name = name;
      m_left = left;
    }

    public Pose2d targetPose() {
      return Constants.States.m_alliance == Alliance.Blue ? m_bluePose : m_redPose;
    }

    public String getName() {
      return m_name;
    }

    public Boolean leftRight() {
      return m_left;
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
  private boolean m_targetsVisible = false;


  //Wheel Calibration
  private static final double k_wheelRadiusMaxVelocity = 0.25; // Rad/Sec
  private static final double k_wheelRadiusRampRate = 0.05; // Rad/Sec^2
  private final  MaxSwerveModule[] modules = new MaxSwerveModule[]{m_frontLeft, m_frontRight, m_backLeft, m_backRight};


  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    m_gyro.reset();
    SmartDashboard.putData("Field", m_field);

    m_orientPID.enableContinuousInput(-180, 180);
    m_orientPID.setIZone(Constants.DriveConstants.k_rotateIZone);

    AutoBuilder.configure(
        this::getPose, // Robot pose supplier
        this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
        this::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        (speeds, feedforwards) -> driveWithChassisSpeedRobotRelative(speeds), // Method that will drive the robot given ROBOT RELATIVE
                                                              // ChassisSpeeds. Also optionally outputs individual
                                                              // module feedforwards
        new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic
                                        // drive trains
            new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
            new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
        ),
        k_pathConfig, // The robot configuration
        () -> {
          // Boolean supplier that controls when the path will be mirrored for the red
          // alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
        this // Reference to this subsystem to set requirements
    );

    PathPlannerLogging.setLogActivePathCallback((poses) -> m_field.getObject("path").setPoses(poses));
  }

  public Command setReefPosition(FieldPosition position) {
    return Commands.runOnce(() -> {m_reefPosition = position;}, this);
  }

  // FIXME: This should be a command factory. -Gavin
  public Command setSource(FieldPosition source) {
    return Commands.runOnce(() -> {m_source = source;}, this);
  }

  public FieldPosition getReefPosition() {
    return m_reefPosition;
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

  // FIXME: Any interface that takes or receives an angle should be using Rotation2d. This protects us from confusing degrees and radians. - Gavin
  // FIXME: This might be better bundled as a method that takes rotation supplier and returns a double supplier. That way it can have its own PIDController. - Gavin
  public double orientPID(double setpoint) {
    double heading = getHeadingInDegrees();
    double rot = m_orientPID.calculate(heading, setpoint);

    // rot += (Constants.DriveConstants.k_rotateF * Math.signum(rot));
    // return Math.abs(heading) < Constants.DriveConstants.k_rotateDeadzone ? 0
    // : rot;
    // FIXME: Should use MathUtil.clamp here. - Gavin
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
    m_targetsVisible = false;
    for(var result : m_tracker.getEstimatedGlobalPose()){
      if (result.isPresent()){
        m_targetsVisible = true;
      }
    }
    SmartDashboard.putNumber("Heading", getHeadingInDegrees());
    SmartDashboard.putNumber("Reef Position", Integer.parseInt(m_reefPosition.getName()));
    SmartDashboard.putString("Source Position", m_source.getName());
    SmartDashboard.putBoolean("Do we see a target", m_targetsVisible);

    // SmartDashboard.putNumber("Rotate Error",
    // getRotationDistanceFromTargetError());
    // // Update the odometry in the periodic block
    SmartDashboard.putNumber("Turn FR Radians",
        m_frontRight.getPosition().angle.getRadians());
    SmartDashboard.putNumber(
        "Turn FL Radians",
        m_frontLeft.getPosition().angle.getRadians());
    SmartDashboard.putNumber(
        "Turn BR Radians",
        m_backRight.getPosition().angle.getRadians());
    SmartDashboard.putNumber(
        "Turn BL Radians", m_backLeft.getPosition().angle.getRadians());
    SmartDashboard.putNumber("Gyro Angle", ParadoxField.normalizeAngle(-m_gyro.getYaw().getValueAsDouble()));
    // spin robot very fast for many rotations eventually back to the original
    // angle, look at the gyro angle and see the offset

    // For efficiency. we could pass in the module states here, to avoid calling it
    // twice. Maybe also currentPos. - Gavin

    Pose2d currentPos = m_tracker.getPose2d();
    //m_tracker.displayRobotPosWithCamera();

    // double yaw = ParadoxField.normalizeAngle(m_gyro.getYaw().getValueAsDouble());
    // if (m_setGyroZero) {
    //   // FIXME: I think this calculation is wrong becauuse it neglects that fact that yaw is clockwise positive. Also, we shouldn't be trying to zero the gyro ourselves, as the PoseEstimator already takes care of this. And it's unsed.  - Gavin
    //   m_gyroZero = ParadoxField.normalizeAngle(currentPos.getRotation().getDegrees() - yaw);
    //   m_setGyroZero = false;
    // }
    // SmartDashboard.putNumber("Gyro offset",
    //     ParadoxField.normalizeAngle(currentPos.getRotation().getDegrees() - yaw - m_gyroZero));
    // SmartDashboard.putNumber("Gyro Zero", m_gyroZero);
    // SmartDashboard.putNumber("Gyro", yaw);
    // SmartDashboard.putNumber("Gyro Diff", ParadoxField.normalizeAngle(currentPos.getRotation().getDegrees() - yaw));
    // SmartDashboard.putNumber("Gyro Est Yaw", ParadoxField.normalizeAngle(currentPos.getRotation().getDegrees()));
    SmartDashboard.putNumber("Robot X", currentPos.getX());
    SmartDashboard.putNumber("Robot Y", currentPos.getY());

    // *********************************************************

    m_field.setRobotPose(m_tracker.getPose2d());
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
    m_tracker.setPose(pose);
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

   // TODO: Consider wrapping this in a command factory that takes a ChassisSpeeds supplier. -Gavin
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
                Rotation2d.fromDegrees(m_tracker.getPose2d().getRotation().getDegrees() +
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

  // FIXME: Consider having a stop() method. - Gavin

  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  // FIXME: This should be a command factory. -Gavin
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
  // FIXME: This is a bad idea. We should not offer this method. Also, it is unused. - Gavin
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_backLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_backRight.resetEncoders();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  // FIXME: This should return a Rotation2d. - Gavin
  public double getHeadingInDegrees() {
    double angle = m_tracker.getPose2d().getRotation().getDegrees();
    return ParadoxField.normalizeAngle(angle);
  }

  public static Command wheelRadiusCharacterization(DriveSubsystem drive) {
    SlewRateLimiter limiter = new SlewRateLimiter(k_wheelRadiusRampRate);
    WheelRadiusCharacterizationState state = new WheelRadiusCharacterizationState();

    return Commands.parallel(
        // Drive control sequence
        Commands.sequence(
            // Reset acceleration limiter
            Commands.runOnce(
                () -> {
                  limiter.reset(0.0);
                }),

            // Turn in place, accelerating up to full speed
            Commands.run(
                () -> {
                  double speed = limiter.calculate(k_wheelRadiusMaxVelocity);
                  drive.driveWithChassisSpeedRobotRelative(new ChassisSpeeds(0.0, 0.0, speed));
                },
                drive)),

        // Measurement sequence
        Commands.sequence(
            // Wait for modules to fully orient before starting measurement
            Commands.waitSeconds(1.0),

            // Record starting measurement
            Commands.runOnce(
                () -> {
                  state.positions = drive.getWheelRadiusCharacterizationPositions();
                  state.lastAngle = drive.getGyroRotation2d();
                  state.gyroDelta = 0.0;
                }),

            //Update gyro delta
            Commands.run(
                    () -> {
                      var rotation = drive.getGyroRotation2d();
                      state.gyroDelta += Math.abs(rotation.minus(state.lastAngle).getRadians());
                      state.lastAngle = rotation;
                    })

                // When cancelled, calculate and print results
                .finallyDo(
                    () -> {
                      double[] positions = drive.getWheelRadiusCharacterizationPositions();
                      double wheelDelta = 0.0;
                      for (int i = 0; i < 4; i++) {
                        wheelDelta += Math.abs(positions[i] - state.positions[i]) / 4.0;
                      }
                      double wheelRadius =
                          (state.gyroDelta * DriveConstants.k_driveBaseRadius) / wheelDelta;

                      NumberFormat formatter = new DecimalFormat("#0.000");
                      System.out.println(
                          "********** Wheel Radius Characterization Results **********");
                      System.out.println(
                          "\tWheel Delta: " + formatter.format(wheelDelta) + " radians");
                      System.out.println(
                          "\tGyro Delta: " + formatter.format(state.gyroDelta) + " radians");
                      System.out.println(
                          "\tWheel Radius: "
                              + formatter.format(wheelRadius)
                              + " meters, "
                              + formatter.format(Units.metersToInches(wheelRadius))
                              + " inches");
                    })));
  }
  
  private static class WheelRadiusCharacterizationState {
    double[] positions = new double[4];
    Rotation2d lastAngle = new Rotation2d();
    double gyroDelta = 0.0;
  }

  public double[] getWheelRadiusCharacterizationPositions() {
    
    double[] values = new double[4];
    for (int i = 0; i < 4; i++) {
      values[i] = modules[i].getWheelRadiusCharacterizationPosition();
    }
    return values;
  }
}
