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

import edu.wpi.first.math.MathUtil;
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
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
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
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class DriveSubsystem extends SubsystemBase {

  private FieldPosition m_reefPosition = FieldPosition.ONE;
  private FieldPosition m_source = FieldPosition.SOURCE_RIGHT;
  public static RobotConfig k_pathConfig;{
  try {
    k_pathConfig = RobotConfig.fromGUISettings();
  } catch (Exception e) {}}

  FieldPosition[] m_reefList = {
    FieldPosition.ONE,
    FieldPosition.TWO,
    FieldPosition.THREE,
    FieldPosition.FOUR,
    FieldPosition.FIVE,
    FieldPosition.SIX,
    FieldPosition.SEVEN,
    FieldPosition.EIGHT,
    FieldPosition.NINE,
    FieldPosition.TEN,
    FieldPosition.ELEVEN,
    FieldPosition.TWELVE,
  };

  private int m_reefIndex = 0;

  public enum FieldPosition {

    // For facing reef
    // ONE(new Pose2d(new Translation2d(6.21, 4.004), Rotation2d.fromDegrees(0)), true, "1", new Translation2d(5.24, 4.08)),
    // TWO(new Pose2d(new Translation2d(5.02, 3.43), Rotation2d.fromDegrees(-60)), false, "2", new Translation2d(4.92, 3.4)),
    // THREE(new Pose2d(new Translation2d(4.74, 3.27), Rotation2d.fromDegrees(-60)), true, "3", new Translation2d(4.92, 3.4)),
    // FOUR(new Pose2d(new Translation2d(4.24, 3.27), Rotation2d.fromDegrees(-120)), false, "4", new Translation2d(4.16, 3.29)),
    // FIVE(new Pose2d(new Translation2d(3.96, 3.43), Rotation2d.fromDegrees(-120)), true, "5", new Translation2d(4.16, 3.29)),
    // SIX(new Pose2d(new Translation2d(3.71, 3.9), Rotation2d.fromDegrees(180)), false, "6", new Translation2d(3.65, 3.94)),
    // SEVEN(new Pose2d(new Translation2d(3.71, 4.19), Rotation2d.fromDegrees(0)), true, "7", new Translation2d(3.65, 3.94)),
    // EIGHT(new Pose2d(new Translation2d(3.96, 4.62), Rotation2d.fromDegrees(120)), false, "8", new Translation2d(4, 4.72)),
    // NINE(new Pose2d(new Translation2d(4.24, 4.78), Rotation2d.fromDegrees(120)), true, "9", new Translation2d(4, 4.72)),
    // TEN(new Pose2d(new Translation2d(5.379, 5.474), Rotation2d.fromDegrees(-120)), false, "10", new Translation2d(4.83, 4.72)),
    // ELEVEN(new Pose2d(new Translation2d(5.379, 5.474), Rotation2d.fromDegrees(-120)), true, "11", new Translation2d(4.83, 4.72)),
    // TWELVE(new Pose2d(new Translation2d(6.21, 4.004), Rotation2d.fromDegrees(0)), false, "12", new Translation2d(5.24, 4.08)),

    // For driving to reef
    ONE(new Pose2d(new Translation2d(5.801, 4.025), Rotation2d.fromDegrees(0)), true, "1"),
    TWO(new Pose2d(new Translation2d(5.158, 2.904), Rotation2d.fromDegrees(-60)), false, "2"),
    THREE(new Pose2d(new Translation2d(5.158, 2.904), Rotation2d.fromDegrees(-60)), true, "3"),
    FOUR(new Pose2d(new Translation2d(3.861, 2.972), Rotation2d.fromDegrees(60)), false, "4"),
    FIVE(new Pose2d(new Translation2d(3.861, 2.972), Rotation2d.fromDegrees(60)), true, "5"),
    SIX(new Pose2d(new Translation2d(3.188, 4.035), Rotation2d.fromDegrees(180)), false, "6"),
    SEVEN(new Pose2d(new Translation2d(3.188, 4.035), Rotation2d.fromDegrees(0)), true, "7"),
    EIGHT(new Pose2d(new Translation2d(3.812, 5.156), Rotation2d.fromDegrees(-60)), false, "8"),
    NINE(new Pose2d(new Translation2d(3.812, 5.156), Rotation2d.fromDegrees(-60)), true, "9"),
    TEN(new Pose2d(new Translation2d(5.148, 5.156), Rotation2d.fromDegrees(-120)), false, "10"),
    ELEVEN(new Pose2d(new Translation2d(5.148, 5.156), Rotation2d.fromDegrees(60)), true, "11"),
    TWELVE(new Pose2d(new Translation2d(5.801, 4.025), Rotation2d.fromDegrees(0)), false, "12"),
    SOURCE_RIGHT(new Pose2d(new Translation2d(1.7, .65), Rotation2d.fromDegrees(52.5)), false, "Right"),
    SOURCE_LEFT(new Pose2d(new Translation2d(1.7, 7.38), Rotation2d.fromDegrees(52.5)), false, "Left");

    private Pose2d m_bluePose;
    private String m_name;
    private boolean m_left;
    private Translation2d m_algaePos = new Translation2d();

    private double fieldX = PositionTrackerPose.k_apriltags.getFieldLength();
    private double fieldY = PositionTrackerPose.k_apriltags.getFieldWidth();

    FieldPosition(Pose2d bluePose, boolean left, String name) {
      m_bluePose = bluePose;
      m_name = name;
      m_left = left;
    }

    FieldPosition(Pose2d bluePose, boolean left, String name, Translation2d algaePos) {
      this(bluePose, left, name);
      m_algaePos = algaePos;
    }


    public Pose2d targetPose() {
      return Constants.States.m_alliance == Alliance.Blue ? m_bluePose : new Pose2d(new Translation2d(fieldX - m_bluePose.getX(), fieldY - m_bluePose.getY()), new Rotation2d());
    }

    public Translation2d algaePos() {
      return Constants.States.m_alliance == Alliance.Blue ? m_algaePos : new Translation2d(fieldX - m_algaePos.getX(), fieldY - m_algaePos.getY());
    }

    public String getName() {
      return m_name;
    }

    public boolean isLeft() {
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
  private static final double k_wheelRadiusMaxVelocity = 0.5; // Rad/Sec
  private static final double k_wheelRadiusRampRate = 0.25; // Rad/Sec^2
  private final  MaxSwerveModule[] modules = new MaxSwerveModule[]{m_frontLeft, m_frontRight, m_backLeft, m_backRight};

  //AScope Pose publisher
  StructPublisher<Pose2d> m_posePublisher = NetworkTableInstance.getDefault()
  .getStructTopic("Robot Pose", Pose2d.struct).publish();


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
    return Commands.runOnce(() -> {m_reefPosition = position;});
  }

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

  public Supplier<Rotation2d> orientPID(DoubleSupplier targetRot) {
    double setpointDegrees = targetRot.getAsDouble();
    double heading = getHeading().getDegrees();
    double rot = MathUtil.clamp(m_orientPID.calculate(heading, setpointDegrees), -Constants.DriveConstants.k_maxRotInput,
    Constants.DriveConstants.k_maxRotInput);
    SmartDashboard.putNumber("Target Rot", setpointDegrees);
    return () -> Rotation2d.fromDegrees(rot);
  }

  public Pose2d getEstimatedFuturePos() {
    return m_futurePos;
  }

  public static boolean m_setGyroZero = true;
  public static double m_gyroZero = 0;


  @Override
  public void periodic() {
    // m_targetsVisible = false;
    // for(var result : m_tracker.getEstimatedGlobalPose()){
    //   if (result.isPresent()){
    //     m_targetsVisible = true;
    //   }
    // }
    // SmartDashboard.putNumber("Heading", getHeading().getDegrees());
    SmartDashboard.putNumber("Reef Position", Integer.parseInt(m_reefPosition.getName()));
    // SmartDashboard.putString("Source Position", m_source.getName());
    // SmartDashboard.putBoolean("Do we see a target", m_targetsVisible);

    // SmartDashboard.putNumber("Rotate Error",
    // getRotationDistanceFromTargetError());
    // // Update the odometry in the periodic block

    // SmartDashboard.putNumber("Turn FR Radians",
    //     m_frontRight.getPosition().angle.getRadians());
    // SmartDashboard.putNumber(
    //     "Turn FL Radians",
    //     m_frontLeft.getPosition().angle.getRadians());
    // SmartDashboard.putNumber(
    //     "Turn BR Radians",
    //     m_backRight.getPosition().angle.getRadians());
    // SmartDashboard.putNumber(
    //     "Turn BL Radians", m_backLeft.getPosition().angle.getRadians());
    // SmartDashboard.putNumber("Gyro Angle", ParadoxField.normalizeAngle(-m_gyro.getYaw().getValueAsDouble()));

    // spin robot very fast for many rotations eventually back to the original
    // angle, look at the gyro angle and see the offset

    // For efficiency. we could pass in the module states here, to avoid calling it
    // twice. Maybe also currentPos. - Gavin

    Pose2d currentPos = m_tracker.getPose2d();
    m_posePublisher.set(currentPos);
    //m_tracker.displayRobotPosWithCamera();

    // double yaw = ParadoxField.normalizeAngle(m_gyro.getYaw().getValueAsDouble());
    // SmartDashboard.putNumber("Gyro offset",
    //     ParadoxField.normalizeAngle(currentPos.getRotation().getDegrees() - yaw - m_gyroZero));
    // SmartDashboard.putNumber("Gyro Zero", m_gyroZero);
    // SmartDashboard.putNumber("Gyro", yaw);
    // SmartDashboard.putNumber("Gyro Diff", ParadoxField.normalizeAngle(currentPos.getRotation().getDegrees() - yaw));
    // SmartDashboard.putNumber("Gyro Est Yaw", ParadoxField.normalizeAngle(currentPos.getRotation().getDegrees()));
    // SmartDashboard.putNumber("Robot X", currentPos.getX());
    // SmartDashboard.putNumber("Robot Y", currentPos.getY());

    // *********************************************************

    m_field.setRobotPose(m_tracker.getPose2d());
    m_tracker.update();
  }

  public Rotation2d getRotationalDistanceFromReef() {
    // return m_reefPosition.targetPose().minus(m_tracker.getPose2d()).getTranslation().getAngle();
    Pose2d targetPose = m_reefPosition.targetPose();
    double xDist = targetPose.getX() - m_tracker.getPose2d().getX();
    double yDist = targetPose.getY() - m_tracker.getPose2d().getY();
    return Rotation2d.fromRadians(ParadoxField.normalizeAngle(
       Math.atan2(yDist, xDist)));
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

  public void stop(){
    drive(0, 0, 0, false, false);
  }

  public Command setX() {
    return Commands.run(() -> {
      m_frontLeft.setDesiredState(
        new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
      m_frontRight.setDesiredState(
        new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
      m_backLeft.setDesiredState(
        new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
      m_backRight.setDesiredState(
        new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    }, this);
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
  // we will never use this so i am not going to comment this out or delete it for now - Paul
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_backLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_backRight.resetEncoders();
  }

  public Command incrementReefPosition(boolean increment) {
    return Commands.runOnce(() -> {
      m_reefIndex += increment ? 1 : -1;
      if (m_reefIndex < 0) {
        m_reefIndex = 11;
      } else if (m_reefIndex > 11) {
        m_reefIndex = 0;
      }
      m_reefPosition = m_reefList[m_reefIndex];
    });
  }

  public Command setReefLeftRight(boolean left) {
    return Commands.runOnce(() -> {
      if (left) {
        m_reefPosition = FieldPosition.ONE;
      } else {
        m_reefPosition = FieldPosition.TWO;
      }
    });
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public Rotation2d getHeading() {
    double angle = m_tracker.getPose2d().getRotation().getDegrees();
    return Rotation2d.fromDegrees(ParadoxField.normalizeAngle(angle));
  }

  public Command wheelRadiusCharacterization(DriveSubsystem drive) {
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
