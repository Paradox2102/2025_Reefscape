// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// import com.pathplanner.lib.config.RobotConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkFlexConfig;

import com.revrobotics.spark.ClosedLoopSlot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation.Alliance;


/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class States {
    public static Alliance m_alliance = Alliance.Blue;
    public static boolean m_autoAim = true;

  }
  
  public static class OperatorConstants {
    public static final int k_DriverControllerPort = 0;
  }

    public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double k_MaxSpeedMetersPerSecond = 4.92;
    public static final double k_MaxAngularSpeed = 2 * Math.PI; // radians per second

    // Angular offsets of the modules relative to the chassis in radians
    public static final double k_FLChassisAngularOffset = -Math.PI / 2;
    public static final double k_FRChassisAngularOffset = 0;
    public static final double k_BLChassisAngularOffset = Math.PI;
    public static final double k_BRChassisAngularOffset = Math.PI / 2;

    public static double k_FLOffset = 4.18 + k_FLChassisAngularOffset;
    public static double k_FROffset = .87 + k_FRChassisAngularOffset;
    public static double k_BLOffset = 2.22 + k_BLChassisAngularOffset;
    public static double k_BROffset = 3.06 + k_BRChassisAngularOffset;

    // SPARK MAX CAN IDs
    public static final int k_FRTurningMotor = 2; // 2
    public static final int k_FLTurningMotor = 4; // 4
    public static final int k_BRTurningMotor = 6; // 6
    public static final int k_BLTurningMotor = 8; // 8

    public static final int k_FRDriveMotor = 1; // 1
    public static final int k_FLDriveMotor = 3; // 3
    public static final int k_BRDriveMotor = 5; // 5
    public static final int k_BLDriveMotor = 7; // 7

    public static final double k_driveDeadband = 0.1;

    public static final double k_rotateF = 0;
    public static final double k_rotateP = .016;
    public static final double k_rotateI = .05;// .01;
    public static final double k_rotateD = .0008;
    public static final double k_rotateIZone = 20; // 70
    public static final double k_rotateDeadzone = 2;
    public static final double k_maxRotInput = .8;

    public static double k_maxSpeedMetersPerSecond = 4.92;
    public static final double k_maxDriveAcceleration = 7.427; // change?
    public static final double k_maxAngularSpeed = Math.PI * 1.5; // radians per second
    public static final double k_maxAngularAcceleration = Math.PI;

    public static final double k_directionSlewRate = 3; // radians per second
    public static final double k_magnitudeSlewRate = 3.25; // percent per second (1 = 100%)
    public static final double k_rotationalSlewRate = 3; // percent per second (1 = 100%)

    public static final SwerveModuleState[] k_defaultState = {
        new SwerveModuleState(0, new Rotation2d(Math.PI / 4)),
        new SwerveModuleState(0, new Rotation2d(3 * Math.PI / 4)),
        new SwerveModuleState(0, new Rotation2d(3 * Math.PI / 4)),
        new SwerveModuleState(0, new Rotation2d(Math.PI / 4))
    };

    public static final double k_lookAheadTimeSeconds = .2;

    public static final boolean k_GyroReversed = false;

    public static final double k_trackWidth = 0.273;
    public static final double k_wheelBase = 0.273;
    public static final double k_driveBaseRadius = Math.hypot(k_trackWidth / 2.0, k_wheelBase / 2.0);
  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T,
    // 13T, or 14T. This changes the drive speed of the module (a pinion gear with
    // more teeth will result in a robot that drives faster).
    public static final int k_DrivingMotorPinionTeeth = 12;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double k_DrivingMotorFreeSpeedRps = 6784.0 / 60.0;
    public static final double k_WheelDiameterMeters = 0.068;
    public static final double k_WheelCircumferenceMeters = k_WheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
    // teeth on the bevel pinion
    public static final double k_DrivingMotorReduction = (45.0 * 22) / (k_DrivingMotorPinionTeeth * 15);
    public static final double k_DriveWheelFreeSpeedRps = (k_DrivingMotorFreeSpeedRps * k_WheelCircumferenceMeters)
        / k_DrivingMotorReduction;
  }

  public static final class AlgaeConstants {
    public static final int k_pivotMotor = 10;
    public static final int k_algaeMotor = 9;
    public static final double k_ticksToDegrees = 45/17.68;
  }
  
  public static final class COConstants {
    public static final int k_coralMotor = 14;
    
    public static final double k_p = 0.00005;
    public static final double k_i = 0.0000001;
    public static final double k_d = 0;
    public static final double k_f = 0.00018;
  }

  public static final class HopperConstants {
    public static final int k_hopperMotor = 13;

    public static final double k_p = 0;
    public static final double k_i = 0;
    public static final double k_d = 0;
    public static final double k_f = 0.00015;
  }

  public static final class ElevatorConstants {
    public static final int k_elevatorMotor = 11;
    public static final int k_elevatorFollower = 12;

    public static final double k_p = 0.8;
    public static final double k_i = 0.000001;
    public static final double k_d = 0.001;

    public static final double k_downP = 0.015;
    public static final double k_downI = 0.000005;
    public static final double k_downD = 0.0025;

    public static final double k_ticksToInches = 4.0/2.67;
  }

  public static final class ClimberConstants{
    public static final int k_climberMotor = 16;
    public static final int k_climberFollower = 17;

    public static final double k_p = 0.005;
    public static final double k_i = 0;
    public static final double k_d = 0;
    public static final double k_f = 0;
    public static final double k_ticksToDegrees = 360;
    public static final double k_resetPosition = 20;
    public static double k_extendPosition = 290;
    public static double k_returnPosition = 110;
  }



  public final class MotorConfigs {
    public static final class SwerveModule {
      public static final SparkFlexConfig drivingConfig = new SparkFlexConfig();
      public static final SparkMaxConfig turningConfig = new SparkMaxConfig();

      public static SparkFlexConfig coastDriveConfig = new SparkFlexConfig();
      public static SparkMaxConfig coastTurnConfig = new SparkMaxConfig();

      static {
        // Use module constants to calculate conversion factors and feed forward gain.
        double drivingFactor = ModuleConstants.k_WheelDiameterMeters * Math.PI
            / ModuleConstants.k_DrivingMotorReduction;
        double turningFactor = 2 * Math.PI;
        double drivingVelocityFeedForward = 1 / ModuleConstants.k_DriveWheelFreeSpeedRps;

        drivingConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(50);
        drivingConfig.encoder
            .positionConversionFactor(drivingFactor) // meters
            .velocityConversionFactor(drivingFactor / 60.0); // meters per second
        drivingConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            // These are example gains you may need to them for your own robot!
            .pid(0.04, 0, 0)
            .velocityFF(drivingVelocityFeedForward)
            .outputRange(-1, 1);

        turningConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(20);
        turningConfig.absoluteEncoder
            // Invert the turning encoder, since the output shaft rotates in the opposite
            // direction of the steering motor in the MAXSwerve Module.
            .inverted(true)
            .positionConversionFactor(turningFactor) // radians
            .velocityConversionFactor(turningFactor / 60.0); // radians per second
        turningConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
            // These are example gains you may need to them for your own robot!
            .pid(1, 0, 0)
            .outputRange(-1, 1)
            // Enable PID wrap around for the turning motor. This will allow the PID
            // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
            // to 10 degrees will go through 0 rather than the other direction which is a
            // longer route.
            .positionWrappingEnabled(true)
            .positionWrappingInputRange(0, 2 * Math.PI);
        coastDriveConfig.apply(drivingConfig);
        coastTurnConfig.apply(turningConfig);
        
        coastDriveConfig.idleMode(IdleMode.kCoast);
        coastTurnConfig.idleMode(IdleMode.kCoast);
      }
    }

    public static final class Climber {
      public static final SparkFlexConfig config = new SparkFlexConfig();
      public static SparkFlexConfig followConfig = new SparkFlexConfig();

      public static SparkFlexConfig coastConfig = new SparkFlexConfig();
      public static SparkFlexConfig fcConfig = new SparkFlexConfig();

      static {

        config.idleMode(IdleMode.kBrake).smartCurrentLimit(80);
        config.absoluteEncoder
              .positionConversionFactor(ClimberConstants.k_ticksToDegrees)
              .zeroOffset(0);
        config.closedLoop
          .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
          .pid(ClimberConstants.k_p, ClimberConstants.k_i, ClimberConstants.k_d)
          .positionWrappingInputRange(0, 360);
        followConfig.apply(config);
        followConfig.follow(ClimberConstants.k_climberMotor, true);


        coastConfig.apply(config);
        coastConfig.idleMode(IdleMode.kCoast);
        fcConfig.apply(coastConfig);
        fcConfig.follow(ClimberConstants.k_climberMotor, true);
      }
    }

    public static final class Elevator {
      public static final SparkFlexConfig leaderConfig = new SparkFlexConfig();
      public static final SparkFlexConfig followerConfig = new SparkFlexConfig();

      public static SparkFlexConfig coastElevatorConfig = new SparkFlexConfig();
      public static SparkFlexConfig coastFollowerConfig = new SparkFlexConfig();

      static {
        leaderConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(80);
        followerConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(80);
      
        leaderConfig.encoder
            .positionConversionFactor(ElevatorConstants.k_ticksToInches);
        leaderConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(ElevatorConstants.k_p, ElevatorConstants.k_i, ElevatorConstants.k_d, ClosedLoopSlot.kSlot0)
            .pid(ElevatorConstants.k_downP, ElevatorConstants.k_downI, ElevatorConstants.k_downD, ClosedLoopSlot.kSlot1);
        leaderConfig.closedLoop.maxMotion
          .maxVelocity(6000)
          .maxAcceleration(12000)
          .allowedClosedLoopError(0.5);
        //coastElevatorConfig.apply(elevatorConfig);

        coastElevatorConfig.idleMode(IdleMode.kCoast);

        followerConfig.follow(ElevatorConstants.k_elevatorMotor, true);
      }

    }

    public static final class Hopper {
      public static final SparkFlexConfig config = new SparkFlexConfig();

      static {
        config.idleMode(IdleMode.kBrake).smartCurrentLimit(80);
        config.closedLoop
          .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
          .pid(HopperConstants.k_p, HopperConstants.k_i, HopperConstants.k_d)
          .velocityFF(HopperConstants.k_f);
      }
    }

    public static final class CoralOuttake {
      public static final SparkFlexConfig config = new SparkFlexConfig();
      //public static final SparkFlexConfig practiceConfig = new SparkFlexConfig();

      //public static SparkFlexConfig coastConfig = new SparkFlexConfig();
      //public static SparkFlexConfig coastPracticeConfig = new SparkFlexConfig();

      static {
        config.idleMode(IdleMode.kBrake).smartCurrentLimit(80);
        config.closedLoop.pid(COConstants.k_p, COConstants.k_i, COConstants.k_d)
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder).velocityFF(COConstants.k_f);
        //coastConfig.apply(config);

        //coastConfig.idleMode(IdleMode.kCoast);

        // practiceConfig.apply(config);
        // practiceConfig.follow(RollerConstants.k_coralMotor, false);

        // coastPracticeConfig.apply(practiceConfig);

        //coastPracticeConfig.idleMode(IdleMode.kCoast);
      }
        }
        
        public static final class Algae {
          public static final SparkFlexConfig pivotConfig = new SparkFlexConfig();

          public static SparkFlexConfig coastAlgaeConfig = new SparkFlexConfig();

          static {
            pivotConfig.idleMode(IdleMode.kBrake)
              .smartCurrentLimit(25)
            .encoder
              .positionConversionFactor(AlgaeConstants.k_ticksToDegrees);
            coastAlgaeConfig.apply(pivotConfig);

            coastAlgaeConfig.idleMode(IdleMode.kCoast);
          }

    }
  }
}
