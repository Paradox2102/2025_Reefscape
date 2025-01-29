// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// import com.pathplanner.lib.config.RobotConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.SparkBase;

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
  }
  
  public static class OperatorConstants {
    public static final int k_DriverControllerPort = 0;
  }

    public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double k_MaxSpeedMetersPerSecond = 4.8;
    public static final double k_MaxAngularSpeed = 2 * Math.PI; // radians per second

    // Angular offsets of the modules relative to the chassis in radians
    public static final double k_FLChassisAngularOffset = -Math.PI / 2;
    public static final double k_FRChassisAngularOffset = 0;
    public static final double k_BLChassisAngularOffset = Math.PI;
    public static final double k_BRChassisAngularOffset = Math.PI / 2;

    public static double k_FLOffset = 4.309 + k_FLChassisAngularOffset;
    public static double k_FROffset = 4.193 + k_FRChassisAngularOffset;
    public static double k_BLOffset = 2.097 + k_BLChassisAngularOffset;
    public static double k_BROffset = 6.239 + k_BRChassisAngularOffset;

    // SPARK MAX CAN IDs
    public static final int k_FRTurningMotor = 2;
    public static final int k_FLTurningMotor = 4;
    public static final int k_BRTurningMotor = 6;
    public static final int k_BLTurningMotor = 8;

    public static final int k_FRDriveMotor = 1;
    public static final int k_FLDriveMotor = 3;
    public static final int k_BRDriveMotor = 5;
    public static final int k_BLDriveMotor = 7;

    public static final double k_driveDeadband = 0.1;

    public static final double k_rotateF = 0;
    public static final double k_rotateP = .016;
    public static final double k_rotateI = .05;// .01;
    public static final double k_rotateD = .0008;
    public static final double k_rotateIZone = 20; // 70
    public static final double k_rotateDeadzone = 2;
    public static final double k_maxRotInput = .8;

    public static double k_maxSpeedMetersPerSecond = 5.74;
    public static final double k_maxDriveAcceleration = 7.427;
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

  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T,
    // 13T, or 14T. This changes the drive speed of the module (a pinion gear with
    // more teeth will result in a robot that drives faster).
    public static final int k_DrivingMotorPinionTeeth = 12;

    // Calculations required for driving motor conversion factors and feed forward
    // FIXME: Integer division!!! -Gavin
    public static final double k_DrivingMotorFreeSpeedRps = 6784 / 60;
    public static final double k_WheelDiameterMeters = 0.0762;
    public static final double k_WheelCircumferenceMeters = k_WheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
    // teeth on the bevel pinion
    public static final double k_DrivingMotorReduction = (45.0 * 22) / (k_DrivingMotorPinionTeeth * 15);
    public static final double k_DriveWheelFreeSpeedRps = (k_DrivingMotorFreeSpeedRps * k_WheelCircumferenceMeters)
        / k_DrivingMotorReduction;
  }

  public static final class PivotConstants {
    public static final int k_pivotMotor = 10;
    
    public static final double k_p = .005;
    public static final double k_i = .011;
    public static final double k_d = .0001;
    public static final double k_f = .02;
    public static final double k_ticksToDegrees = 45.0/1.6;
  }
  
  public static final class RollerConstants {
    public static final int k_LcoralMotor = 12;
    public static final int k_RcoralMotor = 13;
    
    public static final double k_coralP = 0;
    public static final double k_coralI = 0;
    public static final double k_coralD = 0;
    public static final double k_coralF = 0;
    public static final double k_coralTicksToDegrees = 1;

    public static final int k_algaeMotor = 9;
    
    public static final double k_algaeP = 0;
    public static final double k_algaeI = 0;
    public static final double k_algaeD = 0;
    public static final double k_algaeF = 0;
    public static final double k_algaeTicksToDegrees = 1;

    public static final int k_hopperMotor = 0;
  }

  public static final class ElevatorConstants {
    public static final int k_elevatorMotor = 11;
    
    public static final double k_p = 0;
    public static final double k_i = 0;
    public static final double k_d = 0;
    public static final double k_f = 0;
    public static final double k_ticksToInches = 1;
  }

  public static final class WristConstants {
    public static final int k_wristMotor = 14;

    public static final double k_p = 0;
    public static final double k_i = 0;
    public static final double k_d = 0;
    public static final double k_f = 0;
    public static final double k_ticksToDegrees = 1;
  }

  public static final class ClimberConstants{
    public static final int k_climberMotor = 16;
    public static final int k_climberFollower = 17;

    public static final double k_p = 0.25;
    public static final double k_i = 0;
    public static final double k_d = 0;
    public static final double k_f = 0;
    public static final double k_ticksToDegrees = 90/106.94;//90/98.283;
    public static double k_extendPosition = 90;
    public static double k_returnPosition = -150;
  }



  public final class MotorConfigs {
    public static final class SwerveModule {
      public static final SparkMaxConfig drivingConfig = new SparkMaxConfig();
      public static final SparkMaxConfig turningConfig = new SparkMaxConfig();

      public static SparkMaxConfig coastDriveConfig = new SparkMaxConfig();
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
            // FIXME: turningFactor is only coincidentally the right value here because the encoder is in revolutions. With a different encoder, this would be wrong. -Gavin 
            .positionWrappingInputRange(0, turningFactor);
        // FIXME: This is a bug. Java does not have assignment overloading. This will throw away the existing coastDriveConfig and coastTurnConfig objects and replace them with references to drivingConfig and turningConfig. The idleMode() method then modifies the object in place, so this is not creating a separate config object, but seeting coast mode on all configs. -Gavin
        coastDriveConfig = drivingConfig;
        coastTurnConfig = turningConfig;
        
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
        config.encoder
              .positionConversionFactor(ClimberConstants.k_ticksToDegrees);
        config.closedLoop
          .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
          .pid(ClimberConstants.k_p, ClimberConstants.k_i, ClimberConstants.k_d)
          .positionWrappingInputRange(ClimberConstants.k_returnPosition, ClimberConstants.k_extendPosition);
        followConfig.apply(config);
        followConfig.follow(ClimberConstants.k_climberMotor, true);


        coastConfig.apply(config);
        coastConfig.idleMode(IdleMode.kCoast);
        fcConfig.apply(coastConfig);
        fcConfig.follow(ClimberConstants.k_climberMotor, true);
      }
    }

    public static final class Elevator {
      public static final SparkMaxConfig elevatorConfig = new SparkMaxConfig();

      public static SparkMaxConfig coastElevatorConfig = new SparkMaxConfig();

      static {
        elevatorConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(20);
        elevatorConfig.absoluteEncoder
            .positionConversionFactor(ElevatorConstants.k_ticksToInches)
            .velocityConversionFactor(ElevatorConstants.k_ticksToInches);
        elevatorConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
            .pidf(ElevatorConstants.k_p, ElevatorConstants.k_i, ElevatorConstants.k_d, ElevatorConstants.k_f);
        coastElevatorConfig = elevatorConfig;

        coastElevatorConfig.idleMode(IdleMode.kCoast);
      }

    }

    public static final class Hopper {
      public static final SparkMaxConfig config = new SparkMaxConfig();

      static {
        config.idleMode(IdleMode.kBrake).smartCurrentLimit(20);
        config.absoluteEncoder
            .positionConversionFactor(1)
            .velocityConversionFactor(1);
      }
    }

    public static final class CoralOuttake {
      public static final SparkMaxConfig leftConfig = new SparkMaxConfig();
      public static final SparkMaxConfig rightConfig = new SparkMaxConfig();

      public static SparkMaxConfig coastLeftConfig = new SparkMaxConfig();
      public static SparkMaxConfig coastRightConfig = new SparkMaxConfig();

      static {
        leftConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(20);
        leftConfig.absoluteEncoder
        .positionConversionFactor(RollerConstants.k_coralTicksToDegrees)
        .velocityConversionFactor(RollerConstants.k_coralTicksToDegrees);
        coastLeftConfig = leftConfig;

        coastLeftConfig.idleMode(IdleMode.kCoast);

        rightConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(20);
        rightConfig.absoluteEncoder
        .positionConversionFactor(RollerConstants.k_coralTicksToDegrees)
        .velocityConversionFactor(RollerConstants.k_coralTicksToDegrees);
        coastRightConfig = rightConfig;

        coastRightConfig.idleMode(IdleMode.kCoast);
      }
        }
        
        public static final class Algae {
      public static final SparkMaxConfig pivotConfig = new SparkMaxConfig();
      public static final SparkMaxConfig rollerConfig = new SparkMaxConfig();

      public static SparkMaxConfig coastAlgaeConfig = new SparkMaxConfig();
      public static SparkMaxConfig coastRoller = new SparkMaxConfig();

      static {
        pivotConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(80);
        pivotConfig.encoder.positionConversionFactor(PivotConstants.k_ticksToDegrees);
        pivotConfig.absoluteEncoder
            .positionConversionFactor(PivotConstants.k_ticksToDegrees)
            .velocityConversionFactor(PivotConstants.k_ticksToDegrees);
        coastAlgaeConfig = pivotConfig;

        coastAlgaeConfig.idleMode(IdleMode.kCoast);


        rollerConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(20);
        rollerConfig.absoluteEncoder
            .positionConversionFactor(RollerConstants.k_algaeTicksToDegrees)
            .velocityConversionFactor(RollerConstants.k_algaeTicksToDegrees);
        rollerConfig.closedLoop
             .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
             .pid(RollerConstants.k_algaeP, RollerConstants.k_algaeI, RollerConstants.k_algaeD);
        coastRoller = rollerConfig;
  
        coastRoller.idleMode(IdleMode.kCoast);
      }

    }
  }
}
