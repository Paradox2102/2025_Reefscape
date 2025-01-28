// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.PID.AlgaePIDCommand;
import frc.robot.commands.PID.CoralPIDCommand;
import frc.robot.commands.PID.ElevatorPIDCommand;
import frc.robot.commands.drive.DriveCommand;
import frc.robot.commands.drive.DriveToPosition;
import frc.robot.commands.test.TestAlgae;
import frc.robot.commands.test.TestCoralOuttake;
import frc.robot.commands.test.TestElevator;
import frc.robot.commands.test.TestPivot;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.CoralOuttakeSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.HopperSubsystem;

import org.photonvision.PhotonCamera;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private DriveSubsystem m_driveSubsystem = new DriveSubsystem();
  private AlgaeSubsystem m_algaeSubsystem = new AlgaeSubsystem();
  // private CoralOuttakeSubsystem m_coralOuttakeSubsystem = new CoralOuttakeSubsystem();
  // private ClimberSubsystem m_climberSubsystem = new ClimberSubsystem();
  // private HopperSubsystem m_hopperSubsystem = new HopperSubsystem();
  // private ElevatorSubsystem m_elevatorSubsystem = new ElevatorSubsystem();

  private PhotonCamera m_camera1 = new PhotonCamera("camera1");
  //private PhotonCamera m_camera2 = new PhotonCamera("camera2");
  //private PhotonCamera m_alignCamera = new PhotonCamera("align");
  public PositionTrackerPose m_tracker = new PositionTrackerPose(0, 0, m_driveSubsystem, m_camera1);

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.k_DriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    m_driveSubsystem.setTracker(m_tracker);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Subsystem Default Commands
    m_driveSubsystem.setDefaultCommand(new DriveCommand(
      m_driveSubsystem, m_driverController::getLeftX, 
      m_driverController::getLeftY, 
      m_driverController::getRightX));
    // m_algaeSubsystem.setDefaultCommand(new AlgaePIDCommand(m_algaeSubsystem));
    // m_coralOuttakeSubsystem.setDefaultCommand(new CoralPIDCommand(m_coralOuttakeSubsystem));
    // m_climberSubsystem.setDefaultCommand(m_climberSubsystem.stop());
    // m_hopperSubsystem.setDefaultCommand(m_hopperSubsystem.runHopper());
    // m_elevatorSubsystem.setDefaultCommand(new ElevatorPIDCommand(m_elevatorSubsystem));

    //Test
    // m_driverController.a().whileTrue(new TestElevator(m_elevatorSubsystem, 0));
    // m_driverController.b().whileTrue(new TestCoralOuttake(m_coralOuttakeSubsystem, 0));
    m_driverController.x().whileTrue(new TestPivot(m_algaeSubsystem, 45));
    m_driverController.y().whileTrue(new TestPivot(m_algaeSubsystem, 0));
    // Algae
    m_driverController.leftTrigger().whileTrue(m_algaeSubsystem.intake());
    m_driverController.leftBumper().toggleOnTrue(m_algaeSubsystem.outtake());

    // Coral
    // m_driverController.rightBumper().onTrue(m_coralOuttakeSubsystem.ejectCoral());
    m_driverController.rightTrigger().toggleOnTrue(
      new DriveToPosition(m_driveSubsystem, false)
      .until(() -> false/* enter the has game piece condition */)
    );

    // Climb
    // m_driverController.a().whileTrue(m_climberSubsystem.climb(false));
    // m_driverController.b().whileTrue(m_climberSubsystem.climb(true));
      
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
  }
}
