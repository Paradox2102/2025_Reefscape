// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.drive.DriveCommand;
import frc.robot.commands.drive.DriveToPosition;
import frc.robot.commands.operatorCommands.SetElevatorPos;
import frc.robot.commands.operatorCommands.SetReefPos;
import frc.robot.commands.operatorCommands.SetSourcePos;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.CoralOuttakeSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.DriveSubsystem.FieldPosition;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorPosition;
import frc.robot.robotControl.RobotControl;

import org.photonvision.PhotonCamera;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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
  private CoralOuttakeSubsystem m_coralOuttakeSubsystem = new CoralOuttakeSubsystem();
  private ClimberSubsystem m_climberSubsystem = new ClimberSubsystem();
  private HopperSubsystem m_hopperSubsystem = new HopperSubsystem();
  private ElevatorSubsystem m_elevatorSubsystem = new ElevatorSubsystem();

  private final RobotControl m_robotControl = new RobotControl();
  
  // Triggers are self-registering. It's not necessary to store them in a variable if you're only using them once. Putting the button binding next to the command binding might make the code easier to read because you can more easily see what command each button is connected to. -Gavin
  private final Trigger m_L1 = new Trigger(()->m_robotControl.checkButton(1));
  private final Trigger m_L2 = new Trigger(()->m_robotControl.checkButton(2));
  private final Trigger m_L3 = new Trigger(()->m_robotControl.checkButton(3));
  private final Trigger m_L4 = new Trigger(()->m_robotControl.checkButton(4));

  private final Trigger m_reef1 = new Trigger(()->m_robotControl.checkButton(5));
  private final Trigger m_reef2 = new Trigger(()->m_robotControl.checkButton(6));
  private final Trigger m_reef3 = new Trigger(()->m_robotControl.checkButton(7));
  private final Trigger m_reef4 = new Trigger(()->m_robotControl.checkButton(8));
  private final Trigger m_reef5 = new Trigger(()->m_robotControl.checkButton(9));
  private final Trigger m_reef6 = new Trigger(()->m_robotControl.checkButton(10));
  private final Trigger m_reef7 = new Trigger(()->m_robotControl.checkButton(11));
  private final Trigger m_reef8 = new Trigger(()->m_robotControl.checkButton(12));
  private final Trigger m_reef9 = new Trigger(()->m_robotControl.checkButton(13));
  private final Trigger m_reef10 = new Trigger(()->m_robotControl.checkButton(14));
  private final Trigger m_reef11 = new Trigger(()->m_robotControl.checkButton(15));
  private final Trigger m_reef12 = new Trigger(()->m_robotControl.checkButton(16));

  private final Trigger m_leftSource = new Trigger(()->m_robotControl.checkButton(17));
  private final Trigger m_rightSource = new Trigger(()->m_robotControl.checkButton(18));

  private final Trigger m_win = new Trigger(()->m_robotControl.checkButton(19));

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
    // m_algaeSubsystem.setDefaultCommand(m_algaeSubsystem.reset());
    // m_coralOuttakeSubsystem.setDefaultCommand(m_coralOuttakeSubsystem.stop());
    // m_climberSubsystem.setDefaultCommand(m_climberSubsystem.stop());
    // m_hopperSubsystem.setDefaultCommand(m_hopperSubsystem.runHopper());

    // Algae
    // m_driverController.leftTrigger().whileTrue(m_algaeSubsystem.intake());
    // m_driverController.leftBumper().toggleOnTrue(m_algaeSubsystem.outtake());

    // Coral
    //m_driverController.rightBumper().onTrue(m_coralOuttakeSubsystem.ejectCoral());
    m_driverController.rightTrigger().toggleOnTrue(
      new DriveToPosition(m_driveSubsystem, false)
      .until(() -> false/* enter the has game piece condition */)
    );

    // Climb
    m_driverController.a().whileTrue(m_climberSubsystem.climb(false));
    m_driverController.b().whileTrue(m_climberSubsystem.climb(true));

    // Operator UI Controls
    // Elevator Position
    m_L1.onTrue(new SetElevatorPos(m_elevatorSubsystem, ElevatorPosition.L1));
    m_L2.onTrue(new SetElevatorPos(m_elevatorSubsystem, ElevatorPosition.L2));
    m_L3.onTrue(new SetElevatorPos(m_elevatorSubsystem, ElevatorPosition.L3));
    m_L4.onTrue(new SetElevatorPos(m_elevatorSubsystem, ElevatorPosition.L4));

    // Reef Position
    // FIXME: What is this number business? Given that FIRST has officially named the reef branches A-L, it would be much better to use those names. -Gavin
    m_reef1.onTrue(new SetReefPos(m_driveSubsystem, FieldPosition.ONE));
    m_reef2.onTrue(new SetReefPos(m_driveSubsystem, FieldPosition.TWO));
    m_reef3.onTrue(new SetReefPos(m_driveSubsystem, FieldPosition.THREE));
    m_reef4.onTrue(new SetReefPos(m_driveSubsystem, FieldPosition.FOUR));
    m_reef5.onTrue(new SetReefPos(m_driveSubsystem, FieldPosition.FIVE));
    m_reef6.onTrue(new SetReefPos(m_driveSubsystem, FieldPosition.SIX));
    m_reef7.onTrue(new SetReefPos(m_driveSubsystem, FieldPosition.SEVEN));
    m_reef8.onTrue(new SetReefPos(m_driveSubsystem, FieldPosition.EIGHT));
    m_reef9.onTrue(new SetReefPos(m_driveSubsystem, FieldPosition.NINE));
    m_reef10.onTrue(new SetReefPos(m_driveSubsystem, FieldPosition.TEN));
    m_reef11.onTrue(new SetReefPos(m_driveSubsystem, FieldPosition.ELEVEN));
    m_reef12.onTrue(new SetReefPos(m_driveSubsystem, FieldPosition.TWELVE));

    // Choose Source
    // Presumably these are for choosing between the two Coral Stations. I think you're going to want to be able to choose the position within on Coral Station as well. -Gavin
    m_leftSource.onTrue(new SetSourcePos(m_driveSubsystem, FieldPosition.SOURCE_LEFT));
    m_rightSource.onTrue(new SetSourcePos(m_driveSubsystem, FieldPosition.SOURCE_RIGHT));

    // Win Button!!!
    m_win.onTrue(new InstantCommand());
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
