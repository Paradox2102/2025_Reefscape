// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.drive.DriveCommand;
import frc.robot.commands.driverCommands.ScoreBackAwayResetElevator;
import frc.robot.commands.driverCommands.SemiAutoPlaceOnReef;
import frc.robot.commands.driverCommands.AutoIntake;
import frc.robot.commands.driverCommands.IntakeCoral;
import frc.robot.commands.driverCommands.ManualPlaceOnReef;
import frc.robot.commands.operatorCommands.SetReefPos;
import frc.robot.commands.operatorCommands.SetSourcePos;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.CoralOuttakeSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.ClimberSubsystem.ClimberState;
import frc.robot.subsystems.DriveSubsystem.FieldPosition;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorPosition;
import frc.robot.robotControl.RobotControl;

import org.photonvision.PhotonCamera;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
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
  private PivotSubsystem m_pivotSubsystem = new PivotSubsystem();
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

  // private final Trigger m_win = new Trigger(()->m_robotControl.checkButton(19));

  private PhotonCamera m_cameraFL = new PhotonCamera("fl_camera");
  private PhotonCamera m_cameraBL = new PhotonCamera("bl_camera");
  private PhotonCamera m_cameraBR = new PhotonCamera("br_camera");
  private PhotonCamera m_cameraFR = new PhotonCamera("fr_camera");
  //private PhotonCamera m_alignCamera = new PhotonCamera("align_camera");
  public PositionTrackerPose m_tracker = new PositionTrackerPose(0, 0, m_driveSubsystem, m_cameraFL, m_cameraFR, m_cameraBL, m_cameraBR);

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.k_DriverControllerPort);
  private final CommandJoystick m_operatorController = new CommandJoystick(1);
  SendableChooser<Command> m_autoSelect = new SendableChooser<>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    addNamedCommands();
    configureBindings();
    m_driveSubsystem.setTracker(m_tracker);
    updateAutoChooser();
    m_robotControl.start();
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
      m_driveSubsystem, 
      m_driverController::getLeftX, 
      m_driverController::getLeftY, 
      m_driverController::getRightX,
      true));

    m_coralOuttakeSubsystem.setDefaultCommand(m_coralOuttakeSubsystem.holdCoral());
    m_hopperSubsystem.setDefaultCommand(m_hopperSubsystem.stop());
    m_elevatorSubsystem.setDefaultCommand(m_elevatorSubsystem.resetPosition());

    // Algae
    m_driverController.leftTrigger().whileTrue(m_pivotSubsystem.intake()
      .alongWith(m_hopperSubsystem.runHopper(0.6))
    );
    m_driverController.leftBumper().whileTrue(
      m_pivotSubsystem.outtake()
        .alongWith(m_hopperSubsystem.runHopper(-0.6))
      );
    m_driverController.b().toggleOnTrue(
      m_elevatorSubsystem.goToAlgaePosition()
        // .finallyDo(() -> m_elevatorSubsystem.resetPosition())
    );

    // Coral
    m_driverController.rightTrigger().whileTrue(new AutoIntake(m_driverController::getLeftX, m_driverController::getLeftY, m_driverController::getRightX,m_driveSubsystem, m_coralOuttakeSubsystem, m_elevatorSubsystem, m_pivotSubsystem, m_hopperSubsystem));
    // m_driverController.rightBumper().toggleOnTrue(
    //   new ProxyCommand(() -> Commands.either(
    //     new AutoPlaceOnReef(m_driveSubsystem, m_elevatorSubsystem, m_coralOuttakeSubsystem), // on true
    //     new ManualPlaceOnReef(m_elevatorSubsystem, m_driveSubsystem, m_driverController::getLeftX, m_driverController::getLeftY, m_driverController::getRightX), // on false
    //     () -> Constants.States.m_autoAim && m_elevatorSubsystem.getPreset() != ElevatorPosition.L1 // condition
    //   )).finallyDo(() -> new ScoreBackAwayResetElevator(m_driveSubsystem, m_elevatorSubsystem, m_coralOuttakeSubsystem, m_driverController::getLeftX, m_driverController::getLeftY, m_driverController::getRightX).schedule())
    // );
    m_driverController.rightBumper().toggleOnTrue(
      // new DeferredCommand(
      //   () -> Commands.either(
      //       new AutoPlaceOnReef(m_driveSubsystem, m_elevatorSubsystem, m_coralOuttakeSubsystem), // on true
      //       // new SemiAutoPlaceOnReef(m_driverController::getLeftX, m_driverController::getLeftY, m_driveSubsystem, m_elevatorSubsystem, m_coralOuttakeSubsystem), // on true
      //       new ManualPlaceOnReef(m_elevatorSubsystem, m_driveSubsystem, 
      //         m_driverController::getLeftX, 
      //         m_driverController::getLeftY, 
      //         m_driverController::getRightX), // on false
      //       () -> Constants.States.m_autoAim && m_elevatorSubsystem.getPreset() != ElevatorPosition.L1 // condition
      //   ),
      //   Set.of(m_driveSubsystem, m_elevatorSubsystem, m_coralOuttakeSubsystem)
      // ).andThen(new ScoreBackAwayResetElevator(m_driveSubsystem, m_elevatorSubsystem, m_coralOuttakeSubsystem, 
      //         m_driverController::getLeftX, 
      //         m_driverController::getLeftY, 
      //         m_driverController::getRightX))
      // );
      new ProxyCommand(() -> Commands.either(
        // new AutoPlaceOnReef(m_driveSubsystem, m_elevatorSubsystem, m_coralOuttakeSubsystem), // on true
        new SemiAutoPlaceOnReef(m_driverController::getLeftX, m_driverController::getLeftY, m_driverController::getRightX, m_driveSubsystem, m_elevatorSubsystem, m_coralOuttakeSubsystem),
        new ManualPlaceOnReef(m_elevatorSubsystem, m_driveSubsystem, m_driverController::getLeftX, m_driverController::getLeftY, m_driverController::getRightX), // on false
        () -> Constants.States.m_autoAim && m_elevatorSubsystem.getPreset() != ElevatorPosition.L1 // condition
      )).finallyDo(() -> new ScoreBackAwayResetElevator(m_driveSubsystem, m_elevatorSubsystem, m_coralOuttakeSubsystem, m_driverController::getLeftX, m_driverController::getLeftY, m_driverController::getRightX).schedule())
    );
    m_driverController.x().whileTrue(new IntakeCoral(m_coralOuttakeSubsystem, m_hopperSubsystem, m_elevatorSubsystem, m_pivotSubsystem));

    // Climb
    // m_driverController.a().toggleOnTrue(
    //   new ProxyCommand(() -> new PrepareForClimbCommand(m_algaeSubsystem, m_climberSubsystem))
    //     .finallyDo(() -> m_climberSubsystem.setPosition(ClimberState.CLIMB).schedule())
    // );
    // m_driverController.a().whileTrue(m_climberSubsystem.runOut());
    // m_driverController.b().whileTrue(m_climberSubsystem.runIn());
    m_driverController.a().onTrue(
      m_pivotSubsystem.climb()
    );

    // Hopper Pivot
    m_driverController.y().toggleOnTrue(m_pivotSubsystem.climb());

    //m_driverController.y().whileTrue(new DriveToPosition(m_driveSubsystem, true));

    m_driverController.povUp().onTrue(m_elevatorSubsystem.setTargetPos(ElevatorPosition.L4));
    m_driverController.povDown().onTrue(m_elevatorSubsystem.setTargetPos(ElevatorPosition.L1));
    m_driverController.povLeft().onTrue(m_elevatorSubsystem.setTargetPos(ElevatorPosition.L3));
    m_driverController.povRight().onTrue(m_elevatorSubsystem.setTargetPos(ElevatorPosition.L2));

    m_operatorController.button(1).whileTrue(m_hopperSubsystem.runHopper(-.1).alongWith(m_coralOuttakeSubsystem.runSpeed(1500)));
    m_operatorController.button(2).whileTrue(m_hopperSubsystem.runHopper(.1).alongWith(m_coralOuttakeSubsystem.runSpeed(-1500)));
    m_operatorController.button(8).toggleOnTrue(m_elevatorSubsystem.manualMove(() -> m_operatorController.getY()));

    m_operatorController.button(7).onTrue(m_elevatorSubsystem.setAlgaePosition(ElevatorPosition.ALGAE_HIGH));
    m_operatorController.button(9).onTrue(m_elevatorSubsystem.setAlgaePosition(ElevatorPosition.ALGAE_LOW));

    m_operatorController.button(11).whileTrue(m_climberSubsystem.runOut());
    m_operatorController.button(12).whileTrue(m_climberSubsystem.runIn());

    m_operatorController.povRight().onTrue(m_driveSubsystem.incrementReefPosition(true));
    m_operatorController.povLeft().onTrue(m_driveSubsystem.incrementReefPosition(false));

    // Operator UI Controls

    // Elevator Position
    m_L1.onTrue(m_elevatorSubsystem.setTargetPos(ElevatorPosition.L1));
    m_L2.onTrue(m_elevatorSubsystem.setTargetPos(ElevatorPosition.L2));
    m_L3.onTrue(m_elevatorSubsystem.setTargetPos(ElevatorPosition.L3));
    m_L4.onTrue(m_elevatorSubsystem.setTargetPos(ElevatorPosition.L4));

    // Reef Position
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
    m_leftSource.onTrue(new SetSourcePos(m_driveSubsystem, FieldPosition.SOURCE_LEFT));
    m_rightSource.onTrue(new SetSourcePos(m_driveSubsystem, FieldPosition.SOURCE_RIGHT));

    // Win Button!!!
    // m_win.onTrue(new InstantCommand());
  }

  public boolean getThrottle() {
    return m_operatorController.getThrottle() < 0;
  }

  private void updateAutoChooser() {
    m_autoSelect.addOption("Nothing", new InstantCommand());
    m_autoSelect.addOption("Left Scuffed", new PathPlannerAuto("Left Scuffed"));
    m_autoSelect.addOption("Right Scuffed", new PathPlannerAuto("Right Scuffed"));
    m_autoSelect.addOption("399 Push Left", new PathPlannerAuto("399 Push Left"));
    m_autoSelect.addOption("399 Push Right", new PathPlannerAuto("399 Push Right"));
    m_autoSelect.addOption("Center L4", new PathPlannerAuto("Center 1 Auto"));
    m_autoSelect.addOption("Center Push", new PathPlannerAuto("Center Push L1"));
    m_autoSelect.addOption("Left L1", new PathPlannerAuto("left leave"));
    m_autoSelect.addOption("Right L1", new PathPlannerAuto("right leave"));
    m_autoSelect.addOption("Wheel Calibration", m_driveSubsystem.wheelRadiusCharacterization(m_driveSubsystem));
    SmartDashboard.putData(m_autoSelect);
  }

  private void addNamedCommands() {
    NamedCommands.registerCommand("Deploy Elevator", m_elevatorSubsystem.goToPosition());
    NamedCommands.registerCommand("Reset Elevator", m_elevatorSubsystem.resetPosition());
    NamedCommands.registerCommand("Set L1", m_elevatorSubsystem.setTargetPos(ElevatorPosition.L1));
    NamedCommands.registerCommand("Set L2", m_elevatorSubsystem.setTargetPos(ElevatorPosition.L2));
    NamedCommands.registerCommand("Set L3", m_elevatorSubsystem.setTargetPos(ElevatorPosition.L3));
    NamedCommands.registerCommand("Set L4", m_elevatorSubsystem.setTargetPos(ElevatorPosition.L4));
    NamedCommands.registerCommand("Intake Coral", new IntakeCoral(m_coralOuttakeSubsystem, m_hopperSubsystem, m_elevatorSubsystem, m_pivotSubsystem));
    NamedCommands.registerCommand("Score Coral", m_coralOuttakeSubsystem.ejectCoral(m_elevatorSubsystem.isLow));
    NamedCommands.registerCommand("Intake Algae", m_pivotSubsystem.intake());
    NamedCommands.registerCommand("Outtake Algae", m_pivotSubsystem.outtake());
    NamedCommands.registerCommand("Remove Algae", m_elevatorSubsystem.goToAlgaePosition());
    NamedCommands.registerCommand("Stop Coral", m_coralOuttakeSubsystem.stop().alongWith(m_hopperSubsystem.stop()));
    NamedCommands.registerCommand("Back Up", new DriveCommand(m_driveSubsystem, () -> -.2, () -> 0, () -> 0, false));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return m_autoSelect.getSelected();
  }
}
