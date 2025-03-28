// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import org.photonvision.PhotonCamera;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.driverCommands.ScoreBackAwayResetElevator;
import frc.robot.subsystems.CoralOuttakeSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorPosition;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Leave4201Auto extends SequentialCommandGroup {
  /** Creates a new Leave4201Auto. */
  public Leave4201Auto(PivotSubsystem pivotSubsystem, DriveSubsystem drive, Trigger shouldAim, ElevatorSubsystem elevator, CoralOuttakeSubsystem coral, PhotonCamera alignCam) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        elevator.setTargetPos(ElevatorPosition.L4),
        elevator.goToPosition(),
        new DriveForwardCommand(drive, 8, -0.025),
        new WaitCommand(2),
        new ScoreBackAwayResetElevator(pivotSubsystem, alignCam, shouldAim, drive, elevator, coral, () -> 0, () -> 0, () -> 0));
  }
}
