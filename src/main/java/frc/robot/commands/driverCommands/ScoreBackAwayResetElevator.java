// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.driverCommands;

import java.util.concurrent.locks.Condition;
import java.util.function.DoubleSupplier;

import org.photonvision.PhotonCamera;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.drive.ApriltagAimCommand;
import frc.robot.commands.drive.DriveCommand;
import frc.robot.commands.drive.PrecisionAlignOdometrey;
import frc.robot.subsystems.CoralOuttakeSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorPosition;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreBackAwayResetElevator extends SequentialCommandGroup {
  /** Creates a new BackAwayResetElevator. */
  public ScoreBackAwayResetElevator(PhotonCamera alignCam, Trigger shouldAim, DriveSubsystem driveSubsystem, ElevatorSubsystem elevatorSubsystem, CoralOuttakeSubsystem COSubsystem, DoubleSupplier x, DoubleSupplier y, DoubleSupplier rot) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      // new ConditionalCommand(
      //   // new ApriltagAimCommand(alignCam, driveSubsystem), 
      //   new PrecisionAlignOdometrey(driveSubsystem),
      //   new InstantCommand(), 
      //   shouldAim
      // ),
      new ParallelDeadlineGroup(
         COSubsystem.ejectCoral(elevatorSubsystem.isLow),
         new ConditionalCommand(
          elevatorSubsystem.manualMove(() -> -1),
          new InstantCommand(),
          () -> elevatorSubsystem.getPreset() == ElevatorPosition.L1
         )
      ),
      new ConditionalCommand(
        new ParallelDeadlineGroup(
          new WaitCommand(.3),
          new DriveCommand(driveSubsystem, () -> 0, () -> -.2, () -> 0, false)
        ),
        new InstantCommand(),
        () -> elevatorSubsystem.getPreset() == ElevatorPosition.L4
      ),
      new ParallelDeadlineGroup(
        elevatorSubsystem.resetPosition(),
        new DriveCommand(driveSubsystem, x, y, rot, true)
      )
    );
  }
}
