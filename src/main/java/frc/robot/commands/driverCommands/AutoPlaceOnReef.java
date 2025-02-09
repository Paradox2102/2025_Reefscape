// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.driverCommands;

import org.photonvision.PhotonCamera;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.States;
import frc.robot.commands.drive.ApriltagAimCommand;
import frc.robot.commands.drive.DriveToPosition;
import frc.robot.subsystems.CoralOuttakeSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorPosition;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoPlaceOnReef extends SequentialCommandGroup {
  /** Creates a new AutoPlaceOnReef. */
  public AutoPlaceOnReef(DriveSubsystem driveSubsystem, ElevatorSubsystem elevatorSubsystem, CoralOuttakeSubsystem COSubsystem, PhotonCamera camera, ElevatorPosition level) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      elevatorSubsystem.setTargetPos(level),
      new DriveToPosition(driveSubsystem, true)
        .unless(() -> (elevatorSubsystem.getPreset() == ElevatorPosition.L1) || !States.m_autoAim),
      new ParallelCommandGroup(
        new ApriltagAimCommand(camera, driveSubsystem, driveSubsystem.getReefPosition().leftRight())
          .unless(() -> (elevatorSubsystem.getPreset() == ElevatorPosition.L1) || !States.m_autoAim),
        elevatorSubsystem.goToPosition().until(elevatorSubsystem.atPosition)
      ),
      COSubsystem.ejectCoral(),
      elevatorSubsystem.resetPosition()
    );
  }
}
