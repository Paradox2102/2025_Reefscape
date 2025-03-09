// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.driverCommands;

import java.util.function.DoubleSupplier;

import org.photonvision.PhotonCamera;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.drive.ApriltagAimCommand;
import frc.robot.commands.drive.DriveCommand;
import frc.robot.commands.drive.DriveToPosition;
import frc.robot.subsystems.CoralOuttakeSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoPlaceOnReef extends SequentialCommandGroup {
  /** Creates a new AutoPlaceOnReef. */
  public AutoPlaceOnReef(DriveSubsystem driveSubsystem, ElevatorSubsystem elevatorSubsystem, CoralOuttakeSubsystem COSubsystem, DoubleSupplier x, DoubleSupplier y, DoubleSupplier rot, PhotonCamera camera) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new DriveToPosition(driveSubsystem, true),
      new ParallelDeadlineGroup(
        //new PrecisionAlignOdometrey(driveSubsystem),
        elevatorSubsystem.goToPosition()
      ),
      new ApriltagAimCommand(camera, driveSubsystem),
      new ParallelDeadlineGroup(
        new WaitCommand(1), 
        new DriveCommand(driveSubsystem, () -> 0, () -> .1, () -> 0, false, true)
      )
    );
  }
}
