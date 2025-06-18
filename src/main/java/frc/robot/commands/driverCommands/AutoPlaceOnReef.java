// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.driverCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.auto.WaitForCameraCommand;
import frc.robot.commands.drive.DriveToPosition;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoPlaceOnReef extends SequentialCommandGroup {
  /** Creates a new AutoPlaceOnReef. */
  public AutoPlaceOnReef(DriveSubsystem driveSubsystem, ElevatorSubsystem elevatorSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new WaitForCameraCommand(driveSubsystem),
        new DriveToPosition(driveSubsystem),
        elevatorSubsystem.goToPosition()
    );
  }
}
