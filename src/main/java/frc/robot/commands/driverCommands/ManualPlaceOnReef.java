// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.driverCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.drive.DriveCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ManualPlaceOnReef extends ParallelCommandGroup {
  /** Creates a new ManualPlaceOnReef. */
  public ManualPlaceOnReef(ElevatorSubsystem elevatorSubsystem, DriveSubsystem driveSubsystem, DoubleSupplier x, DoubleSupplier y, DoubleSupplier rot) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    double slowMode = .3;
    addCommands(
      new DriveCommand(driveSubsystem, x, y, rot, false, true),
      elevatorSubsystem.goToPosition()
    );
  }
}
