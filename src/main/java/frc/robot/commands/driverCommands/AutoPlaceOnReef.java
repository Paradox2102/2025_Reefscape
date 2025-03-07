// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.driverCommands;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.drive.DriveCommand;
import frc.robot.commands.drive.DriveToPosition;
import frc.robot.subsystems.CoralOuttakeSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoPlaceOnReef extends ParallelCommandGroup {
  /** Creates a new AutoPlaceOnReef. */
  public AutoPlaceOnReef(DriveSubsystem driveSubsystem, ElevatorSubsystem elevatorSubsystem, CoralOuttakeSubsystem COSubsystem, DoubleSupplier x, DoubleSupplier y, DoubleSupplier rot, Supplier<String> position) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new DriveToPosition(driveSubsystem, true),
      // new PathPlannerAuto(position.get()),
      elevatorSubsystem.goToPosition()
      // new DriveCommand(driveSubsystem, x, y, rot, true, true)
    );
  }
}
