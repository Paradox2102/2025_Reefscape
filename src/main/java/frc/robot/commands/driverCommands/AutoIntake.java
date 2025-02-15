// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.driverCommands;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.Constants;
import frc.robot.commands.drive.DriveToPosition;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.CoralOuttakeSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.HopperSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoIntake extends ParallelDeadlineGroup {
  /** Creates a new AutoIntake. */
  public AutoIntake(DriveSubsystem driveSubsystem, CoralOuttakeSubsystem coralOuttakeSubsystem, ElevatorSubsystem elevatorSubsystem, AlgaeSubsystem algaeSubsystem, HopperSubsystem hopperSubsystem) {
    // Add the deadline command in the super() call. Add other commands using
    // addCommands().
    super(new IntakeCoral(coralOuttakeSubsystem, hopperSubsystem, elevatorSubsystem, algaeSubsystem));
    addCommands(
      new DriveToPosition(driveSubsystem, false).unless(() -> !Constants.States.m_autoAim)
    );
  }
}
