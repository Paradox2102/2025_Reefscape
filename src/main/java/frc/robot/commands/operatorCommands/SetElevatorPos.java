// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.operatorCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorPosition;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SetElevatorPos extends InstantCommand {
  ElevatorSubsystem m_subsystem;
  ElevatorPosition m_position;
  public SetElevatorPos(ElevatorSubsystem elevatorSubsystem, ElevatorPosition position) {
    m_subsystem = elevatorSubsystem;
    m_position = position;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_subsystem.setPosition(m_position);
  }

  @Override
  public boolean runsWhenDisabled()
  {
      return true;
  }
}
