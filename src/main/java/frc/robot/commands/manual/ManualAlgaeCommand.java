// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.manual;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ManualAlgaeCommand extends Command {
  private AlgaeSubsystem m_subsystem;
  private DoubleSupplier m_position;
  /** Creates a new ManualAlgaeCommand. */
  public ManualAlgaeCommand(AlgaeSubsystem subsystem, DoubleSupplier position) {
    m_subsystem = subsystem;
    m_position = position;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_position.getAsDouble() < 0) {
      m_subsystem.setPower(-0.1);
    } else if (m_position.getAsDouble() > 0) {
      m_subsystem.setPower(0.1);
    } else {
      m_subsystem.setPower(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.setPoint(m_position.getAsDouble());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
