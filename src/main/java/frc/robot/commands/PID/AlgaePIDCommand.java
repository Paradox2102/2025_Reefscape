// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.PID;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlgaePIDCommand extends Command {
  AlgaeSubsystem m_subsystem;
  /** Creates a new ElevatorPIDCommand. */
  public AlgaePIDCommand(AlgaeSubsystem subsystem) {
    m_subsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_subsystem.setPower(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_subsystem.configurePID(
    SmartDashboard.getNumber("Algae Roller F", 0),
    SmartDashboard.getNumber("Algae Roller P", 0),
    SmartDashboard.getNumber("Algae Roller I", 0),
    SmartDashboard.getNumber("Algae Roller D", 0),
    SmartDashboard.getNumber("Algae Pivot F", 0),
    SmartDashboard.getNumber("Algae Pivot P", 0),
    SmartDashboard.getNumber("Algae Pivot I", 0),
    SmartDashboard.getNumber("Algae Pivot D", 0));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("algae pid set");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
