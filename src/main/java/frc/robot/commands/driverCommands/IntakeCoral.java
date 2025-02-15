// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.driverCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.CoralOuttakeSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.HopperSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IntakeCoral extends Command {
  /** Creates a new IntakeCoral. */
  CoralOuttakeSubsystem m_coSubsystem;
  HopperSubsystem m_hopperSubsystem;
  ElevatorSubsystem m_elevatorSubsystem;
  AlgaeSubsystem m_algaeSubsystem;

  private Timer m_timer = new Timer();
  private static final double k_intakeCurrent = 50;
  private static final double k_stopSpeed = 4000;
  private static final double k_intakeSpeed = 5000;


  public IntakeCoral(CoralOuttakeSubsystem COSubsystem, HopperSubsystem hopperSubsystem, ElevatorSubsystem elevatorSubsystem, AlgaeSubsystem algaeSubsystem) {
    m_coSubsystem = COSubsystem;
    m_hopperSubsystem = hopperSubsystem;
    m_elevatorSubsystem = elevatorSubsystem;
    m_algaeSubsystem = algaeSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_coSubsystem, m_hopperSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_elevatorSubsystem.resetPosition();
    m_algaeSubsystem.setPivotPosition(true);
    m_hopperSubsystem.intake();
    m_timer.reset();
    m_timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_coSubsystem.setSpeed(k_intakeSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
   m_timer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_coSubsystem.getSpeedMotorRPM() < k_stopSpeed && m_timer.get() > .5 && m_coSubsystem.getCurrentDraw() > k_intakeCurrent;
  }
}
