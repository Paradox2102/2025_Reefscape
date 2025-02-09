// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.driverCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralOuttakeSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ScoreCoral extends Command {
  CoralOuttakeSubsystem m_coSubsystem;

  private Timer m_timer = new Timer();
  private static final double k_current = 20;
  private static final double k_outtakePower = .5;
  
  /** Creates a new ScoreCoral. */
  public ScoreCoral(CoralOuttakeSubsystem COSubsystem) {
    m_coSubsystem = COSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_coSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timer.reset();
    m_timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_coSubsystem.setPower(k_outtakePower);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_timer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_coSubsystem.getCurrentDraw() < k_current && m_timer.get() > .25;
  }
}
