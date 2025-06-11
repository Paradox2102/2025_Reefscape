// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.ParadoxField;
import frc.robot.subsystems.DriveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TestGyroCommand extends Command {
  private DriveSubsystem m_subsystem;
  private double m_speed;
  private double m_time;
  private boolean m_return;
  private Timer m_turnTimer;
  private double m_initialAngle;
  
  /** Creates a new TestGyroCommand. */
  public TestGyroCommand(DriveSubsystem subsystem, double speed, double time) {
    m_subsystem = subsystem;
    m_speed = speed;
    m_time = time;
    addRequirements(m_subsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_initialAngle = m_subsystem.getCameraAngleDegrees();
    m_return = false;
    m_turnTimer.reset();
    m_turnTimer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_subsystem.drive(0,0,m_return ? m_speed : -m_speed, false, true);
    if(m_turnTimer.get() > m_time){
      m_return = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(m_subsystem.getCameraAngleDegrees() - m_initialAngle) < 0.25 && m_return == true;
  }
}
