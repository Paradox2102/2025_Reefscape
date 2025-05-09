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
  private double m_angle = 0;
  private double m_angleDiff = 0;
  private double m_totalAngle = 0;
  private Timer m_dwellTimer = new Timer();
  private double m_speed;
  private boolean m_isFinished;
  /** Creates a new TestGyroCommand. */
  public TestGyroCommand(DriveSubsystem subsystem, double speed) {
    m_subsystem = subsystem;
    m_speed = speed;
    addRequirements(m_subsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_totalAngle = 0;
    m_dwellTimer.stop();
    m_dwellTimer.reset();
    m_isFinished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_subsystem.drive(0, 0, m_speed, true, true);
    m_angleDiff = Math.abs(ParadoxField.normalizeAngle(m_angle - m_subsystem.getPose().getRotation().getDegrees()));
    m_angle = ParadoxField.normalizeAngle(m_subsystem.getPose().getRotation().getDegrees());
    m_totalAngle += m_angleDiff;
    if (Math.abs(m_totalAngle) >= 400){
      m_isFinished = true;
    }
    if(m_isFinished){
      m_subsystem.stop();
      m_dwellTimer.start();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_dwellTimer.get() > 1;
  }
}
