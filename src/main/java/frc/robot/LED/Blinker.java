// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.LED;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDSubsystem;

import edu.wpi.first.wpilibj.Timer;

public class Blinker extends Command {
  private final LEDSubsystem m_subsystem;

  private int index = 0;

  private final Timer m_timer = new Timer();
  private Color m_color;

  private final double m_delay;

  /** Creates a new SpeakerAnim. */
  public Blinker(LEDSubsystem subsystem, double delay, Color color) {
    m_subsystem = subsystem;
    m_delay = delay;
    m_color = color;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timer.start();
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_delay == 0) {
      m_subsystem.setAllLEDs(m_color);
    } else {
      if (m_timer.get() >= m_delay) {
        m_timer.reset();
        index += 1;
        if (index > 1) {
          m_subsystem.setAllLEDs(Color.kBlack);
        } else {
          m_subsystem.setAllLEDs(m_color);
        }
        if (index > 2) {
          index = 0;
        }
      }
    }

    m_subsystem.commit();
  }
  /*
   *
   */

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

// index = math.mod(index+1)
