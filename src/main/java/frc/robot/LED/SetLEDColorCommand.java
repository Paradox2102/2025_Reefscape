// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.LED;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.LEDSubsystem;

public class SetLEDColorCommand extends InstantCommand {
  private final LEDSubsystem m_subsystem;
  private final Color m_color;

  public SetLEDColorCommand(LEDSubsystem subsystem, Color color) {
    
    m_subsystem = subsystem;
    m_color = color;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_subsystem);
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {    
    m_subsystem.setAllLEDs(m_color);
    m_subsystem.commit();
    // m_subsystem.start();
  }
}
