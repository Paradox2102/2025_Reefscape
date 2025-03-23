// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PrecisionAlignOdometrey extends InstantCommand {
  String[] redPaths = {"12", "1", "6", "7", "2", "3"};
  DriveSubsystem m_subsystem;

  public PrecisionAlignOdometrey(DriveSubsystem driveSubsystem) {
    m_subsystem = driveSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    String name = m_subsystem.getReefPosition().getName();
    boolean runOtherPath = flip(name);
    // System.out.println("Pos "+name+(flip(name) ? " Red" : ""));
    System.out.println(runOtherPath);
    new PathPlannerAuto("Pos "+name+(runOtherPath ? " Red" : "")).schedule();
  }

  public boolean flip(String name) {
    boolean allianceRed = Constants.States.m_alliance == Alliance.Red;
    if (!allianceRed) {
      return false;
    } else {
      for (int i = 0; i < redPaths.length; i++) {
        if (redPaths[i].equals(name)) {
          return true;
        }
      } 
      return false;
    }
  }
}
