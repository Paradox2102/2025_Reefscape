// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;

public class DriveCommand extends Command {
     /** Creates a new ArcadeDrive. */

  private DriveSubsystem m_subsystem;
  private DoubleSupplier m_getX;
  private DoubleSupplier m_getY;
  private DoubleSupplier m_getRot;
  private boolean m_slowmode;

  // ArcadeDrive is a means to control a differential drive, so this class is misnamed. -Gavin
  public DriveCommand(DriveSubsystem driveSubsystem, DoubleSupplier getX, DoubleSupplier getY, DoubleSupplier getRot, Trigger slowmode) {
    m_subsystem = driveSubsystem;
    m_getX = getX;
    m_getY = getY;
    m_getRot = getRot;
    m_slowmode = slowmode.getAsBoolean();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_subsystem);
  }
  //
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double oldRot = -MathUtil.applyDeadband(m_getRot.getAsDouble(), Constants.DriveConstants.k_driveDeadband);

    double x = -MathUtil.applyDeadband(m_getX.getAsDouble(), Constants.DriveConstants.k_driveDeadband);
    double y = -MathUtil.applyDeadband(m_getY.getAsDouble(), Constants.DriveConstants.k_driveDeadband);
    double rot = oldRot;
    if (m_slowmode) {
      x *= .3;
      y *= .3;
    }
    m_subsystem.drive(
      y, 
      x, 
      rot, 
      true, 
      true,
      new Translation2d(0,0)
      );

    
    // m_swerve.setModuleStates(m_defaultState);
    // System.out.println(String.format("x=%f, y=%f, rot=%f, isFieldRelative=%b", x, y, rot, isFieldRelative)); 
  }
  
  

  // Called once the command ends or is interrupted.
  public void end() {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
