// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.States;
import frc.robot.subsystems.DriveSubsystem;

public class DriveFaceReefCommand extends Command {

  private DriveSubsystem m_subsystem;
  private DoubleSupplier m_getX;
  private DoubleSupplier m_getY;
  private DoubleSupplier m_getRot;
  private boolean m_fieldRelative;
  private boolean m_slowMode = false;
  private final double m_slowModeCoefficient = .3;

  public DriveFaceReefCommand(DriveSubsystem driveSubsystem, DoubleSupplier getX, DoubleSupplier getY, DoubleSupplier getRot, boolean fieldRelative) {
    m_subsystem = driveSubsystem;
    m_getX = getX;
    m_getY = getY;
    m_getRot = getRot;
    m_fieldRelative = fieldRelative;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_subsystem);
  }

  public DriveFaceReefCommand(DriveSubsystem driveSubsystem, DoubleSupplier getX, DoubleSupplier getY, DoubleSupplier getRot, boolean fieldRelative, boolean slowMode) {
    this(driveSubsystem, getX, getY, getRot, fieldRelative); 
    m_slowMode = slowMode;
  }
  //
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double x = -MathUtil.applyDeadband(m_getX.getAsDouble(), Constants.DriveConstants.k_driveDeadband);
    double y = -MathUtil.applyDeadband(m_getY.getAsDouble(), Constants.DriveConstants.k_driveDeadband);
    double rot = States.m_autoAim ? m_subsystem.orientPID(() -> m_subsystem.getRotationalDistanceFromReef().getDegrees()).get().getDegrees() : m_getRot.getAsDouble();
    m_subsystem.drive(
      m_slowMode ? m_slowModeCoefficient * y : y, 
      m_slowMode ? m_slowModeCoefficient * x : x, 
      rot, 
      m_fieldRelative, 
      true
      );
  }
  
  

  // Called once the command ends or is interrupted.
  public void end() {
    m_subsystem.drive(0, 0, 0, true, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
