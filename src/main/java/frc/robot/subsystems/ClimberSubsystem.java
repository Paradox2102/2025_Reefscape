// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.MotorConfigs;

public class ClimberSubsystem extends SubsystemBase {
  // motor
  private SparkFlex m_leadMotor = new SparkFlex(Constants.ClimberConstants.k_climberMotor, MotorType.kBrushless);
  private SparkFlex m_followMotor = new SparkFlex(Constants.ClimberConstants.k_climberFollower, MotorType.kBrushless);
  private SparkClosedLoopController m_pid;
  private RelativeEncoder m_encoder;

  /** Creates a new PivotSubsystem. */
  public ClimberSubsystem() {
    m_leadMotor.configure(MotorConfigs.Climber.config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_followMotor.configure(MotorConfigs.Climber.followConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_pid = m_leadMotor.getClosedLoopController();
    m_encoder = m_leadMotor.getEncoder();
  }

  public void setBrakeMode(boolean brake) {
    // m_pivotMotor.setIdleMode(brake? IdleMode.kBrake : IdleMode.kCoast);
    m_leadMotor.configure((brake ? MotorConfigs.Climber.config : MotorConfigs.Climber.coastConfig), ResetMode.kResetSafeParameters,
    PersistMode.kPersistParameters);
    m_followMotor.configure((brake ? MotorConfigs.Climber.followConfig : MotorConfigs.Climber.fcConfig), ResetMode.kResetSafeParameters,
     PersistMode.kPersistParameters);
  }

  public void setPower(double power) {
    m_leadMotor.set(power);
  }

  public void setAngle(double degrees) {
    m_pid.setReference(degrees, ControlType.kPosition);
  }

  public Command climb(boolean up) {
    return Commands.run(() -> {
      setPower(up ? 90 : 0);
    }, this);
  }

  public Command stop() {
    return Commands.run(() -> {
      setPower(0);
    }, this);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
