// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.MotorConfigs;

public class ClimberSubsystem extends SubsystemBase {
  // motor
  private SparkFlex m_leadMotor = new SparkFlex(ClimberConstants.k_climberMotor, MotorType.kBrushless);
  private SparkFlex m_followMotor = new SparkFlex(ClimberConstants.k_climberFollower, MotorType.kBrushless);
  private AbsoluteEncoder m_encoder;
  private SparkClosedLoopController m_pid;

  private ClimberState m_state = ClimberState.RESET;

  public enum ClimberState {
    RESET(140),
    EXTEND(50),
    CLIMB(170);

    double position;

    ClimberState(double position) {
      this.position = position;
    }

    public double position() {
      return position;
    }
  }

  /** Creates a new PivotSubsystem. */
  public ClimberSubsystem() {
    m_leadMotor.configure(MotorConfigs.Climber.config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_followMotor.configure(MotorConfigs.Climber.followConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_encoder = m_leadMotor.getAbsoluteEncoder();
    m_pid = m_leadMotor.getClosedLoopController();
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

  public double getAngle(){
    return m_encoder.getPosition();
  }

  public Command setPosition(ClimberState state) {
    return Commands.runOnce(() -> {
      m_pid.setReference(state.position(), ControlType.kPosition);
    }, this);
  }

  public Command runIn(){
    return Commands.startEnd(() -> {
      setPower(0.5);},
      () -> {
      setPower(0);},
     this);
  }

  public Command runOut(){
    return Commands.startEnd(() -> {
      setPower(-0.5);},
      () -> {
      setPower(0);},
     this);
  }

  public Command stop() {
    return Commands.runOnce(() -> {
      setPower(0);
    }, this);
  }

  @Override
  public void periodic() {}
}
