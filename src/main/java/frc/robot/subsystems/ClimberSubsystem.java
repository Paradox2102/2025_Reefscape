// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.MotorConfigs;

public class ClimberSubsystem extends SubsystemBase {
  // motor
  private SparkFlex m_leadMotor = new SparkFlex(Constants.ClimberConstants.k_climberMotor, MotorType.kBrushless);
  private SparkFlex m_followMotor = new SparkFlex(Constants.ClimberConstants.k_climberFollower, MotorType.kBrushless);
  private PIDController m_pid = new PIDController(ClimberConstants.k_p, ClimberConstants.k_i, ClimberConstants.k_d);
  private DutyCycleEncoder m_encoder = new DutyCycleEncoder(9);

  /** Creates a new PivotSubsystem. */
  public ClimberSubsystem() {
    m_leadMotor.configure(MotorConfigs.Climber.config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_followMotor.configure(MotorConfigs.Climber.followConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
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
    System.out.println(String.format("%s, %f", "running motor at", power));
  }

  public void setAngle(double degrees) {
    m_pid.setSetpoint(degrees);
  }

  public double getAngle(){
    return (m_encoder.get() - Constants.ClimberConstants.k_resetPosition)*Constants.ClimberConstants.k_ticksToDegrees;
  }

  // FIXME: Should use constants here. -Gavin
  public Command climb(boolean up) {
    // FIXME: Why not runOnce? -Gavin
    return Commands.run(() -> {
      setAngle(up ? -15 : 90);
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
    // FIXME: Why not runOnce? -Gavin
    return Commands.run(() -> {
      setPower(0);
    }, this);
  }

  @Override
  public void periodic() {
    double power = MathUtil.applyDeadband(m_pid.calculate(getAngle()), .1);
    //m_leadMotor.set(power);
    //System.out.println(power);
    //System.out.println(getAngle());
    // This method will be called once per scheduler run
  }
}
