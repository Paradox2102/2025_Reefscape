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

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.MotorConfigs;

public class HopperSubsystem extends SubsystemBase {
  // motor
  private SparkFlex m_motor = new SparkFlex(Constants.HopperConstants.k_hopperMotor, MotorType.kBrushless);
  private RelativeEncoder m_encoder;
  DigitalInput m_beamBreak = new DigitalInput(1);

  private SparkClosedLoopController m_PID = m_motor.getClosedLoopController();

  private static final double k_intakeRPM = 3000;

  //absoulte encoder

  /** Creates a new WristSubsystem. */
  public HopperSubsystem() {
    m_motor.configure(MotorConfigs.Hopper.config, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    m_encoder = m_motor.getEncoder();
  }

  public void setBrakeMode(boolean brake) {
    m_motor.configure(MotorConfigs.Hopper.config, ResetMode.kResetSafeParameters,
    PersistMode.kPersistParameters);
  }

  public boolean getBeamBreak() {
    return m_beamBreak.get();
  }

  public void setPower(double power) {
    m_motor.set(power);
  }

  public void intake() {
    m_PID.setReference(k_intakeRPM, ControlType.kVelocity);
  }

  public Command runHopper(double power) {
    return Commands.run(() -> {
      m_motor.set(power);
    }, this);
  }

  public Command stop() {
    return Commands.runOnce(() -> {
      m_motor.set(.4);
    }, this);
  }

  public double getCurrentDraw() {
    return m_motor.getOutputCurrent();
  }

  public double getSpeed() {
    return m_encoder.getVelocity();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Hopper Current", getCurrentDraw());
    SmartDashboard.putNumber("Hopper Speed", getSpeed());
    // This method will be called once per scheduler run
  }
}
