// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorConfigs;

public class CoralOuttakeSubsystem extends SubsystemBase {
  // motor
  private SparkFlex m_coralLeftMotor = new SparkFlex(0, MotorType.kBrushless);
  private SparkFlex m_coralRightMotor = new SparkFlex(0, MotorType.kBrushless);
  // pid
  private double k_p = 0;
  private double k_i = 0;
  private double k_d = 0;
  private double k_f = 0;
  private PIDController m_PID = new PIDController(k_p, k_i, k_d);

  private RelativeEncoder m_LCoralEncoder = m_coralLeftMotor.getEncoder();
  private RelativeEncoder m_RCoralEncoder = m_coralRightMotor.getEncoder();

  /** Creates a new RollerSubsystem. */
  public CoralOuttakeSubsystem() {
    m_coralLeftMotor.configure(MotorConfigs.Roller.LCoralConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    m_coralRightMotor.configure(MotorConfigs.Roller.RCoralConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  }

  public void setBrakeMode(boolean brake) {
    m_coralLeftMotor.configure((brake ? MotorConfigs.Roller.LCoralConfig : MotorConfigs.Roller.coastLCoralConfig), ResetMode.kResetSafeParameters,
    PersistMode.kPersistParameters);
    m_coralRightMotor.configure((brake ? MotorConfigs.Roller.RCoralConfig : MotorConfigs.Roller.coastRCoralConfig), ResetMode.kResetSafeParameters,
    PersistMode.kPersistParameters);
  }

  public void setPower(double power) {
    m_coralLeftMotor.set(power);
    m_coralRightMotor.set(power);
  }

  public double getLCoralPosition() {
    return m_LCoralEncoder.getPosition();
  }

  public double getRCoralPosition() {
    return m_RCoralEncoder.getPosition();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Raw Left Motor Position", getLCoralPosition());
    SmartDashboard.putNumber("Raw Right Motor Position", getRCoralPosition());
    // This method will be called once per scheduler run
  }
}