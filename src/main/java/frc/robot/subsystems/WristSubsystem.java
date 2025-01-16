// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorConfigs;

public class WristSubsystem extends SubsystemBase {
  // motor
  private SparkFlex m_wristMotor = new SparkFlex(0, MotorType.kBrushless);
  // pid (f term)
  private double k_p = 0;
  private double k_i = 0;
  private double k_d = 0;
  private double k_f = 0;
  private PIDController m_PID = new PIDController(k_p, k_i, k_d);

  private double m_power;
  private RelativeEncoder m_wristEncoder = m_wristMotor.getEncoder();

  //absoulte encoder

  /** Creates a new WristSubsystem. */
  public WristSubsystem() {
    m_wristMotor.configure(MotorConfigs.Wrist.wristConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  }

  public void setBrakeMode(boolean brake) {
    m_wristMotor.configure((brake ? MotorConfigs.Wrist.wristConfig : MotorConfigs.Wrist.coastWristConfig), ResetMode.kResetSafeParameters,
    PersistMode.kPersistParameters);
  }

  public double getWristPosition() {
    return m_wristEncoder.getPosition();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Raw Wrist Position", getWristPosition());
    // This method will be called once per scheduler run
  }
}
