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

public class AlgaeSubsystem extends SubsystemBase {
  // motor
  private SparkFlex m_pivotMotor = new SparkFlex(0, MotorType.kBrushless);
  private SparkFlex m_rollerMotor = new SparkFlex(0, MotorType.kBrushless);

  private RelativeEncoder m_algaeEncoder = m_pivotMotor.getEncoder();

  /** Creates a new RollerSubsystem. */
  public AlgaeSubsystem() {
    m_pivotMotor.configure(MotorConfigs.Algae.pivotConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  }

  public void setBrakeMode(boolean brake) {
    m_pivotMotor.configure((brake ? MotorConfigs.Algae.pivotConfig : MotorConfigs.Algae.coastAlgaeConfig), ResetMode.kResetSafeParameters,
    PersistMode.kPersistParameters);
  }

  public void setPower(double power) {
    m_pivotMotor.set(power);
  }

  public double getAlgaePosition() {
    return m_algaeEncoder.getPosition();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Raw Algaw Position", getAlgaePosition());
    // This method will be called once per scheduler run
  }
}
