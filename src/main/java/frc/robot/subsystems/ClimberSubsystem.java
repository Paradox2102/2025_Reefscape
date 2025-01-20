// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorConfigs;

public class ClimberSubsystem extends SubsystemBase {
  // motor
  private SparkFlex m_pivotMotor = new SparkFlex(0, MotorType.kBrushless);

  /** Creates a new PivotSubsystem. */
  public ClimberSubsystem() {
    m_pivotMotor.configure(MotorConfigs.SwerveModule.drivingConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  }

  public void setBrakeMode(boolean brake) {
    // m_pivotMotor.setIdleMode(brake? IdleMode.kBrake : IdleMode.kCoast);
    m_pivotMotor.configure((brake ? MotorConfigs.Climber.config : MotorConfigs.Climber.coastConfig), ResetMode.kResetSafeParameters,
    PersistMode.kPersistParameters);
  }

  public void setPower(double power) {
    m_pivotMotor.set(power);
  }

  public Command climb(boolean up) {
    return Commands.run(() -> {
      setPower(up ? 1 : -1);
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
