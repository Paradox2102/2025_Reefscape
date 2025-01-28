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

public class HopperSubsystem extends SubsystemBase {
  // motor
  private SparkFlex m_motor = new SparkFlex(28, MotorType.kBrushless);

  //absoulte encoder

  /** Creates a new WristSubsystem. */
  public HopperSubsystem() {
    m_motor.configure(MotorConfigs.Hopper.config, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  }

  public void setBrakeMode(boolean brake) {
    m_motor.configure(MotorConfigs.Hopper.config, ResetMode.kResetSafeParameters,
    PersistMode.kPersistParameters);
  }

  public Command runHopper() {
    return Commands.run(() -> {
      m_motor.set(0.5);
    });
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
