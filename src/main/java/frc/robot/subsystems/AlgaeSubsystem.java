// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.MotorConfigs;

public class AlgaeSubsystem extends SubsystemBase {
  // motor
  private SparkFlex m_pivotMotor = new SparkFlex(Constants.AlgaeConstants.k_pivotMotor, MotorType.kBrushless);
  private SparkFlex m_rollerMotor = new SparkFlex(Constants.AlgaeConstants.k_algaeMotor, MotorType.kBrushless);

  private static final double k_downPower = 1;
  private static final double k_upPower = -.5;

  private boolean m_retracted = true;

  private RollerStates m_rollerState = RollerStates.HOLD;

  private enum RollerStates {
    INTAKE(-.5),
    OUTTAKE(.5),
    HOLD(.2);

    private double m_power;

    private RollerStates(double power) {
      m_power = power;
    }

    public double power() {
      return m_power;
    }
  }

  /** Creates a new RollerSubsystem. */
  public AlgaeSubsystem() {
    m_pivotMotor.configure(MotorConfigs.Algae.pivotConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    m_rollerMotor.configure(MotorConfigs.Algae.rollerConfig, ResetMode.kResetSafeParameters, 
        PersistMode.kPersistParameters);
  }

  public void setBrakeMode(boolean brake) {
    m_pivotMotor.configure((brake ? MotorConfigs.Algae.pivotConfig : MotorConfigs.Algae.coastAlgaeConfig), ResetMode.kResetSafeParameters,
    PersistMode.kPersistParameters);
    m_rollerMotor.configure((brake ? MotorConfigs.Algae.rollerConfig : MotorConfigs.Algae.coastAlgaeConfig), ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  }

  public void setPivotPosition(boolean retracted) {
    m_retracted = retracted;
  }

  public Command intake() {
    return Commands.run(() -> {
      m_rollerState = RollerStates.INTAKE;
      setPivotPosition(false);
    }, this).handleInterrupt(() -> reset()
    );
  }

  public Command outtake() {
    return Commands.run(() -> {
      m_rollerState = RollerStates.OUTTAKE;
      setPivotPosition(false);
    }, this).finallyDo(() -> reset()
    );
  }

  public void reset() {
    m_retracted = true;
    m_rollerState = RollerStates.HOLD;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_pivotMotor.set(m_retracted ? k_upPower : k_downPower);
    m_rollerMotor.set(m_rollerState.power());
  }
}