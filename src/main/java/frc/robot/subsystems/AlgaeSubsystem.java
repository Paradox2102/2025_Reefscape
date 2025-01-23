// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorConfigs;

public class AlgaeSubsystem extends SubsystemBase {
  // motor
  private SparkFlex m_pivotMotor = new SparkFlex(0, MotorType.kBrushless);
  private SparkFlex m_rollerMotor = new SparkFlex(0, MotorType.kBrushless);

  private static final double k_intakePower = .5;
  private static final double k_outtakePower = .5;
  private static final double k_holdAlgaePower = 0;

  private static final double k_resetPositionDegrees = 0;
  private static final double k_intakePositionDegrees = 0;
  private static final double k_outtakePositionDegrees = 0;

  private static final PIDController m_pivotPID = new PIDController(1, 0, 0);

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

  public void setPoint(double pos) {
    m_pivotPID.setSetpoint(pos);
  }

  public double getPivotPosition() {
    return m_algaeEncoder.getPosition();
  }

  public Command intake() {
    return Commands.run(() -> {
      m_rollerMotor.set(k_intakePower);
      m_pivotPID.setSetpoint(k_intakePositionDegrees);
    }, this);
  }

  public Command outtake() {
    return Commands.run(() -> {
      m_rollerMotor.set(k_outtakePower);
      m_pivotPID.setSetpoint(k_outtakePositionDegrees);
    }, this);
  }

  public Command reset() {
    return Commands.run(() -> {
      m_rollerMotor.set(k_holdAlgaePower);
      m_pivotPID.setSetpoint(k_resetPositionDegrees);
    }, this);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Raw Algae Position", getPivotPosition());
    // This method will be called once per scheduler run
    double power = MathUtil.applyDeadband(m_pivotPID.calculate(getPivotPosition()), .1);
    m_pivotMotor.set(power);
  }
}