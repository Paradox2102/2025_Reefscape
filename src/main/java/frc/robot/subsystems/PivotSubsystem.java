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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.MotorConfigs;

public class PivotSubsystem extends SubsystemBase {
  // motor
  private SparkFlex m_pivotMotor = new SparkFlex(Constants.AlgaeConstants.k_pivotMotor, MotorType.kBrushless);
  private RelativeEncoder m_pivotEncoder = m_pivotMotor.getEncoder();

  private static final double k_f = -.25; // for 25 amps

  private PivotStates m_state = PivotStates.RESET;

  private PIDController m_PID = new PIDController(0.1, 0, 0);

  private enum PivotStates {
    RESET(0),
    ALGAE(67),
    CLIMB(60);

    private double m_angleDegrees;

    private PivotStates(double angleDegrees) {
      m_angleDegrees = angleDegrees;
    }

    public double position() {
      return m_angleDegrees;
    }
  }

  /** Creates a new RollerSubsystem. */
  public PivotSubsystem() {
    m_pivotMotor.configure(MotorConfigs.Algae.pivotConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  }

  public void setBrakeMode(boolean brake) {
    m_pivotMotor.configure((brake ? MotorConfigs.Algae.pivotConfig : MotorConfigs.Algae.coastAlgaeConfig), ResetMode.kResetSafeParameters,
    PersistMode.kPersistParameters);
  }

  public double getAngle() {
    return m_pivotEncoder.getPosition();
  }

  public Command intake() {
    return Commands.runOnce(() -> {
      m_state = PivotStates.ALGAE;
    }, this);
  }

  public Command outtake() {
    return Commands.runOnce(() -> {
      m_state = PivotStates.ALGAE;
    }, this).finallyDo(() -> reset());
  }

  public void reset() {
    m_state = PivotStates.RESET;
  }

  public Command climb() {
    return Commands.runOnce(() -> {
      m_state = PivotStates.CLIMB;
    }, this);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_pivotMotor.set(m_PID.calculate(getAngle(), m_state.position()) + k_f);
    SmartDashboard.putNumber("Pivot Angle", getAngle());
  }
}