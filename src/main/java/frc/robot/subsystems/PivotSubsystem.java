// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.drive.RobotDriveBase.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PivotSubsystem extends SubsystemBase {
  //pid
  private double k_p = 0;
  private double k_i = 0;
  private double k_d = 0;
  private double k_f = 0;
  // private SparkFlex k_motor = new SparkFlex(0, MotorType.kBrushless);

  /** Creates a new PivotSubsystem. */
  public PivotSubsystem() {}

  public void setPower(double power) {}

  public double getAngleInDegrees() {
    return 0;
  }

  public void movePosition(double degrees) {} 
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
