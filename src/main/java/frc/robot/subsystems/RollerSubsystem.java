// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

// import edu.wpi.first.wpilibj.drive.RobotDriveBase.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class RollerSubsystem extends SubsystemBase {
  //motor
  // private SparkFlex motor = new SparkFlex(0, MotorType.kBrushless);
  //pid
  private double k_p = 0;
  private double k_i = 0;
  private double k_d = 0;
  private double k_f = 0;
  /** Creates a new RollerSubsystem. */
  public RollerSubsystem() {}
  
  public void setPower(double power) {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
