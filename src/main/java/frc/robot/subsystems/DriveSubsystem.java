// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.SPI;

public class DriveSubsystem extends SubsystemBase {

  DifferentialDrive differentialDrive;
  CANSparkMax frontLeftSparkMax;
  CANSparkMax frontRightSparkMax;
  CANSparkMax rearLeftSparkMax;
  CANSparkMax rearRightSparkMax;
  MotorControllerGroup leftDrive;
  MotorControllerGroup rightDrive;
  AHRS navX;
  

  /** Creates a new Drivetrain. */
  public DriveSubsystem() {

    // Instantiate our motor controllers for the drivetrain
    frontLeftSparkMax = new CANSparkMax(0, MotorType.kBrushless);
    frontRightSparkMax = new CANSparkMax(1, MotorType.kBrushless);
    rearLeftSparkMax = new CANSparkMax(2, MotorType.kBrushless);
    rearRightSparkMax = new CANSparkMax(3, MotorType.kBrushless);

    // Invert the spark maxes
    frontLeftSparkMax.setInverted(true);
    frontRightSparkMax.setInverted(true);
    rearLeftSparkMax.setInverted(true);
    rearRightSparkMax.setInverted(true);

    // Instantiate our two motor controller groups
    leftDrive = new MotorControllerGroup(frontLeftSparkMax, frontRightSparkMax);
    rightDrive = new MotorControllerGroup(rearLeftSparkMax, rearRightSparkMax);

    // Instantiate the NavX
    navX = new AHRS(SPI.Port.kMXP);

    // Instantiate the differential drive
    differentialDrive = new DifferentialDrive(leftDrive, rightDrive);
  }

  /**
   * Set the speed of the drivetrain
   * @param speed
   */
  public void setSpeed(double speed) {
    leftDrive.set(speed);
    rightDrive.stopMotor();
  }

  /**
   * Set the power of the left and right sides of the drivetrain independently
   * @param leftSpeed
   * @param rightSpeed
   */
  public void setSpeed(double leftSpeed, double rightSpeed) {
    leftDrive.set(leftSpeed);
    rightDrive.set(rightSpeed);
  }

  /**
   * Stop the drive motors
   */
  public void stopMotors() {
    leftDrive.set(0);
    rightDrive.set(0);
  }

  /**
   * Get the robot's orientation according to the NavX
   * @return
   */
  public double getOrientation() {
    return navX.getYaw();
  }

  /**
   * Reset the NavX
   */
  public void resetNavX() {
    navX.reset();
  }

  @Override
  public void periodic() {
    
  }
}
