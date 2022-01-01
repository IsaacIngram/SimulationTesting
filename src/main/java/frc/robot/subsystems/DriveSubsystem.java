// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotGearing;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotMotor;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotWheelSize;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SPI;

public class DriveSubsystem extends SubsystemBase {

  DifferentialDrive differentialDrive;
  CANSparkMax frontLeftSparkMax;
  CANSparkMax frontRightSparkMax;
  CANSparkMax rearLeftSparkMax;
  CANSparkMax rearRightSparkMax;
  MotorControllerGroup leftDrive;
  MotorControllerGroup rightDrive;
  RelativeEncoder frontLeftEncoder;
  RelativeEncoder frontRightEncoder;
  RelativeEncoder rearLeftEncoder;
  RelativeEncoder rearRightEncoder;
  AHRS navX;
  DifferentialDrivetrainSim driveSim;
  DifferentialDriveOdometry odometry;
  Field2d field2d;
  

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

    // Instantiate our encoders
    frontLeftEncoder = frontLeftSparkMax.getEncoder();
    frontRightEncoder = frontRightSparkMax.getEncoder();
    rearLeftEncoder = rearLeftSparkMax.getEncoder();
    rearRightEncoder = rearRightSparkMax.getEncoder();

    // Instantiate the NavX
    navX = new AHRS(SPI.Port.kMXP);

    // Instantiate the differential drive
    differentialDrive = new DifferentialDrive(leftDrive, rightDrive);

    // Instantiate simulation components
    driveSim = DifferentialDrivetrainSim.createKitbotSim(KitbotMotor.kDoubleNEOPerSide, KitbotGearing.k10p71, KitbotWheelSize.kSixInch, null);
    
    // Instantiate robot navigation components
    odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getOrientation()));
    field2d = new Field2d();

  }

  /**
   * Set the speed of the drivetrain
   * @param speed
   */
  public void setSpeed(double speed) {
    differentialDrive.tankDrive(speed, speed);
  }

  /**
   * Set the power of the left and right sides of the drivetrain independently
   * @param leftSpeed
   * @param rightSpeed
   */
  public void setSpeed(double leftSpeed, double rightSpeed) {
    differentialDrive.tankDrive(leftSpeed, rightSpeed);
  }

  /**
   * Stop the drive motors
   */
  public void stopMotors() {
    differentialDrive.tankDrive(0, 0);
  }

  /**
   * Get the left side encoder position
   * @return
   */
  public double getLeftPosition() {
    return (frontLeftEncoder.getPosition()+rearLeftEncoder.getPosition())/2;
  }

  /**
   * Get the right side encoder position
   * @return
   */
  public double getRightPosition() {
    return (frontRightEncoder.getPosition()+rearRightEncoder.getPosition())/2;
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

    // Update the odometry based on sensor values
    odometry.update(Rotation2d.fromDegrees(getOrientation()), getLeftPosition(), getRightPosition());

    // Place our robot on the field based on our odometry
    field2d.setRobotPose(odometry.getPoseMeters());
  }

  @Override
  public void simulationPeriodic() {

    // Set the inputs to the drivesim based on the current robot voltage
    // Note: The motor controllers give us a negative output value because they are inverted, so we have to invert that value.
    driveSim.setInputs(-leftDrive.get()*RobotController.getInputVoltage(), -rightDrive.get()*RobotController.getInputVoltage());

    driveSim.update(0.02);

    // Update the NavX data
    setSimDoubleFromDeviceData("navX-Sensor[0]", "Yaw", driveSim.getHeading().getDegrees());

    // Update the Spark Max positions
    setSimDoubleFromDeviceData("SPARK MAX [0]", "Position", driveSim.getLeftPositionMeters());
    setSimDoubleFromDeviceData("SPARK MAX [1]", "Position", driveSim.getLeftPositionMeters());
    setSimDoubleFromDeviceData("SPARK MAX [2]", "Position", driveSim.getRightPositionMeters());
    setSimDoubleFromDeviceData("SPARK MAX [3]", "Position", driveSim.getRightPositionMeters());

    // Update the Spark Max applied outputs
    setSimDoubleFromDeviceData("SPARK MAX [0]", "Applied Output", leftDrive.get());
    setSimDoubleFromDeviceData("SPARK MAX [1]", "Applied Output", leftDrive.get());
    setSimDoubleFromDeviceData("SPARK MAX [2]", "Applied Output", rightDrive.get());
    setSimDoubleFromDeviceData("SPARK MAX [3]", "Applied Output", rightDrive.get());
  }

  /**
   * Modify the JSON of the specified object to set the SIM Double
   * @param deviceName
   * @param doubleName
   * @param value
   */
  public void setSimDoubleFromDeviceData(String deviceName, String doubleName, double value) {
    int device = SimDeviceDataJNI.getSimDeviceHandle(deviceName);
    SimDouble simDouble = new SimDouble(SimDeviceDataJNI.getSimValueHandle(device, doubleName));
    simDouble.set(value);
  }

}
