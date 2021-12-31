// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class DriveTankCommand extends CommandBase {

  DoubleSupplier leftPower;
  DoubleSupplier rightPower;
  DriveSubsystem drivetrain;

  /** Creates a new DriveTank. */
  public DriveTankCommand(DoubleSupplier leftPower, DoubleSupplier rightPower, DriveSubsystem drivetrain) {
    this.leftPower = leftPower;
    this.rightPower = rightPower;
    this.drivetrain = drivetrain;
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetrain.setPower(leftPower.getAsDouble(), rightPower.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.stopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
