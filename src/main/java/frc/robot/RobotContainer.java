/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.commands.DriveTankCommand;
import frc.robot.subsystems.DriveSubsystem;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final DriveSubsystem drivetrain = new DriveSubsystem();

  // Controllers
  private final XboxController driverController = new XboxController(0);

  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    // Set and start the default drive command
    drivetrain.setDefaultCommand(new DriveTankCommand(
      () -> driverController.getLeftY(),
      () -> driverController.getRightY(),
       drivetrain));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

    // Create voltage constraint
    var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(new SimpleMotorFeedforward(ConstantsPW.ksVolts, ConstantsPW.kvVoltSecondsPerMeter, ConstantsPW.kaVoltSecondsSquaredPerMeter), ConstantsPW.kDriveKinematics, 10);
    
    // Create trajector configuration based on the above voltage constraint
    TrajectoryConfig config = new TrajectoryConfig(ConstantsPW.kMaxSpeedMetersPerSecond, ConstantsPW.kMaxAccelerationMetersPerSecondSquared)
    .setKinematics(ConstantsPW.kDriveKinematics)
    .addConstraint(autoVoltageConstraint);

    // Generate a trajectory to follow
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
      // Start at the origin facing the +x direction
      new Pose2d(0, 0, new Rotation2d(0)),
      List.of(
        new Translation2d(1, 1),
        new Translation2d(2, -1)
      ),
      new Pose2d(3, 0, new Rotation2d(0)),
      config
    );

    // Generate a command based on our trajectory
    RamseteCommand ramseteCommand = new RamseteCommand(
      exampleTrajectory,
      drivetrain::getPose,
      new RamseteController(ConstantsPW.kRamseteB, ConstantsPW.kRamseteZeta),
      new SimpleMotorFeedforward(ConstantsPW.ksVolts, ConstantsPW.kvVoltSecondsPerMeter, ConstantsPW.kaVoltSecondsSquaredPerMeter),
      ConstantsPW.kDriveKinematics,
      drivetrain::getWheelSpeeds,
      new PIDController(ConstantsPW.kPDriveVel, 0, 0),
      new PIDController(ConstantsPW.kPDriveVel, 0,0),
      drivetrain::setVolts,
      drivetrain
    );

    return ramseteCommand;

  }
}
