// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class RobotContainer {
  public static final SwerveModuleConstants FLModule = new SwerveModuleConstants(2, 3,
      SwerveModuleConstants.cancoderTLOffset, 10, false, false);
  public static final SwerveModuleConstants FRModule = new SwerveModuleConstants(4, 5,
      SwerveModuleConstants.cancoderTROffset, 11, false, false);
  public static final SwerveModuleConstants BLModule = new SwerveModuleConstants(6, 7,
      SwerveModuleConstants.cancoderBLOffset, 12, false, false);
  public static final SwerveModuleConstants BRModule = new SwerveModuleConstants(8, 9,
      SwerveModuleConstants.cancoderBROffset, 13, false, false);


  // public static final SwerveModuleConstants FLModule = new SwerveModuleConstants(8, 9,
  //     SwerveModuleConstants.cancoderBROffset, 13, false, false);

  public static final CommandPS5Controller controller = new CommandPS5Controller(0);

  public RobotContainer() {
    // swerve = Drivebase.getInstance();
    configureBindings();
  }

  /**
   * Configures the bindings for the controller.
   */
  private void configureBindings() {
    controller.cross().onTrue(new InstantCommand(() -> Drivebase.getInstance().resetGyro()));

  }

  /**
   * Returns the state of the swerve module based on the controller input.
   * @return the state of the swerve module based on the controller input.
   */
  public static SwerveModuleState stateFromController() {
    double y = -controller.getLeftY();
    double x = controller.getLeftX();
    if (Math.abs(x) < 0.1)
      x = 0;
    if (Math.abs(y) < 0.1)
      y = 0;

    return new SwerveModuleState(Math.sqrt(x * x + y * y) * 0.5, new Rotation2d(x, y));
  }

  /**
   * Returns the speeds of the chassis based on the controller input.
   * @return the speeds of the chassis based on the controller input.
   */
  public static ChassisSpeeds speedsFromController() {
    double x = controller.getLeftX();
    double y = -controller.getLeftY();
    double rot = controller.getRightX();

    if (Math.abs(x) < 0.2)
      x = 0;
    if (Math.abs(y) < 0.2)
      y = 0;
    if (Math.abs(rot) < 0.2)
      rot = 0;

    return new ChassisSpeeds(1.5 * x, 1.5 * y, rot);
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

  /**
   * Calculates the deadband for a given value.
   * @param value The value to calculate the deadband for.
   * @return The deadband for the given value.
   */
  public static double calculateDeadBand(double value) {
    if (Math.abs(value) < 0.2)
      return 0;
    return value;
  }

  /**
   * Calculates the raw axis value, applying deadband and scaling.
   * @param axis the raw axis value to calculate deadband and scaling for
   * @return the axis value after applying deadband and scaling
   */
  public static double getRawAxis(int axis) {
    return calculateDeadBand(controller.getRawAxis(axis));
  }
}
