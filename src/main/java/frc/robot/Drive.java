// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class Drive extends CommandBase {
  Chassis chassis = Chassis.getInstance();
  public Drive() {
    addRequirements(chassis);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    chassis.drive(RobotContainer.getRawAxis(1) * SwerveModuleConstants.freeSpeedMetersPerSecond,
        -RobotContainer.getRawAxis(0) * SwerveModuleConstants.freeSpeedMetersPerSecond,
        RobotContainer.getRawAxis(2) * 10);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    chassis.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}