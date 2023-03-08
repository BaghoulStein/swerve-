package frc.robot.Autos;

import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.commands.FollowPathWithEvents;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.Drivebase;
import frc.robot.SwerveModuleConstants;

public final class Autos {

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }

  /**
   * Generates a trajectory for the robot to follow.
   *
   * @param swerve The swerve module to use for the trajectory.
   * @param trajectory The trajectory to follow.
   * @param isReversed Whether the robot should be reversed.
   * @return The command to follow the trajectory.
   */
  public static CommandBase exampleAuto(Drivebase swerve) {
    boolean onTheFly = false; // Use the defined path from PathPlanner
    PathPlannerTrajectory trajectory;
    if (onTheFly) {
      // Simple path with holonomic rotation. Stationary start/end. Max velocity of 4
      // m/s and max accel of 3 m/s^2
      trajectory = PathPlanner.generatePath(
          new PathConstraints(4, 3),
          new PathPoint(new Translation2d(0, 0), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0)),
          // position, heading(direction of travel), holonomic rotation
          new PathPoint(new Translation2d(3, 5), Rotation2d.fromDegrees(90), Rotation2d.fromDegrees(90)),
          // position, heading(direction of travel), holonomic rotation
          new PathPoint(new Translation2d(5, 5), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0))
      // position, heading(direction of travel), holonomic rotation
      );
    } else {
      // List<PathPlannerTrajectory> trajectories = PathPlanner.loadPathGroup("2 piece", new PathConstraints(4, 3));
      trajectory = PathPlanner.loadPath("test path", new PathConstraints(4, 3));
      HashMap<String, Command> eventMap = new HashMap<>();
      eventMap.put("event1", new PrintCommand("Passed marker 1"));
      eventMap.put("event2", new InstantCommand());

      SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
          swerve::getPose,
          swerve::resetOdometry,
          new PIDConstants(SwerveModuleConstants.yAutoPID.getP(), SwerveModuleConstants.yAutoPID.getI(),
              SwerveModuleConstants.yAutoPID.getI()),
          new PIDConstants(SwerveModuleConstants.angleAutoPID.getP(),
              SwerveModuleConstants.angleAutoPID.getI(),
              SwerveModuleConstants.angleAutoPID.getD()),
          swerve::setChassisSpeeds,
          eventMap,
          swerve);
      return Commands.sequence(autoBuilder.fullAuto(trajectory));
      // return new FollowPathWithEvents(new FollowTrajectory(swerve, trajectory, false), trajectory.getMarkers(), eventMap);
    }

    return Commands.sequence(swerve.followTrajectory(trajectory, true));
  }

}