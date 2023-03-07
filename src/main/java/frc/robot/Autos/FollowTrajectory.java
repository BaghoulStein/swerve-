package frc.robot.Autos;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Drivebase;
import frc.robot.SwerveModuleConstants;

public class FollowTrajectory extends SequentialCommandGroup {

  public FollowTrajectory(Drivebase chassis, PathPlannerTrajectory trajectory, boolean resetOdometry) {

    addRequirements(chassis);
    if (resetOdometry)
      chassis.reset(trajectory.getInitialHolonomicPose());
    addCommands(
        new PPSwerveControllerCommand(
            trajectory,
            chassis::getPose,
            SwerveModuleConstants.xAutoPID.createPIDController(),
            SwerveModuleConstants.yAutoPID.createPIDController(),
            SwerveModuleConstants.angleAutoPID.createPIDController(),
            chassis::setChassisSpeeds,
            chassis));
  }
}
