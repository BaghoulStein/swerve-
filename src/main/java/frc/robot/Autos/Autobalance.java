package frc.robot.Autos;

import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.Drivebase;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Autobalance extends PIDCommand {
  //TODO: DO THIS 
  public Autobalance() {
    super(
        // The controller that the command will use
        Constants.AutobalancePIDF.createPIDController(),
        // This should return the measurement
        Drivebase.getInstance()::getPitch,
        // This should return the setpoint (can also be a constant)
        () -> 0,

        // This uses the output
        output -> {
          // Use the output here
        });
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
