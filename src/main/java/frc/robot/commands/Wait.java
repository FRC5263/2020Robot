package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class Wait extends CommandBase {
  public Wait() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }
  long timeoutSeconds = System.currentTimeMillis();
  long end = 5000;
  boolean done = false;

  public void DoNothing () {
    System.out.println("Waiting...");
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    timeoutSeconds = System.currentTimeMillis();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
      if (timeoutSeconds < end) {
          done = false;
      }
      else {
          done = true;
      }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return done;
  }

}