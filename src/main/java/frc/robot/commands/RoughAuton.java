/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.MotorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * An example command that uses an example subsystem.
 */
public class RoughAuton extends CommandBase {
    private final Timer m_timer = new Timer();
    private DriveTrainSubsystem driveTrainSubsystem;
    private MotorSubsystem conveyor;
    private ShooterSubsystem shooter;
    private MotorSubsystem intake;

    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public RoughAuton(DriveTrainSubsystem subsystem, MotorSubsystem conveyor, ShooterSubsystem shooter, MotorSubsystem intake) {
    driveTrainSubsystem = subsystem;
    this.conveyor = conveyor;
    this.shooter = shooter;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      m_timer.reset();
      m_timer.start();
    }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
          // Drive for 2 seconds
    if (m_timer.get() < 5.0) {
        driveTrainSubsystem.drive(0, 0.5, 0); // drive forwards half speed
      } else {
        driveTrainSubsystem.drive(0, 0, 0); // stop robot
      }
      conveyor.powerMotor(.5);
    shooter.ShootBall(.8);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
