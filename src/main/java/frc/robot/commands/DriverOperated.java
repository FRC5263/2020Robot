/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.MotorSubsystem;
import edu.wpi.first.wpilibj.GenericHID;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj.Joystick;

public class DriverOperated extends CommandBase {

  DriveTrainSubsystem driveTrain;
  MotorSubsystem conveyor;
  IntakeSubsystem intake;
  Joystick controller1 = new Joystick(0);
  Joystick controller2 = new Joystick(1);
  ShooterSubsystem shooter;

  /**
   * Creates a new DriverOperated.
   */
  public DriverOperated(DriveTrainSubsystem driveTrain, MotorSubsystem conveyor, IntakeSubsystem intake, ShooterSubsystem shooter) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.driveTrain = driveTrain;
    this.conveyor = conveyor;
    this.intake = intake;
    this.shooter = shooter;
  }


// Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double forwardSpeed = controller1.getRawAxis(5);
    double horizontalSpeed = controller1.getRawAxis(4);
    double rotation = controller1.getRawAxis(0);
    driveTrain.drive(horizontalSpeed, forwardSpeed, rotation);

    //  dont know how we are using controller so fix that soon "double intakeSpeed = controller2.getRawAxis();""
    //intake.powerMotor(intakeSpeed);


    
    double shooterTrigger = controller1.getRawAxis(2);
    controller1.setRumble(RumbleType.kRightRumble, shooterTrigger);

    shooter.ShootBall(shooterTrigger);


    int dpadDirection = controller1.getPOV();
    if (dpadDirection == 0) {
      conveyor.powerMotor(.30);
    } else if (dpadDirection == 180) {
      conveyor.powerMotor(-.30);
    } else {
      conveyor.powerMotor(0);
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.drive(0, 0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
