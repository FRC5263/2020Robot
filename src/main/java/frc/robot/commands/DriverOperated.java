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
  MotorSubsystem intake;
  MotorSubsystem pivot;
  Joystick controller1 = new Joystick(0);
  Joystick controller2 = new Joystick(1);
  ShooterSubsystem shooter;
  MotorSubsystem kicker;

  /**
   * Creates a new DriverOperated.
   */
  public DriverOperated(DriveTrainSubsystem driveTrain, MotorSubsystem conveyor, ShooterSubsystem shooter,
      MotorSubsystem intake, MotorSubsystem kicker) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.driveTrain = driveTrain;
    this.conveyor = conveyor;
    this.intake = intake;
    this.shooter = shooter;
    this.kicker = kicker;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // driving: left stick
    double forwardSpeed = controller1.getRawAxis(5);
    double horizontalSpeed = controller1.getRawAxis(4);
    double rotation = controller1.getRawAxis(0);
    
    double conveyorTrigger = controller2.getRawAxis(3);
    controller2.setRumble(RumbleType.kRightRumble, conveyorTrigger);
    conveyor.powerMotor(conveyorTrigger);
    
    double shooterTrigger = controller2.getRawAxis(2);
    controller2.setRumble(RumbleType.kLeftRumble, shooterTrigger);
    shooter.ShootBall(shooterTrigger);

    double aim = controller2.getRawAxis(1);
    shooter.Aim(aim);
    controller2.setRumble(RumbleType.kRightRumble, shooterTrigger);
    
    
    //Slow down driving controls
    boolean slowButton = controller1.getRawButton(2);
    Double slowDownFactor = .3;
    if(slowButton){
      driveTrain.drive(slowDownFactor * horizontalSpeed, slowDownFactor * -forwardSpeed, slowDownFactor * rotation);
    }else{
      driveTrain.drive(horizontalSpeed, -forwardSpeed, rotation);
    }

    boolean intakeButton = controller2.getRawButton(6);
    if (intakeButton) {
      intake.powerMotor(-.3);
      controller2.setRumble(RumbleType.kLeftRumble, 1);
    } else {
      intake.powerMotor(0);
      controller2.setRumble(RumbleType.kLeftRumble, 0);
    }

    int dpadDirection = controller2.getPOV();
    if (dpadDirection == 0) {
      conveyor.powerMotor(1);
    } else if (dpadDirection == 180) {
      conveyor.powerMotor(-1);
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
