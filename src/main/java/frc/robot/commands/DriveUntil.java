/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import edu.wpi.first.wpilibj.Ultrasonic;

import edu.wpi.first.wpilibj.Gyro;

import java.frc.robot.subsystems.DriveTrainSubsystem;


public class DriveUntil extends CommandBase {
  Ultrasonic ultrasonicSensor;
  double distanceInches;
  DriveTrainSubsystem driveTrain;
  double XSpeed;
  /**
   * Creates a new DriveInterrupt.
   */
  public DriveUntil(Ultrasoniic ultrasonicSensor, double distanceInches, DriveTrainSubsystem driveTrain, double XSpeed) {
    this.ultrasonicSensor = ultrasonicSensor;
    this.distanceInches = distanceInches;
    this.driveTrain = driveTrain;
    this.XSpeed = XSpeed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    //implement logic 
    // IF sonic distance is higher than distanceInches, use DriveTrainSubsystem.drive() to drive the robot forwards
    if (ultrasonicSensor.getDistanceInches() >= distanceInches) {
      driveTrain.drive(XSpeed, 0, 0);
    }
    //ultrasonicSensor.getDistanceInches();
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
