/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.SpeedController;

import edu.wpi.first.wpilibj.drive.MecanumDrive;

import edu.wpi.first.wpilibj.Encoder;

import com.kauailabs.navx.frc.AHRS;


public class DriveTrainSubsystem extends SubsystemBase {
  

  private MecanumDrive driveTrain;
  private AHRS navx;
  private Encoder backRightEncoder;
  private Encoder backLeftEncoder;
  private Encoder frontRightEncoder;
  private Encoder frontLeftEncoder;
  private SpeedController backRightMotor;
  private SpeedController backLeftMotor;
  private SpeedController frontRightMotor;
  private SpeedController frontLeftMotor;
  
  /**
   * Creates a new DriveTrainSubsystem.
   */
  public DriveTrainSubsystem(SpeedController backRightMotor, SpeedController backLeftMotor, SpeedController frontRightMotor, SpeedController frontLeftMotor) {
    this.backRightMotor = backRightMotor;
    this.backLeftMotor = backLeftMotor;
    this.frontLeftMotor = frontLeftMotor;
    this.frontRightMotor = frontRightMotor;
  
    this.driveTrain = new MecanumDrive(frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor);
  
  }

  public void drive(double ySpeed, double xSpeed, double zRotation) {
    driveTrain.driveCartesian(ySpeed, xSpeed, zRotation);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
