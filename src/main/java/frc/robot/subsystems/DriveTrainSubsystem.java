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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
  
  private static final double wheelDiameterInches = 6.0;
  private final static double encoderClicksPerRevolution = 360;

  /**
   * Creates a new DriveTrainSubsystem.
   */
  public DriveTrainSubsystem(SpeedController backRightMotor, SpeedController backLeftMotor, SpeedController frontRightMotor, SpeedController frontLeftMotor, Encoder backRightEncoder, Encoder backLeftEncoder, Encoder frontRightEcoder, Encoder frontLeftEncoder, AHRS navx) {
    this.backRightMotor = backRightMotor;
    this.backLeftMotor = backLeftMotor;
    this.frontLeftMotor = frontLeftMotor;
    this.frontRightMotor = frontRightMotor;
    this.backLeftEncoder = backLeftEncoder;
    this.backRightEncoder = backRightEncoder;
    this.frontLeftEncoder = frontLeftEncoder;
    this.frontRightEncoder = frontRightEcoder;
    this.navx = navx;
    this.driveTrain = new MecanumDrive(frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor);

    System.out.println("drivetrain subsystem intalized");

  
  }

  public DriveTrainSubsystem(SpeedController backRightMotor, SpeedController backLeftMotor, SpeedController frontRightMotor, SpeedController frontLeftMotor) {
    this.backRightMotor = backRightMotor;
    this.backLeftMotor = backLeftMotor;
    this.frontLeftMotor = frontLeftMotor;
    this.frontRightMotor = frontRightMotor;
    this.driveTrain = new MecanumDrive(frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor);

    System.out.println("drivetrain subsystem intalized");

  
  }

  public void drive(double horizontalSpeed, double forwardSpeed, double rotation) {
    driveTrain.driveCartesian(horizontalSpeed, forwardSpeed, rotation);
  }

  @Override
  public void periodic() {
    updatedashboard();
    // This method will be called once per scheduler run
  }
    public DriveTrainSubsystem() {
      navx.reset();
    }
  protected void updatedashboard() {
    SmartDashboard.putNumber("Back Left Encoder", backLeftEncoder.get());
    SmartDashboard.putNumber("Back Right Encoder", backRightEncoder.get());
    SmartDashboard.putNumber("Front Left Encoder", frontLeftEncoder.get());
    SmartDashboard.putNumber("Front Right Encoder", frontRightEncoder.get());
  }
}