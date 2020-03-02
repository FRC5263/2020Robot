/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.MecanumDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.MecanumDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.AnalogGyro;

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
  public DriveTrainSubsystem(SpeedController backRightMotor, SpeedController backLeftMotor,
      SpeedController frontRightMotor, SpeedController frontLeftMotor, Encoder backRightEncoder,
      Encoder backLeftEncoder, Encoder frontRightEcoder, Encoder frontLeftEncoder, AHRS navx) {
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

  public DriveTrainSubsystem(SpeedController backRightMotor, SpeedController backLeftMotor,
      SpeedController frontRightMotor, SpeedController frontLeftMotor) {
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
    if (frontLeftEncoder != null) {
      SmartDashboard.putNumber("Front Left Encoder", frontLeftEncoder.get());
    }
    if (frontRightEncoder != null) {
    SmartDashboard.putNumber("Front Right Encoder", frontRightEncoder.get());
    }
    if (backLeftEncoder != null) {
      SmartDashboard.putNumber("Back Left Encoder", backLeftEncoder.get());
    }
    if (backRightEncoder != null) {
      SmartDashboard.putNumber("Back Right Encoder", backRightEncoder.get());
    }
  }
//Need to find out how many meters per second the chassis goes
  public static final double kMaxSpeed = 3.0; //  meters per second
  public static final double kMaxAngularSpeed = Math.PI; // 1/2 rotation per second
//Need to change motors to correct motor types
  private final SpeedController m_frontLeftMotor = new PWMVictorSPX (1);
  private final SpeedController m_frontRightMotor = new PWMVictorSPX (2);
  private final SpeedController m_backLeftMotor = new PWMVictorSPX(3);
  private final SpeedController m_backRightMotor = new PWMVictorSPX(4);

  private final Encoder m_frontLeftEncoder = new Encoder(0, 1);
  private final Encoder m_frontRightEncoder = new Encoder(2, 3);
  private final Encoder m_backLeftEncoder = new Encoder(4, 5);
  private final Encoder m_backRightEncoder = new Encoder(6, 7);
//Need to put in values of how far each wheel is relative to the Robot's center
  private final Translation2d FrontLeftLocation = new Translation2d(0.4318, 0.4318);
  private final Translation2d FrontRightLocation = new Translation2d(0.4318, -0.4318);
  private final Translation2d BackLeftLocation = new Translation2d(-0.4318, 0.4318);
  private final Translation2d BackRightLocation = new Translation2d(-0.4318, -0.4318);

  private final PIDController m_frontLeftPIDController = new PIDController(1, 0, 0);
  private final PIDController m_frontRightPIDController = new PIDController(1, 0, 0);
  private final PIDController m_backLeftPIDController = new PIDController(1, 0, 0);
  private final PIDController m_backRightPIDController = new PIDController(1, 0, 0);

  private final AnalogGyro m_gyro = new AnalogGyro(0);

  private final MecanumDriveKinematics Kinematics = new MecanumDriveKinematics(FrontLeftLocation, FrontRightLocation,
      BackLeftLocation, BackRightLocation);

  private final MecanumDriveOdometry m_odometry = new MecanumDriveOdometry(Kinematics, getAngle());
  // Note that the values below for the SimpleMotor is an estimate and a guess,
  // and may need to be changed.
  private final SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(1,3);

  public Rotation2d getAngle() {

    return Rotation2d.fromDegrees(-m_gyro.getAngle());
  }
  // Since there's two encoders, this takes the ones that we have and mirrors them to the opposite
  public MecanumDriveWheelSpeeds getCurrentState() {
    return new MecanumDriveWheelSpeeds(
        backRightEncoder.getRate(),
        backLeftEncoder.getRate(),
        backLeftEncoder.getRate(),
        backRightEncoder.getRate()
        );
  }
  public void setSpeeds(MecanumDriveWheelSpeeds speeds) {
    final double frontLeftFeedforward = m_feedforward.calculate(speeds.frontLeftMetersPerSecond);
    final double frontRightFeedforward = m_feedforward.calculate(speeds.frontRightMetersPerSecond);
    final double backLeftFeedforward = m_feedforward.calculate(speeds.rearLeftMetersPerSecond);
    final double backRightFeedforward = m_feedforward.calculate(speeds.rearRightMetersPerSecond);

    final double frontLeftOutput = m_frontLeftPIDController.calculate(
        frontLeftEncoder.getRate(), speeds.frontLeftMetersPerSecond
    );
    final double frontRightOutput = m_frontRightPIDController.calculate(
        frontRightEncoder.getRate(), speeds.frontRightMetersPerSecond
    );
    final double backLeftOutput = m_backLeftPIDController.calculate(
        backLeftEncoder.getRate(), speeds.rearLeftMetersPerSecond
    );
    final double backRightOutput = m_backRightPIDController.calculate(
        backRightEncoder.getRate(), speeds.rearRightMetersPerSecond
    );

    frontLeftMotor.setVoltage(frontLeftOutput + frontLeftFeedforward);
    frontRightMotor.setVoltage(frontRightOutput + frontRightFeedforward);
    backLeftMotor.setVoltage(backLeftOutput + backLeftFeedforward);
    backRightMotor.setVoltage(backRightOutput + backRightFeedforward);
  }
  @SuppressWarnings("ParameterName")
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    var mecanumDriveWheelSpeeds = Kinematics
        .toWheelSpeeds(
        fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
            xSpeed, ySpeed, rot, getAngle()
        ) : new ChassisSpeeds(xSpeed, ySpeed, rot)
    );
    mecanumDriveWheelSpeeds.normalize(kMaxSpeed);
    setSpeeds(mecanumDriveWheelSpeeds);
  }
  public void updateOdometry() {
    m_odometry.update(getAngle(), getCurrentState());
  }
}