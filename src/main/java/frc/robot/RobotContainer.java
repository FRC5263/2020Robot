/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.RoughAuton;
import frc.robot.commands.Wait;
import frc.robot.subsystems.DialSubsystem;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.MotorSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Ultrasonic;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.commands.DriverOperated;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.VictorSP;
import com.kauailabs.navx.frc.AHRS;
import frc.robot.commands.GyroDriverOperated;
/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  private ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);
  private DialSubsystem dialSubsystem = new DialSubsystem(m_colorSensor);


  private SpeedController backRightMotor = new WPI_TalonSRX(5);
  private SpeedController backLeftMotor = new WPI_TalonSRX(3);
  private SpeedController frontRightMotor = new WPI_TalonSRX(4);
  private SpeedController frontLeftMotor = new WPI_TalonSRX(2);

  private AHRS navx = new AHRS();


  private DriveTrainSubsystem driveTrain = new DriveTrainSubsystem(backRightMotor, backLeftMotor, frontRightMotor, frontLeftMotor, navx);


  
  private SpeedController conveyorMotor = new WPI_VictorSPX(10);
  private MotorSubsystem conveyor = new MotorSubsystem(conveyorMotor);

  private SpeedController intakeMotor = new WPI_TalonSRX(7);
  private MotorSubsystem intake = new MotorSubsystem(intakeMotor);


  private SpeedController shooterMotor1 = new WPI_VictorSPX(6);
  private SpeedController shooterMotor2 = new VictorSP(0);
  private SpeedController shooterDirectionMotor = new WPI_VictorSPX(9);
  private ShooterSubsystem shooter = new ShooterSubsystem(shooterMotor1, shooterMotor2, shooterDirectionMotor);
  

  private SpeedController kickerMotor = new WPI_VictorSPX(8);
  private MotorSubsystem kicker = new MotorSubsystem(kickerMotor);

  private final Command m_autoCommand = new RoughAuton(driveTrain, conveyor, shooter, intake);
  
  // private MotorSubsystem dial = new MotorSubsystem(new VictorSP(1));
  // private DriverOperated m_teleOp = new DriverOperated(driveTrain, conveyor, shooter, intake, kicker);
  private GyroDriverOperated m_teleOp = new GyroDriverOperated(driveTrain, conveyor, shooter, intake, kicker);
  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoCommand;
  }

  
  public Command getTeleOpCommand() {
    return m_teleOp;
  }
}
