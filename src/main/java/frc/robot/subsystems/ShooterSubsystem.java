/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SpeedController;

public class ShooterSubsystem extends SubsystemBase {

 private SpeedController shootMotor1;
 private SpeedController shootMotor2;
  private double speedMultiplier = 0.6;


  public ShooterSubsystem(SpeedController shootMotor1, SpeedController shootMotor2) {
    this.shootMotor1 = shootMotor1;
    this.shootMotor2 = shootMotor2;
  }

  public void ShootBall(double shootPower) {
    shootMotor1.set(-shootPower * speedMultiplier);
    shootMotor2.set(shootPower * speedMultiplier);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
