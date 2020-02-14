/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.SpeedController;

public class MotorSubsystem extends SubsystemBase {
  
  private SpeedController motor;
  private double scalarPower = 1.0;

    /**
   * Basic motor power subsystem.
   * @param motor - SpeedController object to control motor
   */
  public MotorSubsystem(SpeedController motor) {
    this.motor = motor;
  }

  /**
   * Basic motor power subsystem.
   * @param motor - SpeedController object to control motor
   * @param scalarPower - limit power by giving a decimal value between 0 and 1. 
   */
  public MotorSubsystem(SpeedController motor, double scalarPower) {
    this.motor = motor;
    this.scalarPower = scalarPower;
  }

  public void powerMotor(double speed) {
    motor.set(speed * scalarPower);
  }

  /**
   * Models input power to the curve of power^(1/3) for inputs with dead zones close to zero. 
   */
  public void curvePowerMotor(double power){
    double adjustedPower = Math.pow(Math.abs(power), (1.0 / 3.0));
    if (power < 0) {
        adjustedPower *= -1;
    }
    this.motor.set(adjustedPower);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
