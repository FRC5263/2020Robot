/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DialSubsystem extends SubsystemBase {
    ColorSensorV3 colorSensor;
    final ColorMatch m_colorMatcher = new ColorMatch();
    final Color kBlueTarget = ColorMatch.makeColor(0.136, 0.434, 0.432);
    final Color kGreenTarget = ColorMatch.makeColor(0.192, 0.561, 0.245);
    final Color kRedTarget = ColorMatch.makeColor(0.521, 0.338, 0.125);
    final Color kYellowTarget = ColorMatch.makeColor(0.331, 0.552, 0.116);
    final Color kBlackTarget = ColorMatch.makeColor(0.307, 0.474, .223);

    public DialSubsystem(ColorSensorV3 colorSensor) {

        this.colorSensor = colorSensor;
        m_colorMatcher.addColorMatch(kBlueTarget);
        m_colorMatcher.addColorMatch(kGreenTarget);
        m_colorMatcher.addColorMatch(kRedTarget);
        m_colorMatcher.addColorMatch(kYellowTarget);
        m_colorMatcher.addColorMatch(kBlackTarget);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run

        Color detectedColor = colorSensor.getColor();
        String colorString;
        ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);

        if (match.color == kBlueTarget) {
            colorString = "Blue";
        } else if (match.color == kRedTarget) {
            colorString = "Red";
        } else if (match.color == kGreenTarget) {
            colorString = "Green";
        } else if (match.color == kYellowTarget) {
            colorString = "Yellow";
        } else if (match.color == kBlackTarget) {
            colorString = "Black";
        } else {
            colorString = "Unknown";
        }
        SmartDashboard.putNumber("Red", detectedColor.red);
        SmartDashboard.putNumber("Green", detectedColor.green);
        SmartDashboard.putNumber("Blue", detectedColor.blue);
        SmartDashboard.putNumber("Confidence", match.confidence);
        SmartDashboard.putString("Detected Color", colorString);
    }
}
