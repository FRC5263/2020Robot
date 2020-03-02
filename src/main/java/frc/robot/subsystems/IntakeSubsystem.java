package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
    private Ultrasonic ultraSonic;
    /**
     * Creates a new ExampleSubsystem.
     */
    public IntakeSubsystem(Ultrasonic ultraSonic) {
        this.ultraSonic = ultraSonic;
        // Starts the ultrasonic sensor running in automatic mode
        ultraSonic.setAutomaticMode(true);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("UltraSonicReading", ultraSonic.getRangeInches()/12);
    // This method will be called once per scheduler run
  }
}
