package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
    private AnalogInput ultraSonicInput;
    /**
     * Creates a new ExampleSubsystem.
     */
    public IntakeSubsystem(AnalogInput ultraSonicInput) {
        this.ultraSonicInput = ultraSonicInput;
  }

  @Override
  public void periodic() {
    ultraSonicInput.getValue();
    SmartDashboard.putNumber("UltraSonicReading", ultraSonicInput.getValue()*0.125);
    // This method will be called once per scheduler run
  }
}
