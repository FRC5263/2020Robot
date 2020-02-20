package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogInput;
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
    // This method will be called once per scheduler run
  }
}
