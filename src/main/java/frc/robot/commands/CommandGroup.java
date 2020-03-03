package frc.robot.commands;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Wait;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.commands.AutonInit;

/**
 * A complex auto command that drives forward, releases a hatch, and then drives
 * backward.
 */
public class CommandGroup extends SequentialCommandGroup {

  /**
   * Creates a new CommandGroup
   * 
   *
   *
   */
  public  CommandGroup(DriveTrainSubsystem driveTrainSubsystem) {
    addCommands(  
       new Wait(),
       new AutonInit(driveTrainSubsystem)
       
       );
  }

  

}