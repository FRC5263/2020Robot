package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Wait;
import com.kauailabs.navx.frc.AHRS;





/**
 * A complex auto command that drives forward, releases a hatch, and then drives backward.
 */
public class CommandGroup extends SequentialCommandGroup {
  private AHRS Navx;
  
  /**
   * Creates a new CommandGroup.
 * @param <Navx>
   *
   *
   */
  public <Navx> CommandGroup(Wait wait, Navx navx) {
    addCommands(  
       ((AHRS) navx).reset(),
       new Wait(),
    
       );
  }

  

}