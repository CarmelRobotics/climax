package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.BTS;
import frc.robot.subsystems.Shooter;

public class RunBTS extends Command {
    // Called once the command ends or is interrupted.
    BTS bts;
    public RunBTS(BTS b){
        bts = b;
    }
    @Override
    public void initialize(){
        
    }
    @Override
    public void execute(){
        bts.set(0.25);
    }
    @Override
    public void end(boolean interrupted)
    {
    }

    // Returns true when the command should end.
  @Override
    public boolean isFinished()
    {
     return false;
     } 
}   
