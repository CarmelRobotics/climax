package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShootMaxxer;

public class ShootSpeaker extends Command {
    // Called once the command ends or is interrupted.
    ShootMaxxer shooter;
    public ShootSpeaker(ShootMaxxer s){
        shooter = s;
    }
    @Override
    public void initialize(){
        
    }
    @Override
    public void execute(){
        shooter.autoShoot();
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
