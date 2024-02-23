package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class ShootNote extends Command {
    // Called once the command ends or is interrupted.
    Shooter shooter;
    public ShootNote(Shooter s){
        shooter = s;
    }
    @Override
    public void initialize(){
        
    }
    @Override
    public void execute(){
        shooter.shoot(1);
    }
    @Override
    public void end(boolean interrupted)
    {
        shooter.shoot(0);
    }

    // Returns true when the command should end.
  @Override
    public boolean isFinished()
    {
     return false;
     } 
}   
