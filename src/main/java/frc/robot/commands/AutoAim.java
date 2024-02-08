package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;
import frc.robot.subsystems.ShootMaxxer;

public class AutoAim extends Command {
    // Called once the command ends or is interrupted.
    ShootMaxxer shooter;
    double angle;
    public AutoAim(ShootMaxxer b, double angle){
        shooter = b;
        this.angle = angle;
    }
    @Override
    public void initialize(){
        
    }
    @Override
    public void execute(){
        
    }
    @Override
    public void end(boolean ainterrupted)
    {
        shooter.pivotToAngle(angle);
    }

    // Returns true when the command should end.
  @Override
    public boolean isFinished()
    {
     return true;
     } 
}   
