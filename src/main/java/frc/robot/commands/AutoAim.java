package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.BTS;
import frc.robot.subsystems.Shooter;

public class AutoAim extends Command {
    // Called once the command ends or is interrupted.
    Shooter shooter;
    double angle;
    public AutoAim(Shooter b, double angle){
        shooter = b;
        this.angle = angle;
    }
    @Override
    public void initialize(){
        
    }
    @Override
    public void execute(){
        shooter.pivotToAngle(angle);
    }
    @Override
    public void end(boolean ainterrupted)
    {
    }

    // Returns true when the command should end.
  @Override
    public boolean isFinished()
    {
     return false;
     } 
}   