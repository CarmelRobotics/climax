package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class PivotManual extends Command {
    // Called once the command ends or is interrupted.
    Shooter shooter;
    double speed;
    public PivotManual(Shooter s, double x){
        shooter = s;
        speed = x;
    }
    @Override
    public void initialize(){
        
    }
    @Override
    public void execute(){
        shooter.pivot(speed);
    }
    @Override
    public void end(boolean interrupted)
    {
        shooter.pivot(0);
    }

    // Returns true when the command should end.
  @Override
    public boolean isFinished()
    {
     return false;
     } 
}   
