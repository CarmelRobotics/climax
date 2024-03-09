package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter.ShooterState;

public class ShootNote extends Command {
    // Called once the command ends or is interrupted.
    Shooter shooter;
    double speed;
    public ShootNote(Shooter s, double x){
        shooter = s;
        speed = x;
        shooter.setShoot(ShooterState.SHOOTING);
    }
    @Override
    public void initialize(){
        
    }
    @Override
    public void execute(){
        shooter.shoot(-speed);
    }
    @Override
    public void end(boolean interrupted)
    {
        shooter.shoot(0);
        shooter.setShoot(ShooterState.DEFAULT);
    }

    // Returns true when the command should end.
  @Override
    public boolean isFinished()
    {
     return false;
     } 
}   
