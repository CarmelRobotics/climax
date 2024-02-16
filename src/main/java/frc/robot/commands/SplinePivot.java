package frc.robot.commands;

import edu.wpi.first.math.proto.Spline;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Shooter;
import frc.robot.subsystems.ShootMaxxer;

public class SplinePivot extends Command {
    // Called once the command ends or is interrupted.
    ShootMaxxer shooter;
    Timer timey;
    double amountMove;
    double timeMove;
    double endAngle;
    public SplinePivot(ShootMaxxer s, double targetAngle){
        shooter = s;
        timey = new Timer();
        amountMove = targetAngle - shooter.getDegree();
        timeMove = Math.abs(amountMove * shooter.secondsPerDegree);
        endAngle = targetAngle;
    }
    @Override
    public void initialize(){
        timey.start();
    }
    @Override
    public void execute(){
        shooter.pivot(0.2);
    }
    @Override
    public void end(boolean interrupted)
    {
        shooter.pivot(0);
        timey.stop();
        timey.reset();
        shooter.setAngle(endAngle);
    }

    // Returns true when the command should end.
  @Override
    public boolean isFinished()
    {
     return timey.get() >= timeMove;
     } 
}
