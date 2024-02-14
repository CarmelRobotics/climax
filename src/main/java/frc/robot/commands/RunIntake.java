package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.BTS;
import frc.robot.subsystems.IntakeMaxxer;
import frc.robot.subsystems.ShootMaxxer;

public class RunIntake extends Command {
    // Called once the command ends or is interrupted.
    IntakeMaxxer intake;
    public RunIntake(IntakeMaxxer intake){
        this.intake = intake;
    }
    @Override
    public void initialize(){
        
    }
    @Override
    public void execute(){
        intake.runIntake(1);
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
