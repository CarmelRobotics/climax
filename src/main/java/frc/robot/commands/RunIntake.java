package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.BTS;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class RunIntake extends Command {
    // Called once the command ends or is interrupted.
    Intake intake;
    public RunIntake(Intake intake){
        this.intake = intake;
    }
    @Override
    public void initialize(){
        
    }
    @Override
    public void execute(){
        intake.runIntake(-1);
        //0.4 optimal speed
    }
    @Override
    public void end(boolean interrupted)
    {
        intake.runIntake(0);
    }

    // Returns true when the command should end.
  @Override
    public boolean isFinished()
    {
     return false;
     } 
}   
