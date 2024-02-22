package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.BTS;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveSubsystem;

public class setHeadingCorrection extends Command {
    // Called once the command ends or is interrupted.
    SwerveSubsystem swerve;
    boolean state;
    public setHeadingCorrection(SwerveSubsystem b, boolean s){
        swerve = b;
        state = s;
    }
    @Override
    public void initialize(){
        System.out.println("set heading correction to: " + state);
    }
    @Override
    public void execute(){
        
    }
    @Override
    public void end(boolean interrupted)
    {
        swerve.setHeadingCorrection(interrupted);
    }

    // Returns true when the command should end.
  @Override
    public boolean isFinished()
    {
     return true;
     } 
}   
