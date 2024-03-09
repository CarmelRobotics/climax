package frc.robot.commands;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.WPI_PigeonIMU;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.BTS;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveSubsystem;

public class ZeroGyro extends Command {
    // Called once the command ends or is interrupted.
    SwerveSubsystem swerve;
    // PigeonIMU pigeon;
    public ZeroGyro(SwerveSubsystem b){
      this.swerve = b;
        //PigeonIMU pigeon = new PigeonIMU(20);
    }
    @Override
    public void initialize(){
        System.out.println("Zeroing gyro");
        //pigeon.setCompassAngle(0);
       
    }
    @Override
    public void execute(){
        swerve.zeroGyro();
        //  pigeon.setCompassAngle(0);
    }
    @Override
    public void end(boolean interrupted)
    {
        swerve.zeroGyro();
      //   System.out.println("swerve zeroed");
      // WPI_PigeonIMU pigeon = (WPI_PigeonIMU) swerve.getSwerveController().().getIMU();
      // pigeon.setYaw(0);
       //   pigeon.setCompassAngle(0);
    }

    // Returns true when the command should end.
  @Override
    public boolean isFinished()
    {
     return true;
     } 
}   
