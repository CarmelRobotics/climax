package frc.robot.subsystems;

import com.ctre.phoenix.platform.DeviceType;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import frc.robot.RockinTalon;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

//its BTS but its actually a NTS
//hasty response reference lmao
public class BTS extends SubsystemBase{
    TalonFX BTSMotor;
    public BTS(){
        BTSMotor = new TalonFX(10);
    }
    public void set(double speed){
        BTSMotor.set(-speed);
    }
    @Override
    public void periodic(){
        
    }
    
    
}
