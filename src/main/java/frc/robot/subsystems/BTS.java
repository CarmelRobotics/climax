package frc.robot.subsystems;

import com.ctre.phoenix.platform.DeviceType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import frc.robot.RockinTalon;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

//its BTS but its actually a NTS
//hasty response reference lmao
public class BTS extends SubsystemBase{
    RockinTalon BTSMotor;
    public BTS(){
        BTSMotor = new RockinTalon(10);
    }
    public void set(double speed){
        BTSMotor.set(speed);
    }
    
    
}
