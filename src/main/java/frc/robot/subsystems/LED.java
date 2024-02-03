package frc.robot.subsystems;

import edu.wpi.first.hal.LEDJNI;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LED extends SubsystemBase {
    private SerialPort spi = new SerialPort(9600, SerialPort.Port.kUSB1 );
    public LED(){
        
    }
}
