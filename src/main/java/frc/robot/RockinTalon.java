package frc.robot;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;

public class RockinTalon extends TalonFX {
    public RockinTalon(int deviceId) {
        super(deviceId);
    }

    @Override
    public void set(double speed){
        setControl(new DutyCycleOut(speed,false,false,false,false));
    }
}
