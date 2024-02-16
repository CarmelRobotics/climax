package frc.robot.subsystems;
import frc.robot.Constants;

import edu.wpi.first.hal.LEDJNI;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LED extends SubsystemBase {
    private SerialPort arduino_comunication = new SerialPort(9600, SerialPort.Port.kUSB1 );
    private Constants.LED_Controls state;

    public LED() {
        state = Constants.LED_Controls.BOOTING;
    }

    public void setState(Constants.LED_Controls newState) { state = newState; write(); }

    public Constants.LED_Controls getState() { return state; }

    private static byte[] getComunication(Constants.LED_Controls state) {
        byte[] out = new byte[2];
        switch (state) {
            case AUTO:
                out[0] = 0; out[1] = 0;
                break;
            case DEFAULT:
                out[0] = 0; out[1] = 1;
                break;
            case SHOOTING:
                out[0] = 1; out[1] = 0;
                break;
            case BOOTING:
                out[0] = 1; out[1] = 1;
                break;
            default:
                out[0] = 1; out[1] = 1;
                break;
        }
        return out;
    } 

    public void write() {
        arduino_comunication.write(getComunication(state), 2);
    }
}
