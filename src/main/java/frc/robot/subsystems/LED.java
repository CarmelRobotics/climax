package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LED extends SubsystemBase {
    private pinManager pins;
    private STATUS mode;

    public boolean testState = true;
    public DigitalOutput test = new DigitalOutput(6);

    public LED() { pins = new pinManager(new int[] {2,3,4,5}); mode = STATUS.BOOTING; }

    public void setMode(STATUS status) {
        switch (mode) {
            case BOOTING, AUTO -> { if (status != STATUS.SHOOTING) mode = status; }
            case VIBE -> { mode = STATUS.VIBE; }
            case SHOOTING, DEFAULT -> { mode = status; }
            case ERROR -> { if (status != STATUS.DEFAULT) mode = status; }
            default -> { mode = status; }
        }
        //System.out.println("<LED INTERNALS> - " + status + " mode enabled");
    }

    public STATUS getState() { return mode; }

    public void defaultState() { if (mode != STATUS.VIBE) { setMode(STATUS.DEFAULT); } else { mode = STATUS.DEFAULT; } }

    @Override
    public void periodic(){
        switch (mode) {
            case BOOTING:
                pins.setBinary("1000");
                break;
            case DEFAULT:
                pins.setBinary("1001");
                break;
            case SHOOTING:
                pins.setBinary("1010");
                break;
            case AUTO:
                pins.setBinary("1011");
                break;
            case VIBE:
                pins.setBinary("1100");
                break;
            case ERROR:
                pins.setBinary("1101");
                break;
            default:
                break;
        }
        test.set(testState);
    }

    public static enum STATUS {
        BOOTING,
        DEFAULT,
        SHOOTING,
        AUTO,
        VIBE,
        ERROR
    }

    private class pinManager {
        private DigitalOutput[] pins;
        private int[] pinIDs;
        public pinManager(int[] npinIDs) {
            pinIDs = npinIDs;
            pins = new DigitalOutput[pinIDs.length];
            for (int i = 0; i < pinIDs.length; i++)
                pins[i] = new DigitalOutput(pinIDs[i]);
        }

        public void writeBoolean(boolean[] data) {
            if (data.length <= pins.length) {
                for (int i = 0; i < data.length; i++) {
                    DigitalOutput pin = pins[i];
                    boolean dat = data[i];
                    pin.set(dat);
                    // pins[i].set(data[i]);
                    // System.out.println("<LED INTERNALS> - PIN " + pin.getChannel() + " set to value " + dat);
                }
            }
        }

        public void clearOut() {
            for (int i = 0; i < pins.length; i++) {
                pins[i].set(false);
            }
        }

        public void setBinary(String data) {
            boolean[] ndata = new boolean[data.length()];
            for (int i = 0; i < data.length(); i++)
                ndata[i] = (data.charAt(i) == '1');
            writeBoolean(ndata);
        }
    }
}
