package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
    public NetworkTable limelight;
    public NetworkTableInstance inst = NetworkTableInstance.getDefault();
    double x;
    double y;
    public Vision() {
        limelight = inst.getTable("limelight");    
    }

    @Override
    public void periodic() {
        Transform2d botpose = getRelativeBotPose();
        SmartDashboard.putNumber("robot x", botpose.getX());
        SmartDashboard.putNumber("robot y", botpose.getY());
        SmartDashboard.putNumber("robot angle", botpose.getRotation().getDegrees());
        SmartDashboard.putBoolean("limelight found", limelight.containsKey("botpose"));
        SmartDashboard.putBoolean("AprilTag found",targetFound());
        SmartDashboard.putNumber("AprilTag ID", getAprilTagId());
    }
    public Transform2d getRelativeBotPose() {
        Transform2d botpose = getBotPose();
         x = botpose.getX() - 7.9;
         y = botpose.getY() - 2.74;

        botpose = new Transform2d(new Translation2d(x, y), botpose.getRotation());
        return botpose;
    }
    public double getPoseX(){
        return x;
    }
    public double getPoseY(){
        return y;
    }
    public Transform2d getBotPose() {
        Transform2d t;
        double[] botpose = limelight.getEntry("botpose").getDoubleArray(new double[1]);
        if (!limelight.getEntry("botpose").exists() || botpose.length < 6) {
            return new Transform2d();
        }
        t = new Transform2d(new Translation2d(botpose[0], botpose[1]), new Rotation2d(botpose[5] * (3.14/180.0)));
        return t;
    }
    public boolean targetFound(){
        double id;
        double[] tagid = limelight.getEntry("tid").getDoubleArray(new double[1]);
        id = tagid[0];
        return(!(id == 0));
    }
    public double getAprilTagId(){
        double id;
        double[] tagid = limelight.getEntry("tid").getDoubleArray(new double[1]);
        id = tagid[0];
        return id;
    }
    
}