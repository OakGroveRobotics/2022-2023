package org.firstinspires.ftc.teamcode.Lift;

import com.arcrobotics.ftclib.hardware.SimpleServo;
import java.util.HashMap;

public class BeltDrive {

    private final HashMap<String, Double> Positions = new HashMap<>();

    SimpleServo[] BeltDrive;

    public BeltDrive(SimpleServo drive){
        this(drive, 0.0, 1.0, false);
    }

    public BeltDrive(SimpleServo drive, double MIN, double MAX){
        this(drive, MIN, MAX, false);
    }

    public BeltDrive(SimpleServo drive, double MIN, double MAX, boolean INVERT){
        this.BeltDrive[0] = drive;
        BeltDrive[0].setRange(MIN, MAX);
        drive.setInverted(INVERT);
    }

    public BeltDrive(SimpleServo drive, double MIN, double MAX, SimpleServo... followers){
        this.BeltDrive = new SimpleServo[followers.length + 1];
        this.BeltDrive[0] = drive;
        BeltDrive[0].setRange(MIN, MAX);
        for(int i = 0; i < followers.length; i++){
            BeltDrive[i+1] = followers[i];
            BeltDrive[i+1].setRange(MIN, MAX);
        }
    }

    public BeltDrive(SimpleServo drive, double MIN, double MAX, int[] index, SimpleServo... followers){
        this(drive,MIN,MAX,followers);
        this.invert(index);
    }

    public void invert(int index){ BeltDrive[index].setInverted(true); }

    public void invert(int[] index){
        for(int ind: index) {
            BeltDrive[ind].setInverted(true);
        }
    }

    public void addPosition(String name, double position){
        Positions.put(name, position);
    }

    public void setPosition(String name){
        double tmp = Positions.get(name);
        for(SimpleServo servo: BeltDrive) {
            servo.setPosition(tmp);
        }
    }
    public double getPosition() { return BeltDrive[0].getPosition(); }

    public void rotateBy(double position) {
        for (SimpleServo servo : BeltDrive) { servo.rotateBy(position); }
    }

}
