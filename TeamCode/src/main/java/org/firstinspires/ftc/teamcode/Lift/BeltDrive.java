package org.firstinspires.ftc.teamcode.Lift;

import com.arcrobotics.ftclib.hardware.SimpleServo;

public class BeltDrive {

    String[] Positions = null;

    SimpleServo[] group;

    public BeltDrive(SimpleServo drive){
        this(drive, 0.0, 1.0, false);
    }

    public BeltDrive(SimpleServo drive, double MIN, double MAX){
        this(drive, MIN, MAX, false);
    }

    public BeltDrive(SimpleServo drive, double MIN, double MAX, SimpleServo... followers){
        this.group[0] = drive;
    }

    public BeltDrive(SimpleServo drive, double MIN, double MAX, boolean INVERT){
        this.group[0] = drive;
        drive.setRange(MIN, MAX);
        drive.setInverted(INVERT);
    }

    public void setPosition(String Position, double position){
        //TODO
    }

}
