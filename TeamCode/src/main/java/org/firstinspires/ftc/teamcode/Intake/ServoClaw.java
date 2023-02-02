package org.firstinspires.ftc.teamcode.Intake;

import com.arcrobotics.ftclib.hardware.SimpleServo;

public class ServoClaw {

    double clawRange;

    double open = 1.0;
    double close = 0.0;

    SimpleServo Claw;

    public ServoClaw(SimpleServo Claw){ //Simple Constructor that takes in a SimpleServo Object
        this(Claw, 0.0, 1.0, false);
    }

    public ServoClaw(SimpleServo Claw, double MIN, double MAX){ //Overloaded Constructor that doesn't invert the claw direction
        this(Claw, MIN, MAX, false);
    }

    public ServoClaw(SimpleServo Claw, double MIN, double MAX, boolean INVERT){ //Overloaded Constructor that optionally inverts the claw direction
        this.Claw = Claw;
        this.Claw.setRange(MIN, MAX);
        this.Claw.setInverted(INVERT);
    }

    public void Close(){
        Claw.setPosition(close);
    }

    public void setClose(double close){ this.close = close;}

    public void Open(){
        Claw.setPosition(open);
    }

    public void setOpen(double open){ this.open = open;}

}
