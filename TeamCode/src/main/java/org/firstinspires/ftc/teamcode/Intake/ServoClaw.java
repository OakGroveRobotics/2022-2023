package org.firstinspires.ftc.teamcode.Intake;

import static com.arcrobotics.ftclib.util.MathUtils.clamp;

import com.arcrobotics.ftclib.hardware.SimpleServo;


import java.util.HashMap;


public class ServoClaw {

    double open;
    double close;

    SimpleServo[] Claw;

    public ServoClaw(SimpleServo Claw){ //Simple Constructor that takes in a SimpleServo Object
        this(Claw, 0.0, 1.0, false);
    }

    public ServoClaw(SimpleServo Claw, double MIN, double MAX){ //Overloaded Constructor that doesn't invert the claw direction
        this(Claw, MIN, MAX, false);
    }

    public ServoClaw(SimpleServo Claw, double MIN, double MAX, boolean INVERT){ //Overloaded Constructor that optionally inverts the claw direction
        this.Claw = new SimpleServo[1];
        this.Claw[0] = Claw;
        this.Claw[0].setRange(MIN, MAX);
        this.Claw[0].setInverted(INVERT);
        init();
    }

    public ServoClaw(SimpleServo Claw, double MIN, double MAX, int[] INVERT, SimpleServo... followers){
        this.Claw = new SimpleServo[followers.length+1];
        this.Claw[0] = Claw;
        this.Claw[0].setRange(MIN, MAX);
        for(int i = 0; i < followers.length; i++){
            this.Claw[i+1] = followers[i];
            this.Claw[i+1].setRange(MIN, MAX);
        }
        initInvert(INVERT);
        init();

    }

    public void initInvert(int[] index){
        for(int ind: index) {
            Claw[ind].setInverted(true);
        }
    }

    public void Close(){
        for(SimpleServo servo: Claw) {
            servo.setPosition(close);
        }
    }

    public void setClose(double pos){close = pos;}

    public void Open(){
        for(SimpleServo servo: Claw) {
            servo.setPosition(open);
        }
    }

    public void setOpen(double pos){open = pos;}


    public void setPos(double pos){
        double temp = clamp(pos, close, open);

        for(SimpleServo servo: Claw) {
            servo.setPosition(temp);
        }
    }

    public double getPos(){
        return Claw[0].getPosition();
    }

    public boolean isOpen(){
        return Claw[0].getPosition() == open;
    }
    public boolean isClosed(){
        return Claw[0].getPosition() == close;
    }

    public void invert(){
        double tempPos = this.getPos();


    }
    private void init(){
        this.setClose(0.0);
        this.setOpen(1.0);
    }


}
