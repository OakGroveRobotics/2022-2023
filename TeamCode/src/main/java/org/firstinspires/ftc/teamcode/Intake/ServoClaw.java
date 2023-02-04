package org.firstinspires.ftc.teamcode.Intake;

import com.arcrobotics.ftclib.hardware.SimpleServo;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;

public class ServoClaw {

    List<Double> Pos = new LinkedList<>();

    double open = 1.0;
    double close = 0.0;

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
        invert(INVERT);
        init();

    }

    public void invert(int[] index){
        for(int i = 0; i < index.length; i++) {
            Claw[index[i]].setInverted(true);
        }
    }

    public void Close(){
        for(SimpleServo servo: Claw) {
            servo.setPosition(close);
        }
    }

    public void setClose(double close){ this.close = close;}

    public void Open(){
        for(SimpleServo servo: Claw) {
            servo.setPosition(open);
        }
    }

    public void setOpen(double open){ this.open = open;}

    public void addPosition(double pos){
        Pos.add(1, pos);
    }

    public void addPosition(double pos, int index){
        Pos.add(index,pos);
    }

    public void setPos(int index){
        for(SimpleServo servo: Claw){
            servo.setPosition(Pos.get(index));
        }
    }

    private void init(){
        Pos.add(close);
        Pos.add(open);
    }


}
