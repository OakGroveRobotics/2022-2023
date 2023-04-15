package org.firstinspires.ftc.teamcode.Lift;

import static com.arcrobotics.ftclib.hardware.motors.Motor.*;
import static com.arcrobotics.ftclib.hardware.motors.Motor.ZeroPowerBehavior.*;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;

import java.util.ArrayList;

public class CascadedLift {

    MotorGroup Lift;
    double MAX_POWER= 1.0;

    ArrayList<Integer> Positions = new ArrayList<>();

    public CascadedLift(MotorGroup Lift){
        this(Lift, 1.0, BRAKE);
    }

    public CascadedLift(MotorGroup Lift, double MAX_POWER){
        this(Lift, MAX_POWER, BRAKE);
    }

    public CascadedLift(MotorGroup Lift, double MAX_POWER, ZeroPowerBehavior ZPB){
        this.Lift = Lift;
        this.MAX_POWER = MAX_POWER;
        this.Lift.setZeroPowerBehavior(ZPB);
    }

    public void setInverted(boolean invert){ Lift.setInverted(invert); }

    public double getMaxPower(){ return MAX_POWER;}

    public void setMaxPower(double MAX_POWER){
        this.MAX_POWER = MAX_POWER;
    }

    public void Extend(double Power){
        Lift.setRunMode(RunMode.RawPower);
        Lift.set(Power);
    }

    public void Extend(Boolean extend){
        Lift.setRunMode(RunMode.RawPower);
        Lift.set(MAX_POWER);
    }

    public void Retract(double Power){
        Lift.setRunMode(RunMode.RawPower);
        Lift.set(-Power);
    }

    public void Retract(Boolean retract){
        Lift.setRunMode(RunMode.RawPower);
        Lift.set(-MAX_POWER);
    }

    public void setVeloCoefficients(double kp, double ki, double kd){ Lift.setVeloCoefficients(kp, ki, kd); }

    public void setDistancePerPulse(double distancePerPulse){ Lift.setDistancePerPulse(distancePerPulse); }

    public void setPositionTolerance(double positionTolerance){ Lift.setPositionTolerance(positionTolerance); }

    public void addPosition(int pos){ Positions.add(pos); }

    public void addPosition(int index, int pos){ Positions.add(index, pos); }

    public boolean atTargetPosition(){ return Lift.atTargetPosition(); }

    public void setTargetPosition(int index){ Lift.setTargetPosition( Positions.get(index)); }

    public void setRunMode( RunMode runmode){ Lift.setRunMode(runmode);}

    public void resetPosition(){ Lift.resetEncoder(); }

    public void goToPosition(){
        if(!(Lift.atTargetPosition())){
            Lift.set(MAX_POWER);
        }
    }
    public void goToPosition(double speed){
        if(!(Lift.atTargetPosition())){
            Lift.set(speed);
        }
    }
    public double get(){
        return Lift.get();
    }

}
