package org.firstinspires.ftc.teamcode.Lift;

import static com.arcrobotics.ftclib.hardware.motors.Motor.*;
import static com.arcrobotics.ftclib.hardware.motors.Motor.ZeroPowerBehavior.*;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;

public class CascadedLift {

    MotorGroup Lift;
    double MAX_POWER= 1.0;

    public CascadedLift(MotorGroup Lift){
        this(Lift, 1.0, BRAKE);
    }

    public CascadedLift(MotorGroup Lift, double MAX_POWER){
        this(Lift, MAX_POWER, BRAKE);
    }

    public CascadedLift(MotorGroup Lift, double MAX_POWER, ZeroPowerBehavior ZPB){
        this.Lift = Lift;
        this.MAX_POWER = MAX_POWER;
        Lift.setZeroPowerBehavior(ZPB);
    }

    public double setMaxPower(){ return MAX_POWER;}

    public void setMaxPower(double MAX_POWER){
        this.MAX_POWER = MAX_POWER;
    }

    public void Extend(double Power){
        Lift.set(Power);
    }

    public void Extend(Boolean extend){
        Lift.set(MAX_POWER);
    }

    public void Retract(double Power){
        Lift.set(-Power);
    }

    public void Retract(Boolean retract){
        Lift.set(-MAX_POWER);
    }

}
