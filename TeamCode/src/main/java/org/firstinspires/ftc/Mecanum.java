package org.firstinspires.ftc.teamcode.Drivebase;

import static com.arcrobotics.ftclib.hardware.motors.Motor.RunMode.*;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.Motor.ZeroPowerBehavior.*;

public class Mecanum extends FourMotor{

    Motor[] drive;

    //EZ declaration Rawpower runmode
    public Mecanum(Motor frontLeft, Motor frontRight, Motor rearLeft, Motor rearRight){
        new Mecanum(frontLeft, frontRight, rearLeft, rearRight, RawPower, Motor.ZeroPowerBehavior.FLOAT);
    }

    //Declare Motor objects, set runmode
    public Mecanum(Motor frontLeft, Motor frontRight, Motor rearLeft, Motor rearRight, Motor.RunMode runmode, Motor.ZeroPowerBehavior zeropowerbehavior){
        drive = new Motor[]{frontLeft, frontRight, rearLeft, rearRight};

        for (Motor x : drive) {
            x.setRunMode(runmode);
            x.setZeroPowerBehavior(zeropowerbehavior);
        }
    }

    public void stop() {
        for (Motor x : drive) {x.stopMotor();}
    }

    public void setMAX_SPEED(double MAX_SPEED) {
        super.setMAX_SPEED(MAX_SPEED);
    }

    public double getMAX_SPEED() {
        return super.MAX_SPEED;
    }

    public void setINVERTED(double[] Invert) {
        super.setINVERTED(Invert);
    }

    public double[] getINVERTED() {
        return super.DIRECTION;
    }

    public void  move(double[] power) {
        double[] MODIFIED_POWER = super.getMODIFIED_POWER(power);
        for (int i = 0; i < drive.length; i++) {
            drive[i].set(MODIFIED_POWER[i]);
        }

    }
}
