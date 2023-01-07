package org.firstinspires.ftc.teamcode.Drivebase;

import static com.arcrobotics.ftclib.hardware.motors.Motor.RunMode.RawPower;


import com.arcrobotics.ftclib.hardware.motors.Motor;

import java.util.Arrays;

public class Mecanum extends FourMotor{

    Motor[] drive = new Motor[4];

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
        return MAX_SPEED;
    }

    public void moveFieldCentric(double FORWARD_VEL, double STRAFE_VEL, double ROTATE_VEL, double GYRO) {

        double X_ROT = STRAFE_VEL* Math.cos(GYRO) - FORWARD_VEL * Math.sin(GYRO);
        double Y_ROT = FORWARD_VEL * Math.cos(GYRO) + STRAFE_VEL * Math.sin(GYRO);

        double[] power = new double[4];
        power[0] = Y_ROT + X_ROT + ROTATE_VEL;
        power[1] = -Y_ROT + X_ROT + ROTATE_VEL;
        power[2] = Y_ROT - X_ROT + ROTATE_VEL;
        power[3] = -Y_ROT - X_ROT + ROTATE_VEL;

        power = normalize(power);

        for(double x : power){ x = x * MAX_SPEED;}

        this.move(power);
    }

    public void moveRobotCentric(double FORWARD_VEL, double STRAFE_VEL, double ROTATE_VEL) {

        double[] power = new double[4];
        power[0] = FORWARD_VEL + STRAFE_VEL + ROTATE_VEL;
        power[1] = -FORWARD_VEL + STRAFE_VEL + ROTATE_VEL;
        power[2] = FORWARD_VEL - STRAFE_VEL + ROTATE_VEL;
        power[3] = -FORWARD_VEL - STRAFE_VEL + ROTATE_VEL;

        move(power);
    }

    private double[] normalize(double[] power) {

        double[] power_normalized = new double[power.length];
        double temp = Arrays.stream(power).max().getAsDouble();
        double max_power = Math.max(Math.abs(temp), 1);


        for( int i = 0; i < power.length; i++) {
            power_normalized[i] = power[i] / max_power;
        }

        return power_normalized;
    }

    private void move(double[] power) {
        for( int i = 0; i < power.length; i++){ drive[i].set(power[i]);}
    }
}