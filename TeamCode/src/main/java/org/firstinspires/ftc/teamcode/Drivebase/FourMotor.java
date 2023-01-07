package org.firstinspires.ftc.teamcode.Drivebase;


public abstract class FourMotor {

        public static double MAX_SPEED = 1.0;
        public static double[] DIRECTION = {1.0, 1.0, 1.0, 1.0};

        public FourMotor() {}

        public void setMAX_SPEED(double MAX_SPEED){this.MAX_SPEED = MAX_SPEED;} //Set Max Speed

        public double getMAX_SPEED() {return this.MAX_SPEED;} //get Max Speed

        public void setINVERTED(double[] DIRECTION) {this.DIRECTION = DIRECTION;} //set the direction of motor rotation

        public double[] getINVERTED() {return this.DIRECTION;} //get the direction of motor rotation

        public double[] getMODIFIED_POWER(double[] powers){
                double[] MODIFIED_POWER = new double[powers.length];
                for( int i = 0; i < powers.length; i++){
                        if(Math.abs(powers[i]) >= MAX_SPEED){ MODIFIED_POWER[i] = MAX_SPEED * Math.signum(powers[i]) * DIRECTION[i];}
                        else{MODIFIED_POWER[i] = powers[i] * DIRECTION[i];}
                }
                return MODIFIED_POWER;
        }
}
