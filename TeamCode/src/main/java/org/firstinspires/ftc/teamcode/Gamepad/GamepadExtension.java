

package org.firstinspires.ftc.teamcode.Gamepad;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys.Button;
import com.qualcomm.robotcore.hardware.Gamepad;


/**
 * An extended gamepad for more advanced toggles, key events,
 * and other control processors.
 */
public class GamepadExtension extends GamepadEx{

    double DZLY = 0;
    double DZLX = 0;
    double DZRY = 0;
    double DZRX = 0;



    /**
     * The constructor, that contains the gamepad object from the
     * opmode.
     *
     * @param gamepad the gamepad object from the opmode
     */
    public GamepadExtension(Gamepad gamepad) {
        super(gamepad);
    }

    @Override
    public boolean getButton(Button button) {
        return super.getButton(button);
    }

    public void setDeadzone(double leftY, double leftX, double rightY, double rightX){ //Setter method for Deadzones around each Gamepad stick axis
         DZLY = leftY;
         DZLX = leftX;
         DZRY = rightY;
         DZRX = rightX;
    }
    @Override
    public double getLeftY() { //returns the LeftY axis of the gamepad if it's value is greater than the deadzone
        double ly = super.getLeftY(); //utilizes GamepadEx's getter method (Since we don't declare a gamepad in this extension)
        return Math.abs(ly) < DZLY ? 0 : ly; // returns 0 if the absolute value of LeftY() is less than set deadzone, otherwise returns LeftY()
    }

    @Override
    public double getLeftX() {
        double lx = super.getLeftX();
        return Math.abs(lx) < DZLX ? 0 : lx;
    }

    @Override
    public double getRightY() {
        double ry = super.getRightY();
        return Math.abs(ry) < DZRY ? 0 : ry;
    }

    @Override
    public double getRightX() {
        double rx = super.getRightX();
        return Math.abs(rx) < DZRX ? 0 : rx;
    }

}