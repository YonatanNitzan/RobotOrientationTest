package org.usfirst.frc.team2212.robot;

import edu.wpi.first.wpilibj.Joystick;

public class OI /*VAVOI*/ {
    public static Joystick right = new Joystick(0);
    public static Joystick left = new Joystick(1);
    public static double getLeft(){
        return -left.getY()*0.4;
    }
    public static double getRight(){
        return -right.getY()*0.4;
    }
}
