package org.firstinspires.ftc.teamcode.Variables;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;


public class TeleOP_Variables {

    // Motor's && Servo's Variable Names
    public static DcMotor motorFL = null;
    public static DcMotor motorFR = null;
    public static DcMotor motorBL = null;
    public static DcMotor motorBR = null;
    public static DcMotor motorLiftyLift;
    public static DcMotor motorRiseyRise;
    public static Servo FlippyFlip;
    public static Servo FlooppyFloop;
    public static Servo GearServo;
    public static Servo LeftClaw;
    public static Servo RightClaw;



    // Position for the Claws to close
    public static double Open = 0.33;
    public static double Close = 0;


    // Encoder Values to stop the Linear Slides instead of using a Magnetic Limit Switch
    public static final int slideySlideMax = 7700;
    public static final int slideySlideMin = 100;

    }
