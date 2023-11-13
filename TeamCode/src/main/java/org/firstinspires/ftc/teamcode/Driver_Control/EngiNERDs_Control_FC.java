package org.firstinspires.ftc.teamcode.Driver_Control;

import static org.firstinspires.ftc.teamcode.Variables.TeleOP_Variables.Close;
import static org.firstinspires.ftc.teamcode.Variables.TeleOP_Variables.FlippyFlip;
import static org.firstinspires.ftc.teamcode.Variables.TeleOP_Variables.FlooppyFloop;
import static org.firstinspires.ftc.teamcode.Variables.TeleOP_Variables.GearServo;
import static org.firstinspires.ftc.teamcode.Variables.TeleOP_Variables.LeftClaw;
import static org.firstinspires.ftc.teamcode.Variables.TeleOP_Variables.Open;
import static org.firstinspires.ftc.teamcode.Variables.TeleOP_Variables.RightClaw;
import static org.firstinspires.ftc.teamcode.Variables.TeleOP_Variables.motorBL;
import static org.firstinspires.ftc.teamcode.Variables.TeleOP_Variables.motorBR;
import static org.firstinspires.ftc.teamcode.Variables.TeleOP_Variables.motorFL;
import static org.firstinspires.ftc.teamcode.Variables.TeleOP_Variables.motorFR;
import static org.firstinspires.ftc.teamcode.Variables.TeleOP_Variables.motorLiftyLift;
import static org.firstinspires.ftc.teamcode.Variables.TeleOP_Variables.motorRiseyRise;
import static org.firstinspires.ftc.teamcode.Variables.TeleOP_Variables.slideySlideMax;
import static org.firstinspires.ftc.teamcode.Variables.TeleOP_Variables.slideySlideMin;

import android.graphics.Path;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name="EngiNERDs Control Field Centric", group="Linear Opmode")
//@Disabled
public class EngiNERDs_Control_FC extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        // A way to store values that the gamepad enters
        Gamepad currentGamepad1 = new Gamepad();
        Gamepad currentGamepad2 = new Gamepad();

        // A way to store values that gamepad enters
        Gamepad previousGamepad1 = new Gamepad();
        Gamepad previousGamepad2 = new Gamepad();

        // Declare our motors
        // Make sure your ID's match your configuration
        motorFL = hardwareMap.dcMotor.get("motorFL");
        motorBL = hardwareMap.dcMotor.get("motorBL");
        motorFR = hardwareMap.dcMotor.get("motorFR");
        motorBR = hardwareMap.dcMotor.get("motorBR");
        motorLiftyLift = hardwareMap.dcMotor.get("motorLiftyLift");
        motorRiseyRise = hardwareMap.dcMotor.get("motorRiseyRise");

        LeftClaw = hardwareMap.servo.get("LeftClaw");
        RightClaw = hardwareMap.servo.get("RightClaw");
        GearServo = hardwareMap.servo.get("GearServo");
        FlippyFlip = hardwareMap.servo.get("FlippyFlip");
        FlooppyFloop = hardwareMap.servo.get("FlooppyFloop");

        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
        // Setting the motor Direction, so the motors rotate correctly (Default Direction = Forward)

        motorFL.setDirection(DcMotor.Direction.FORWARD);
        motorFR.setDirection(DcMotor.Direction.REVERSE);
        motorBR.setDirection(DcMotor.Direction.REVERSE);
        motorBL.setDirection(DcMotor.Direction.FORWARD);
        motorRiseyRise.setDirection(DcMotorSimple.Direction.FORWARD);
        motorLiftyLift.setDirection(DcMotorSimple.Direction.FORWARD);

        LeftClaw.setDirection(Servo.Direction.REVERSE);
        FlippyFlip.setDirection(Servo.Direction.REVERSE);
        FlooppyFloop.setDirection(Servo.Direction.REVERSE);

        // Retrieve the IMU from the hardware map
        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

        motorRiseyRise.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLiftyLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorRiseyRise.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLiftyLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Setting the motor Power for Driver Control
        motorFL.setPower(0);
        motorBL.setPower(0);
        motorFR.setPower(0);
        motorBR.setPower(0);
        motorRiseyRise.setPower(0);
        motorLiftyLift.setPower(0);

        LeftClaw.setPosition(0);
        RightClaw.setPosition(0);
        FlooppyFloop.setPosition(1);
        FlippyFlip.setPosition(0);


        boolean Right_Claw_Toggle = false;

        boolean Left_Claw_Toggle = false;


        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad2);

            // Stored values of the gamepad inputs
            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);

            double RaiseandLower = -gamepad2.left_stick_y;
            int LiftyLiftPos = motorLiftyLift.getCurrentPosition();
            int RiseyRisePos = motorRiseyRise.getCurrentPosition();

            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // Rotate the movement direction counter to the bot's rotation
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX = rotX * 1.1;  // Counteract imperfect strafing

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            if(gamepad1.right_trigger != 0) {
                motorFL.setPower(frontLeftPower);
                motorBL.setPower(backLeftPower);
                motorFR.setPower(frontRightPower);
                motorBR.setPower(backRightPower);
            }

            if(gamepad1.left_trigger != 0) {
                motorFL.setPower(frontLeftPower *.3);
                motorBL.setPower(backLeftPower *.3);
                motorFR.setPower(frontRightPower *.3);
                motorBR.setPower(backRightPower *.3);
            }
            else {
                    motorFL.setPower(frontLeftPower * .7);
                    motorBL.setPower(backLeftPower * .7);
                    motorFR.setPower(frontRightPower *.7);
                    motorBR.setPower(backRightPower * .7);

            }

            if (LiftyLiftPos >= slideySlideMin && RiseyRisePos >= slideySlideMin
                    && LiftyLiftPos <= slideySlideMax && RiseyRisePos <= slideySlideMax) {

                // If you are trying to raise the linear Slide
                // Then raise the linear slides!
                if (gamepad2.right_trigger != 0) {
                    motorRiseyRise.setPower(1);
                    motorLiftyLift.setPower(1);
                }


                // if you are trying to lower the linear Slide
                // Then lower the linear slides!
                if (gamepad2.left_trigger != 0) {
                    motorRiseyRise.setPower(-1);
                    motorLiftyLift.setPower(-1);
                }

                // If you are not pushing on the joystick the power = 0
                // This is mainly to prevent stick drift
                else {
                    motorRiseyRise.setPower(0);
                    motorLiftyLift.setPower(0);
                }
            }

            if (LiftyLiftPos < slideySlideMin || RiseyRisePos < slideySlideMin) {

                if (gamepad2.right_trigger != 0) {
                    motorRiseyRise.setPower(1);
                    motorLiftyLift.setPower(1);
                } else {
                    motorRiseyRise.setPower(0);
                    motorLiftyLift.setPower(0);
                }

            }

            if (LiftyLiftPos > slideySlideMax || RiseyRisePos > slideySlideMax) {
                if (gamepad2.left_trigger !=0) {
                    motorRiseyRise.setPower(-1);
                    motorLiftyLift.setPower(-1);
                }
                else {
                    motorRiseyRise.setPower(0);
                    motorLiftyLift.setPower(0);
                }
            }


            // Toggle / Close & Open for the Right claw
            if (currentGamepad2.right_bumper && !previousGamepad2.right_bumper) {
                // This will set intakeToggle to true if it was previously false
                // and intakeToggle to false if it was previously true,
                // providing a toggling behavior.
                Right_Claw_Toggle = !Right_Claw_Toggle;
            }

            if (Right_Claw_Toggle) {
                RightClaw.setPosition(Open);
            }
            else {
                RightClaw.setPosition(Close);
            }


            // Toggle / Close & Open for the Left claw
            if (currentGamepad2.left_bumper && !previousGamepad2.left_bumper) {
                // This will set intakeToggle to true if it was previously false
                // and intakeToggle to false if it was previously true,
                // providing a toggling behavior.
                Left_Claw_Toggle = !Left_Claw_Toggle;
            }

            if (Left_Claw_Toggle) {
                LeftClaw.setPosition(Open);
            }
            else {
                LeftClaw.setPosition(Close);
            }

            if(Math.abs(gamepad2.right_stick_y) <= -0.5) {
                FlippyFlip.setPosition((FlippyFlip.getPosition() + 0.0005 * Math.signum(gamepad2.right_stick_y)));
                FlooppyFloop.setPosition((FlooppyFloop.getPosition() - 0.0005 * Math.signum(gamepad2.right_stick_y)));
            }

            if(Math.abs(gamepad2.right_stick_y) >= 0.5) {
                FlippyFlip.setPosition((FlippyFlip.getPosition() - 0.0005 * Math.signum(gamepad2.right_stick_y)));
                FlooppyFloop.setPosition((FlooppyFloop.getPosition() + 0.0005 * Math.signum(gamepad2.right_stick_y)));
            }

            if(Math.abs(gamepad2.left_stick_y) <= -0.5) {
                GearServo.setPosition((GearServo.getPosition() + 0.0005 * Math.signum(gamepad2.left_stick_y)));
                GearServo.setDirection(Servo.Direction.REVERSE);
            }

            if(Math.abs(gamepad2.left_stick_y) >= 0.5) {
                GearServo.setPosition((GearServo.getPosition() - 0.0005 * Math.signum(gamepad2.left_stick_y)));
                GearServo.setDirection(Servo.Direction.REVERSE);
            }


            // This button choice was made so that it is hard to hit on accident,
            // it can be freely changed based on preference.
            // The equivalent button is start on Xbox-style controllers.
            if (gamepad1.back) {
                imu.resetYaw();
            }

            telemetry.addData("Arm Position Flippy", FlippyFlip.getPosition());
            telemetry.addData("Arm Position Floopy", FlooppyFloop.getPosition());
            telemetry.addData("Left Claw Position", LeftClaw.getPosition());
            telemetry.addData("Right Claw Position", RightClaw.getPosition());
            telemetry.addData("LiftyLift Position", LiftyLiftPos);
            telemetry.addData("RiseyRise Position", RiseyRisePos);
        }
    }
}