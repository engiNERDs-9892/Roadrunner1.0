package org.firstinspires.ftc.teamcode.Driver_Control;

import static org.firstinspires.ftc.teamcode.Variables.TeleOP_Variables.Close;
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
import static org.firstinspires.ftc.teamcode.Variables.TeleOP_Variables.GearServo;
import static org.firstinspires.ftc.teamcode.Variables.TeleOP_Variables.FlooppyFloop;
import static org.firstinspires.ftc.teamcode.Variables.TeleOP_Variables.FlippyFlip;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name="EngiNERDs Control Robot Centric", group="Linear Opmode")
//@Disabled

public class EngiNERDs_Control_RC extends LinearOpMode {
    @Override
    public void runOpMode() {

        // A way to store values that the gamepad enters
        Gamepad currentGamepad1 = new Gamepad();
        Gamepad currentGamepad2 = new Gamepad();

        // A way to store values that gamepad enters
        Gamepad previousGamepad1 = new Gamepad();
        Gamepad previousGamepad2 = new Gamepad();


        // HardwareMap Section (Used to talk to the driver hub for the configuration)

        // Declare our Motors
        // Make sure your ID's match your configuration

        motorFL = hardwareMap.dcMotor.get("motorFL");
        motorFR = hardwareMap.dcMotor.get("motorFR");
        motorBL = hardwareMap.dcMotor.get("motorBL");
        motorBR = hardwareMap.dcMotor.get("motorBR");
        motorLiftyLift = hardwareMap.dcMotor.get("motorLiftyLift");
        motorRiseyRise = hardwareMap.dcMotor.get("motorRiseyRise");

        // Declare our Servos
        // Make sure your ID's match your configuration

        LeftClaw = hardwareMap.servo.get("LeftClaw");
        RightClaw = hardwareMap.servo.get("RightClaw");
        GearServo = hardwareMap.servo.get("GearServo");
        FlippyFlip = hardwareMap.servo.get("FlippyFlip");
        FlooppyFloop = hardwareMap.servo.get("FlooppyFloop");

        // Resets the Encoder Position to 0 so that we can use Encoders for our Driver Control instead
        // of Magnetic Limit Switches
        motorRiseyRise.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLiftyLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Tells the motors to Run using those specific Encoders
        motorRiseyRise.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLiftyLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Setting the motor Power for Driver Control
        motorFL.setPower(0);
        motorBL.setPower(0);
        motorFR.setPower(0);
        motorBR.setPower(0);
        motorRiseyRise.setPower(0);
        motorLiftyLift.setPower(0);

        // Setting the position for the Servos for Driver Control
        LeftClaw.setPosition(0);
        RightClaw.setPosition(0);
        FlooppyFloop.setPosition(1);
        FlippyFlip.setPosition(0);


        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
        // Setting the motor / servo Direction, so the motors or servos rotate correctly (Default Direction = Forward)
        motorFL.setDirection(DcMotor.Direction.FORWARD);
        motorFR.setDirection(DcMotor.Direction.REVERSE);
        motorBR.setDirection(DcMotor.Direction.REVERSE);
        motorBL.setDirection(DcMotor.Direction.FORWARD);
        motorRiseyRise.setDirection(DcMotorSimple.Direction.FORWARD);
        motorLiftyLift.setDirection(DcMotorSimple.Direction.FORWARD);

        LeftClaw.setDirection(Servo.Direction.REVERSE);
        FlippyFlip.setDirection(Servo.Direction.REVERSE);
        FlooppyFloop.setDirection(Servo.Direction.REVERSE);
        GearServo.setDirection(Servo.Direction.REVERSE);

        // Toggels so that the Claws can be opened and closed using the same button
        boolean Right_Claw_Toggle = false;

        boolean Left_Claw_Toggle = false;


        waitForStart();


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Stored values of the gamepad inputs
            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad2);

            // Stored values of the gamepad inputs
            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);


            // Encoder tracker for the LS so it doesn't go over the limit / to high or low
            int LiftyLiftPos = motorLiftyLift.getCurrentPosition();
            int RiseyRisePos = motorRiseyRise.getCurrentPosition();

            // Variable used for Regular speed (To find the direction that the stick needs to be in)
            double max;

            // The code below talks about the Y-axis (Up and Down / Forward and Backwards)

            double axial = gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value

            // The code below talks about the X-axis (Left and Right)

            double lateral = -gamepad1.left_stick_x; // The bottom two are inverted because motor direction is changed

            // The code below talks about Z-Axis (Spinning around)

            double yaw = -gamepad1.right_stick_x;


            // Combine the joystick requests for each axis-motion to determine each wheel's power
            // And direction for Regular speed
            double leftFrontPower = (axial + lateral + yaw);
            double rightFrontPower = (axial - lateral - yaw);
            double leftBackPower = (axial - lateral + yaw);
            double rightBackPower = (axial + lateral - yaw);


            ////////////////////////////////////////////////////////////////////////////////////////////
            // use LEFT joystick to go Forward/Backwards & left/Right, and RIGHT joystick to Rotate.///
            //////////////////////////////////////////////////////////////////////////////////////////


            // This calculates the direction & power for Regular Speed
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            // sets the wheels to do whatever the calculation above tells it to do for Regular Speed
            if (max > 1.0) {
                leftFrontPower /= max;
                rightFrontPower /= max;
                leftBackPower /= max;
                rightBackPower /= max;
            }




            // If the Right Trigger is pressed set the Values of the motor to 100% power
            if (gamepad1.right_trigger != 0) {

                motorFL.setPower(leftFrontPower);
                motorBL.setPower(leftBackPower);
                motorBR.setPower(rightBackPower);
                motorFR.setPower(rightFrontPower);

            }

            // If the Left Trigger is pressed set the Values of the motor to 30% power
            else if (gamepad1.left_trigger != 0) {
                motorFL.setPower(leftFrontPower * .3);
                motorBL.setPower(leftBackPower * .3);
                motorBR.setPower(rightBackPower * .3);
                motorFR.setPower(rightFrontPower * .3);

            }

            // If No Trigger is pressed set the Values of the motor to 70% (Base value for motors)
            else {
                motorFL.setPower(leftFrontPower * .7);
                motorBL.setPower(leftBackPower * .7);
                motorFR.setPower(rightFrontPower * .7);
                motorBR.setPower(rightBackPower * .7);
            }



            // Statement = If encoder value is between 100 and 7700 be able to go both up and down
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
                else {
                    motorRiseyRise.setPower(0);
                    motorLiftyLift.setPower(0);
                }
            }

            // Statement = If encoder value is less than 100, on be able to Raise Linear Slides
            if (LiftyLiftPos < slideySlideMin || RiseyRisePos < slideySlideMin) {

                // If you are trying to raise the linear Slide
                // Then raise the linear slides!
                if (gamepad2.right_trigger != 0) {
                    motorRiseyRise.setPower(1);
                    motorLiftyLift.setPower(1);
                }
                // If you are not pushing on the joystick the power = 0
                else {
                    motorRiseyRise.setPower(0);
                    motorLiftyLift.setPower(0);
                }

            }

            // Statement = If encoder value is more than 7700, on be able to Lower Linear Slides
            if (LiftyLiftPos > slideySlideMax || RiseyRisePos > slideySlideMax) {

                // if you are trying to lower the linear Slide
                // Then lower the linear slides!
                if (gamepad2.left_trigger !=0) {
                    motorRiseyRise.setPower(-1);
                    motorLiftyLift.setPower(-1);
                }
                // If you are not pushing on the joystick the power = 0
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

            // Opens the claws after the 1st press of the bumper and alternates once pressed again
            if (Right_Claw_Toggle) {
                RightClaw.setPosition(Open);
            }
            // Closes the claws on the 2nd press of the bumper and alternates once pressed again
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

            // Opens the claws after the 1st press of the bumper and alternates once pressed again
            if (Left_Claw_Toggle) {
                LeftClaw.setPosition(Open);
            }
            // Closes the claws on the 2nd press of the bumper and alternates once pressed again
            else {
                LeftClaw.setPosition(Close);
            }



            // Statement = If you are pushing up on the right joystick, then rotate the arms behind the robot
            if(Math.abs(gamepad2.right_stick_y) <= -0.5) {

                // This rotates the arms Clockwise so that the arms rotate behind the robot (Facing the backboard idealy)
                // FlippyFlip adds to its current position due to the value starting at zero
                FlippyFlip.setPosition((FlippyFlip.getPosition() + 0.0005 * Math.signum(gamepad2.right_stick_y)));

                // FloopyFloop subtracts from its current position due to the value starting at One
                FlooppyFloop.setPosition((FlooppyFloop.getPosition() - 0.0005 * Math.signum(gamepad2.right_stick_y)));
            }

            // Statement = If you are pushing down on the right joystick, then rotate the arms to in front of the robot
            if(Math.abs(gamepad2.right_stick_y) >= 0.5) {

                // This rotates the arms Counter Clockwise so that the arms rotate in front of the robot
                // FlippyFlip subtracts from its current position due to the value starting at zero
                FlippyFlip.setPosition((FlippyFlip.getPosition() - 0.0005 * Math.signum(gamepad2.right_stick_y)));

                // FloopyFloop adds to its current position due to the value starting at One
                FlooppyFloop.setPosition((FlooppyFloop.getPosition() + 0.0005 * Math.signum(gamepad2.right_stick_y)));
            }


            // Telemetry for the drivers so they can see if the system is running smoothly
            telemetry.addData("LiftyLift Position", LiftyLiftPos);
            telemetry.addData("RiseyRise Position", RiseyRisePos);
            telemetry.addData("Left Claw Position", LeftClaw.getPosition());
            telemetry.addData("Right Claw Position", RightClaw.getPosition());
            telemetry.addData("Arm Position", FlippyFlip.getPosition());
            telemetry.addData("Arm Position", FlooppyFloop.getPosition());
            updateTelemetry(telemetry);
        }
    }
}




