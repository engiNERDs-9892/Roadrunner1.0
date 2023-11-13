package org.firstinspires.ftc.teamcode.Testing;

import static org.firstinspires.ftc.teamcode.Variables.TeleOP_Variables.GearServo;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Gear Servo Test", group="Linear Opmode")
//@Disabled

public class GearServo_Test extends LinearOpMode {
    @Override
    public void runOpMode() {

        GearServo = hardwareMap.servo.get("GearServo");

        GearServo.setPosition(0);

        GearServo.setDirection(Servo.Direction.REVERSE);

        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Statement = If you are pushing up on the left joystick rotate the wrist behind the robot
            if(Math.abs(gamepad2.left_stick_y) <= -0.5) {

                // This rotate the gear Clockwise so that the wrist joint rotates to the back of the Robot
                // by adding to its current position
                GearServo.setPosition((GearServo.getPosition() + 0.0005 * Math.signum(gamepad2.left_stick_y)));
            }

            // Statement = If you are pushing down on the left joystick rotate the wrist in front of the robot
            if(Math.abs(gamepad2.left_stick_y) >= 0.5) {

                // This rotate the gear CounterClockwise so that the wrist joint rotates to the front of the Robot
                // by subtracting from its current position
                GearServo.setPosition((GearServo.getPosition() - 0.0005 * Math.signum(gamepad2.left_stick_y)));

            }

            telemetry.addData("GearServo Position", GearServo.getPosition());
            updateTelemetry(telemetry);

        }
    }
}




