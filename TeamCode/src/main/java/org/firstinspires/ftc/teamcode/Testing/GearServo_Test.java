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


        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            if(Math.abs(gamepad2.left_stick_y) <= -0.5) {
                GearServo.setPosition((GearServo.getPosition() + 0.0005 * Math.signum(gamepad2.left_stick_y)));
                GearServo.setDirection(Servo.Direction.REVERSE);
            }

            if(Math.abs(gamepad2.left_stick_y) >= 0.5) {
                GearServo.setPosition((GearServo.getPosition() - 0.0005 * Math.signum(gamepad2.left_stick_y)));
                GearServo.setDirection(Servo.Direction.REVERSE);
            }

            telemetry.addData("GearServo Position", GearServo.getPosition());
            updateTelemetry(telemetry);

        }
    }
}




