/**
 * Team:    Syosset Syborgs
 * ID:      10696
 *
 * Author:  Rohan Ghotra        GitHub User: Rohan75
 * Version: v2.0
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="FinishedTeleOp", group="TeleOp")
public class TeleOp extends OpMode {

    // Drive System
    private DcMotor FL, FR, BL, BR;

    // Sub-Systems
    private DcMotor actuator;
    private Servo shoulder, elbow, wrist;
    private CRServo intake;
    private boolean shouldIntake;

    @Override
    public void init() {
        // Drive System
        FL = hardwareMap.get(DcMotor.class, "frontLeft");
        FR = hardwareMap.get(DcMotor.class, "frontRight");
        BL = hardwareMap.get(DcMotor.class, "backLeft");
        BR = hardwareMap.get(DcMotor.class, "backRight");

        FL.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.REVERSE);

        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Sub-Systems
        actuator = hardwareMap.get(DcMotor.class, "actuator");
        shoulder = hardwareMap.get(Servo.class, "shoulder");
        elbow = hardwareMap.get(Servo.class, "elbow");
        wrist = hardwareMap.get(Servo.class, "wrist");
        intake = hardwareMap.get(CRServo.class, "intake");
    }

    @Override
    public void loop() {
        double RT = gamepad1.right_trigger;
        double LT = gamepad1.left_trigger;
        double LY = -gamepad1.left_stick_y;
        double RY = -gamepad1.right_stick_y;

        FL.setPower(LY + RT - LT);
        BL.setPower(LY - RT + LT);
        FR.setPower(RY + LT - RT);
        BR.setPower(RY - LT + RT);

        actuator.setPower(gamepad2.right_trigger - gamepad2.left_trigger);

        if (gamepad1.a || gamepad2.a) {
            intake.setPower(shouldIntake ? .75 : 0);
            shouldIntake = !shouldIntake;
        }
    }

}
