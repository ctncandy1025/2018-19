/**
 * Team:    Syosset Syborgs
 * ID:      10696
 *
 * Author:  Rohan Ghotra        GitHub User: Rohan75
 * Version: v1.0
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="TeleOp", group="TeleOp")
public class TeleOp extends OpMode {

    DcMotor FL, FR, BL, BR;

    @Override
    public void init() {
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

    }

}
