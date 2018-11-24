{
    "status": "success"
}


package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.util.List;
import java.util.ArrayList;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

public class Autonomous extends LinearOpMode {

    // Drive system
    private static DcMotor FL, FR, BL, BR;

    // Sub-systems
    private static DcMotor actuator;
    private static Servo shoulder, elbow, wrist;
    private static CRServo intake;

    // Encoder values
    private static final int LENGTH = 14, WIDTH = 18, WHEEL_RADIUS = 2, TICKS_PER_ROT = 1440, GEAR_RATIO = 60,
            TICKS_PER_DEGREE = TICKS_PER_ROT * GEAR_RATIO / 360;
    private static final double TICKS_PER_INCH = TICKS_PER_ROT * GEAR_RATIO / 2 * Math.PI * WHEEL_RADIUS,
            TURN_RADIUS = Math.sqrt(Math.pow(LENGTH/2) + Math.pow(WIDTH/2));
    
    // VuForia + TensorFlow declarations
    private static final String VUFORIA_KEY = "AU4FDNH/////AAABmRVll+1KLUOvurFViKcVqNpTWXwbjf31OgDqmYXYdxSUgDANPXZ2u318WCLJ72KycjRKeDz92M2BmL8lNtnE5seRlMt7ES28yoMbkp1ic0xgmLAo1f1tXFBySj9WFJD708PscrGLeGr8vbbhDb6Zmv1t4i+ZEsHZVQyijBGH6t+egvvwjYdqA+tOZdujYxX1TWmGXJQVwI1PpA9YFh8+m2DfFteZ7zyyirYeFllW03a8yHZZyeVF0GycAPn9nBkPEDWEFodD9S2/mKJLdq/FyPIyfbh/9v6K8biIL0UyyOzIKEI3N542XZspTv6RkWeKcMwqOZqaPXBtDWnX8uxJ3gaq0Zbb2t12cBBBjVj5d5a/";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;
    private ArrayList<minerals> sampling;
    
    public enum minerals {
        GOLD, SILVER
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initVuforia();
        
        waitForStart();

        Drive.mapDriveSystem();
        
        while (opModeIsActive) {
            // Sampling
            while (opModeIsActive && sampling == null) {
                sampling = objectDetection(tfod.getUpdatedRecognitions());
            }
            if (sampling.indexOf(minerals.GOLD) == 0) {
                Drive.moveX("inches", -16);
                Drive.moveY("inches", 10);
            } else if (sampling.indexOf(minerals.GOLD) == 1) {
                Drive.moveY("inches", 10);
            } else {
                Drive.moveX("inches", 16);
                Drive.moveY("inches", 10);
            }
        }
    }

    // Instantiating subsystem motors/servos
    public void mapSubSystems() {
        shoulder = hardwareMap.get(Servo.class, "shoulder");
        elbow = hardwareMap.get(Servo.class, "elbow");
        wrist = hardwareMap.get(Servo.class, "wrist");
        intake = hardwareMap.get(CRServo.class, "intake");
    }

    // Class with methods used in drive system
    public static class Drive {
        // Instantiating motors for four wheel drive; mecanum wheels
        public void mapDriveSystem() {
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

            FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        
        // standard drive; enter inch/geree measurement, and the units (use negative for moving backward)
        private static void moveZ(String type, double value) {
            type = type.toLowerCase();
            if (type == "inches") {
                FL.setTargetPosition((int)(FL.getCurrentPosition() + (value * TICKS_PER_INCH)));
                FL.setPower(.5);
                FR.setTargetPosition((int)(FR.getCurrentPosition() + (value * TICKS_PER_INCH)));
                FR.setPower(.5);
                BL.setTargetPosition((int)(BL.getCurrentPosition() + (value * TICKS_PER_INCH)));
                BL.setPower(.5);
                BR.setTargetPosition((int)(BR.getCurrentPosition() + (value * TICKS_PER_INCH)));
                BR.setPower(.5);
            }
            else if (type == "degrees") {
                FL.setTargetPosition((int)(FL.getCurrentPosition() + (value * TICKS_PER_DEGREE)));
                FL.setPower(.5);
                FR.setTargetPosition((int)(FR.getCurrentPosition() + (value * TICKS_PER_DEGREE)));
                FR.setPower(.5);
                BL.setTargetPosition((int)(BL.getCurrentPosition() + (value * TICKS_PER_DEGREE)));
                BL.setPower(.5);
                BR.setTargetPosition((int)(BR.getCurrentPosition() + (value * TICKS_PER_DEGREE)));
                BR.setPower(.5);
            }
        }

        // strafing; enter inch/degree measurement, and the units (use negative for strafing left)
        public static void moveX(String type, double value) {
            type = type.toLowerCase();
            if (type == "inches") {
                FL.setTargetPosition((int)(FL.getCurrentPosition() + (value * TICKS_PER_INCH)));
                FL.setPower(.5);
                FR.setTargetPosition((int)(FR.getCurrentPosition() - (value * TICKS_PER_INCH)));
                FR.setPower(.5);
                BL.setTargetPosition((int)(BL.getCurrentPosition() - (value * TICKS_PER_INCH)));
                BL.setPower(.5);
                BR.setTargetPosition((int)(BR.getCurrentPosition() + (value * TICKS_PER_INCH)));
                BR.setPower(.5);
            }
            else if (type == "degrees") {
                FL.setTargetPosition((int)(FL.getCurrentPosition() + (value * TICKS_PER_DEGREE)));
                FL.setPower(.5);
                FR.setTargetPosition((int)(FR.getCurrentPosition() - (value * TICKS_PER_DEGREE)));
                FR.setPower(.5);
                BL.setTargetPosition((int)(BL.getCurrentPosition() - (value * TICKS_PER_DEGREE)));
                BL.setPower(.5);
                BR.setTargetPosition((int)(BR.getCurrentPosition() + (value * TICKS_PER_DEGREE)));
                BR.setPower(.5);
            }
        }

        // turning; positive degrees for right, neagtive degrees for left
        public static void turnY(double degrees) {
            double turnInches = (degrees/360) * (2 * Math.PI * TURN_RADIUS);
            FL.setTargetPosition((int)(FL.getCurrentPosition() + (turnInches * TICKS_PER_INCH / 2)));
            FR.setTargetPosition((int)(FR.getCurrentPosition() - (turnInches * TICKS_PER_INCH / 2)));
            BL.setTargetPosition((int)(BL.getCurrentPosition() + (turnInches * TICKS_PER_INCH / 2)));
            BR.setTargetPosition((int)(BR.getCurrentPosition() - (turnInches * TICKS_PER_INCH / 2)));
        }
    }

    // Method for controlling linear actuator movement
    public void controlActuator(double degrees) {
        actuator.setTargetPosition((int)(actuator.getCurrentPosition() + (degrees * TICKS_PER_DEGREE)));
    }
    
    public ArrayList<minerals> objectDetection(List<Recognition> objectsFound) {
        ArrayList<minerals> order;
        
        if (objectsFound.size() == 3) {
            int goldPos = 0;
            int silverPos1 = 0;
            int silverPos2 = 0;
            
            for (Recognition object : objectsFound) {
                if (object.getLabel().equals("Gold Mineral")) {
                    goldPos = (int) object.getLeft();
                } else if (silverPos1 == 0) {
                    silverPos1 = (int) object.getLeft();
                } else {
                    silverPos2 = (int) object.getLeft();
                }
            }
            
            if (goldPos < silverPos1 && goldPos < silverPos2) {
                order.add(0, minerals.GOLD);
                order.add(1, minerals.SILVER);
                order.add(2, minerals.SILVER);
            } else if (silverPos1 < goldPos && goldPos < silverPos2) {
                order.add(0, minerals.SILVER);
                order.add(1, minerals.GOLD);
                order.add(2, minerals.Silver);
            } else if (silverPos1 < goldPos && silverPos2 < goldPos) {
                order.add(0, minerals.SILVER);
                order.add(1, minerals.SILVER);
                orer.add(2, minerals.GOLD);
            } else {
                return null;
            }
        }
        
        if (order.size() == 3) {
            return order;
        } else {
            return null;
        }
    }
    
    public void initVuforia() {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

}
