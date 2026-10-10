package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

@Config
@TeleOp(name = "Limelight 3A Python Test", group = "Test")
public class limelight extends OpMode {

    private Limelight3A limelight;

    private DcMotorEx frontLeftMotor;
    private DcMotorEx frontRightMotor;
    private DcMotorEx backLeftMotor;
    private DcMotorEx backRightMotor;

    // Tracking variables
    private double customTx = 0.0;
    private double customTy = 0.0;
    private double customRadius = 0.0;
    private double standardTx = 0.0;
    private boolean targetFound = false;

    private double imageCenterX = 320.0;
    private double horizontalFOV = 54.5;

    private double kP = 0.02;
    private double deadBand = 0.7;


    @Override
    public void init() {
        // 1. Hardware Map Setup (Must match your Robot Configuration name)
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        frontLeftMotor = hardwareMap.get(DcMotorEx.class, "FL_Motor");
        frontRightMotor = hardwareMap.get(DcMotorEx.class, "FR_Motor");
        backLeftMotor = hardwareMap.get(DcMotorEx.class, "BL_Motor");
        backRightMotor = hardwareMap.get(DcMotorEx.class, "BR_Motor");

        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeftMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER); // set motor mode
        backLeftMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER); //set motor mode
        frontRightMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER); // set motor mode
        backRightMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER); // set motor mode

        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // 2. Configure Limelight settings
        limelight.setPollRateHz(200);  // High refresh rate for responsive updates
        limelight.pipelineSwitch(5);   // Ensure this matches your Python pipeline index
        limelight.start();

        telemetry.addData("Limelight", "Initialized on Pipeline 5");
        telemetry.update();
    }

    @Override
    public void loop() {
        /**
        LLResult result = limelight.getLatestResult();

        // Layer 1: Hardware Connection Check
        if (result == null) {
            telemetry.addData("Status", "DISCONNECTED / NO DATA FROM HARDWARE");
            telemetry.addData("Fix", "Check USB/Ethernet connection & hardwareMap name");
        }
        /*
        // Layer 2: Pipeline Target Check
        else if (!result.isValid()) {
            telemetry.addData("Status", "CONNECTED - NO TARGET OR PIPELINE CRASH");
            telemetry.addData("Fix", "Check Limelight Web UI log for Python script errors");
        }
        */
        // Layer 3: Valid Target Found
        /**
        else {
            standardTx = result.getTx();
            double[] pythonData = result.getPythonOutput();

            // Check if the custom pythonData array was returned properly
            if (pythonData == null) {
                telemetry.addData("Status", "TARGET LOCKED - BUT pythonData IS NULL");
                telemetry.addData("Fix", "Ensure pipeline type is set to 'Python' in Web UI");
            }
            else if (pythonData.length < 8) {
                telemetry.addData("Status", "TARGET LOCKED - INVALID ARRAY LENGTH");
                telemetry.addData("Array Length Received", pythonData.length);
                telemetry.addData("Fix", "Python script must return exactly 8 elements in llpython");
            }
            else {
                // Successfully parsed 8-element Python output!
                targetFound = (pythonData[0] == 1.0);

                if (targetFound) {
                    customTx = pythonData[1];     // Custom X center (Pixel coordinate)
                    customTy = pythonData[2];     // Custom Y center (Pixel coordinate)
                    customRadius = pythonData[3]; // Custom Radius

                    telemetry.addData("Status", "TARGET LOCKED & PYTHON ARRAY READ SUCCESS");
                    telemetry.addData("Standard tx (Deg)", standardTx);
                    telemetry.addData("Custom Python X (Pixel)", customTx);
                    telemetry.addData("Custom Python Y (Pixel)", customTy);
                    telemetry.addData("Custom Radius", customRadius);

                    // Example steering math using custom Python X
                    double imageCenterX = 320.0; // Adjust for your frame width (e.g., 640/2)
                    double errorX = customTx - imageCenterX;
                    double turnPower = errorX * 0.002;

                    telemetry.addData("Calculated Steering Power", turnPower);
                } else {
                    telemetry.addData("Status", "CONNECTED - Python returned hasTarget = 0");
                }
            }
         }

        telemetry.update();
         */

        LLResult result = limelight.getLatestResult();

        if (result != null) {
            standardTx = result.getTx();
            double[] pythonData = result.getPythonOutput();

            targetFound = (pythonData[0] == 1.0);

            if (targetFound) {
                customTx = pythonData[1];     // Custom X center (Pixel coordinate)
                customTy = pythonData[2];     // Custom Y center (Pixel coordinate)
                customRadius = pythonData[3]; // Custom Radius

                double pixelOffset = customTx - imageCenterX;
                double customTxDeg = (pixelOffset / imageCenterX) * (horizontalFOV / 2.0);

                telemetry.addData("Status", "TARGET LOCKED & PYTHON ARRAY READ SUCCESS");
                telemetry.addData("Standard tx (Deg)", standardTx);
                telemetry.addData("Custom Python X (Deg)", customTxDeg);
                telemetry.addData("Custom Python X (Pixel)", customTx);
                telemetry.addData("Custom Python Y (Pixel)", customTy);
                telemetry.addData("Custom Radius", customRadius);

                double turnPower;

                if (Math.abs(customTxDeg) < deadBand) {
                    turnPower = 0;
                }
                else {
                    turnPower = customTxDeg * kP;
                }
                
                turnPower = Range.clip(turnPower, -0.5, 0.5);

                frontLeftMotor.setPower(turnPower);
                backLeftMotor.setPower(turnPower);
                frontRightMotor.setPower(-turnPower);
                backRightMotor.setPower(-turnPower);

            }
            else {
                telemetry.addData("Status", "CONNECTED - Python returned hasTarget = 0");
            }
        }
    }

    @Override
    public void stop() {
        if (limelight != null) {
            limelight.stop();
        }
    }
}