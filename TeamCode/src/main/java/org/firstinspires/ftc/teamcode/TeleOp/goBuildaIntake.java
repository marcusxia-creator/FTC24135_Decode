package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp
public class goBuildaIntake extends OpMode {

    private DcMotorEx intakeMotor;
    private double power = 0;

    private ElapsedTime debounceTimer = new ElapsedTime();
    private double DEBOUNCE_THRESHOLD = 0.25;

    private int target_tick = 200;//mm
    //1150 RPM

    @Override
    public void init() {
        intakeMotor = hardwareMap.get(DcMotorEx.class, "Intake_Motor");
        //intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //intakeMotor.setTargetPositionTolerance(4);
    }

    @Override
    public void loop() {
        if (gamepad1.dpad_right && debounceTimer.seconds() >= DEBOUNCE_THRESHOLD) {
            debounceTimer.reset();
            power += 0.1;
        }

        if (gamepad1.dpad_left && debounceTimer.seconds() >= DEBOUNCE_THRESHOLD) {
            debounceTimer.reset();
            power -= 0.1;
        }

        /*
        if (gamepad1.dpad_up && debounceTimer.seconds() >= DEBOUNCE_THRESHOLD) {
            debounceTimer.reset();
            target_tick += 10;
        }

        if (gamepad1.dpad_down && debounceTimer.seconds() >= DEBOUNCE_THRESHOLD) {
            debounceTimer.reset();
            target_tick -= 10;
        }
         */

        if (gamepad1.a && debounceTimer.seconds() >= DEBOUNCE_THRESHOLD) {
            debounceTimer.reset();
            intakeMotor.setPower(power);
            int driveTick = target_tick + intakeMotor.getCurrentPosition();
            intakeMotor.setTargetPosition(driveTick);
            intakeMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        if (gamepad1.b && debounceTimer.seconds() >= DEBOUNCE_THRESHOLD) {
            debounceTimer.reset();
            intakeMotor.setPower(power);
            int driveTick = intakeMotor.getCurrentPosition() - target_tick;
            intakeMotor.setTargetPosition(driveTick);
            intakeMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        power = Range.clip(power, -1, 1);
        intakeMotor.setPower(power);

        telemetry.addData("motor_power", power);
        telemetry.addData("motor_velocity", intakeMotor.getVelocity());
        telemetry.addData("target_tick", target_tick);
        telemetry.addData("motor_position", intakeMotor.getCurrentPosition());
        telemetry.addLine("dpad up for increase target tick");
        telemetry.addLine("dpad dow for decrease target tick");
        telemetry.addLine("dpad right for increase power");
        telemetry.addLine("dpad left for decrease power");
        telemetry.addLine("a for slides up");
        telemetry.addLine("b for slides down");
        telemetry.update();
    }
}
