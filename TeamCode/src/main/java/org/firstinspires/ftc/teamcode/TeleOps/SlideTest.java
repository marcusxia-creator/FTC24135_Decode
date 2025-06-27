/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.TeleOps;

import org.opencv.core.Point;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/*
 * Demonstrates an empty iterative OpMode
 */
@TeleOp(name = "SlideTest")
public class SlideTest extends OpMode {

  private ElapsedTime runtime = new ElapsedTime();

  public Point slidePoint;

  public RobotHardware robot;

  GamepadEx gamepad;

  /**
   * This method will be called once, when the INIT button is pressed.
   */
  @Override
  public void init() {
    robot=new RobotHardware(hardwareMap);
    robot.init(hardwareMap);
    telemetry=new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    telemetry.addData("Status", "Initialized");
    gamepad=new GamepadEx(gamepad1);
    slidePoint=new Point(0,0);

    robot.intakeArmServo.setPosition(RobotActionConfig.intake_Arm_Pick);
    robot.intakeWristServo.setPosition(RobotActionConfig.intake_Wrist_Grab);

    advancedIntake.runToPoint(robot,slidePoint,DistanceUnit.INCH);
  }

  /**
   * This method will be called repeatedly during the period between when
   * the INIT button is pressed and when the START button is pressed (or the
   * OpMode is stopped).
   */
  @Override
  public void init_loop() {
  }

  /**
   * This method will be called once, when the START button is pressed.
   */
  @Override
  public void start() {
    runtime.reset();
  }

  /**
   * This method will be called repeatedly during the period between when
   * the START button is pressed and when the OpMode is stopped.
   */
  @Override
  public void loop() {
    telemetry.addData("Status", "Run Time: " + runtime.toString());

    slidePoint.x+=0.1*gamepad.getLeftX();
    if(slidePoint.x<-7){slidePoint.x=-7;}
    if(slidePoint.x>7){slidePoint.x=7;}
    telemetry.addData("X", slidePoint.x);

    slidePoint.y+=0.1*gamepad.getLeftY();
    if(slidePoint.y<0){slidePoint.y=0;}
    if(slidePoint.y>14){slidePoint.y=14;}
    telemetry.addData("Y", slidePoint.y);

    advancedIntake.runToPoint(robot,slidePoint,DistanceUnit.INCH);
  }

  /**
   * This method will be called once, when this OpMode is stopped.
   * <p>
   * Your ability to control hardware from this method will be limited.
   */
  @Override
  public void stop() {

  }
}
