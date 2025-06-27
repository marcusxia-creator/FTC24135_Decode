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

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.TeleOps.coarsevisionproc.FindBestSample;
import org.firstinspires.ftc.teamcode.TeleOps.coarsevisionproc.Sample;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.opencv.core.Point;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/*
 * Demonstrates an empty iterative OpMode
 */
@TeleOp(name = "CoarseVisionTest")
public class CoarseVisionTest extends OpMode {

  private ElapsedTime runtime = new ElapsedTime();

  public Point slidePoint;

  public RobotHardware robot;

  public GamepadEx gamepad;

  public VisionPortal portal;
  public List<ColorBlobLocatorProcessor> useProcessors;

  public Sample bestSample;

  /**
   * This method will be called once, when the INIT button is pressed.
   */
  @Override
  public void init() {
    robot=new RobotHardware(hardwareMap);
    robot.init(hardwareMap);

    telemetry=new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    gamepad=new GamepadEx(gamepad1);

    ColorBlobLocatorProcessor blueColorLocator = new ColorBlobLocatorProcessor.Builder()
            .setTargetColorRange(org.firstinspires.ftc.vision.opencv.ColorRange.BLUE)         // use a predefined color match
            .setContourMode(ColorBlobLocatorProcessor.ContourMode.ALL_FLATTENED_HIERARCHY)    // exclude blobs inside blobs
            .setRoi(ImageRegion.asUnityCenterCoordinates(-1.0, 1.0, 1.0, -1.0))  // search central 1/4 of camera view
            .setDrawContours(true)                        // Show contours on the Stream Preview
            .setBlurSize(7)                               // Smooth the transitions between different colors in image
            .build();

    ColorBlobLocatorProcessor yellowColorLocator = new ColorBlobLocatorProcessor.Builder()
            .setTargetColorRange(org.firstinspires.ftc.vision.opencv.ColorRange.YELLOW)         // use a predefined color match
            .setContourMode(ColorBlobLocatorProcessor.ContourMode.ALL_FLATTENED_HIERARCHY)    // exclude blobs inside blobs
            .setRoi(ImageRegion.asUnityCenterCoordinates(-1.0, 1.0, 1.0, -1.0))  // search central 1/4 of camera view
            .setDrawContours(true)                        // Show contours on the Stream Preview
            .setBlurSize(7)                               // Smooth the transitions between different colors in image
            .build();

    ColorBlobLocatorProcessor redColorLocator = new ColorBlobLocatorProcessor.Builder()
            .setTargetColorRange(ColorRange.RED)         // use a predefined color match
            .setContourMode(ColorBlobLocatorProcessor.ContourMode.ALL_FLATTENED_HIERARCHY)    // exclude blobs inside blobs
            .setRoi(ImageRegion.asUnityCenterCoordinates(-1.0, 1.0, 1.0, -1.0))  // search central 1/4 of camera view
            .setDrawContours(true)                        // Show contours on the Stream Preview
            .setBlurSize(7)                               // Smooth the transitions between different colors in image
            .build();

    portal = new VisionPortal.Builder()
            .addProcessors(blueColorLocator,yellowColorLocator,redColorLocator)
            .setCameraResolution(new Size(320, 240))
            .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
            .build();

    useProcessors=new ArrayList<>(Arrays.asList(blueColorLocator,yellowColorLocator));

    telemetry.addData("Status", "Initialized");

    slidePoint=new Point(0,0);

    robot.intakeArmServo.setPosition(RobotActionConfig.intake_Arm_Coarse);
    robot.intakeWristServo.setPosition(RobotActionConfig.intake_Wrist_Coarse);

    advancedIntake.runToPoint(robot,slidePoint,DistanceUnit.INCH);
  }

  /**
   * This method will be called repeatedly during the period between when
   * the INIT button is pressed and when the START button is pressed (or the
   * OpMode is stopped).
   */
  @Override
  public void init_loop() {
    telemetry.addData("preview on/off", "... Camera Stream\n");

    bestSample= FindBestSample.findBestSample(useProcessors,RobotActionConfig.CamPos,RobotActionConfig.Arducam);

    if(bestSample!=null) {

      telemetry.addLine(" Area Density Aspect  Center");

      telemetry.addLine(String.format("%5d  %4.2f   %5.2f  (%3d,%3d)  (%3d,%3d)  (%3d,%3d)",
              bestSample.blob.getContourArea(), bestSample.blob.getDensity(), bestSample.blob.getAspectRatio(), (int) bestSample.blob.getBoxFit().center.x, (int) bestSample.blob.getBoxFit().center.y, (int) bestSample.ViscenterPoint.x, (int) bestSample.ViscenterPoint.y, (int) bestSample.relPos.x, (int) bestSample.relPos.y));

    }
    FtcDashboard.getInstance().startCameraStream(portal, 0);
    telemetry.update();
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

    robot.intakeArmServo.setPosition(RobotActionConfig.intake_Arm_Pick);
    robot.intakeWristServo.setPosition(RobotActionConfig.intake_Wrist_Grab);

    advancedIntake.runToPoint(robot,bestSample.relPos,DistanceUnit.CM);
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
