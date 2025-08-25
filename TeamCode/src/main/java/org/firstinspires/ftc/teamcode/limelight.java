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

package org.firstinspires.ftc.teamcode;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvWebcam;
import com.acmerobotics.dashboard.FtcDashboard;


/*
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name="Limelight", group="Iterative OpMode")
//@Disabled
public class limelight extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontRight = null;
    private DcMotor frontLeft = null;
    private DcMotor backRight = null;
    private DcMotor backLeft = null;
    public int timer = 0;
    public double averageTA = 0;
    private Limelight3A limelight;


    public static double[] addX(int n, double arr[], double x) {
        double newarr[] = new double[n + 1];
        for (int i = 0; i < n; i++) {
            newarr[i] = arr[i];
        }

        newarr[n] = x;

        return newarr;
    }
    public double[] results = new double[1];

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Javi", "Initialized");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        //telemetry.setMsTransmissionInterval(11);
        //limelight.pipelineSwitch(0);
        //NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
        //String jsonResults = limelightTable.getEntry("json").getString("");
        //limelight.start();
        /*if (!jsonResults.isEmpty()) {
            try {
                // Parse the JSON string into a JSONObject
                JSONObject jsonObject = new JSONObject(jsonResults);

                // Access specific data within the JSON
                // Example for Detector results (assuming "Detector" key exists and contains an array)
                telemetry.addData("json", jsonObject);
                if (jsonObject.has("Detector")) {
                    JSONArray detectorArray = jsonObject.getJSONArray("Detector");
                    for (int i = 0; i < detectorArray.length(); i++) {
                        JSONObject detection = detectorArray.getJSONObject(i);
                        String className = detection.getString("class");
                        double confidence = detection.getDouble("conf");
                        double tx = detection.getDouble("tx");
                        double ty = detection.getDouble("ty");

                        telemetry.addData("Detected: " + className + ", Confidence: " + confidence, ", TX: " + tx + ", TY: " + ty);
                    }
                }

                // You can access other keys like "Fiducial", "ColorResults", etc., similarly
                // based on the Limelight JSON Results Specification:
                // https://docs.limelightvision.io/docs/docs-limelight/apis/json-results-specification

            } catch (Exception e) {
                telemetry.addData("Error parsing Limelight JSON: ", e.getMessage());
            }
        }
        else {
            telemetry.addData("No JSON results from Limelight.", "");
        }
         */

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        /*frontRight  = hardwareMap.get(DcMotor.class, "front_right");
        frontLeft = hardwareMap.get(DcMotor.class, "front_left");
        backRight = hardwareMap.get(DcMotor.class, "back_right");
        backLeft = hardwareMap.get(DcMotor.class, "back_left");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
*/
        // Tell the driver that initialization is complete.
        telemetry.addData("Javi", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits START
     */
    @Override
    public void start() {
    }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {
        // TIMER is here to sample average of data samples from the last 5 ticks (to eliminate noise potentially)
        timer += 1;
        // Setup a variable for each drive wheel to save power level for telemetry
        // Choose to drive using either Tank Mode, or POV Mode
        // Comment out the method that's not used.  The default below is POV.

        // POV Mode uses left stick to go forward, and right stick to turn.
        // - This uses basic math to combine motions and is easier to drive straight.
        //double drive = -gamepad1.left_stick_y;
        //double turn  =  gamepad1.right_stick_x;
        double drive = 0;
        double turn = 0;
        double strafe = 0;

        LLResult result = limelight.getLatestResult();
        telemetry.addData("result", result);

        double frPower = drive - turn - strafe;
        double flPower = drive + turn + strafe;
        double brPower = drive - turn + strafe;
        double blPower = drive + turn - strafe;
        // getting rid of smaller inputs
        if (frPower < 0.3 && frPower > -0.3) { frPower = 0; }
        if (flPower < 0.3 && flPower > -0.3) { flPower = 0; }
        if (brPower < 0.3 && brPower > -0.3) { brPower = 0; }
        if (blPower < 0.3 && blPower > -0.3) { blPower = 0; }
        /*
        frontRight.setPower(frPower);
        frontLeft.setPower(flPower);
        backRight.setPower(brPower);
        backLeft.setPower(blPower);
        */

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "time: " + runtime.toString());


    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        telemetry.addData("results", results);
    }


}
