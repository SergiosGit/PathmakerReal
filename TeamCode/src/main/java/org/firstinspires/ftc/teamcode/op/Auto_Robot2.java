//
// PathMaker.java
//
// This is the main module showing the use of the PathMaker library that can be used
// for autonomous control of a Mecanum wheel drive train for the FTC competition.
// This implementation uses the FTC Dashboard for developing the robot path settings
// including simulating robot movements that can be displayed on the dashboard.
//
// MIT License
// Copyright (c) 2023 bayrobotics.org
//

// This is for robot 2, the second robot built by the Bay team in the 22/23 season

package org.firstinspires.ftc.teamcode.op;

import android.graphics.Color;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.hw.DriveTrain;
import org.firstinspires.ftc.teamcode.hw.MyIMU;
import org.firstinspires.ftc.teamcode.pathmaker.PathManager;
import org.firstinspires.ftc.teamcode.pathmaker.RobotPoseSimulation;

//@Autonomous(name = "Robot2 Testing", group = "Test")
@TeleOp(name = "Robot2 Testing", group = "Test")
@Config
public class Auto_Robot2 extends LinearOpMode {

    public static double thisForwardPower = 0;
    public static double thisStrafePower = 0;
    public static double thisTurnPower = 0.2;
    public static double thisHeadingDrive = 0;
    public static int thisPathNumber = 0;
    public static int runTest_ms = 100;
    public static boolean runDriveTest = true;
    public static int thisNumberSteps = 2;
    public static int setZone = 1;
    public static String rampAlgorithm = "linear";
    public static boolean runAgain = true;


    static FtcDashboard dashboard = FtcDashboard.getInstance();
    static Telemetry telemetry = dashboard.getTelemetry();
    DriveTrain driveTrain = new DriveTrain(this);

    @Override
    public void runOpMode() throws InterruptedException {
        GameSetup.init("robot2",rampAlgorithm);
        //final TouchSensor limitSwitch;
        //limitSwitch = hardwareMap.get(TouchSensor.class, "limitSwitch");

        // Initialize Color Sensor
        final NormalizedColorSensor colorSensor;
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "colorSensor");
        // If necessary, turn ON the white LED (if there is no LED switch on the sensor)
        if (colorSensor instanceof SwitchableLight) {
            ((SwitchableLight) colorSensor).enableLight(true);
        }
        colorSensor.setGain(15);
        NormalizedRGBA colors = colorSensor.getNormalizedColors();
        final float[] hsvValues = new float[3];

        // Initialize pose, including IMU
        if (GameSetup.SIMULATION) {
            RobotPoseSimulation.initializePose(0, 0, 0);
        } else {
            RobotPose.initializePose(this, driveTrain, telemetry);
        }

        int j=0;

        while (opModeInInit()){
            if (colors == null) {
                telemetry.addData("null colors",0);
            } else {
                // Update the hsvValues array by passing it to Color.colorToHSV()
                colorSensor.getNormalizedColors();
                Color.colorToHSV(colors.toColor(), hsvValues);
                //telemetry.addData("limitSwitch",limitSwitch.isPressed());
                telemetry.addData("A reading sensors...", j++);
                telemetry.addData("Colors Alpha", "%.3f", colors.alpha);
                telemetry.addData("Colors Red", "%.3f", colors.red);
                telemetry.addData("Colors Green", "%.3f", colors.green);
                telemetry.addData("Colors Blue", "%.3f", colors.blue);
                telemetry.addData("hsv[0] Hue", "%.3f", hsvValues[0]);
                telemetry.addData("hsv[1] Saturation", "%.3f", hsvValues[1]);
                telemetry.addData("hsv[2] Value", "%.3f", hsvValues[2]);
                /* If this color sensor also has a distance sensor, display the measured distance.
                 * Note that the reported distance is only useful at very close range, and is impacted by
                 * ambient light and surface reflectivity. */
                if (colorSensor instanceof DistanceSensor) {
                    telemetry.addData("Distance (cm)", "%.3f", ((DistanceSensor) colorSensor).getDistance(DistanceUnit.CM));
                }
            }
            MyIMU.updateTelemetry(telemetry);
            telemetry.update();
            sleep(200);
        }

        waitForStart();

        if (opModeIsActive()) {
            while (!isStopRequested()) {
                if (runAgain) {
                    if (runDriveTest) {
                        telemetry.addData("runDriveTest, thisPathNumber", thisPathNumber);
                        RobotPose.initializePose(this, driveTrain, telemetry);
                        if (thisPathNumber == -1) {
                            WheelPowerManager.setDrivePower(driveTrain, thisForwardPower, thisStrafePower, thisTurnPower, thisHeadingDrive);
                            RobotPose.readPose();
                        } else if (thisPathNumber == 0) {
                            // use dashboard parameters
                            ParallelAction.init();
                            PathManager.moveRobot(dashboard, telemetry);
                            MyIMU.updateTelemetry(telemetry);
                            UpdateTelemetry.params(telemetry);
                            sleep(runTest_ms);
                        } else if (thisPathNumber == 1) {
                            telemetry.addData("runDriveTest, thisPathNumber", thisPathNumber);
                            PathDetails.setPath_startToJunction();
                            PathManager.moveRobot(dashboard, telemetry);
                        }
                        WheelPowerManager.setDrivePower(driveTrain, 0, 0, 0, 0);
                    } else {
                        PathDetails.setPath_startToJunction();
                        PathManager.moveRobot(dashboard, telemetry);
                        for (int i = 0; i < thisNumberSteps; i++) {

                            PathDetails.setPath_junctionDeliver();
                            PathManager.moveRobot(dashboard, telemetry);

                            PathDetails.setPath_junctionBackOff();
                            PathManager.moveRobot(dashboard, telemetry);

                            PathDetails.setPath_junctionToStack();
                            PathManager.moveRobot(dashboard, telemetry);

                            PathDetails.setPath_stack();
                            PathManager.moveRobot(dashboard, telemetry);

                            PathDetails.setPath_stackToJunction();
                            PathManager.moveRobot(dashboard, telemetry);
                        }
                        PathDetails.setPath_parking(setZone);
                        PathManager.moveRobot(dashboard, telemetry);
                    }
                    runAgain = false;
                } // end if runAgain
                sleep(200);
            }
        }
    }
}
