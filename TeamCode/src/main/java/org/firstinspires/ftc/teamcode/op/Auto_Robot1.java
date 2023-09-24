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
package org.firstinspires.ftc.teamcode.op;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pathmaker.PathManager;
import org.firstinspires.ftc.teamcode.pathmaker.RobotPoseSimulation;
import org.firstinspires.ftc.teamcode.hw.DriveTrain;

@Config
@Autonomous
public class Auto_Robot1 extends LinearOpMode {

    public static double thisForwardPower = 0;
    public static double thisStrafePower = 0;
    public static double thisTurnPower = 0.2;
    public static double thisHeadingDrive = 0;
    public static int thisPathNumber = 0;
    public static int runTest_ms = 100;
    public static boolean runDriveTest = true;
    public static int thisNumberSteps = 2;
    public static int setZone = 1;



    static FtcDashboard dashboard = FtcDashboard.getInstance();
    static Telemetry telemetry = dashboard.getTelemetry();
    DriveTrain driveTrain = new DriveTrain(this);

    @Override
    public void runOpMode() throws InterruptedException {
        GameSetup.init("robot1","linear");
        //final TouchSensor limitSwitch;
        //limitSwitch = hardwareMap.get(TouchSensor.class, "limitSwitch");
        //final ColorRangeSensor colorRangeSensor;

        waitForStart();
        if (GameSetup.SIMULATION) {
            RobotPoseSimulation.initializePose(0, 0, 0);
        } else {
            RobotPose.initializePose(this, driveTrain, telemetry);
        }


        if (opModeIsActive()) {
            if (isStopRequested()) return;
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
                PathManager.moveRobot(dashboard,telemetry);
            }
        }
    }
}
