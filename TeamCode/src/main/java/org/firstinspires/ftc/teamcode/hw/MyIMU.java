// MIT License
// Copyright (c) 2023 bayrobotics.org
//
package org.firstinspires.ftc.teamcode.hw;
//import com.qualcomm.hardware.bosch.BHI260IMU;
//import com.qualcomm.hardware.bosch.BNO055IMU;

import static com.qualcomm.hardware.rev.RevHubOrientationOnRobot.xyzOrientation;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class MyIMU {
    // see https://stemrobotics.cs.pdx.edu/node/7266.html
    //BNO055IMU imu;
    //BHI260IMU imu2;
    static  IMU imu;
    Orientation             lastAngles = new Orientation();
    double                  globalAngle;
    private LinearOpMode myOpMode = null;   // gain access to methods in the calling OpMode.
    public MyIMU(LinearOpMode opMode){
        myOpMode = opMode;
    }
    public void setOpMode(LinearOpMode opMode){
        myOpMode = opMode;
    }
    public static YawPitchRollAngles orientation;
    public static AngularVelocity angularVelocity;


    public void initMyIMU(){
        imu = myOpMode.hardwareMap.get(IMU.class, "imu");
        double xRotation = 0;  // enter the desired X rotation angle here.
        double yRotation = 0;  // enter the desired Y rotation angle here.
        double zRotation = 0;  // enter the desired Z rotation angle here.
        Orientation hubRotation = xyzOrientation(xRotation, yRotation, zRotation);
        // Now initialize the IMU with this mounting orientation
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(hubRotation);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        // sdk v 8 ... ??
        // Retrieve the IMU from the hardware map
        // imu = myOpMode.hardwareMap.get(IMU.class, "imu");
        //IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
        //        RevHubOrientationOnRobot.LogoFacingDirection.UP,
        //        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
    }
    public void resetAngle()
    {
        globalAngle = 0;
        imu.resetYaw();
    }
    public static void updateTelemetry(Telemetry telemetry){
        // Retrieve Rotational Angles and Velocities
        // Retrieve Rotational Angles and Velocities
        orientation = imu.getRobotYawPitchRollAngles();
        angularVelocity = imu.getRobotAngularVelocity(AngleUnit.DEGREES);

        telemetry.addData("Yaw (Z)", "%.2f Deg. (Heading)", orientation.getYaw(AngleUnit.DEGREES));
        telemetry.addData("Pitch (X)", "%.2f Deg.", orientation.getPitch(AngleUnit.DEGREES));
        telemetry.addData("Roll (Y)", "%.2f Deg.\n", -90+orientation.getRoll(AngleUnit.DEGREES));
        telemetry.addData("Yaw (Z) velocity", "%.2f Deg/Sec", angularVelocity.zRotationRate);
        telemetry.addData("Pitch (X) velocity", "%.2f Deg/Sec", angularVelocity.xRotationRate);
        telemetry.addData("Roll (Y) velocity", "%.2f Deg/Sec", angularVelocity.yRotationRate);
        telemetry.update();

    }
    public double getAngle_deg()
    {
        // X (firstAngle) axis is for heading with controller logo to the left and USB backwards
        // Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();
        return -angles.getYaw(AngleUnit.DEGREES);
    }
    public double getAngle_rad(){
        return getAngle_deg() / 180 * Math.PI;
    }
}
