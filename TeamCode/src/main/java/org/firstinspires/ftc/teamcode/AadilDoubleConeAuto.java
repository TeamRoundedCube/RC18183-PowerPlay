package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;


@Autonomous
public class AadilDoubleConeAuto extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    HardwareFullBot robot = new HardwareFullBot();

    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    int LEFT = 1;
    int MIDDLE = 2;
    int RIGHT = 3;

    AprilTagDetection tagOfInterest = null;


    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        robot.b_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.b_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.f_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.f_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.f_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.f_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.b_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.b_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(960,720, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        while (!isStarted() && !isStopRequested())
        {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if(currentDetections.size() != 0)
            {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections)
                {
                    if(tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT)
                    {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if(tagFound)
                {
                    tagToTelemetry(tagOfInterest);
                }
                else
                {

                    if(tagOfInterest == null)
                    {
                        telemetry.addLine("(The tag has never been seen)");
                    }
                    else
                    {
                        telemetry.addLine("\nwe HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            }
            else
            {

                if(tagOfInterest == null)
                {
                }
                else
                {
                    telemetry.addLine("\nwe HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.addData("Left Encoder Position", robot.f_left.getCurrentPosition());
            telemetry.addData("Right Encoder Position", robot.f_right.getCurrentPosition());            telemetry.update();
            sleep(20);
            robot.claw.setPosition(0.3);
        }


        waitForStart();

        if (opModeIsActive()) {
            strafeLeft(0.5,1430); //1430
            moveArm(1, -2150);
            driveStraight(0.3,100);
            robot.claw.setPosition(0);
            sleep(500);
            driveStraight(0.3,-100);
            robot.claw.setPosition(0.3);
            sleep(500);
            strafeRight(0.5,-350); //1430
            sleep(200);
            moveTurret(0.4, -188);
            moveArm(0.7,-550);
            robot.claw.setPosition(0);
            driveStraight(.5, -800);
            robot.claw.setPosition(0.3);
            moveArm(0.7,-2000);
            driveStraight(.5, 800);
            moveTurret(0.4, 40);
            moveArm(.7, 3100);


            //    strafeLeft(1,1500);s

            //  moveArm(1, -2800);  //3100 is maximum
        /*    driveStraight(0.2, 200);
            robot.claw.setPosition(0);
            driveBack(0.2, 200);
            tagToTelemetry(tagOfInterest);
            strafeLeft(1,500);*/
            //  sleep(3000);

            //  moveTurret(0.4, -170);
            //  sleep(2000);
            //  moveTurret(0.2, 0);
            // sleep(3000);
            // moveArm(0.2, 0);
//            if (tagOfInterest.id == MIDDLE) {
//            }
//            if (tagOfInterest.id == LEFT){
//                //  strafeLeft(1,500);
//                driveStraight(0.5, -700);
//            }
//            if (tagOfInterest.id == RIGHT){
//                //          strafeLeft(1,400);
//                driveStraight(0.5, 600);
//            }
//            if (tagOfInterest.id != MIDDLE || tagOfInterest.id != LEFT || tagOfInterest.id != RIGHT){
//                //  driveBack(0.5, 400);
//                //  System.out.println("hggyg");
//            }

        }

    }
    public void driveBack(double speed, int distance){
        double maxLeftSpeed = 1;
        double maxRightSpeed = 1;
        robot.f_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.f_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.b_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.b_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.f_left.setTargetPosition(-distance);
        robot.b_left.setTargetPosition(-distance);
        robot.f_right.setTargetPosition(-distance);
        robot.b_right.setTargetPosition(-distance);
        robot.f_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.b_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.f_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.b_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.f_left.setPower(maxLeftSpeed * speed);
        robot.b_left.setPower(maxLeftSpeed * speed);
        robot.f_right.setPower(maxRightSpeed * speed);
        robot.b_right.setPower(maxRightSpeed * speed);
        while (opModeIsActive() && robot.f_left.isBusy() && robot.f_right.isBusy()) {
            idle();
        }
        robot.f_left.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        robot.b_left.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        robot.f_right.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        robot.b_right.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        robot.f_left.setPower(0);
        robot.b_left.setPower(0);
        robot.f_right.setPower(0);
        robot.b_right.setPower(0);
    }
    public void driveStraight(double speed, int distance){
        double maxLeftSpeed = 1;
        double maxRightSpeed = 1;
        robot.f_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.f_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.b_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.b_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.f_left.setTargetPosition(distance);
        robot.b_left.setTargetPosition(distance);
        robot.f_right.setTargetPosition(distance);
        robot.b_right.setTargetPosition(distance);
        robot.f_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.b_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.f_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.b_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.f_left.setPower(maxLeftSpeed * speed);
        robot.b_left.setPower(maxLeftSpeed * speed);
        robot.f_right.setPower(maxRightSpeed * speed);
        robot.b_right.setPower(maxRightSpeed * speed);
        while (opModeIsActive() && robot.f_left.isBusy() && robot.f_right.isBusy()) {
            idle();
        }
        robot.f_left.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        robot.b_left.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        robot.f_right.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        robot.b_right.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        robot.f_left.setPower(0);
        robot.b_left.setPower(0);
        robot.f_right.setPower(0);
        robot.b_right.setPower(0);
    }

    public void turnLeft(double speed, double distance){
        double maxLeftSpeed = 1;
        double maxRightSpeed = 1;

        // robot.left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // robot.right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        if (opModeIsActive()) {
            double i = robot.f_right.getCurrentPosition();
            double j = robot.f_left.getCurrentPosition();
            double leftTarget = j + distance;
            double rightTarget = i - distance;


            while (opModeIsActive() && i > rightTarget && j < leftTarget) {
                if(i > rightTarget){
                    robot.f_right.setPower(-(maxRightSpeed * speed));
                    robot.b_right.setPower(-(maxRightSpeed * speed));
                    i = robot.f_right.getCurrentPosition();

                }
                if(j < leftTarget) {
                    robot.f_left.setPower(maxLeftSpeed * speed);
                    robot.b_left.setPower(maxLeftSpeed * speed);
                    j = robot.f_left.getCurrentPosition();

                }


            }

            robot.f_left.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            robot.b_left.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            robot.f_right.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            robot.b_right.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            robot.f_left.setPower(0);
            robot.b_left.setPower(0);
            robot.f_right.setPower(0);
            robot.b_right.setPower(0);
        }
    }

    /**
     * Turns the robot right based on the given parameters speed and seconds.
     * @param speed
     * @param distance
     */
    public void turnRight(double speed, double distance){
        double maxLeftSpeed = 1;
        double maxRightSpeed = 1;

        // robot.left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // robot.right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        if (opModeIsActive()) {
            double i = robot.f_right.getCurrentPosition();
            double j = robot.f_left.getCurrentPosition();
            double leftTarget = j - distance;
            double rightTarget = i + distance;


            while (opModeIsActive() && i < rightTarget && j > leftTarget) {
                if(i < rightTarget){
                    robot.f_right.setPower(maxRightSpeed * speed);
                    robot.b_right.setPower(maxRightSpeed * speed);
                    i = robot.f_right.getCurrentPosition();
                }
                if(j>leftTarget) {
                    robot.f_left.setPower(-(maxLeftSpeed * speed));
                    robot.b_left.setPower(-(maxLeftSpeed * speed));
                    j = robot.f_left.getCurrentPosition();
                }


            }


            robot.f_left.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            robot.b_left.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            robot.f_right.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            robot.b_right.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            robot.f_left.setPower(0);
            robot.b_left.setPower(0);
            robot.f_right.setPower(0);
            robot.b_right.setPower(0);
        }
    }
    /**
     * strafe the robot right based on the given parameters speed and seconds.
     * @param speed
     * @param distance
     */
    public void strafeLeft(double speed, int distance){
        double maxLeftSpeed = 1;
        double maxRightSpeed = 1;
        robot.f_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.f_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.b_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.b_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.f_left.setTargetPosition(-distance);
        robot.b_left.setTargetPosition(distance);
        robot.f_right.setTargetPosition(distance);
        robot.b_right.setTargetPosition(-distance);
        robot.f_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.b_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.f_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.b_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.f_left.setPower(maxLeftSpeed * speed);
        robot.b_left.setPower(maxLeftSpeed * speed);
        robot.f_right.setPower(maxRightSpeed * speed);
        robot.b_right.setPower(maxRightSpeed * speed);
        while (opModeIsActive() && robot.f_left.isBusy() && robot.f_right.isBusy()) {
            idle();
        }
        robot.f_left.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        robot.b_left.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        robot.f_right.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        robot.b_right.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        robot.f_left.setPower(0);
        robot.b_left.setPower(0);
        robot.f_right.setPower(0);
        robot.b_right.setPower(0);
    }

    /**
     * Strafe the robot left based on the given parameters speed and seconds.
     * @param speed
     * @param distance
     */
    public void strafeRight(double speed, int distance){
        double maxLeftSpeed = 1;
        double maxRightSpeed = 1;
        robot.f_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.f_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.b_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.b_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.f_left.setTargetPosition(distance);
        robot.b_left.setTargetPosition(-distance);
        robot.f_right.setTargetPosition(-distance);
        robot.b_right.setTargetPosition(distance);
        robot.f_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.b_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.f_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.b_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.f_left.setPower(maxLeftSpeed * speed);
        robot.b_left.setPower(maxLeftSpeed * speed);
        robot.f_right.setPower(maxRightSpeed * speed);
        robot.b_right.setPower(maxRightSpeed * speed);
        while (opModeIsActive() && robot.f_left.isBusy() && robot.f_right.isBusy()) {
            idle();
        }
        robot.f_left.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        robot.b_left.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        robot.f_right.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        robot.b_right.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        robot.f_left.setPower(0);
        robot.b_left.setPower(0);
        robot.f_right.setPower(0);
        robot.b_right.setPower(0);
    }


    public void moveArm(double speed, int target) {
        robot.arm.setPower(speed);
        robot.arm.setTargetPosition(target);
        robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (robot.arm.isBusy()) {
            sleep(1);
        }
        robot.arm.setPower(0);
    }
    public void moveTurret(double speed, int target) {
        robot.turret.setPower(speed);
        // robot.turret.setTargetPosition((target/360) * (int)positionConversionFactor);
        robot.turret.setTargetPosition(target);
        robot.turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //       while (robot.turret.isBusy()) {

        //        }
        //         robot.turret.setPower(0);
    }
    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
    }
}