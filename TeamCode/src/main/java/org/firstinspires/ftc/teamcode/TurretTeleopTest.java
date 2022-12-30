//  ____       ____  ____        ___   ______________             _______          ____       ____
// |    |    /    /  \   \      /  /  |    _______   \           /       \        |    \     |   |
// |    |   /    /    \   \    /  /   |   |       |   \         /   / \   \       |     \    |   |
// |    |  /    /      \   \  /  /    |   |_______|   /        /   /   \   \      |      \   |   |
// |    | /    /        \   \/  /     |     ___     _/        /   /     \   \     |   |\  \  |   |
// |    | \    \         |     |      |    |   \    \        /   /_______\   \    |   | \  \ |   |
// |    |  \    \        |     |      |    |    \    \      /   /_________\   \   |   |  \  \|   |
// |    |   \    \       |     |      |    |     \    \    /   /           \   \  |   |   \      |
// |____|    \____\      |_____|      |____|      \____\  /___/             \___\ |___|    \_____|

package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;


//Created by XXXX XX/XX/2022 @ X:XXpm
//Purpose: Teleop for qualifier robot

@TeleOp(name = "TurretTeleop")
public class TurretTeleopTest extends OpMode {
    HardwareFullBot robot = new HardwareFullBot();
    float turnPower;
    float forwardPower;
    float strafePower;
    double maxLeftSpeed = .6;
    double maxRightSpeed = .6;
    double maxTurret = 0.4;
    int armPosition = 0;
    int nextninety = 1;
    //int lastArmPosition = 0;
    final double positionConversionFactor = 375;
    double convertedArmPosition;
    boolean invertDirection = false;
    boolean cycleMode = true;


    // Code to run ONCE when the driver hits INIT
    @Override
    public void init() {

        robot.init(hardwareMap);

        telemetry.addData("Mode", cycleMode);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //  robot.gyro.calibrate();
        //  while(robot.gyro.isCalibrating()) {
        //      sleep(50);
        //  }
        // Tell the driver that initialization is complete.
     //   telemetry.addData("Status", "Initialized");
       // telemetry.update();
    }

    //Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
    @Override
    public void init_loop() {
        robot.b_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.b_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.f_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.f_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robot.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        if(cycleMode) {
            robot.turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        } else {
            robot.turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

    }

    //Code to run ONCE when the driver hits PLAY
    @Override
    public void start() {
        robot.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        if (cycleMode) {
             robot.turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }


    //Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
    @Override
    public void loop() {
        convertedArmPosition = (int)(((double)robot.turret.getCurrentPosition() / (double)positionConversionFactor) * -360);
        telemetry.addData("turret position", robot.turret.getCurrentPosition());
        telemetry.addData("corrected position", convertedArmPosition);
 //       telemetry.addData("leftstickx", gamepad2.left_stick_x);


        //    telemetry.addData("f_Left Encoder Position", robot.f_left.getCurrentPosition());
        //    telemetry.addData("f_Right Encoder Position", robot.f_right.getCurrentPosition());
        //    telemetry.addData("b_Left Encoder Position", robot.b_left.getCurrentPosition());
        //    telemetry.addData("b_Right Encoder Position", robot.b_right.getCurrentPosition());

        telemetry.addData("Arm", robot.arm.getCurrentPosition());
        telemetry.addData("Trigger", gamepad1.right_trigger);
        telemetry.addData("invert", invertDirection);
        //  telemetry.addData("gyro", robot.gyro.getHeading());
        telemetry.update();
        //Variables

        turnPower = -gamepad1.right_stick_x; //Turn robot
        forwardPower = -gamepad1.left_stick_y; //FOrward and Back
        strafePower = -gamepad1.left_stick_x; //FOrward and Back
        //Gamepad1
        if (invertDirection) {
            forwardPower *= -1;
            strafePower *= -1;
            turnPower *= -1;
        }
        if (Math.abs(gamepad1.right_stick_x) > 0.1) {

            robot.f_left.setPower(turnPower * maxLeftSpeed);
            robot.b_left.setPower(turnPower * maxLeftSpeed);
            robot.f_right.setPower(-turnPower * maxRightSpeed);
            robot.b_right.setPower(-turnPower * maxRightSpeed);
        }
        else if (Math.abs(gamepad1.left_stick_y) > 0.1) {
            // Forward (Left stick Y)
            robot.f_left.setPower(forwardPower * maxLeftSpeed);
            robot.b_left.setPower(forwardPower * maxLeftSpeed);
            robot.f_right.setPower(forwardPower * maxRightSpeed);
            robot.b_right.setPower(forwardPower * maxRightSpeed);
        }
        else if (Math.abs(gamepad1.left_stick_x) > 0.1) {
            // Forward (Left stick Y)
            robot.f_left.setPower(-strafePower * maxLeftSpeed);
            robot.b_left.setPower(strafePower * maxLeftSpeed);
            robot.f_right.setPower(strafePower * maxRightSpeed);
            robot.b_right.setPower(-strafePower * maxRightSpeed);
        }
        else if (gamepad1.right_trigger > 0.1) {
            robot.claw.setPosition(0);
            sleep(500);
        }
        else if (gamepad1.left_trigger > 0.1){
            robot.claw.setPosition(0.3);
            sleep(500);
        }
      //   Gamepad 2
       /*cheese yourmom
        how to get a girlfriend
        step one do nothing for 18 years
      big chungus wil haunt you for 2000 days*/
        else if (gamepad2.dpad_down) {
            moveArm(0.5, 0);
            if (cycleMode) {
                moveTurret(maxTurret, 0);
            }
            armPosition = 0;
            sleep(500);
        } else if (gamepad2.dpad_up) {
            moveArm(1, -3100); //-2800
            sleep(500);
            if (cycleMode) {
                moveTurret(maxTurret, -188);
            }
            armPosition = 3;
            sleep(500);
        } else if (gamepad2.y) {
            moveArm(1, -2100);
            sleep(500);
            if (cycleMode) {
                moveTurret(maxTurret, -188);
            }
            armPosition = 2;
            sleep(500);
        } else if (gamepad2.a) {
            moveArm(1, -1250);
            sleep(500);
            if (cycleMode) {
                moveTurret(maxTurret, -188);
            }
            armPosition = 1;
            sleep(500);
        }
        else if (!cycleMode && Math.abs(gamepad2.left_stick_x) > 0.1) {
            if (armPosition > 0) {
                    robot.turret.setPower(gamepad2.left_stick_x * maxTurret);
            }
        }
    //    else if (cycleMode && gamepad2.left_bumper) {
    //        if (armPosition > 0) {
    //            moveTurret(maxTurret, -94);//(-94 * nextninety));
             //   nextninety++;
    //        }
    //    }
    //    else if (cycleMode && gamepad2.right_bumper) {
    //        if (armPosition > 0) {
    //            moveTurret(maxTurret, 94);//(94 * nextninety));
            //    nextninety--;
    //        }
    //    }
        else {
            robot.f_left.setPower(0);
            robot.b_left.setPower(0);
            robot.f_right.setPower(0);
            robot.b_right.setPower(0);
            robot.turret.setPower(0);

        }
       /* if (armPosition > 0){
            if (gamepad2.left_stick_x > 0.1 && convertedArmPosition < 270) {
                robot.turret.setPower(gamepad2.left_stick_x * 0.7);
                lastArmPosition = robot.turret.getCurrentPosition();
            }
            else if (gamepad2.left_stick_x < -0.1 && convertedArmPosition > -5) {
                robot.turret.setPower(gamepad2.left_stick_x * 0.7);
                lastArmPosition = robot.turret.getCurrentPosition();
            }
            else{
                robot.turret.setTargetPosition(lastArmPosition);
            }

        }
        else{
            if (Math.cos(Math.toRadians(convertedArmPosition)) > 0) {
                invertDirection = false;
                robot.turret.setTargetPosition(0);

            }
            else {
                robot.turret.setTargetPosition(180);
                invertDirection = true;
            }
        }*/
     //   if(nextninety == 0) {
      //      nextninety = 1;
      //  }
        if (Math.cos(Math.toRadians(convertedArmPosition)) > 0) {
            invertDirection = false;

        }
        else {
            invertDirection = true;
        }

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        robot.arm.setPower(0);
        robot.arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public final void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    public void moveArm(double speed, int target) {
        robot.arm.setPower(speed);
        robot.arm.setTargetPosition(target);
        robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //   while (robot.arm.isBusy()) {
        //       sleep(1);
        //   }
        //     robot.arm.setPower(0);
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
}