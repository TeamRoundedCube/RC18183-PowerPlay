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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


//Created by Kyran 11/20/2022 @ 2:09pm
//Purpose: Teleop for qualifier robot
//Original Code from 27th november 2022

@Disabled
@TeleOp(name = "ogTeleop")
public class originalTeleOp extends OpMode {
    HardwareFullBot robot = new HardwareFullBot();
    float turnPower;
    float forwardPower;
    float strafePower;
    double maxLeftSpeed = 0.7;
    double maxRightSpeed = 0.7;
    double maxArm = 1;
    double maxTurret = 0.15;


    // Code to run ONCE when the driver hits INIT
    @Override
    public void init() {

        robot.init(hardwareMap);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    //Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
    @Override
    public void init_loop() {
      //  robot.b_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      //  robot.b_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      //  robot.f_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      //  robot.f_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robot.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //robot.turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    //Code to run ONCE when the driver hits PLAY
    @Override
    public void start() {
        robot.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }


    //Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
    @Override
    public void loop() {
        //Telemetry Debugging

        // DRIVE MOTOR ENCODERS
            telemetry.addData("f_Left Encoder Position", robot.f_left.getCurrentPosition());
            telemetry.addData("f_Right Encoder Position", robot.f_right.getCurrentPosition());
            telemetry.addData("b_Left Encoder Position", robot.b_left.getCurrentPosition());
            telemetry.addData("b_Right Encoder Position", robot.b_right.getCurrentPosition());

        // ARM & TURRET MOTOR ENCODERS
        telemetry.addData("Arm", robot.arm.getCurrentPosition());
        telemetry.addData("turret", robot.turret.getCurrentPosition());

        // GYRO SENSOR HEADING
        //  telemetry.addData("gyro", robot.gyro.getHeading());

        telemetry.update();


        //Variables
        turnPower = -gamepad1.right_stick_x; //Turn robot
        forwardPower = -gamepad1.left_stick_y; //Forward and Back
        strafePower = -gamepad1.left_stick_x; //Left to Right

        //Gamepad1
        if (Math.abs(gamepad1.right_stick_x) > 0.1 ) {
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
        // Gamepad 2
        else if (Math.abs(gamepad2.left_stick_x) > 0.1) {
            if (robot.arm.getCurrentPosition() < 500) {}
            else {
                robot.turret.setPower(gamepad2.left_stick_x * maxTurret);
            }
        }
        else if (gamepad2.dpad_down) {
            moveArm(0.5, 0);
        } else if (gamepad2.dpad_up) {
            moveArm(1, 2100);
        } else if (gamepad2.y) {
            moveArm(1, 1500);
        } else if (gamepad2.a) {
            moveArm(1, 1000);
        }
        else if (gamepad1.right_trigger > 0.1) {
            robot.claw.setPosition(0);
        }
        else if (gamepad1.left_trigger > 0.1){
            robot.claw.setPosition(0.3);
        }
        else {
            robot.f_left.setPower(0);
            robot.b_left.setPower(0);
            robot.f_right.setPower(0);
            robot.b_right.setPower(0);
            robot.turret.setPower(0);
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
        robot.arm.setPower(speed * maxArm);
        robot.arm.setTargetPosition(target);
        robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //   while (robot.arm.isBusy()) {
        //       sleep(1);
        //   }
        //     robot.arm.setPower(0);
    }

}