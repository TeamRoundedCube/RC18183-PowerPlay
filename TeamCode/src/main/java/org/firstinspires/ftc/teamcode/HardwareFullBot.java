/*       ________                   ________           ________
        /        \                 /        \         |  _____ \
       /   ____   \               /   ____   \        | |     | \
      /   /    \   \             /   /    \   \       | |      \ \
     /   /      \   \           /   /      \   \      | |       \ \
    /   /________\   \         /   /________\   \     | |        | |
   /   /__________\   \       /   /__________\   \    | |       / /
  /   /            \   \     /   /            \   \   | |      / /
 /   /              \   \   /   /              \   \  | |_____| /
/   /                \   \ /   /                \   \ |________/
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorMRGyro;

public class HardwareFullBot {

    public DcMotorEx f_right;
    public DcMotorEx f_left;
    public DcMotorEx b_right;
    public DcMotorEx b_left;
    public DcMotorEx turret;
    public DcMotorEx arm;
    public Servo claw;

    public GyroSensor gyro;

    HardwareMap hwMap;

    public HardwareFullBot() {

    }

    public void init(HardwareMap ahwMap) {
        hwMap=ahwMap;

        // Define and Initialize Motors
        f_right  = hwMap.get(DcMotorEx.class, "f_right");
        f_left = hwMap.get(DcMotorEx.class, "f_left");
        b_right  = hwMap.get(DcMotorEx.class, "b_right");
        b_left = hwMap.get(DcMotorEx.class, "b_left");
        turret = hwMap.get(DcMotorEx.class, "turret");
        arm = hwMap.get(DcMotorEx.class, "arm");
        claw = hwMap.get(Servo.class, "claw");

        gyro = hwMap.gyroSensor.get("gyro");

        f_left.setDirection(DcMotorEx.Direction.FORWARD);
        b_left.setDirection(DcMotorEx.Direction.FORWARD);
        f_right.setDirection(DcMotorEx.Direction.REVERSE);
        b_right.setDirection(DcMotorEx.Direction.REVERSE);
      //  armleft.setDirection(DcMotorEx.Direction.FORWARD);
      //  armright.setDirection(DcMotorEx.Direction.REVERSE);


        //left.setPower(0);
        //right.setPower(0);


//        left.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
//        right.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        //   armright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //   armleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



 //       left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
 //       right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //   armleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //   armright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }
}

