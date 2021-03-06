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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;

/**
 * This file provides the teleop driving for margarine bot.
 * The code is structured as an Iterative OpMode
 *
 * This OpMode uses the common Pushbot hardware class to define the devices on the robot.
 * All device access is managed through the Hardware_2017 class.
 *
 * This particular OpMode executes a basic omni drive base
 *
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Teleop: Margarine bot", group="Margarine")
//@Disabled
public class Teleop_2017 extends OpMode{

    /* Declare OpMode members. */
    Hardware_2017 robot       = new Hardware_2017(); // use the class created to define a Pushbot's hardware

    double leftsticky;
    double leftstickx;
    double armsticky;
    double liftstick;
    double RT;
    double LT;
    double turn;
    double left;
    double right;
    double multiplier;
    double temp;
    boolean clicked = false;
    int state = 1;

    double          clawOffset  = 0.5 ;
    final double    CLAW_SPEED  = 0.005 ;  // sets rate to move servo
    double          clawOffset2 = 0.665;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {

    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

        //Get controller input
        leftsticky = -gamepad1.left_stick_y;
        leftstickx = gamepad1.left_stick_x;
        turn = -gamepad1.right_stick_x;
        RT = gamepad1.right_trigger;
        LT = gamepad1.left_trigger;
        armsticky = -gamepad2.left_stick_y;
        liftstick = -gamepad2.right_stick_y;

        //Mutate linear input into a cubic curve
        leftstickx = Math.pow(leftstickx, 3);
        leftsticky = Math.pow(leftsticky, 3);
        RT         = Math.pow(RT, 3);
        LT         = Math.pow(LT, 3);
        armsticky  = Math.pow(armsticky, 5);
        liftstick  = Math.pow(liftstick, 5);

        //Set motor powers to be in interavls of 0.2
        ranged(leftstickx);
        ranged(leftsticky);

        //drive robot
        drive();



        //Set value for turning while moving
        if (turn > 0.2) {
            right = 1;
            left = -turn;
        } else if (turn < -0.2) {
            right = turn;
            left = 1;
        } else {
            right = 1;
            left = 1;
        }

        //Check if robot is not moving
        if (leftstickx == 0 && leftsticky == 0) {
            multiplier = 1;
        } else {
            multiplier = 0;
        }



        //Use gamepad left & right Bumpers to open and close the claw
        if (gamepad1.right_bumper || gamepad2.right_bumper)
            clawOffset -= CLAW_SPEED;
        else if (gamepad1.left_bumper || gamepad2.left_bumper)
            clawOffset += CLAW_SPEED;

        //Move both servos to new position.  Assume servos are mirror image of each other.
        clawOffset = Range.clip(clawOffset, 0, 1);
        robot.Right.setPosition(1 - clawOffset);
        robot.Left.setPosition(clawOffset);

        //for one driver
        //Change trigger setting, 1 as default
        //1 -- move lower arm
        //2 -- move upper arm
        //Check trigger setting then set motor power
//        if ((gamepad1.a || gamepad2.a) && !clicked) {
//            clicked = true;
//        } else if ((!gamepad1.a || !gamepad2.a) && clicked) {
//            clicked = false;
//            if (state == 1) {
//                state = 2;
//            } else {
//                state = 1;
//            }
//        }
//
//        if (state == 1) {
//            robot.Arm.setPower(RT - LT);
//        } else {
//            robot.Lift.setPower((RT - LT)/2);
//        }

        robot.Arm.setPower(armsticky);
        robot.Lift.setPower(liftstick);

        if (gamepad1.dpad_left || gamepad2.dpad_left) {
            robot.Turntable.setPower(1);
        } else if (gamepad1.dpad_left || gamepad2.dpad_left) {
            robot.Turntable.setPower(-1);
        } else {
            robot.Turntable.setPower(0);
        }

        telemetry.addData("right pos", robot.Right.getPosition());
        telemetry.addData("left pos", robot.Left.getPosition());
        telemetry.update();

    }

    //Set the power of each motor to be the
    //(x direction + y direction) * turn rate + turn rate for robot not moving
        /*
        *  FL ________ FR
        *    /        \
        *   /          \
        *
        *
        *   \          /
        *    \________/
        *  BL           BR
        * */
    public void drive() {
        if (leftstickx < leftsticky && Math.abs(leftstickx) < leftsticky) {
            //Robot moving forward: Direction of arm
            robot.FL.setPower((leftstickx + leftsticky) * left + (turn * multiplier));
            robot.FR.setPower((-leftstickx + leftsticky) * right + -(turn * multiplier));
            robot.BL.setPower((-leftstickx + leftsticky) * left + (turn * multiplier));
            robot.BR.setPower((leftstickx + leftsticky) * right + -(turn * multiplier));
        } else if (leftstickx >= leftsticky && leftstickx > 0) {
            //Robot moving right: Direction right of arm
            robot.FL.setPower((leftstickx + leftsticky) * left + (turn * multiplier));
            robot.FR.setPower((-leftstickx + leftsticky) * left + (turn * multiplier));
            robot.BL.setPower((-leftstickx + leftsticky) * right + -(turn * multiplier));
            robot.BR.setPower((leftstickx + leftsticky) * right + -(turn * multiplier));
        } else if (leftstickx > leftsticky && leftstickx < Math.abs(leftsticky)) {
            //Robot moving back: Direction opposite of arm
            robot.FL.setPower((leftstickx + leftsticky) * right + -(turn * multiplier));
            robot.FR.setPower((-leftstickx + leftsticky) * left + (turn * multiplier));
            robot.BL.setPower((-leftstickx + leftsticky) * right + -(turn * multiplier));
            robot.BR.setPower((leftstickx + leftsticky) * left + (turn * multiplier));
        } else if (leftstickx <= leftsticky && leftstickx < 0) {
            //Robot moving left: Direction left of arm
            robot.FL.setPower((leftstickx + leftsticky) * right + -(turn * multiplier));
            robot.FR.setPower((-leftstickx + leftsticky) * right + -(turn * multiplier));
            robot.BL.setPower((-leftstickx + leftsticky) * left + (turn * multiplier));
            robot.BR.setPower((leftstickx + leftsticky) * left + (turn * multiplier));
        } else {
            //In place turning
            robot.FL.setPower(-turn);
            robot.FR.setPower(turn);
            robot.BL.setPower(-turn);
            robot.BR.setPower(turn);
        }
    }


    /** This method returns a clipped version of the motor power given by the joystick values
     * Clips the values in intervals of 0.2
     *
     * @param power
     * @return power
     */
    public double ranged(double power) {
        if (power < 0.2) {
            power = 0;
        } else if (power <= 0.4) {
            power = 0.4;
        } else if (power <= 0.6) {
            power = 0.6;
        } else if (power <= 0.8) {
            power = 0.8;
        } else {
            power = 1;
        }
        return power;
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}
