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

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Prototype Test 3")
public class ProtoOpMode3 extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor winchDrive = null;
    private DcMotor leftFront = null;
    private DcMotor leftRear = null;
    private DcMotor rightFront = null;
    private DcMotor rightRear = null;
    private Servo leftClaw;
    private Servo rightClaw;
    private boolean closed = false;
    private boolean pressed = false;
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        winchDrive = hardwareMap.get(DcMotor.class, "winch");
        leftClaw = hardwareMap.get(Servo.class, "servo_left");
        rightClaw = hardwareMap.get(Servo.class, "servo_right");

        leftFront = hardwareMap.get(DcMotor.class, "left_front");;
        leftRear = hardwareMap.get(DcMotor.class, "left_rear");;
        rightFront = hardwareMap.get(DcMotor.class, "right_front");;
        rightRear = hardwareMap.get(DcMotor.class, "right_rear");;

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.REVERSE);

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected direc tly to the battery
        winchDrive.setDirection(DcMotor.Direction.FORWARD);
        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
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
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        // Choose to drive using either Tank Mode, or POV Mode
        // Comment out the method that's not used.  The default below is POV.

        if (gamepad1.dpad_up) {
            leftFront.setPower(-.6);
            rightFront.setPower(-.6);
            leftRear.setPower(-.6);
            rightRear.setPower(-.6);
        } else if (gamepad1.dpad_down){
            leftFront.setPower(.6);
            rightFront.setPower(.6);
            leftRear.setPower(.6);
            rightRear.setPower(.6);
        } else if (gamepad1.dpad_left) {
            leftFront.setPower(.6);
            rightFront.setPower(-.6);
            leftRear.setPower(-.6);
            rightRear.setPower(.6);
        } else if (gamepad1.dpad_right){
            leftFront.setPower(-.6);
            rightFront.setPower(.6);
            leftRear.setPower(.6);
            rightRear.setPower(-.6);
        }

        if (gamepad1.a) {
            if (!closed && !pressed){
                leftClaw.setPosition(0.6);
                rightClaw.setPosition(0.3);
                closed = !closed;
            }else if (closed && !pressed) {
                leftClaw.setPosition(0.0);
                rightClaw.setPosition(9.6);
                closed = !closed;
            }
            pressed = true;
        }else {
            pressed = false;
        }
        if (gamepad1.left_bumper)
            winchDrive.setPower(1);
        else if (gamepad1.right_bumper)
            winchDrive.setPower(-1);
        else
            winchDrive.setPower(0);

        double speed = .75;
        double r = Math.hypot(-gamepad1.left_stick_x, gamepad1.left_stick_y);
        double robotAngle = Math.atan2(gamepad1.left_stick_y, -gamepad1.left_stick_x) - Math.PI / 4;
        double rightX = -gamepad1.right_stick_x;
        final double v1 = speed * r * Math.cos(robotAngle) + rightX;
        final double v2 = speed * r * Math.sin(robotAngle) - rightX;
        final double v3 = speed * r * Math.sin(robotAngle) + rightX;
        final double v4 = speed * r * Math.cos(robotAngle) - rightX;

        leftFront.setPower(v1);
        rightFront.setPower(v2);
        leftRear.setPower(v3);
        rightRear.setPower(v4);

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        //telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
