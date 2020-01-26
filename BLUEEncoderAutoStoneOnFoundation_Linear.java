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

package org.firstinspires.ftc.teamcode.BlueAuto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file illustrates the concept of driving a path based on encoder counts.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByTime;
 *
 *  This code ALSO requires that the drive Motors have been configured such that a positive
 *  power command moves them forwards, and causes the encoders to count UP.
 *
 *   The desired path in this example is:
 *   - Drive forward for 48 inches
 *   - Spin right for 12 Inches
 *   - Drive Backwards for 24 inches
 *   - Stop and close the claw.
 *
 *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 *  that performs the actual movement.
 *  This methods assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="BLUEEncoderAuto_StoneOnFoundation", group="Pushbot")
//@Disabled
public class BLUEEncoderAutoStoneOnFoundation_Linear extends LinearOpMode {

    /* Declare OpMode members. */
    //org.firstinspires.ftc.teamcode.HardwarePushbot robot   = new HardwarePushbot();   // Use a Pushbot's hardware
    private ElapsedTime     runtime = new ElapsedTime();
    DcMotor motorFR;
    DcMotor motorFL;
    DcMotor motorBR;
    DcMotor motorBL;
    DcMotor armExtendA;
    DcMotor armExtendB;
    DcMotor gripperL;
    DcMotor gripperR;
    Servo claw;
    Servo   rotate;


    static final double     COUNTS_PER_MOTOR_REV    = 1140 ;    // AndyMark Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP; if no gear put 1.0
    static final double     WHEEL_DIAMETER_INCHES   = 3.0 ;     // **NEEDS UPDATE** For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                      (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     speed             = 1.0;
    //static final double     TURN_SPEED              = 0.2;

    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */

        motorFR    = hardwareMap.dcMotor.get("motor front right");
        motorFL    = hardwareMap.dcMotor.get("motor front left");
        motorBL    = hardwareMap.dcMotor.get("motor back left");
        motorBR    = hardwareMap.dcMotor.get("motor back right");
        armExtendA = hardwareMap.dcMotor.get("arm extend A");
        armExtendB = hardwareMap.dcMotor.get("arm extend B");
        gripperL   = hardwareMap.dcMotor.get("gripper left");
        gripperR   = hardwareMap.dcMotor.get("gripper right");
        claw       = hardwareMap.servo.get("claw");
        rotate     = hardwareMap.servo.get("rotate");

        motorFL.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        motorBL.setDirection(DcMotor.Direction.REVERSE);
        motorFR.setDirection(DcMotor.Direction.FORWARD); //set to FORWARD if AndyMark
        motorBR.setDirection(DcMotor.Direction.FORWARD);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();


        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0",  "Starting at %7d :%7d :%7d :%7d",
                motorFL.getCurrentPosition(),
                motorFR.getCurrentPosition(),
                motorBL.getCurrentPosition(),
                motorBR.getCurrentPosition());
        telemetry.update();


        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        // Starting from the left side of the field, robot aligned with the rightmost stone (closest to the bridge)

        //open claw
        rotate.setPosition(0.85);
        claw.setPosition(0.2);
        sleep(500);

        // S1: Forward 40 Inches with 5 Sec timeout
        encoderDrive(speed, 28, 28, 2);

        // S2: Turn left 90 degrees with 5 Sec timeout
        encoderDrive(speed, -17.5, 17.5, 1);

        // S3: close the claw and grab the stone
        claw.setPosition(1.0);

        sleep(500);     // pause for servos to move

        // S4: Turn left 90 degrees
        encoderDrive(speed, -17.5, 17.5, 1);

        //S5: retreat a little bit
        encoderDrive(speed, 5, 5, 0.5);

        //S6: Turn left 90 degrees
        encoderDrive(speed, -17.5, 17.5, 1);

        //Go near foundation
        encoderDrive(speed, -77.5, -77.5, 3.5);

        //Turn left 90 degrees to face foundation
        encoderDrive(speed, -17.5, 17.5, 1);

        //extend arm
        armExtendB.setPower(-0.4);
        sleep(1500);

        //Turn left 90 degrees
        encoderDrive(speed, -17.5, 17.5, 1);

        //Drop the stone
        claw.setPosition(0.2);
        sleep(500);

        //Turn right 90 degrees
        encoderDrive(speed, 17.5, -17.5, 1);

        //retract arm
        armExtendB.setPower(0.4);
        sleep(1500);

        armExtendB.setPower(0);

        //S5: retreat a little bit
        encoderDrive(speed, -5, -5, 0.5);

        //close and retract the claw
        claw.setPosition(0.8);
        rotate.setPosition(-0.3);

        //turn right 90 degrees
        encoderDrive(speed, 17.5, -17.5, 1);

        /*

        //Go back to loading zone
        encoderDrive(speed, 82.5, 82.5, 3.5);

        //Turn right 90 degrees to face stones
        encoderDrive(speed, 17.5, -17.5, 1);

        //approach stones
        encoderDrive(speed, 25, 25, 2);

        //open claw
        rotate.setPosition(0.8);
        claw.setPosition(0.2);
        sleep(1000);

        // Turn left 90 degrees
        encoderDrive(speed, -17.5, 17.5, 1);

        //close the claw and grab the stone
        claw.setPosition(1.0);
        sleep(1000);     // pause for servos to move

        // Turn right 90 degrees
        encoderDrive(speed, 17.5, -17.5, 1);

        //S5: retreat a little bit
        encoderDrive(speed, -5, -5, 0.5);

        //S6: Turn right to face the bridge
        encoderDrive(speed, 17.5, -17.5, 1);

        //Go near foundation
        encoderDrive(speed, 70.5, 70.5, 3.5);

        //Turn left 90 degrees to face foundation
        encoderDrive(speed, -17.5, 17.5, 1);

        //extend arm
        armExtendB.setPower(-0.4);
        sleep(1500);

        //Turn left 90 degrees
        encoderDrive(speed, -17.5, 17.5, 1);

        //Drop the stone
        claw.setPosition(0.2);
        sleep(500);

        //retract arm
        armExtendB.setPower(0.4);
        sleep(1500);

        //close and retract the claw
        claw.setPosition(0.8);
        rotate.setPosition(-0.3);
*/
        //Park under the bridge
        encoderDrive(speed, 50, 50, 4);



        telemetry.addData("Path", "Complete");
        telemetry.update();

    }

    /*
     *  Method to perform a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */


    public void encoderDrive(double speed,
                             double LeftDistance, double RightDistance,
                             double timeoutS) {
        int LeftTarget;
        int RightTarget;
       /*
        int newFLTarget;
        int newFRTarget;
        int newBLTarget;
        int newBRTarget;
        */
        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            LeftTarget = motorFL.getCurrentPosition() + (int)(LeftDistance * COUNTS_PER_INCH);
            RightTarget = motorFR.getCurrentPosition() + (int)(RightDistance * COUNTS_PER_INCH);

            motorFL.setTargetPosition(LeftTarget);
            motorFR.setTargetPosition(RightTarget);
            motorBL.setTargetPosition(LeftTarget);
            motorBR.setTargetPosition(RightTarget);

            // Turn On RUN_TO_POSITION
            motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.

            runtime.reset();

            motorFL.setPower(speed);
            motorFR.setPower(speed);
            motorBL.setPower(speed);
            motorBR.setPower(speed);

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                   (runtime.seconds() < timeoutS) &&
                   (motorFL.isBusy() || motorFR.isBusy() || motorBL.isBusy() || motorBR.isBusy() ))
            {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", LeftTarget, RightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d :%7d :%7d",
                                            motorFL.getCurrentPosition(),
                                            motorFR.getCurrentPosition(),
                                            motorBL.getCurrentPosition(),
                                            motorBR.getCurrentPosition());
                telemetry.addData("motor pwr:", "%.2f", speed);
                telemetry.update();
            }

            // Stop all motion;
            motorFL.setPower(0);
            motorFR.setPower(0);
            motorBL.setPower(0);
            motorBR.setPower(0);

            // Turn off RUN_TO_POSITION
            motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

              //sleep(250);   // optional pause after each move
        }
    }

}
