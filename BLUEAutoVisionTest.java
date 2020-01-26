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
import org.firstinspires.ftc.teamcode.VisionStuff.VuforiaTest;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.robot;

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

@Autonomous(name="BLUEVisionTest", group="Pushbot")
@Disabled
public class BLUEAutoVisionTest extends robot {

    VuforiaTest.skystonePos pos;
    int stoneDiff = 32;
    static final double     speed             = 1;
    //VuforiaTest vuforiaScan = new VuforiaTest(vuforia);

    @Override
    public void runOpMode() throws InterruptedException {

        pos = vuforiaTest.vuforiascan(false, false);

        roboInit();

        rotate.setPosition(0.3);
        claw.setPosition(0);
        sleep(500);
/*
        switch (pos) {
            case LEFT:
                stoneDiff = 0;
                //claw.setPosition(1);
                break;
            case RIGHT:
                stoneDiff = 32;
                //claw.setPosition(1);
                break;
            case CENTER:
                stoneDiff = 16;
                //claw.setPosition(1);
                break;
        }
*/
        if (pos == VuforiaTest.skystonePos.CENTER) {
            encoderDrive(speed,28,28,28,28,2);

            encoderDrive(speed, -17.5, 17.5, -17.5, 17.5, 1);

            grab();

            //move away from quarry
            encoderDrive(speed, -15, 15, 15, -15, 1);

            //go past tape
            encoderDrive(speed,45, 45,45,45,3);

            release();

            //go back to quarry
            encoderDrive(speed, -45-stoneDiff, -45-stoneDiff, -45-stoneDiff,-45-stoneDiff, 6);

            encoderDrive(speed,15,-15,-15,15,1);

            grab();

            //move away from quarry
            encoderDrive(speed, -15, 15, 15, -15, 1);

            //go past tape
            encoderDrive(speed,70, 70,70,70,3);

            release();

            encoderDrive(speed,-5,-5,-5,-5,1);

            telemetry.addData("Path", "Complete");
            telemetry.update();

        } else if (pos == VuforiaTest.skystonePos.LEFT) {
            encoderDrive(speed,28,28,28,28,2);

            encoderDrive(speed, -17.5, 17.5, -17.5, 17.5, 1);

            encoderDrive(speed,8,8,8,8,1);

            grab();

            //move away from quarry
            encoderDrive(speed, -15, 15, 15, -15, 1);

            //go past tape
            encoderDrive(speed,40, 40,40,40,3);

            release();

            encoderDrive(speed,-40-stoneDiff,-40-stoneDiff,-40-stoneDiff,-40-stoneDiff,4.5);

            encoderDrive(speed,15,-15,-15,15,1);

            grab();

            //move away from quarry
            encoderDrive(speed, -15, 15, 15, -15, 1);

            //go past tape
            encoderDrive(speed,60, 60,60,60,3.5);

            release();

            encoderDrive(speed,-5,-5,-5,-5,1);

            telemetry.addData("Path", "Complete");
            telemetry.update();

        } else {
            encoderDrive(speed,28,28,28,28,2);

            encoderDrive(speed, -17.5, 17.5, -17.5, 17.5, 1);

            encoderDrive(speed,-8,-8,-8,-8,1);

            grab();

            //move away from quarry
            encoderDrive(speed, -15, 15, 15, -15, 1);

            //go past tape
            encoderDrive(speed,60, 60,60,60,4);

            release();

            encoderDrive(speed,-60-stoneDiff,-60-stoneDiff,-60-stoneDiff,-60-stoneDiff,7);

            encoderDrive(speed,15,-15,-15,15,1);

            grab();

            //move away from quarry
            encoderDrive(speed, -15, 15, 15, -15, 1);

            //go past tape
            encoderDrive(speed,-80, -80,-80,-80,5.5);

            release();

            encoderDrive(speed,-5,-5,-5,-5,1);

            telemetry.addData("Path", "Complete");
            telemetry.update();
        }




    }


}