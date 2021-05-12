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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="AutonomousCode", group="Linear Opmode")
//@Disabled
public class AutonomousCode extends UsefulFunctions {
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        Initialise();
        InitialiseVision();

        sleep(500);
        /*for(int i = 1; i < 2; i = 1)
        {
            telemetry.addData("Ring size:", detector.count);
            telemetry.addData("One ring value:", detector.oneRingTotal);
            telemetry.addData("Four ring value:", detector.fourRingsTotal);
            telemetry.update();

        }*/

        waitForStart();
        runtime.reset();

        sleep(500);

        String startCount = detector.count;
        telemetry.addData("Current ring count:", startCount);
        //telemetry.addData( "Crt ticks fl fr bl br", crticksfl + " " + crticksfr + "  " + crticksbl + " " + crticksbr);
        telemetry.update();

        /*MoveRotation(750, true);
        sleep(100);
        AutonomousMove(245, 0); //old x_mm = 123
        sleep(100);
        MoveRotation(750, false);
        CorrectAngle(0);
        telemetry.addData("Current angle:", crtangle.firstAngle);*/

        double angleOffset = 0;
        if(startCount == "ZERO") {
            AutonomousMove(in_to_mm(3 * 24), 0);
            AutonomousMove(0, in_to_mm(0.25 * 24));
            CorrectAngle(0);

            AutonomousMove(-in_to_mm(14), 0);
            AutonomousMove(0, -in_to_mm(18.4));
            CorrectAngle(0);
        } else if(startCount == "ONE") {
            AutonomousMove(in_to_mm(3.1 * 24), 0);
            MoveRotation(750, false);
            AutonomousMove(150, 0);
            CorrectAngle(0);

            AutonomousMove(-in_to_mm(36), 0);
            AutonomousMove(0, -in_to_mm(6));
        } else if(startCount == "FOUR") {

            angleOffset = 1;
            AutonomousMove(in_to_mm(4.5 * 24), 0);
            AutonomousMove(0, in_to_mm(0.25 * 24));
            CorrectAngle(0);

            AutonomousMove(-in_to_mm(4 + 2 * 24), 0);
            AutonomousMove(0, -in_to_mm(20));
            CorrectAngle(0);
        }

        telemetry.addData( "Crt ticks fl fr bl br", crticksfl + " " + crticksfr + "  " + crticksbl + " " + crticksbr);
        telemetry.update();

        AddToLaunchAngle(startAngle  + angleOffset);
        launchMotor.setPower(1);
        sleep(500);
        for(int i = 0; i < 3; i++)
        {
            if(i == 2)
                AddToLaunchAngle(-1);
            sleep(150);
            launchServoThread.run();
        }
        launchMotor.setPower(0);
        AddToLaunchAngle(-currentLaunchAngle);

        AutonomousMove(in_to_mm(16), 0);

        while (opModeIsActive()) {
            //telemetry.addData( "Status", "Run Time: " + runtime.toString());
            //telemetry.update();
        }

        /*double temptfl = crticksfl, temptfr = crticksfr, temptbl = crticksbl, temptbr = crticksbr;
        telemetry.addData( "Initial ticks fl fr bl br", crticksfl + " " + crticksfr + "  " + crticksbl + " " + crticksbr);
        telemetry.update();

        AutonomousMove(in_to_mm(4 * 24), 0);
        UpdateOrientation();
        UpdateTicks();
        telemetry.addData("Initial ticks fl fr bl br", crticksfl + " " + crticksfr + "  " + crticksbl + " " + crticksbr);
        telemetry.addData("Crt ticks fl fr bl br", (crticksfl - temptfl) + " " + (crticksfr - temptfr) + "  " + (crticksbl - temptbl) + " " + (crticksbr - temptbr));
        telemetry.update();

        while (opModeIsActive()) {
        }*/
    }
}
