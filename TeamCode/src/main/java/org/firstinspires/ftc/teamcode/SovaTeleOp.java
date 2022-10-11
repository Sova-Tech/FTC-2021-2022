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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;



@TeleOp(name="SovaTechTeleOp", group="Linear Opmode")
public class SovaTechTeleOp extends LinearOpMode {

    // declaratii
    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor topLeft = null;
    private DcMotor topRight = null;
    private DcMotor bottomRight = null;
    private DcMotor bottomLeft = null;

    private DcMotorEx macara = null;
    private DcMotor carusel = null;
    private DcMotor intake = null;
    Servo gheara;

    double powerTopLeft = 0;
    double powerTopRight = 0;
    double powerBottomRight = 0;
    double powerBottomLeft = 0;


    boolean deschis = false;

    /**
     * configurare hardware map:
     *   W1     W2
     *       *
     *   W4     W3
     */

    static final double     COUNTS_PER_MOTOR_REV    = 28;
    static final double     GEAR_REDUCTION    = 80;
    static final double     COUNTS_PER_GEAR_REV    = COUNTS_PER_MOTOR_REV * GEAR_REDUCTION;
    static final double     COUNTS_PER_DEGREE    = COUNTS_PER_GEAR_REV/360;

    private ElapsedTime runtime_carusel= new ElapsedTime();


    @Override
    public void runOpMode() {

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        double a;

        //instantierea motoarelor
        topLeft = hardwareMap.get(DcMotor.class, "left_top");
        topRight = hardwareMap.get(DcMotor.class, "right_top");
        bottomRight = hardwareMap.get(DcMotor.class, "right_bottom");
        bottomLeft = hardwareMap.get(DcMotor.class, "left_bottom");

        macara = hardwareMap.get(DcMotorEx.class, "macara");
        gheara = hardwareMap.get(Servo.class, "gheara");
        carusel = hardwareMap.get(DcMotor.class, "carusel");
        intake = hardwareMap.get(DcMotor.class, "intake");

        /// de verificat directiile
        topLeft.setDirection(DcMotor.Direction.FORWARD);
        topRight.setDirection(DcMotor.Direction.REVERSE);
        bottomRight.setDirection(DcMotor.Direction.REVERSE);
        bottomLeft.setDirection(DcMotor.Direction.FORWARD);



        macara.setDirection(DcMotorEx.Direction.FORWARD);

        macara.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        macara.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        macara.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);




        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {


            int telemetrie = (int)macara.getCurrentPosition();
            telemetry.addData( "macara ", telemetrie );


            double vx = gamepad1.left_stick_x;
            double vy = gamepad1.left_stick_y;
            double r = -gamepad1.right_stick_x;


            powerTopLeft = Range.clip((vy - vx + r)/2, -0.7, 0.7);
            powerTopRight = Range.clip((vy + vx - r)/2, -0.7, 0.7);
            powerBottomRight = Range.clip((vy - vx - r)/2, -0.7, 0.7);
            powerBottomLeft = Range.clip((vy + vx + r)/2, -0.7, 0.7);

            
            /**
             *  left_trigger si right_trigger sunt pentru slow/fast mode
             */
            if(gamepad1.left_trigger > 0)
            {
                powerTopLeft/=2;
                powerTopRight/=2;
                powerBottomLeft/=2;
                powerBottomRight/=2;
            }

            if(gamepad1.right_trigger > 0)
            {
                powerTopLeft/=0.7;
                powerTopRight/=0.7;
                powerBottomLeft/=0.7;
                powerBottomRight/=0.7;
            }

            
            topLeft.setPower(powerTopLeft);
            topRight.setPower(powerTopRight);
            bottomRight.setPower(powerBottomRight);
            bottomLeft.setPower(powerBottomLeft);


            
            if(gamepad1.right_bumper){
                macara.setPower(-0.5);
            }
            else if (gamepad1.left_bumper){
                macara.setPower(0.5);
            }
            else {
                macara.setPower(0);
            }



            //carusel
            runtime_carusel.reset();
            if( gamepad1.y )
            {
                carusel.setPower(-1);
            }
            else
            {
                carusel.setPower(0);
            }


            
            //gheara
            if( gamepad1.b ) {

                deschis = false;
                gheara.setPosition( 0.3 );
                a = gheara.getPosition();
                telemetry.addData("pozitie servo ", a );

            }

            if( gamepad1.a ) {
                
                deschis = true;
                gheara.setPosition( 0.8 );
                a = gheara.getPosition();
                telemetry.addData("pozitie servo ", a );
                
            }
            
            
            if( gamepad1.right_stick_button ){
                intake.setPower( -1 );
            }
            else {
                if( gamepad1.left_stick_button ){
                    intake.setPower( 1 );
                }
                else
                    intake.setPower(0);
            }



            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();


        }

    }

    public void move_macara( double angle_tar ){


        double target_angle = angle_tar + angle_tar/10;
        double initial_angle = (double) (macara.getCurrentPosition()) / COUNTS_PER_DEGREE;

        int angle = (int)( target_angle - initial_angle );

        int armPosition = (int) (COUNTS_PER_DEGREE * angle);
        int macara_current = macara.getCurrentPosition();
        
        int target = macara_current + armPosition;
        macara.setTargetPosition(target);
        macara.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        
        while( opModeIsActive() && macara.isBusy() ){
            macara.setPower(0.3);
        }


    }

}
