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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;



@Autonomous(name = "SovaTechAutonomous", group = "Linear Opmode")
public class SovaTechAutonomous extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime macara_runtime = new ElapsedTime();

    // declararea motoarelor
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

    boolean deschis = true;


    int MILISECONDS_PER_90_DEGREES = 400;

    static final double COUNTS_PER_MOTOR_REV  = 28*80;
    static final double COUNTS_PER_DEGREE     = COUNTS_PER_MOTOR_REV/360;
    static final double COUNTS_PER_90_DEGREES_DOUBLE = COUNTS_PER_DEGREE*90;



    @Override
    public void runOpMode() {



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

        macara.setDirection(DcMotor.Direction.FORWARD);

        macara.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        macara.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        macara.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();
        runtime.reset();

        /**
         * configurare hardware map:
         *   W1     W2
         *       *
         *   W4     W3
         */

        if( opModeIsActive() ){


            move_gheara();
            sleep(50);

            miscare( 1, 0, 80 );
            sleep(100);

            miscare( 0, -1, 1000 );
            sleep(100);

            run_carusel();
            sleep(100);



            miscare( 1, 0, 80 );

            miscare( 0, 1, 1870 );
            sleep(100);


            miscare( 1, 0, 100 );


            rotate( 165, 1 ); // GRIJA LA UNGHIUL ASTA (180-200)
            sleep(100);

            miscare_cu_macara( -1, 0, 150, 300 );
            sleep(400);

            move_gheara();
            sleep(100);

            miscare( 1, 0, 200 );
            sleep(100);

            move_gheara();

            move_macara( 0 );
            sleep(100);



            rotate( 76, -1 );
            sleep(100);

            miscare( 0, 1, 975 );
            sleep(100);


            move_gheara();



            miscare( 1, 0, 1400 );
            sleep(100);



        }

    }


    /**  */
    public void miscare( double sin, double cos, int number_miliseconds ){

        // din inversiune
        double vx = -sin;
        double vy = -cos;


        if (opModeIsActive()) {

            /**
             * powerTopLeft     =  vx + vy;
             * powerTopRight    =  vx - vy;
             * powerBottomRight =  vx + vy;
             * powerBottomLeft  =  vx - vy;
             */

            powerTopLeft     =  (vx + vy)/2;
            powerTopRight    =  (vx - vy)/2;
            powerBottomRight =  (vx + vy)/2;
            powerBottomLeft  =  (vx - vy)/2;

            powerTopLeft = Range.clip( powerTopLeft , -0.7, 0.7 );
            powerTopRight = Range.clip( powerTopRight , -0.7, 0.7 );
            powerBottomRight = Range.clip( powerBottomRight , -0.7, 0.7 );
            powerBottomLeft = Range.clip( powerBottomLeft , -0.7, 0.7 );


            topLeft.setPower( powerTopLeft );
            topRight.setPower( powerTopRight );
            bottomRight.setPower( powerBottomRight );
            bottomLeft.setPower( powerBottomLeft );

            sleep(number_miliseconds);

            topLeft.setPower( 0 );
            topRight.setPower( 0 );
            bottomRight.setPower( 0 );
            bottomLeft.setPower( 0 );


        }

        sleep(100);

    }

    /** pentru varianta de robot cu intake */
    public void miscare_cu_intake( double sin, double cos, int number_miliseconds ){

        // din inversiune
        double vx = -sin;
        double vy = -cos;


        if (opModeIsActive()) {

            /**
             * powerTopLeft     =  vx + vy;
             * powerTopRight    =  vx - vy;
             * powerBottomRight =  vx + vy;
             * powerBottomLeft  =  vx - vy;
             */

            powerTopLeft     =  (vx + vy)/2;
            powerTopRight    =  (vx - vy)/2;
            powerBottomRight =  (vx + vy)/2;
            powerBottomLeft  =  (vx - vy)/2;

            powerTopLeft = Range.clip( powerTopLeft , -0.7, 0.7 );
            powerTopRight = Range.clip( powerTopRight , -0.7, 0.7 );
            powerBottomRight = Range.clip( powerBottomRight , -0.7, 0.7 );
            powerBottomLeft = Range.clip( powerBottomLeft , -0.7, 0.7 );


            topLeft.setPower( powerTopLeft );
            topRight.setPower( powerTopRight );
            bottomRight.setPower( powerBottomRight );
            bottomLeft.setPower( powerBottomLeft );

            intake.setPower( -1 );

            sleep(number_miliseconds);

            topLeft.setPower( 0 );
            topRight.setPower( 0 );
            bottomRight.setPower( 0 );
            bottomLeft.setPower( 0 );

            intake.setPower( 0 );


        }

        sleep(100);

    }

    /** pentru erori */
    public void rotate( int angle, int dir ){

        double milis = (double)( (double) angle/90) * (double)MILISECONDS_PER_90_DEGREES;

        int miliseconds = (int)milis;

        double power = 0.7;

        dir=-dir;

        topLeft.setPower( power*dir );
        topRight.setPower( -power*dir );
        bottomRight.setPower( -power*dir );
        bottomLeft.setPower( power*dir );

        sleep(miliseconds);

        topLeft.setPower( 0 );
        topRight.setPower( 0 );
        bottomRight.setPower( 0 );
        bottomLeft.setPower( 0 );

        sleep(100);


    }

    public void run_carusel( ){

        carusel.setPower( -1 );
        sleep(2700);

        carusel.setPower(0);
        sleep(100);

    }

    public void move_gheara(){

        if( deschis ){
            deschis = false;
            gheara.setPosition( 0.0 );

            sleep(500);
        } else {
            deschis = true;
            gheara.setPosition( 1.0 );

            sleep(500);
        }

    }


    public void move_macara( int angle_tar ){


        double target_angle = angle_tar;
        double initial_angle = (double) (macara.getCurrentPosition()) / COUNTS_PER_DEGREE;

        int angle = (int)( target_angle - initial_angle );

        int armPosition = (int) (COUNTS_PER_DEGREE * angle);
        int macara_current = macara.getCurrentPosition();

        int target = macara_current + armPosition;
        macara.setTargetPosition(target);
        macara.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);


        while( opModeIsActive() && macara.isBusy() ){
            macara.setPower(0.5);
        }


        macara.setPower(0);
        macara.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

    }


    public void miscare_cu_macara( double sin, double cos, double tar_angle, int number_miliseconds ){

        double target_angle = tar_angle + tar_angle/10;
        double initial_angle = (double) (macara.getCurrentPosition()) / COUNTS_PER_DEGREE;

        int angle = (int)( target_angle - initial_angle );

        int armPosition = (int) (COUNTS_PER_DEGREE * angle);
        int macara_current = macara.getCurrentPosition();

        int target = macara_current + armPosition;
        macara.setTargetPosition(target);
        macara.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);


        double vx = -sin;
        double vy = -cos;


        powerTopLeft     =  (vx + vy)/2;
        powerTopRight    =  (vx - vy)/2;
        powerBottomRight =  (vx + vy)/2;
        powerBottomLeft  =  (vx - vy)/2;

        powerTopLeft = Range.clip( powerTopLeft , -0.7, 0.7 );
        powerTopRight = Range.clip( powerTopRight , -0.7, 0.7 );
        powerBottomRight = Range.clip( powerBottomRight , -0.7, 0.7 );
        powerBottomLeft = Range.clip( powerBottomLeft , -0.7, 0.7 );


        while( macara.isBusy() && opModeIsActive() ){
            macara.setPower(0.3);
        }



        macara_runtime.reset();

        while( opModeIsActive() && macara_runtime.milliseconds() <= number_miliseconds ){

            macara.setPower(0.1);

            topLeft.setPower( powerTopLeft );
            topRight.setPower( powerTopRight );
            bottomRight.setPower( powerBottomRight );
            bottomLeft.setPower( powerBottomLeft );


        }

        topLeft.setPower( 0 );
        topRight.setPower( 0 );
        bottomRight.setPower( 0 );
        bottomLeft.setPower( 0 );


        macara.setPower(0);
        macara.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        macara.setPower(0);


        sleep(100);
        sleep(200);

    }



}
