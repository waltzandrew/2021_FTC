package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import java.util.List;

@Autonomous(name = "Autov1")

public class AutoV1 extends LinearOpMode {

    private ElapsedTime time = new ElapsedTime();
    Hardware r = new Hardware();

    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String QUAD = "Quad";
    private static final String SINGLE = "Single";
    private static final String VUFORIA_KEY = "ARBKom//////AAABmQ6j5Q7euktykQmWnMdF5GKAEmU17d+XyTd31FAnr9ICsUpVzyCSwHOUoi6PAoGUuPNBk3LXi1SLZgfOen62wPzq9PhCzJsKMKHSW2BBRWZb+/2Zciy+jsvae89X+CMXyOXong09iiFyUSVipop+UufmDqdjVnp4n1DaGkLilCxwqCdN8NCVdLjlbvlzwfQkQ7xgEswiN01pRaig8bVHxVsq+FdamOdRNmBJhtuAjrZK52hK+9IT6GwM6mxMJjWEF3yrEup2/G/jODEXJAQST8OPx9yKZt+8NKObk4Fs0U5WkVWmUilmxvNCmPwq8snqs76w7DJANEsxA32YaPBTa51kfMnHCKdOuTgf+Gd5gEX5";

    private static final double RADIUS = 4;
    private static final double CPR = 1120;
    private static final int CPI = (int)(CPR/(2*Math.PI*RADIUS));


    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    private boolean ran = false;

    private enum stack
    {
        ONE, FOUR, NONE;
    }

    stack stackSize = stack.NONE;



    @Override
    public void runOpMode() {

        r.init(hardwareMap);

        resetAll();
        initVuforia();
        initTfod();

        if (tfod != null) {
            tfod.activate();
            // Uncomment the following line if you want to adjust the magnification and/or the aspect ratio of the input images.
            //tfod.setZoom(2.5, 1.78);
        }
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            moveDistance(0.5, 24, 4);

            time.reset();
            while (opModeIsActive() && !ran && time.seconds() < 3) {
                if (tfod != null) {
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Object Detected", updatedRecognitions.size());

                        // step through the list of recognitions and display boundary info.
                        int i = 0;
                        for (Recognition recognition : updatedRecognitions) {
                            telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                            telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                    recognition.getLeft(), recognition.getTop());
                            telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                    recognition.getRight(), recognition.getBottom());
                            if(recognition.getLabel() == QUAD)
                            {
                                stackSize = stack.FOUR;
                                ran = true;
                            }
                            if(recognition.getLabel() == SINGLE)
                            {
                                stackSize = stack.ONE;
                                        ran = true;
                            }
                        }
                        telemetry.update();
                    }
                }
            }
            switch(stackSize)
            {
                case NONE: //target zone a
                    moveDistance(0.5, 60, 4);
                    sleep(100);
                   // moveDistance(0.5, -60, 4);
                    break;

                case ONE: //target zone b
                    moveDistance(0.5, 37, 4);
                    sleep(100);

                 //   moveDistance(0.5, -37, 4);;
                    break;
                    
                case FOUR: //target zone c
        moveDistance(0.5, 80, 4);
        sleep(100);
        //moveDistance(0.5, -80, 4);

                    break;


            }

        }

        if (tfod != null) {
            tfod.shutdown();
        }
    }

    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        parameters.cameraMonitorFeedback = VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, QUAD, SINGLE);
    }
    //---------------------------Methods and Algorithms------------------------
    public void setAll(double power)
    {
        r.frontright.setPower(power);
        r.frontleft.setPower(power);
        r.backright.setPower(power);
        r.backleft.setPower(power);
    }

    public void setAllEncoder()
    {
        r.frontright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        r.frontleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        r.backright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        r.backleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void setAllRun()
    {
        r.frontright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        r.frontleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        r.backright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        r.backleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void setAllTarget(int target)
    {
        r.frontright.setTargetPosition(r.frontright.getCurrentPosition() + target);
        r.frontleft.setTargetPosition(r.frontleft.getCurrentPosition() + target);
        r.backright.setTargetPosition(r.backright.getCurrentPosition() + target);
        r.backleft.setTargetPosition(r.backleft.getCurrentPosition() + target);
    }

    public void resetAll()
    {
        r.frontright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        r.frontleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        r.backright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        r.backleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void moveTime(double power, long time)
    {
        setAll(power);
        sleep(time);
        setAll(0);
    }

    public void setRight(double power)
    {
        r.frontright.setPower(power);
        r.backright.setPower(power);
    }

    public void setLeft(double power)
    {
        r.backleft.setPower(power);
        r.frontleft.setPower(power);
    }
    public void moveDistance(double power, int distance, double timeoutS)
    {
        int target = CPI * distance; //counts per inch * distance(inches)
        double leftPower;
        double rightPower;
        double error;
        double errorMargin = 15; //may need to be tuned
        double kP = 0.004;        //may need to be tuned (if too low - correction is weak, if too high - too jerky)

        setAllTarget(target);          //set target distance
        setAllRun();                  // set mode to run to position
        setAll(Math.abs(power));     //  give power(postive)
        time.reset();               //   start timeout clock

        while(opModeIsActive() && time.seconds()<timeoutS && r.backleft.isBusy())//&& (r.frontright.isBusy() || r.frontleft.isBusy() || r.backleft.isBusy() || r.backright.isBusy()))
        {
           error = ((r.frontleft.getCurrentPosition()) - ((r.frontright.getCurrentPosition())));
           if(Math.abs(error) < errorMargin) error = 0;

           leftPower = power - (error * kP);
           rightPower = power + (error * kP);

           setRight(rightPower);
           setLeft(leftPower);

            telemetry.addData("FrontLeft",  "Running at %7d ", r.frontleft.getCurrentPosition());
            telemetry.addData("FrontRight", "Running at %7d ", r.frontright.getCurrentPosition());
            telemetry.addData("BackLeft",   "Running at %7d ", r.backleft.getCurrentPosition());
            telemetry.addData("BackRight",  "Running at %7d ", r.backright.getCurrentPosition());
          //telemetry.update();
        }
        setAll(0);
        setAllEncoder();
    }
}
