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

import android.graphics.Bitmap;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;
import com.vuforia.Image;

import org.firstinspires.ftc.robotcontroller.external.samples.ConceptVuforiaNavigation;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.opencv.android.Utils;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;

import ftc.vision.BeaconColorResult;
import ftc.vision.BeaconProcessor;
import ftc.vision.ImageProcessor;
import ftc.vision.ImageProcessorResult;
import ftc.vision.ImageUtil;

/**
 * This OpMode illustrates the basics of using the Vuforia engine to determine
 * the identity of Vuforia VuMarks encountered on the field. The code is structured as
 * a LinearOpMode. It shares much structure with {@link ConceptVuforiaNavigation}; we do not here
 * duplicate the core Vuforia documentation found there, but rather instead focus on the
 * differences between the use of Vuforia for navigation vs VuMark identification.
 *
 * @see ConceptVuforiaNavigation
 * @see VuforiaLocalizer
 * @see VuforiaTrackableDefaultListener
 * see  ftc_app/doc/tutorial/FTC_FieldCoordinateSystemDefinition.pdf
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained in {@link ConceptVuforiaNavigation}.
 */

@Autonomous(name="Autonomous: Test", group ="Concept")
//@Disabled
public class Vuforia_test extends LinearOpMode {

    public static final String TAG = "Vuforia VuMark Sample";

    OpenGLMatrix lastLocation = null;

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    VuforiaLocalizer vuforia;

    Hardware_2017 robot = new Hardware_2017();
    private ElapsedTime runtime = new ElapsedTime();

    public enum Direction {
        FORWARD  (1),
        BACKWARD (2),
        LEFT     (3),
        RIGHT    (4);

        public final int direction;

        Direction(int number) {this.direction = number;}
    }

    @Override public void runOpMode() throws InterruptedException {

//========================================================== Start of Autonomous Program ==========================================================//

//========================================================== Set-up ==========================================================//

        /*
         * To start up Vuforia, tell it the view that we wish to use for camera monitor (on the RC phone);
         * If no camera monitor is desired, use the parameterless constructor instead (commented out below).
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        // OR...  Do Not Activate the Camera Monitor View, to save power
        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        /*
         * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
         * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
         * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
         * web site at https://developer.vuforia.com/license-manager.
         *
         * Vuforia license keys are always 380 characters long, and look as if they contain mostly
         * random data. As an example, here is a example of a fragment of a valid key:
         *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
         * Once you've obtained a license key, copy the string from the Vuforia web site
         * and paste it in to your code onthe next line, between the double quotes.
         */
        parameters.vuforiaLicenseKey = "AdMfwt3/////AAAAGb1teOGtfEIMp2su4rECQC8XtaXGZ31RT/lGzygte/bMFzL08u3r8XwZIq+LZyydLQ2c/zPv9keO43DluoznWwtCL7F8VT23sSONaoyNjho5L7W/Xc4e1Ee5WKYq2v3X9Fjrhfos5kbo1g9bn4X27IUnjEEgviP0AmanRhc8rC1OnCS2HczyvE0KLjhSBBN5GLyu5BqgwQ+RhjKJZY79E3vvUxj35w5wjKGCVoPpCNASH73AenBLtIA3ZxvXWJAvFA46D0Ntu0N/HGKyjQpPPVfd5vXRCiY32VXrPNCTy2OLqIrcjCA947tneXmNCp9M3Ylc/7MUPw1S5WH4mOXLQBnhco3aIiPTW5dkqItRDP6q";

        /*
         * We also indicate which camera on the RC that we wish to use.
         * Here we chose the back (HiRes) camera (for greater range), but
         * for a competition robot, the front camera might be more convenient.
         */
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        parameters.useExtendedTracking = false;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        this.vuforia.setFrameQueueCapacity(3);
        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true);

        /**
         * Load the data set containing the VuMarks for Relic Recovery. There's only one trackable
         * in this data set: all three of the VuMarks in the game were created from this one template,
         * but differ in their instance id information.
         * @see VuMarkInstanceId
         */
        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary

        telemetry.addData(">", "Press Play to start");
        telemetry.update();
        waitForStart();

        relicTrackables.activate();

//========================================================== Set-up ==========================================================//

//========================================================== Jewel-Detection ==========================================================//

        Image rgb = null;

        VuforiaLocalizer.CloseableFrame frame = vuforia.getFrameQueue().take(); //takes the frame at the head of the queue
        long numImages = frame.getNumImages();

        for (int i = 0; i < numImages; i++) {
            if (frame.getImage(i).getFormat() == PIXEL_FORMAT.RGB565) {
                rgb = frame.getImage(i);
                break;
            }
        }

        if (rgb != null) {
            /*rgb is now the Image object that we've used in the video*/
            Bitmap bm = Bitmap.createBitmap(rgb.getWidth(), rgb.getHeight(), Bitmap.Config.RGB_565);
            bm.copyPixelsFromBuffer(rgb.getPixels());

            //put the image into a MAT for OpenCV
            Mat tmp = new Mat(rgb.getWidth(), rgb.getHeight(), CvType.CV_8UC4);
            Utils.bitmapToMat(bm, tmp);

            frame.close();

            long frameTime = System.currentTimeMillis();
//            ImageUtil.saveImage("a", tmp, Imgproc.COLOR_RGBA2BGR, "0_camera", frameTime);
            ImageProcessor processor = new BeaconProcessor();
            ImageProcessorResult processorResult =  processor.process(frameTime, tmp, true);
            BeaconColorResult colorResult = (BeaconColorResult) processorResult.getResult();

            //color results from processing images
            BeaconColorResult.BeaconColor leftColor = colorResult.getLeftColor();
            BeaconColorResult.BeaconColor rightColor = colorResult.getRightColor();
            telemetry.addData("Result", colorResult);

        }

        telemetry.update();
        Thread.sleep(1000);

        //lower jewel knocker and remove desired jewel

//========================================================== Jewel-Detection ==========================================================//


//========================================================== Pictograph-Detection ==========================================================//

        while (opModeIsActive()) {
            /**
             * See if any of the instances of {@link relicTemplate} are currently visible.
             * {@link RelicRecoveryVuMark} is an enum which can have the following values:
             * UNKNOWN, LEFT, CENTER, and RIGHT. When a VuMark is visible, something other than
             * UNKNOWN will be returned by {@link RelicRecoveryVuMark#from(VuforiaTrackable)}.
             */
            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
            if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
                /* Found an instance of the template. In the actual game, you will probably
                 * loop until this condition occurs, then move on to act accordingly depending
                 * on which VuMark was visible. */
                telemetry.addData("VuMark", "%s visible", vuMark);

                /* For fun, we also exhibit the navigational pose. In the Relic Recovery game,
                 * it is perhaps unlikely that you will actually need to act on this pose information, but
                 * we illustrate it nevertheless, for completeness. */
                OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) relicTemplate.getListener()).getPose();
                telemetry.addData("Pose", format(pose));

                /* We further illustrate how to decompose the pose into useful rotational and
                 * translational components */
                if (pose != null) {
                    VectorF trans = pose.getTranslation();
                    Orientation rot = Orientation.getOrientation(pose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

                    // Extract the X, Y, and Z components of the offset of the target relative to the robot
                    double tX = trans.get(0);
                    double tY = trans.get(1);
                    double tZ = trans.get(2);

                    // Extract the rotational components of the target relative to the robot
                    double rX = rot.firstAngle;
                    double rY = rot.secondAngle;
                    double rZ = rot.thirdAngle;
                }
                //move certain distance based on which VuMark it is
                if (vuMark == RelicRecoveryVuMark.CENTER) {

                } else if (vuMark == RelicRecoveryVuMark.LEFT) {

                } else if (vuMark == RelicRecoveryVuMark.RIGHT) {

                }

                //lower arm and release the glyph
            } else {
                telemetry.addData("VuMark", "not visible");
                //move in direction of pictograph to detect it
            }
        }
    }

//========================================================== Pictograph-Detection ==========================================================//


//========================================================== End of Autonomous Program ==========================================================//

    /** Drives the robot in the desired direction "direction" at power "power" for "time" seconds
     *
     * @param direction
     * @param power
     * @param time
     */
    public void driveMotors(Direction direction, int power, float time) {
        //set power
        if (direction == Direction.FORWARD) {
            robot.FL.setPower(power);
            robot.FR.setPower(power);
            robot.BL.setPower(power);
            robot.BR.setPower(power);
        } else if (direction == Direction.BACKWARD) {
            robot.FL.setPower(-power);
            robot.FR.setPower(-power);
            robot.BL.setPower(-power);
            robot.BR.setPower(-power);
        } else if (direction == Direction.LEFT) {
            robot.FL.setPower(-power);
            robot.FR.setPower(power);
            robot.BL.setPower(power);
            robot.BR.setPower(-power);
        } else if (direction == Direction.RIGHT) {
            robot.FL.setPower(power);
            robot.FR.setPower(-power);
            robot.BL.setPower(-power);
            robot.BR.setPower(power);
        }
        runtime.reset();
        while(opModeIsActive() && runtime.seconds() <= time) {
            //wait
        }
        robot.FL.setPower(0);
        robot.FR.setPower(0);
        robot.BL.setPower(0);
        robot.BR.setPower(0);
    }
//    public Image getImage(){
//        VuforiaLocalizer.CloseableFrame frame = null;
//        try{
//            frame = vuforia.getFrameQueue().take();
//            long numImages = frame.getNumImages();
//            Image rgbImage = null;
//            for (int i = 0; i < numImages; i++) {
//                Image img = frame.getImage(i);
//                int fmt = img.getFormat();
//                if (fmt == PIXEL_FORMAT.RGB565) {
//                    rgbImage = frame.getImage(i);
//                    frame.close();
//                    break;
//                }
//            }
//            return rgbImage;
//        }
//        catch(InterruptedException exc){
//            return null;
//        }
//        finally{
//            if (frame != null) frame.close();
//        }
//
//    }

    String format(OpenGLMatrix transformationMatrix) {
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }
}
