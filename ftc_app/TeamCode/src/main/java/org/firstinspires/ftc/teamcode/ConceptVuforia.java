package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;
import android.os.Environment;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.KeyPoint;
import org.opencv.core.Mat;
import org.opencv.core.MatOfKeyPoint;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.features2d.FeatureDetector;
import org.opencv.imgproc.Imgproc;

import java.io.File;
import java.io.FileOutputStream;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.security.Key;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import static java.lang.Math.floor;
import static org.opencv.core.CvType.CV_8U;


@Autonomous(name="Concept: Vuforia Rover Nav", group ="Concept")

public class ConceptVuforia extends LinearOpMode {

    //API Key
    private static final String VUFORIA_KEY = "AXWZcLL/////AAAAGYCcsRtBUkNxiyu+Ou2RGpFDCqMl4iCECw3Qy/FOuiFNCZXzyruu03x1guQYrlkaopplIvPIL65vlGHAyu2NJsfP9DnpqJUSVvuxUOPUfaeFpu2PCulZ7xaOmfdAyF1JSjGDhdz8h01EUB2Surp5vHhqqVhkuBCCui6Vf8Eyy9E6wa5Fs2Y+cInVC/5FEw/+xNYbeZ9aJu8qE1iqU/b78Kzcha3m8FXRsnZl8XYBzGUGndE5A/aT9t+vgcTi8YaBLNvrGOQg+HjVFZIErRJFRgxlZNTr0I6/1BGG8CUg9Ef0f3FjuDoeoTs1BhtPV/jJ8frgQu5YSB9bWxKe91Z1EiNxp5TwzPIt21nAdZlGVNzc";

    @Override public void runOpMode() throws InterruptedException {

        //Loading in OpenCV so it works
        System.loadLibrary("opencv_java3");

        //The parameters for creating an instance of Vuforia
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters params = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        params.vuforiaLicenseKey = VUFORIA_KEY;
        params.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        //Creating a localizer instance of Vuforia
        VuforiaLocalizer vuforia = ClassFactory.getInstance().createVuforia(params);

        //We enable converting a Frame object to a Bitmap image
        vuforia.enableConvertFrameToBitmap();

        //We set the size of the BlockQueue of ClosedFrames to 1, because we only want the most recent image taken
        vuforia.setFrameQueueCapacity(1);

        //Loading in the 4 assets it needs to find and assigning names to them
        VuforiaTrackables targets = vuforia.loadTrackablesFromAsset("RoverRuckus");
        targets.get(0).setName("Blue-Rover");
        targets.get(1).setName("Red-Footprint");
        targets.get(2).setName("Front-Craters");
        targets.get(3).setName("Back-Space");

        telemetry.addData(">", "Press Play to begin");
        telemetry.update();

        waitForStart();

        //Variable declaration
        Mat resizeImageOutput = new Mat();
        Mat hsvThresholdOutput = new Mat();
        Mat blurOutput = new Mat();
        MatOfKeyPoint findBlobsOutput = new MatOfKeyPoint();


        int resizeImageInterpolation = Imgproc.INTER_CUBIC;

        BlurType blurType = BlurType.get("Median Filter");
        double blurRadius = 12.612612612612606;

        double findBlobsMinArea = 2000.0;
        double[] findBlobsCircularity = {0.0, 1.0};
        boolean findBlobsDarkBlobs = false;

        double[] hsvThresholdHue = {0.0035798949192634433, 66.9305748015407};
        double[] hsvThresholdSaturation = {105.43392580392587, 254.9483142931346};
        double[] hsvThresholdValue = {0.0, 255.0};

        Bitmap bm;
        int timer=0;
        Mat locations = new Mat();

        while (opModeIsActive()) {

            //We dequeue the top of the queue, getting the most recent frame. Then convert to BMP. If it is null, then we do it agane until it is not null.
            do {
                bm = vuforia.convertFrameToBitmap(vuforia.getFrameQueue().take());
            } while (bm == null);

            //Converting the BMP to Matrix for use with OpenCV
            Mat m = new Mat(bm.getWidth(), bm.getHeight(), CV_8U);
            Utils.bitmapToMat(bm, m);

            //Here we resize the image using resizeImage taken from GRIP
            double resizeImageWidth = m.width()/3;
            double resizeImageHeight = m.height()/3;
            resizeImage(m, resizeImageWidth, resizeImageHeight, resizeImageInterpolation, resizeImageOutput);

            Mat blurInput = resizeImageOutput;
            blur(blurInput, blurType, blurRadius, blurOutput);

            // Step Find_Blobs0:
            Mat findBlobsInput = blurOutput;
            findBlobs(findBlobsInput, findBlobsMinArea, findBlobsCircularity, findBlobsDarkBlobs, findBlobsOutput);

            // Step HSV_Threshold0:
            Mat hsvThresholdInput = resizeImageOutput;
            hsvThreshold(hsvThresholdInput, hsvThresholdHue, hsvThresholdSaturation, hsvThresholdValue, hsvThresholdOutput);

            //Start of Peter's Code: it should create a new mat, locations, which stores the coords of the nonzero (white pixels), then stores it in a matofpoint format, which is supposedly easier to read?
            Core.findNonZero(hsvThresholdOutput,locations);

            Point[] arrayOfPoints = new MatOfPoint(locations).toArray();
            KeyPoint[] arrayOfPointsB = findBlobsOutput.toArray();
            double stackx = 0;
            double stacky = 0;

            for ( Point p : arrayOfPoints) {

                stackx = stackx+p.x;
                stacky=stacky+p.y;

            }

            double hsvX = stackx/arrayOfPoints.length;
            double hsvY = stacky/arrayOfPoints.length;
            double distSq;
            double radSq;
            ArrayList<KeyPoint> silverBalls = new ArrayList<>();
            ArrayList<Point> minerals = new ArrayList<>();

            for(int i = 0; i<arrayOfPointsB.length; i++) {
                KeyPoint c = arrayOfPointsB[i];
                distSq = Math.pow(c.pt.x - hsvX, 2) + Math.pow(c.pt.y - hsvY, 2);
                radSq = Math.pow(c.size/2 + 50, 2);
                if(distSq > radSq) {
                    silverBalls.add(arrayOfPointsB[i]);
                }
            }

            for (int i = 1; i < silverBalls.size(); i++) {
                for(int j = i ; j > 0 ; j--){
                    if(silverBalls.get(j).size > silverBalls.get(j-1).size){
                        Collections.swap(silverBalls, j, j-1);
                    }
                }
            }

            if(silverBalls.size() > 1) {
                 silverBalls.subList(2, silverBalls.size()).clear();
            }

            for ( KeyPoint b : silverBalls) {
                Point n = new Point(b.pt.x, 0);
                minerals.add(n);
                Imgproc.circle (
                        resizeImageOutput,                 //Matrix obj of the image
                        new Point(b.pt.x, b.pt.y),    //Center of the circle
                        (int)b.size/2,                    //Radius
                        new Scalar(255, 127, 0),  //Scalar object for color
                        6                      //Thickness of the circle
                );
            }

            Imgproc.circle (
                    resizeImageOutput,                 //Matrix obj of the image
                    new Point(hsvX, hsvY),    //Center of the circle
                    50,                    //Radius
                    new Scalar(0, 127, 255),  //Scalar object for color
                    6                      //Thickness of the circle
            );

            Point gold = new Point(hsvX, 1);
            minerals.add(gold);

            for (int i = 1; i < minerals.size(); i++) {
                for(int j = i ; j > 0 ; j--){
                    if(minerals.get(j).x < minerals.get(j-1).x){
                        Collections.swap(minerals, j, j-1);
                    }
                }
            }

            String s = "";

            for(int l = 0; l<minerals.size(); l++) {
                if(minerals.get(l).y == 1) {
                    s += "GOLD ";
                } else {
                    s += "SILVER ";
                }

            }
            timer++;
// Test comment please ignore
            telemetry.addData("timer", timer);
            telemetry.addData(">", s);
            telemetry.update();

            // End of Peter's Code


            //Creating an empty bitmap to send our finished Matrix to, then converting it.
            //Bitmap b = Bitmap.createBitmap(resizeImageOutput.cols(), resizeImageOutput.rows(), Bitmap.Config.RGB_565);
            //Utils.matToBitmap(resizeImageOutput, b);
            //Bitmap blur = Bitmap.createBitmap(blurOutput.cols(), blurOutput.rows(), Bitmap.Config.RGB_565);
            //Utils.matToBitmap(blurOutput, blur);
            //Saving image to filesystem with random name, using SaveImage (below)

            //SaveImage(b);

            //break;

            }
        }
    /**
     * An indication of which type of filter to use for a blur.
     * Choices are BOX, GAUSSIAN, MEDIAN, and BILATERAL
     */
    enum BlurType{
        BOX("Box Blur"), GAUSSIAN("Gaussian Blur"), MEDIAN("Median Filter"),
        BILATERAL("Bilateral Filter");

        private final String label;

        BlurType(String label) {
            this.label = label;
        }

        public static BlurType get(String type) {
            if (BILATERAL.label.equals(type)) {
                return BILATERAL;
            }
            else if (GAUSSIAN.label.equals(type)) {
                return GAUSSIAN;
            }
            else if (MEDIAN.label.equals(type)) {
                return MEDIAN;
            }
            else {
                return BOX;
            }
        }

        @Override
        public String toString() {
            return this.label;
        }
    }

    /**
     * Softens an image using one of several filters.
     * @param input The image on which to perform the blur.
     * @param type The blurType to perform.
     * @param doubleRadius The radius for the blur.
     * @param output The image in which to store the output.
     */
    private void blur(Mat input, BlurType type, double doubleRadius,
                      Mat output) {
        int radius = (int)(doubleRadius + 0.5);
        int kernelSize;
        switch(type){
            case BOX:
                kernelSize = 2 * radius + 1;
                Imgproc.blur(input, output, new Size(kernelSize, kernelSize));
                break;
            case GAUSSIAN:
                kernelSize = 6 * radius + 1;
                Imgproc.GaussianBlur(input,output, new Size(kernelSize, kernelSize), radius);
                break;
            case MEDIAN:
                kernelSize = 2 * radius + 1;
                Imgproc.medianBlur(input, output, kernelSize);
                break;
            case BILATERAL:
                Imgproc.bilateralFilter(input, output, -1, radius, radius);
                break;
        }
    }

    /**
     * Detects groups of pixels in an image.
     * @param input The image on which to perform the find blobs.
     * @param minArea The minimum size of a blob that will be found
     * @param circularity The minimum and maximum circularity of blobs that will be found
     * @param darkBlobs The boolean that determines if light or dark blobs are found.
     * @param blobList The output where the MatOfKeyPoint is stored.
     */
    private void findBlobs(Mat input, double minArea, double[] circularity,
                           Boolean darkBlobs, MatOfKeyPoint blobList) {
        FeatureDetector blobDet = FeatureDetector.create(FeatureDetector.SIMPLEBLOB);
        try {
            File tempFile = File.createTempFile("config", ".xml");

            StringBuilder config = new StringBuilder();

            config.append("<?xml version=\"1.0\"?>\n");
            config.append("<opencv_storage>\n");
            config.append("<thresholdStep>10.</thresholdStep>\n");
            config.append("<minThreshold>50.</minThreshold>\n");
            config.append("<maxThreshold>220.</maxThreshold>\n");
            config.append("<minRepeatability>2</minRepeatability>\n");
            config.append("<minDistBetweenBlobs>10.</minDistBetweenBlobs>\n");
            config.append("<filterByColor>1</filterByColor>\n");
            config.append("<blobColor>");
            config.append((darkBlobs ? 0 : 255));
            config.append("</blobColor>\n");
            config.append("<filterByArea>1</filterByArea>\n");
            config.append("<minArea>");
            config.append(minArea);
            config.append("</minArea>\n");
            config.append("<maxArea>");
            config.append(Integer.MAX_VALUE);
            config.append("</maxArea>\n");
            config.append("<filterByCircularity>1</filterByCircularity>\n");
            config.append("<minCircularity>");
            config.append(circularity[0]);
            config.append("</minCircularity>\n");
            config.append("<maxCircularity>");
            config.append(circularity[1]);
            config.append("</maxCircularity>\n");
            config.append("<filterByInertia>1</filterByInertia>\n");
            config.append("<minInertiaRatio>0.1</minInertiaRatio>\n");
            config.append("<maxInertiaRatio>" + Integer.MAX_VALUE + "</maxInertiaRatio>\n");
            config.append("<filterByConvexity>1</filterByConvexity>\n");
            config.append("<minConvexity>0.95</minConvexity>\n");
            config.append("<maxConvexity>" + Integer.MAX_VALUE + "</maxConvexity>\n");
            config.append("</opencv_storage>\n");
            FileWriter writer;
            writer = new FileWriter(tempFile, false);
            writer.write(config.toString());
            writer.close();
            blobDet.read(tempFile.getPath());
        } catch (IOException e) {
            e.printStackTrace();
        }

        blobDet.detect(input, blobList);
    }

    //Taken from StackOverflow, added in random image naming so it doesnt overwrite
    private void SaveImage(Bitmap finalBitmap) {

        String root = Environment.getExternalStorageDirectory().toString();
        File myDir = new File(root + "/saved_images");
        if(myDir.mkdirs()) {
            telemetry.addData(">", "File saved!");
        }
        double random = floor(Math.random() * 500 + 1);
        String fname = "tmp" + random + ".jpg";
        File file = new File (myDir, fname);
        try {
            FileOutputStream out = new FileOutputStream(file);
            finalBitmap.compress(Bitmap.CompressFormat.JPEG, 90, out);
            out.flush();
            out.close();

        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    //GRIP Function
    private void resizeImage(Mat input, double width, double height,
                             int interpolation, Mat output) {
        Imgproc.resize(input, output, new Size(width, height), 0.0, 0.0, interpolation);
    }

    //GRIP Function
    private void hsvThreshold(Mat input, double[] hue, double[] sat, double[] val,
                              Mat out) {
        Imgproc.cvtColor(input, out, Imgproc.COLOR_RGB2HSV);
        Core.inRange(out, new Scalar(hue[0], sat[0], val[0]),
                new Scalar(hue[1], sat[1], val[1]), out);
    }





}
