/*
 * Copyright (c) 2020 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

public class TripleCamera
{
    OpenCvCamera frontCam;
    OpenCvCamera leftCam;
    OpenCvCamera rightCam;
    HardwareMap hardwareMap;

    FrontPipeline frontPipeline;
    LeftPipeline leftPipeline;
    RightPipeline rightPipeline;

    public TripleCamera(HardwareMap hwMap) {
        hardwareMap = hwMap;
    }

    public void initCamera() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        int[] viewportContainerIds = OpenCvCameraFactory.getInstance()
                .splitLayoutForMultipleViewports(
                        cameraMonitorViewId, //The container we're splitting
                        3, //The number of sub-containers to create
                        OpenCvCameraFactory.ViewportSplitMethod.VERTICALLY); //Whether to split the container vertically or horizontally

        frontCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, viewportContainerIds[0]);
        leftCam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "leftCam"), viewportContainerIds[1]);
        rightCam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "rightCam"), viewportContainerIds[2]);

        frontCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened()
            {
                frontCam.setPipeline(frontPipeline);
                frontCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });

        leftCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened()
            {
                leftCam.setPipeline(leftPipeline);
                leftCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });

        rightCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened()
            {
                rightCam.setPipeline(rightPipeline);
                rightCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });
    }

    public static class FrontPipeline extends OpenCvPipeline {
        /*
         * An enum to define the parking position
         */
        public enum SignalPosition {
            LEFT,
            CENTER,
            RIGHT
        }
        public enum FrontAligned {
            YES,
            NO
        }

        // box fills
        static final Scalar BLUE = new Scalar(0, 0, 255);
        static final Scalar BLACK = new Scalar(0, 0, 0);
        static final Scalar ORANGE = new Scalar(255, 69, 0);
        static final Scalar YELLOW = new Scalar(255, 255, 0);

        // box outlines
        static final Scalar GREEN = new Scalar(0, 255, 0);

        /*
         * The core values which define the location and size of the sample regions
         */
        static final Point SLEEVE_TOPLEFT_ANCHOR_POINT = new Point(145,40);
        static final int SLEEVE_REGION_WIDTH = 25;
        static final int SLEEVE_REGION_HEIGHT = 15;

        static final Point AUTO_ALIGN_TOPLEFT_ANCHOR_POINT = new Point(0,40);
        static final int AUTO_ALIGN_REGION_WIDTH = 145;
        static final int AUTO_ALIGN_REGION_HEIGHT = 15;

        /*
         * Points which actually define the sample region rectangles, derived from above values
         *
         * Example of how points A and B work to define a rectangle
         *
         *   ------------------------------------
         *   | (0,0) Point A                    |
         *   |                                  |
         *   |                                  |
         *   |                                  |
         *   |                                  |
         *   |                                  |
         *   |                                  |
         *   |                  Point B (70,50) |
         *   ------------------------------------
         *
         */
        Point sleeve_pointA = new Point(
                SLEEVE_TOPLEFT_ANCHOR_POINT.x,
                SLEEVE_TOPLEFT_ANCHOR_POINT.y);
        Point sleeve_pointB = new Point(
                SLEEVE_TOPLEFT_ANCHOR_POINT.x + SLEEVE_REGION_WIDTH,
                SLEEVE_TOPLEFT_ANCHOR_POINT.y + SLEEVE_REGION_HEIGHT);

        Point auto_align_pointA = new Point(
                AUTO_ALIGN_TOPLEFT_ANCHOR_POINT.x,
                AUTO_ALIGN_TOPLEFT_ANCHOR_POINT.y);
        Point auto_align_pointB = new Point(
                AUTO_ALIGN_TOPLEFT_ANCHOR_POINT.x + AUTO_ALIGN_REGION_WIDTH,
                AUTO_ALIGN_TOPLEFT_ANCHOR_POINT.y + AUTO_ALIGN_REGION_HEIGHT);

        /*
         * Working variables
         */
        Mat sleeve_Y, sleeve_Cr, sleeve_Cb;
        Mat auto_align_Y, auto_align_Cr, auto_align_Cb;
        Mat YCrCb = new Mat();
        Mat YImage = new Mat();
        Mat CrImage = new Mat();
        Mat CbImage = new Mat();
        int sleeve_avgY, sleeve_avgCr, sleeve_avgCb;
        int auto_align_avgY, auto_align_avgCr, auto_align_avgCb;

        // Volatile since accessed by OpMode thread w/o synchronization
        private volatile SignalPosition position = SignalPosition.RIGHT;
        private volatile String StringPos = "CENTER";

        private volatile FrontAligned aligned = FrontAligned.NO;
        private volatile String StringAligned = "NO";

        /*
         * This function takes the RGB frame, converts to YCrCb,
         * and extracts the Y channel to the 'Y' variable
         */
        void inputToChannels(Mat input) {
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(YCrCb, YImage, 0);
            Core.extractChannel(YCrCb, CrImage, 1);
            Core.extractChannel(YCrCb, CbImage, 2);
        }

        @Override
        public void init(Mat firstFrame) {
            /*
             * We need to call this in order to make sure the 'Y'
             * object is initialized, so that the submats we make
             * will still be linked to it on subsequent frames. (If
             * the object were to only be initialized in processFrame,
             * then the submats would become delinked because the backing
             * buffer would be re-allocated the first time a real frame
             * was crunched)
             */
            inputToChannels(firstFrame);

            /*
             * Submats are a persistent reference to a region of the parent
             * buffer. Any changes to the child affect the parent, and the
             * reverse also holds true.
             */
            sleeve_Y = YImage.submat(new Rect(sleeve_pointA, sleeve_pointB));
            sleeve_Cr = CrImage.submat(new Rect(sleeve_pointA, sleeve_pointB));
            sleeve_Cb = CbImage.submat(new Rect(sleeve_pointA, sleeve_pointB));

            auto_align_Y = YImage.submat(new Rect(auto_align_pointA, auto_align_pointB));
            auto_align_Cr = CrImage.submat(new Rect(auto_align_pointA, auto_align_pointB));
            auto_align_Cb = CbImage.submat(new Rect(auto_align_pointA, auto_align_pointB));
        }

        @Override
        public Mat processFrame(Mat input) {
            /*
             * Overview of what we're doing:
             *
             * We first convert to YCrCb color space, from RGB color space.
             * Why do we do this? Well, in the RGB color space, chroma and
             * luma are intertwined. In YCrCb, chroma and luma are separated.
             * YCrCb is a 3-channel color space, just like RGB. YCrCb's 3 channels
             * are Y, the luma channel (which essentially just a B&W image), the
             * Cr channel, which records the difference from red, and the Cb channel,
             * which records the difference from blue. Because chroma and luma are
             * not related in YCrCb, vision code written to look for certain values
             * in the Cr/Cb channels will not be severely affected by differing
             * light intensity, since that difference would most likely just be
             * reflected in the Y channel.
             *
             * After we've converted to YCrCb, we extract just the 2nd channel, the
             * Cb channel. We do this because stones are bright yellow and contrast
             * STRONGLY on the Cb channel against everything else, including SkyStones
             * (because SkyStones have a black label).
             *
             * We then take the average pixel value of 3 different regions on that Cb
             * channel, one positioned over each stone. The brightest of the 3 regions
             * is where we assume the SkyStone to be, since the normal stones show up
             * extremely darkly.
             *
             * We also draw rectangles on the screen showing where the sample regions
             * are, as well as drawing a solid rectangle over top the sample region
             * we believe is on top of the SkyStone.
             *
             * In order for this whole process to work correctly, each sample region
             * should be positioned in the center of each of the first 3 stones, and
             * be small enough such that only the stone is sampled, and not any of the
             * surroundings.
             */

            /*
             * Get the Cb channel of the input frame after conversion to YCrCb
             */
            inputToChannels(input);

            /*
             * Compute the average pixel value of each submat region. We're
             * taking the average of a single channel buffer, so the value
             * we need is at index 0. We could have also taken the average
             * pixel value of the 3-channel image, and referenced the value
             * at index 2 here.
             */
            sleeve_avgY = (int) Core.mean(sleeve_Y).val[0];
            sleeve_avgCr = (int) Core.mean(sleeve_Cr).val[0];
            sleeve_avgCb = (int) Core.mean(sleeve_Cb).val[0];

            auto_align_avgY = (int) Core.mean(sleeve_Y).val[0];
            auto_align_avgCr = (int) Core.mean(sleeve_Cr).val[0];
            auto_align_avgCb = (int) Core.mean(sleeve_Cb).val[0];

            /*
             * Draw a rectangle showing sample sleeve box on the screen.
             * Simply a visual aid. Serves no functional purpose.
             */
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    sleeve_pointA, // First point which defines the rectangle
                    sleeve_pointB, // Second point which defines the rectangle
                    GREEN, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines

            /*
             * Draw a rectangle showing sample sleeve box on the screen.
             * Simply a visual aid. Serves no functional purpose.
             */
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    auto_align_pointA, // First point which defines the rectangle
                    auto_align_pointB, // Second point which defines the rectangle
                    GREEN, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines

            /*
             * Now that we found the min, we actually need to go and
             * figure out which sample region that value was from
             */
            if(sleeve_avgCb - sleeve_avgCr > 20) {
                position = SignalPosition.LEFT; // Record our analysis
                StringPos = "RIGHT";

                /*
                 * Draw a solid rectangle on top of the chosen region.
                 * Simply a visual aid. Serves no functional purpose.
                 */
                Imgproc.rectangle(
                        input, // Buffer to draw on
                        sleeve_pointA, // First point which defines the rectangle
                        sleeve_pointB, // Second point which defines the rectangle
                        BLUE, // The color the rectangle is drawn in
                        -1); // Negative thickness means solid fill
            }
            else if(sleeve_avgCr - sleeve_avgCb > 20) {
                position = SignalPosition.RIGHT; // Record our analysis
                StringPos = "LEFT";

                /*
                 * Draw a solid rectangle on top of the chosen region.
                 * Simply a visual aid. Serves no functional purpose.
                 */
                Imgproc.rectangle(
                        input, // Buffer to draw on
                        sleeve_pointA, // First point which defines the rectangle
                        sleeve_pointB, // Second point which defines the rectangle
                        ORANGE, // The color the rectangle is drawn in
                        -1); // Negative thickness means solid fill
            }
            else{
                position = SignalPosition.CENTER; // Record our analysis
                StringPos = "CENTER";

                /*
                 * Draw a solid rectangle on top of the chosen region.
                 * Simply a visual aid. Serves no functional purpose.
                 */
                Imgproc.rectangle(
                        input, // Buffer to draw on
                        sleeve_pointA, // First point which defines the rectangle
                        sleeve_pointB, // Second point which defines the rectangle
                        BLACK, // The color the rectangle is drawn in
                        -1); // Negative thickness means solid fill
            }

            if(sleeve_avgCb > 95 && sleeve_avgCb < 110) {
                aligned = FrontAligned.YES; // Record our analysis
                StringAligned = "YES";

                /*
                 * Draw a solid rectangle on top of the chosen region.
                 * Simply a visual aid. Serves no functional purpose.
                 */
                Imgproc.rectangle(
                        input, // Buffer to draw on
                        auto_align_pointA, // First point which defines the rectangle
                        auto_align_pointB, // Second point which defines the rectangle
                        YELLOW, // The color the rectangle is drawn in
                        -1); // Negative thickness means solid fill
            }

            /*
             * Render the 'input' buffer to the viewport. But note this is not
             * simply rendering the raw camera feed, because we called functions
             * to add some annotations to this buffer earlier up.
             */
            return input;
        }

        /*
         * Call this from the OpMode thread to obtain the latest analysis
         */
        public SignalPosition getAnalysis()
        {
            return position;
        }

        public String getStringAnalysis()
        {
            return StringPos;
        }

        public String getAlignedAnalysis()
        {
            return StringAligned;
        }

        public int getSleeve_avgY() { return sleeve_avgY; }
        public int getSleeve_avgCr() { return sleeve_avgCr; }
        public int getSleeve_avgCb() { return sleeve_avgCb; }

        public int getAuto_align_avgY() {
            return auto_align_avgY;
        }
        public int getAuto_align_avgCr() {
            return auto_align_avgCr;
        }
        public int getAuto_align_avgCb() {
            return auto_align_avgCb;
        }
    }

    public static class LeftPipeline extends OpenCvPipeline {
        public enum LeftAligned {
            YES,
            NO
        }

        // box fills
        static final Scalar YELLOW = new Scalar(255, 255, 0);

        // box outlines
        static final Scalar GREEN = new Scalar(0, 255, 0);

        static final Point LEFT_ALIGN_TOPLEFT_ANCHOR_POINT = new Point(0,40);
        static final int LEFT_ALIGN_REGION_WIDTH = 145;
        static final int LEFT_ALIGN_REGION_HEIGHT = 15;

        /*
         * Points which actually define the sample region rectangles, derived from above values
         *
         * Example of how points A and B work to define a rectangle
         *
         *   ------------------------------------
         *   | (0,0) Point A                    |
         *   |                                  |
         *   |                                  |
         *   |                                  |
         *   |                                  |
         *   |                                  |
         *   |                                  |
         *   |                  Point B (70,50) |
         *   ------------------------------------
         *
         */
        Point left_align_pointA = new Point(
                LEFT_ALIGN_TOPLEFT_ANCHOR_POINT.x,
                LEFT_ALIGN_TOPLEFT_ANCHOR_POINT.y);
        Point left_align_pointB = new Point(
                LEFT_ALIGN_TOPLEFT_ANCHOR_POINT.x + LEFT_ALIGN_REGION_WIDTH,
                LEFT_ALIGN_TOPLEFT_ANCHOR_POINT.y + LEFT_ALIGN_REGION_HEIGHT);

        /*
         * Working variables
         */
        Mat left_align_Y, left_align_Cr, left_align_Cb;
        Mat YCrCb = new Mat();
        Mat YImage = new Mat();
        Mat CrImage = new Mat();
        Mat CbImage = new Mat();
        int left_align_avgY, left_align_avgCr, left_align_avgCb;

        // Volatile since accessed by OpMode thread w/o synchronization
        private volatile LeftAligned aligned = LeftAligned.NO;
        private volatile String StringAligned = "NO";

        /*
         * This function takes the RGB frame, converts to YCrCb,
         * and extracts the Y channel to the 'Y' variable
         */
        void inputToChannels(Mat input) {
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(YCrCb, YImage, 0);
            Core.extractChannel(YCrCb, CrImage, 1);
            Core.extractChannel(YCrCb, CbImage, 2);
        }

        @Override
        public void init(Mat firstFrame) {
            /*
             * We need to call this in order to make sure the 'Y'
             * object is initialized, so that the submats we make
             * will still be linked to it on subsequent frames. (If
             * the object were to only be initialized in processFrame,
             * then the submats would become delinked because the backing
             * buffer would be re-allocated the first time a real frame
             * was crunched)
             */
            inputToChannels(firstFrame);

            /*
             * Submats are a persistent reference to a region of the parent
             * buffer. Any changes to the child affect the parent, and the
             * reverse also holds true.
             */
            left_align_Y = YImage.submat(new Rect(left_align_pointA, left_align_pointB));
            left_align_Cr = CrImage.submat(new Rect(left_align_pointA, left_align_pointB));
            left_align_Cb = CbImage.submat(new Rect(left_align_pointA, left_align_pointB));
        }

        @Override
        public Mat processFrame(Mat input) {
            /*
             * Overview of what we're doing:
             *
             * We first convert to YCrCb color space, from RGB color space.
             * Why do we do this? Well, in the RGB color space, chroma and
             * luma are intertwined. In YCrCb, chroma and luma are separated.
             * YCrCb is a 3-channel color space, just like RGB. YCrCb's 3 channels
             * are Y, the luma channel (which essentially just a B&W image), the
             * Cr channel, which records the difference from red, and the Cb channel,
             * which records the difference from blue. Because chroma and luma are
             * not related in YCrCb, vision code written to look for certain values
             * in the Cr/Cb channels will not be severely affected by differing
             * light intensity, since that difference would most likely just be
             * reflected in the Y channel.
             *
             * After we've converted to YCrCb, we extract just the 2nd channel, the
             * Cb channel. We do this because stones are bright yellow and contrast
             * STRONGLY on the Cb channel against everything else, including SkyStones
             * (because SkyStones have a black label).
             *
             * We then take the average pixel value of 3 different regions on that Cb
             * channel, one positioned over each stone. The brightest of the 3 regions
             * is where we assume the SkyStone to be, since the normal stones show up
             * extremely darkly.
             *
             * We also draw rectangles on the screen showing where the sample regions
             * are, as well as drawing a solid rectangle over top the sample region
             * we believe is on top of the SkyStone.
             *
             * In order for this whole process to work correctly, each sample region
             * should be positioned in the center of each of the first 3 stones, and
             * be small enough such that only the stone is sampled, and not any of the
             * surroundings.
             */

            /*
             * Get the Cb channel of the input frame after conversion to YCrCb
             */
            inputToChannels(input);

            /*
             * Compute the average pixel value of each submat region. We're
             * taking the average of a single channel buffer, so the value
             * we need is at index 0. We could have also taken the average
             * pixel value of the 3-channel image, and referenced the value
             * at index 2 here.
             */
            left_align_avgY = (int) Core.mean(left_align_Y).val[0];
            left_align_avgCr = (int) Core.mean(left_align_Cr).val[0];
            left_align_avgCb = (int) Core.mean(left_align_Cb).val[0];

            /*
             * Draw a rectangle showing sample sleeve box on the screen.
             * Simply a visual aid. Serves no functional purpose.
             */
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    left_align_pointA, // First point which defines the rectangle
                    left_align_pointB, // Second point which defines the rectangle
                    GREEN, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines

            /*
             * Now that we found the min, we actually need to go and
             * figure out which sample region that value was from
             */

            if(left_align_avgCb > 95 && left_align_avgCb < 110) {
                aligned = LeftAligned.YES; // Record our analysis
                StringAligned = "YES";

                /*
                 * Draw a solid rectangle on top of the chosen region.
                 * Simply a visual aid. Serves no functional purpose.
                 */
                Imgproc.rectangle(
                        input, // Buffer to draw on
                        left_align_pointA, // First point which defines the rectangle
                        left_align_pointB, // Second point which defines the rectangle
                        YELLOW, // The color the rectangle is drawn in
                        -1); // Negative thickness means solid fill
            }

            /*
             * Render the 'input' buffer to the viewport. But note this is not
             * simply rendering the raw camera feed, because we called functions
             * to add some annotations to this buffer earlier up.
             */
            return input;
        }

        /*
         * Call this from the OpMode thread to obtain the latest analysis
         */
        public String getAlignedAnalysis()
        {
            return StringAligned;
        }

        public int getLeft_align_avgY() {
            return left_align_avgY;
        }
        public int getLeft_align_avgCr() {
            return left_align_avgCr;
        }
        public int getLeft_align_avgCb() {
            return left_align_avgCb;
        }
    }

    public static class RightPipeline extends OpenCvPipeline {
        public enum RightAligned {
            YES,
            NO
        }

        // box fills
        static final Scalar YELLOW = new Scalar(255, 255, 0);

        // box outlines
        static final Scalar GREEN = new Scalar(0, 255, 0);

        static final Point RIGHT_ALIGN_TOPRIGHT_ANCHOR_POINT = new Point(0,40);
        static final int RIGHT_ALIGN_REGION_WIDTH = 145;
        static final int RIGHT_ALIGN_REGION_HEIGHT = 15;

        /*
         * Points which actually define the sample region rectangles, derived from above values
         *
         * Example of how points A and B work to define a rectangle
         *
         *   ------------------------------------
         *   | (0,0) Point A                    |
         *   |                                  |
         *   |                                  |
         *   |                                  |
         *   |                                  |
         *   |                                  |
         *   |                                  |
         *   |                  Point B (70,50) |
         *   ------------------------------------
         *
         */
        Point right_align_pointA = new Point(
                RIGHT_ALIGN_TOPRIGHT_ANCHOR_POINT.x,
                RIGHT_ALIGN_TOPRIGHT_ANCHOR_POINT.y);
        Point right_align_pointB = new Point(
                RIGHT_ALIGN_TOPRIGHT_ANCHOR_POINT.x + RIGHT_ALIGN_REGION_WIDTH,
                RIGHT_ALIGN_TOPRIGHT_ANCHOR_POINT.y + RIGHT_ALIGN_REGION_HEIGHT);

        /*
         * Working variables
         */
        Mat right_align_Y, right_align_Cr, right_align_Cb;
        Mat YCrCb = new Mat();
        Mat YImage = new Mat();
        Mat CrImage = new Mat();
        Mat CbImage = new Mat();
        int right_align_avgY, right_align_avgCr, right_align_avgCb;

        // Volatile since accessed by OpMode thread w/o synchronization
        private volatile TripleCamera.RightPipeline.RightAligned aligned = TripleCamera.RightPipeline.RightAligned.NO;
        private volatile String StringAligned = "NO";

        /*
         * This function takes the RGB frame, converts to YCrCb,
         * and extracts the Y channel to the 'Y' variable
         */
        void inputToChannels(Mat input) {
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(YCrCb, YImage, 0);
            Core.extractChannel(YCrCb, CrImage, 1);
            Core.extractChannel(YCrCb, CbImage, 2);
        }

        @Override
        public void init(Mat firstFrame) {
            /*
             * We need to call this in order to make sure the 'Y'
             * object is initialized, so that the submats we make
             * will still be linked to it on subsequent frames. (If
             * the object were to only be initialized in processFrame,
             * then the submats would become delinked because the backing
             * buffer would be re-allocated the first time a real frame
             * was crunched)
             */
            inputToChannels(firstFrame);

            /*
             * Submats are a persistent reference to a region of the parent
             * buffer. Any changes to the child affect the parent, and the
             * reverse also holds true.
             */
            right_align_Y = YImage.submat(new Rect(right_align_pointA, right_align_pointB));
            right_align_Cr = CrImage.submat(new Rect(right_align_pointA, right_align_pointB));
            right_align_Cb = CbImage.submat(new Rect(right_align_pointA, right_align_pointB));
        }

        @Override
        public Mat processFrame(Mat input) {
            /*
             * Overview of what we're doing:
             *
             * We first convert to YCrCb color space, from RGB color space.
             * Why do we do this? Well, in the RGB color space, chroma and
             * luma are intertwined. In YCrCb, chroma and luma are separated.
             * YCrCb is a 3-channel color space, just like RGB. YCrCb's 3 channels
             * are Y, the luma channel (which essentially just a B&W image), the
             * Cr channel, which records the difference from red, and the Cb channel,
             * which records the difference from blue. Because chroma and luma are
             * not related in YCrCb, vision code written to look for certain values
             * in the Cr/Cb channels will not be severely affected by differing
             * light intensity, since that difference would most likely just be
             * reflected in the Y channel.
             *
             * After we've converted to YCrCb, we extract just the 2nd channel, the
             * Cb channel. We do this because stones are bright yellow and contrast
             * STRONGLY on the Cb channel against everything else, including SkyStones
             * (because SkyStones have a black label).
             *
             * We then take the average pixel value of 3 different regions on that Cb
             * channel, one positioned over each stone. The brightest of the 3 regions
             * is where we assume the SkyStone to be, since the normal stones show up
             * extremely darkly.
             *
             * We also draw rectangles on the screen showing where the sample regions
             * are, as well as drawing a solid rectangle over top the sample region
             * we believe is on top of the SkyStone.
             *
             * In order for this whole process to work correctly, each sample region
             * should be positioned in the center of each of the first 3 stones, and
             * be small enough such that only the stone is sampled, and not any of the
             * surroundings.
             */

            /*
             * Get the Cb channel of the input frame after conversion to YCrCb
             */
            inputToChannels(input);

            /*
             * Compute the average pixel value of each submat region. We're
             * taking the average of a single channel buffer, so the value
             * we need is at index 0. We could have also taken the average
             * pixel value of the 3-channel image, and referenced the value
             * at index 2 here.
             */
            right_align_avgY = (int) Core.mean(right_align_Y).val[0];
            right_align_avgCr = (int) Core.mean(right_align_Cr).val[0];
            right_align_avgCb = (int) Core.mean(right_align_Cb).val[0];

            /*
             * Draw a rectangle showing sample sleeve box on the screen.
             * Simply a visual aid. Serves no functional purpose.
             */
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    right_align_pointA, // First point which defines the rectangle
                    right_align_pointB, // Second point which defines the rectangle
                    GREEN, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines

            /*
             * Now that we found the min, we actually need to go and
             * figure out which sample region that value was from
             */

            if(right_align_avgCb > 95 && right_align_avgCb < 110) {
                aligned = TripleCamera.RightPipeline.RightAligned.YES; // Record our analysis
                StringAligned = "YES";

                /*
                 * Draw a solid rectangle on top of the chosen region.
                 * Simply a visual aid. Serves no functional purpose.
                 */
                Imgproc.rectangle(
                        input, // Buffer to draw on
                        right_align_pointA, // First point which defines the rectangle
                        right_align_pointB, // Second point which defines the rectangle
                        YELLOW, // The color the rectangle is drawn in
                        -1); // Negative thickness means solid fill
            }

            /*
             * Render the 'input' buffer to the viewport. But note this is not
             * simply rendering the raw camera feed, because we called functions
             * to add some annotations to this buffer earlier up.
             */
            return input;
        }

        /*
         * Call this from the OpMode thread to obtain the latest analysis
         */
        public String getAlignedAnalysis()
        {
            return StringAligned;
        }

        public int getRight_align_avgY() {
            return right_align_avgY;
        }
        public int getRight_align_avgCr() {
            return right_align_avgCr;
        }
        public int getRight_align_avgCb() {
            return right_align_avgCb;
        }
    }
}
