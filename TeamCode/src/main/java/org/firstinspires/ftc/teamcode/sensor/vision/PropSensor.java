package org.firstinspires.ftc.teamcode.sensor.vision;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

public class PropSensor {
    private final OpenCvCamera camera;
    private final PropSensorPipeline pipeline;
    private boolean isOpened = false;
    private int propPosition = 0;

    private static final double[]
            BLUE_UPPER = {150, 255, 60},
            BLUE_LOWER = {0, 85, 0},
            RED_UPPER = {66, 62, 249},
            RED_LOWER = {0, 0, 107};
  
    public PropSensor(HardwareMap hardwareMap, boolean isRed) {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        WebcamName name = hardwareMap.get(WebcamName.class, "Webcam 1");
        camera = OpenCvCameraFactory.getInstance().createWebcam(name, cameraMonitorViewId);
  
        pipeline = new PropSensorPipeline(isRed);

        initializeCamera();
    }

    private void initializeCamera() {
        camera.setPipeline(pipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
                isOpened = true;
            }

            @Override
            public void onError(int errorCode) {}
        });
    }

    public int propPosition() {
        return propPosition;
    }

    public OpenCvCamera getCamera() {
        return camera;
    }

    public boolean getIsOpened() {
        return isOpened;
    }

    private class PropSensorPipeline extends OpenCvPipeline {
        // left, center, right = 0, 1, 2

        private final int[] average = new int[2];
        private final double[] upper, lower;

        public PropSensorPipeline(boolean isRed) {
            this.upper = isRed ? RED_UPPER : BLUE_UPPER;
            this.lower = isRed ? RED_LOWER : BLUE_LOWER;
        }

        @Override
        public Mat processFrame(Mat input) {
            Imgproc.cvtColor(input, input, Imgproc.COLOR_RGB2BGR, 0);
            Mat mask = createMask(input, upper, lower);
            Imgproc.cvtColor(mask, mask, Imgproc.COLOR_BGR2GRAY, 0);
            Rect rect = Imgproc.boundingRect(mask);

            Scalar color = new Scalar(0, 255, 0);
            Point midpoint = new Point(average[0], average[1]);

            Imgproc.line(input, new Point(input.width() / 3.0, 0), new Point(input.width() / 3.0, input.height()), color, 5);
            Imgproc.line(input, new Point(2 * input.width() / 3.0, 0), new Point(2 * input.width() / 3.0, input.height()), color, 5);
            Imgproc.rectangle(input, rect, color, 5);
            Imgproc.circle(input, midpoint, 2, color, 10);

            double x = midpoint.x / input.cols();
            propPosition = x <= 1.0 / 3.0 ? 0 : x <= 2.0 / 3.0 ? 1 : 2;

            return input;
        }

        private Mat createMask(Mat input, double[] upper, double[] lower) {
            int counter = 1;
            average[0] = 0;
            average[1] = 0;
            double[] white = {255, 255, 255};
            double[] black = {0, 0, 0};
            Mat filtered = new Mat();
            input.copyTo(filtered);

            for(int i = 0; i < input.rows(); i++) {
                for(int j = 0; j < input.cols(); j++) {
                    double[] data = input.get(i, j);
                    if (data[2] < upper[2] && data[2] > lower[2] && data[1] < upper[1] && data[1] > lower[1] && data[0] < upper[0] && data[0] > lower[0]) {
                        filtered.put(i, j, white);
                        counter++;
                        average[0] += j;
                        average[1] += i;
                    } else {
                        filtered.put(i, j, black);
                    }
                }
            }
            average[0] /= counter;
            average[1] /= counter;

            return filtered;
        }
    }
}
