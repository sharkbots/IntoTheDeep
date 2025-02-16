//package org.firstinspires.ftc.teamcode.common.vision;
//
//import com.acmerobotics.dashboard.config.Config;
//
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.firstinspires.ftc.teamcode.common.vision.sampleDetection.SampleDetectionPipeline;
//import org.openftc.easyopencv.OpenCvCameraFactory;
//import org.openftc.easyopencv.OpenCvCameraRotation;
//import org.openftc.easyopencv.OpenCvWebcam;
//import org.openftc.easyopencv.OpenCvCamera;
//
//@Config
//public class CVManager {
//    public static boolean shouldStream = false;
//    OpenCvWebcam webcam;
//
//    SampleDetectionPipeline sample;
//    public CVManager(){
//        webcam = OpenCvCameraFactory.getInstance()
//                .createWebcam(op.hardwareMap.get(WebcamName.class, "Webcam 2"));
//        sample = new SampleDetectionPipeline();
////            if(isTeleop) {
//        startStreamin();
////            }
//    }
//    public double[] getCenter(){
//        return sample.getCenter();
//    }
//    public void startStreamin(){
//        webcam.openCameraDeviceAsync(
//                new OpenCvCamera.AsyncCameraOpenListener() {
//                    @Override
//                    public void onOpened() {
//                        /*
//                         * Tell the webcam to start streaming images to us! Note that you must make sure
//                         * the resolution you specify is supported by the camera. If it is not, an exception
//                         * will be thrown.
//                         *
//                         * Keep in mind that the SDK's UVC driver (what OpenCvWebcam uses under the hood) only
//                         * supports streaming from the webcam in the uncompressed YUV image format. This means
//                         * that the maximum resolution you can stream at and still get up to 30FPS is 480p (640x480).
//                         * Streaming at e.g. 720p will limit you to up to 10FPS and so on and so forth.
//                         *
//                         * Also, we specify the rotation that the webcam is used in. This is so that the image
//                         * from the camera sensor can be rotated such that it is always displayed with the image upright.
//                         * For a front facing camera, rotation is defined assuming the user is looking at the screen.
//                         * For a rear facing camera or a webcam, rotation is defined assuming the camera is facing
//                         * away from the user.
//                         */
//                        webcam.setPipeline(sample);
//                        webcam.startStreaming(
//                                1280, 720, OpenCvCameraRotation.UPRIGHT, OpenCvWebcam.StreamFormat.MJPEG);
//
//                        webcam.setViewportRenderer(OpenCvCamera.ViewportRenderer.SOFTWARE);
////                        dashboard.startCameraStream(webcam,5);
//
////                        if(shouldStream)
//                        dashboard.startCameraStream(webcam, 4);
//                        LOGGER.log("Camera Streaming now!");
//                    }
//
//                    @Override
//                    public void onError(int errorCode) {
//                        /*
//                         * This will be called if the camera could not be opened
//                         */
//                    }
//                });
//    }
//    public int getCurrent(){
//        return sample.getColor();
//    }
//    public void swapInt(int swap){
//        sample.setColor(swap);
//    }
//    public void swapNext(){
//        int newy = sample.getColor()+1;
//        if (newy > 2) newy = 0;
//        sample.setColor(newy);
//    }
//    public double getLatency(){
//        return Math.min(1/ webcam.getFps(),.2/*,0.1+webcam.getTotalFrameTimeMs()*/);
//    }
//    public void swapRed(){
//        sample.setColor(0);
//    }
//    public void swapBlue(){
//        sample.setColor(1);
//    }
//    public void swapYellow(){
//        sample.setColor(2);
//    }
//    public void resetCenter(){
//        sample.resetCenter();
//    }
//}
