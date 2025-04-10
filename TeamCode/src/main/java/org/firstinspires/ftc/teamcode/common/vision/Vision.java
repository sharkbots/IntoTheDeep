package org.firstinspires.ftc.teamcode.common.vision;

import static org.firstinspires.ftc.teamcode.common.utils.Globals.CAMERA_EXPOSURE_MILLIS;
import static org.firstinspires.ftc.teamcode.common.utils.Globals.CAMERA_GAIN;
import static org.firstinspires.ftc.teamcode.common.utils.Globals.CAMERA_STREAM_HEIGHT;
import static org.firstinspires.ftc.teamcode.common.utils.Globals.CAMERA_STREAM_WIDTH;
import static org.firstinspires.ftc.teamcode.common.utils.Globals.CAMERA_WHITE_BALANCE_TEMPERATURE;

import static java.lang.Thread.sleep;

import android.util.Size;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.WhiteBalanceControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.Arrays;
import java.util.List;
import java.util.concurrent.TimeUnit;

@Config
public class Vision {
    public CVCamera cvCamera;
    public Limelight limelight;

    public Vision(){
        cvCamera = new CVCamera();
        limelight = new Limelight();
        cvCamera.cameraInRange();
    }

    public class Limelight {
        private Limelight3A limelight;
        List<Point> sGroundCoordinates = Arrays.asList(
                new Point(0,4), new Point(0,7), new Point(0,10), new Point(0,13), new Point(0,16), new Point(-3,4), new Point(-3,7), new Point(-3,10), new Point(-3,13), new Point(-3,16), new Point(-6,7), new Point(-6,10), new Point(-6,13), new Point(-6,16), new Point(-9,16), new Point(3,4), new Point(3,7), new Point(3,10), new Point(3,13), new Point(3,16), new Point(3,19), new Point(6,7), new Point(6,10), new Point(6,13), new Point(6,16), new Point(6,19), new Point(9,16));
        List<Point> sPixelPoints  = Arrays.asList(
                new Point(160,57), new Point(160,118), new Point(160,161), new Point(160,192), new Point(160,218), new Point(72,55), new Point(87,117), new Point(98,160), new Point(106,193), new Point(112,218), new Point(14,117), new Point(35,159), new Point(52,193), new Point(64,219), new Point(17,220), new Point(250,56), new Point(233,119), new Point(222,162), new Point(214,195), new Point(208,219), new Point(205,238), new Point(308,120), new Point(286,163), new Point(270,195), new Point(257,218), new Point(248,237), new Point(306,219)   );

        Mat mHomography = null;
        LLResult latestResults = null;
        List<LLResultTypes.DetectorResult> latestDetectorResults = null;
        LLResultTypes.DetectorResult closestResult = null;

        /** Constructor */
        public Limelight() {
            initializeHomography();
        }

        public void setLimelight(Limelight3A limelight) {
            this.limelight = limelight;
        }

        public void start(){
            limelight.start();
        }

        public void setPipeline(int pipeline){
            limelight.pipelineSwitch(pipeline);
        }

        public LLStatus getStatus(){
            return limelight.getStatus();
        }

        public void setLatestResults(){
            LLResult results = limelight.getLatestResult();
            if (results != null){
                this.latestResults = limelight.getLatestResult();
            }
        }

        public LLResult getLatestResults(){
            return this.latestResults;
        }

        public List<LLResultTypes.DetectorResult> getDetectorResults(){
            if (latestResults != null){
                this.latestDetectorResults = latestResults.getDetectorResults();
                return latestResults.getDetectorResults();
            }
            else return null;
        }

        // TODO: refactor
        public float[] getCenterOffset(LLResultTypes.DetectorResult detection){
            float [] center = computeGroundPosition(320-detection.getTargetXPixels(), detection.getTargetYPixels());
            float [] result = new float[2];
            result[0] = center[1];
            result[1] = -center[0]+4.53f;
            return result;
        }

        public LLResultTypes.DetectorResult getClosestResult(){
            // TODO: add proper election algorithm
            latestDetectorResults = getDetectorResults();
            if (latestDetectorResults != null && !latestDetectorResults.isEmpty()){
                this.closestResult = latestDetectorResults.get(0);
                return this.closestResult;
            }
            else return null;
        }

        public float[] getClosestOffset(){
            return getCenterOffset(getClosestResult());
        }

        public String getClassName(){
            return getClosestResult().getClassName();
        }

        /** Compute homography from image **/
        void initializeHomography()
        {
            MatOfPoint2f imgMat = new MatOfPoint2f();
            imgMat.fromList(sPixelPoints);
            MatOfPoint2f groundMat = new MatOfPoint2f();
            groundMat.fromList(sGroundCoordinates);
            mHomography =  Calib3d.findHomography(imgMat, groundMat);
        }

        /**
         * Compute ground position from an input pixel
         * @param x : pixel x position in upper left corner reference
         * @param y : pixel y position in upper left corner reference
         * @return Ground position associated to the pixel
         */
        public float[] computeGroundPosition(double x, double y) {
            Point pixel = new Point(x,y);
            Mat src = new Mat(1, 1, CvType.CV_32FC2);
            src.put(0, 0, new float[]{(float) pixel.x, (float) pixel.y});
            Mat dst = new Mat();
            Core.perspectiveTransform(src, dst, mHomography);
            float[] result = new float[2];
            dst.get(0, 0, result);
            return result;
        }
    }

    public static class CVCamera {
        public boolean shouldStream = false;
        OpenCvWebcam webcam;
        CameraName camera;
        VisionPortal visionPortal;
        SampleDetectionPipeline sample;

        ExposureControl exposureControl = null;
        GainControl gainControl = null;
        WhiteBalanceControl whiteBalanceControl = null;

        private final long CAMERA_DEFAULT_EXPOSURE_LENGTH_MILLIS = 15;
        private final int CAMERA_DEFAULT_GAIN = 0;
        private final int CAMERA_DEFAULT_WHITE_BALANCE_TEMPERATURE = 4600;

        public CVCamera() {
            sample = new SampleDetectionPipeline();
        }

        public void setWebcamName (CameraName camera) {
            this.camera = camera;
        }

        public void setVisionPortal(VisionPortal visionPortal) {
            this.visionPortal = visionPortal;
        }

        public void startCamera(){
            visionPortal = new VisionPortal.Builder()
                    .setCamera(camera)
                    .setCameraResolution(new Size(CAMERA_STREAM_WIDTH, CAMERA_STREAM_HEIGHT)) // 1024 768
                    .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                    .addProcessors(sample)
                    .enableLiveView(true)
                    .build();

            visionPortal.setProcessorEnabled(sample, false);
        }

        public void resumeStreaming(){
            visionPortal.resumeStreaming();
        }

        public void setManualCameraControls() {
            exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            gainControl = visionPortal.getCameraControl(GainControl.class);
            whiteBalanceControl = visionPortal.getCameraControl(WhiteBalanceControl.class);


            if (exposureControl.getMode()!= ExposureControl.Mode.Manual){
                setExposureMode(ExposureControl.Mode.Manual);
                try{
                    sleep(50);
                } catch (Exception e){

                }
            }

            setExposureControl(CAMERA_EXPOSURE_MILLIS, TimeUnit.MILLISECONDS);

            if (whiteBalanceControl.getMode()!= WhiteBalanceControl.Mode.MANUAL){
                setWhiteBalanceMode(WhiteBalanceControl.Mode.MANUAL);
                try{
                    sleep(50);
                } catch (Exception e){

                }
            }
            setWhiteBalanceControl(CAMERA_WHITE_BALANCE_TEMPERATURE);


            setGainControl(CAMERA_GAIN);

        }

        public void setAutoCameraControls(){

            exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            gainControl = visionPortal.getCameraControl(GainControl.class);
            whiteBalanceControl = visionPortal.getCameraControl(WhiteBalanceControl.class);


            if (exposureControl.getMode()!= ExposureControl.Mode.AperturePriority){
                setExposureMode(ExposureControl.Mode.AperturePriority);
                try{
                    sleep(50);
                } catch (Exception e){

                }
            }
            setExposureControl(CAMERA_DEFAULT_EXPOSURE_LENGTH_MILLIS, TimeUnit.MILLISECONDS);

            if (whiteBalanceControl.getMode()!= WhiteBalanceControl.Mode.AUTO){
                setWhiteBalanceMode(WhiteBalanceControl.Mode.AUTO);
                try{
                    sleep(50);
                } catch (Exception e){

                }
            }
            setWhiteBalanceControl(CAMERA_DEFAULT_WHITE_BALANCE_TEMPERATURE);

            setGainControl(CAMERA_DEFAULT_GAIN);

        }

        public void setExposureMode(ExposureControl.Mode mode){
            exposureControl.setMode(mode);
        }

        public void setExposureControl(long duration, TimeUnit unit){
            exposureControl.setExposure(duration, unit);
        }

        public void setWhiteBalanceMode(WhiteBalanceControl.Mode mode){
            whiteBalanceControl.setMode(mode);
        }

        public void setWhiteBalanceControl(int temperature){
            whiteBalanceControl.setWhiteBalanceTemperature(temperature);
        }

        public void setGainControl(int gain){
            gainControl.setGain(gain);
        }

        public void setProcessorEnabled(VisionProcessor processor, boolean enabled){
            this.visionPortal.setProcessorEnabled(processor, enabled);
        }

        public VisionPortal.CameraState getCameraState(){
            if (visionPortal != null) return visionPortal.getCameraState();
            return null;
        }

        public void closeCamera(){
            if (visionPortal != null){
                visionPortal.stopLiveView();
                visionPortal.stopStreaming();
                visionPortal.close();
            }
        }


        public double getParallaxYCm(int yPixelVal){
            return 4E-05*(yPixelVal*yPixelVal) + 0.011*yPixelVal + 0.5037;
        }

        public double getParallaxXCm(int xPixelVal, int yPixelVal){
            return xPixelVal/((-1.0/75.0)*yPixelVal+40.0);
        }

        public double getLatency(){
            return Math.min(1/ webcam.getFps(),.2/*,0.1+webcam.getTotalFrameTimeMs()*/);
        }

        public boolean cameraInRange(){
            return cameraYInRange() && cameraHeadingInRange(); //&& cameraXInRange();
        }

        public boolean cameraXInRange(){
            return Math.abs(getCameraXOffset()) <0.3;
        }

        public boolean cameraYInRange(){
            return Math.abs(getCameraYOffset()) < 0.5;
        }

        public boolean cameraHeadingInRange(){
            return Math.abs(getCameraHeadingOffsetDegrees()) < (20.0);
        }

        public double getCameraXOffset(){
            if (sample.closestCenter[0] == 0){
                return 0;
            }
            else {
                return -(getParallaxXCm(sample.closestCenter[0].intValue(), sample.closestCenter[1].intValue()) - getParallaxXCm(CAMERA_STREAM_WIDTH/2, sample.closestCenter[1].intValue()))/2.54;
            }
        }
        public double getCameraYOffset(){
            if (sample.closestCenter[1] == 0){
                return 0;
            }
            else {
                return (getParallaxYCm(sample.closestCenter[1].intValue()) - getParallaxYCm(CAMERA_STREAM_HEIGHT/2))/2.54;
            }
        }
        public double getCameraHeadingOffsetDegrees(){
            return ((sample.closestCenter[3]-(90+0))%180); // +0 is current angle claw
        }


        public double getLatestValidCameraXOffset(){
            if (sample.latestValidCenter[0] == 0){
                return 0;
            }
            else {
                return -(getParallaxXCm(sample.latestValidCenter[0].intValue(), sample.latestValidCenter[1].intValue()) - getParallaxXCm(CAMERA_STREAM_WIDTH/2, sample.latestValidCenter[1].intValue()))/2.54;
            }
        }
        public double getLatestValidCameraYOffset(){
            if (sample.latestValidCenter[1] == 0){
                return 0;
            }
            else {
                return (getParallaxYCm(sample.latestValidCenter[1].intValue()) - getParallaxYCm(CAMERA_STREAM_HEIGHT/2))/2.54;
            }
        }
        public double getLatestValidCameraHeadingOffsetDegrees(){
            return ((sample.latestValidCenter[3]-(90+0))%180); // +0 is current angle claw
        }


        public double getCameraOffsetMagnitude(){
            if(getCameraXOffset() == 0 && getCameraYOffset() == 0){
                return 0.0;
            }
            else {
                return Math.sqrt(getCameraXOffset()*getCameraXOffset() + getCameraYOffset()*getCameraYOffset());
            }

        }

        public double getCameraZOffset(){
            return sample.closestCenter[2]/2.54;
        }

        public int getCurrent(){
            return sample.getColor();
        }

        public Double[] getCenter(){
            return sample.getCenter();
        }

        public void swapInt(int swap){
            sample.setColor(swap);
        }
        public void swapNext(){
            int newy = sample.getColor()+1;
            if (newy > 2) newy = 0;
            sample.setColor(newy);
        }

        public void swapRed(){
            sample.setColor(0);
        }
        public void swapBlue(){
            sample.setColor(1);
        }
        public void swapYellow(){
            sample.setColor(2);
        }
        public void resetCenter(){
            sample.resetCenter();
        }
    }
}
