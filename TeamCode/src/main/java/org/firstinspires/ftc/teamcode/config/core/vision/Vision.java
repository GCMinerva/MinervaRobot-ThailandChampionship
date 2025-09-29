package org.firstinspires.ftc.teamcode.config.core.vision;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.localization.Pose;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.config.core.util.Color;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

@Config
public class Vision extends OpenCvPipeline {
    private boolean isOn = false;
    private boolean alwaysOn = false;
    public void turnOn() {
        isOn = true;
        cnt = cnt2 = 0;
    }
    public void turnOff() {
        isOn = false;
    }
    MultipleTelemetry mtelemetry;
    private static double fx = 1.432084260087562598e+03 / 3;
    private static double fy = 1.430189983900187599e+03 / 3;
    private static double cx = 9.506506547840623398e+02 / 3;
    private static double cy = 5.352683775399959814e+02 / 3;

    private static double k1 = 3.536317189134511457e-02;
    private static double k2 = -1.755007930191257726e-01;
    private static double p1 = -9.696624226567563960e-05;
    private static double p2 = -6.668043622214608413e-04;
    private static double k3 = 1.416944005594701628e-01;

    Mat cameraMatrix = new Mat(3, 3, CvType.CV_64F);
    Mat distCoeffs = new Mat(1, 5, CvType.CV_64F);
    Mat H = new Mat(3, 3, CvType.CV_64F);

    Size imageSize = new Size(640, 360);

    Mat newCameraMatrix;

    Mat laplacianKernel = new Mat(3, 3, CvType.CV_32F);

    private Color color;
    public Pose startPose;
    public int need;
    public int cnt = 0;
    public int cnt2 = 0;

    public Vision(Telemetry telemetry, Color color) {
        this.mtelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        this.color = color;
        this.alwaysOn = false;
        this.isOn = false;
        this.startPose = null;
        this.need = 4;
        this.cnt = 0;
        this.cnt2 = 0;

        cameraMatrix.put(0, 0, fx, 0, cx);
        cameraMatrix.put(1, 0, 0, fy, cy);
        cameraMatrix.put(2, 0, 0, 0, 1);

        distCoeffs.put(0, 0, k1, k2, p1, p2, k3);

//        H.put(0, 0, 1.013984044714722366e+00, 1.178549537172913109e+00, -6.752577943995709653e+00);
//        H.put(1, 0, 7.226215846771814888e-02, 2.688241934510303466e+00, -5.084124927163707497e+01);
//        H.put(2, 0, -2.864660819719514482e-05, 4.428567450598048030e-03, 1.000000000000000000e+00);

        H.put(0, 0, 1.024259893590453663e+00, 1.072143124182987739e+00, 9.643569257193563615e+00);
        H.put(1, 0, 1.555072604522374224e-01, 2.559102118676020332e+00, -4.737499556558831415e+01);
        H.put(2, 0, 9.473936195649239561e-05, 4.170561343819388471e-03, 1.000000000000000000e+00);

        newCameraMatrix = Calib3d.getOptimalNewCameraMatrix(
                cameraMatrix, distCoeffs, imageSize, 1, imageSize, null);

        laplacianKernel.put(0, 0, 1, 1, 1);
        laplacianKernel.put(1, 0, 1, -8, 1);
        laplacianKernel.put(2, 0, 1, 1, 1);
    }
    private static double c = 20.0 / 25.0;

    private final Mat warped = new Mat();
    private final Mat imgFloat = new Mat();
    private final Mat laplacian = new Mat();
    private final Mat sharpened = new Mat();
    private final Mat hsv = new Mat();
    private final Mat yellowMask = new Mat();
    private final Mat redMask = new Mat();
    private final Mat redMask1 = new Mat();
    private final Mat redMask2 = new Mat();
    private final Mat blueMask = new Mat();
    private final Mat mask = new Mat();
    private final Mat roi = new Mat();
    private final Mat edges = new Mat();
    private final Mat dilated = new Mat();
    private final Mat resMask = new Mat();
    private final Mat output = new Mat();
    private final Mat hierarchy = new Mat();
    private final Mat invMask = new Mat();
    private final Mat labels = new Mat();
    private final Mat stats = new Mat();
    private final Mat centroids = new Mat();
    private final MatOfPoint2f contour2f = new MatOfPoint2f();

    private final List<MatOfPoint> contours = new ArrayList<>();
    private final List<MatOfPoint> allBoxes = new ArrayList<>();

    private final List<MatOfPoint> small = new ArrayList<>();
    private final List<Double> smallAreas = new ArrayList<>();

    private final Mat kernel3 = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3, 3));
    private final Mat kernelDilate = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3, 3));

    private final Mat kernelOpen = Imgproc.getStructuringElement(
            Imgproc.MORPH_RECT, new Size(3, 3));
    private final Scalar boxColor = new Scalar(0, 255, 0);

    public static double MIN_AREA = 900*c*c;
    public static double MAX_AREA = 4000*c*c;
    public static double MIN_SMALL_AREA = 400*c*c;
    public static double MAX_H = 105*c;
    public static double MAX_W = 45*c;
    public static double MIN_RECT_AREA = 1200*c*c;
    public static double MIN_SMALL_RECT_AREA = 300*c*c;

    private final Scalar CLIP_MIN = Scalar.all(0);
    private final Scalar CLIP_MAX = Scalar.all(255);
//    private final Scalar YELLOW_MIN = new Scalar(15, 80, 200);
//    private final Scalar YELLOW_MAX = new Scalar(30, 255, 255);
//    private final Scalar RED1_MIN = new Scalar(0, 100, 120);
//    private final Scalar RED1_MAX = new Scalar(8, 255, 255);
//    private final Scalar RED2_MIN = new Scalar(175, 100, 120);
//    private final Scalar RED2_MAX = new Scalar(179, 255, 255);
//    private final Scalar BLUE_MIN = new Scalar(100, 20, 100);
//    private final Scalar BLUE_MAX = new Scalar(140, 255, 255);

    // BOOTH
//    private final Scalar YELLOW_MIN = new Scalar(15, 80, 200);
//    private final Scalar YELLOW_MAX = new Scalar(30, 255, 255);
//    private final Scalar RED1_MIN = new Scalar(0, 100, 120);
//    private final Scalar RED1_MAX = new Scalar(8, 255, 255);
//    private final Scalar RED2_MIN = new Scalar(175, 100, 120);
//    private final Scalar RED2_MAX = new Scalar(179, 255, 255);
//    private final Scalar BLUE_MIN = new Scalar(100, 20, 100);
//    private final Scalar BLUE_MAX = new Scalar(140, 255, 255);

    // FIELD
    // mask1 = cv2.inRange(hsv, (8,40,220), (40,255,255))
    //mask1 = cv2.inRange(hsv, (150,140,205), (180,255,255))
    //mask2 = cv2.inRange(hsv, (0,140,205), (1,255,255))
    //mask1 = cv2.inRange(hsv, (100,80,100), (140,255,255))
    private final Scalar YELLOW_MIN = new Scalar(8, 40, 200);
    private final Scalar YELLOW_MAX = new Scalar(40, 255, 255);
    private final Scalar RED1_MIN = new Scalar(0, 140, 205);
    private final Scalar RED1_MAX = new Scalar(1, 255, 255);
    private final Scalar RED2_MIN = new Scalar(150, 140, 205);
    private final Scalar RED2_MAX = new Scalar(180, 255, 255);
    private final Scalar BLUE_MIN = new Scalar(100, 80, 100);
    private final Scalar BLUE_MAX = new Scalar(140, 255, 255);
    private final List<MatOfPoint> smallContours = new ArrayList<>();
    private class ConsRect {
        public RotatedRect rect;
        public double area;
        public int[] indices;

        public ConsRect(RotatedRect rect, double area, int[] indices) {
            this.rect = rect;
            this.area = area;
            this.indices = indices;
        }
    }
    private final List<ConsRect> rectangles = new ArrayList<>();

    private Mat fillHoles(Mat mask) {
// Invert the mask
        Core.bitwise_not(mask, invMask);

// Connected components with stats
        int numLabels = Imgproc.connectedComponentsWithStats(invMask, labels, stats, centroids);

// Loop over all labels (skip background at index 0)
        for (int label = 1; label < numLabels; label++) {
            int area = (int) stats.get(label, Imgproc.CC_STAT_AREA)[0];

            if (area < 10) {
                // Get bounding box of component
                int left = (int) stats.get(label, Imgproc.CC_STAT_LEFT)[0];
                int top = (int) stats.get(label, Imgproc.CC_STAT_TOP)[0];
                int width = (int) stats.get(label, Imgproc.CC_STAT_WIDTH)[0];
                int height = (int) stats.get(label, Imgproc.CC_STAT_HEIGHT)[0];

                // Fill region in original mask where label matches
                for (int y = top; y < top + height; y++) {
                    for (int x = left; x < left + width; x++) {
                        if ((int) labels.get(y, x)[0] == label) {
                            mask.put(y, x, 255);  // Fill the hole
                        }
                    }
                }
            }
        }

        return mask;
    }

    List<Sample> detectedRects = new ArrayList<>();

    List<Sample> dominatedRects = new ArrayList<>();

    @Override
    public Mat processFrame(Mat input) {
        if (!alwaysOn) {
            if (!isOn) {
                return input;
            }
        }
        ++cnt;

        Imgproc.warpPerspective(input, warped, H, new Size(640, 360));

        warped.convertTo(imgFloat, CvType.CV_32F);

        Imgproc.filter2D(imgFloat, laplacian, -1, laplacianKernel);

        Core.subtract(imgFloat, laplacian, sharpened);

        Core.min(sharpened, CLIP_MAX, sharpened);
        Core.max(sharpened, CLIP_MIN, sharpened);
        sharpened.convertTo(sharpened, CvType.CV_8U);

        Imgproc.cvtColor(sharpened, hsv, Imgproc.COLOR_RGB2HSV);  // sharpened is in RGB

        if (color == Color.YELLOW) {
            Core.inRange(hsv, YELLOW_MIN, YELLOW_MAX, yellowMask);
            fillHoles(yellowMask);
            Imgproc.morphologyEx(yellowMask, mask, Imgproc.MORPH_OPEN, kernel3, new Point(-1, -1), 3);
        } else if (color == Color.BLUE) {
            Core.inRange(hsv, BLUE_MIN, BLUE_MAX, blueMask);
            fillHoles(blueMask);
            Imgproc.morphologyEx(blueMask, mask, Imgproc.MORPH_OPEN, kernel3, new Point(-1, -1), 3);
        } else if (color == Color.RED) {
            Core.inRange(hsv, RED1_MIN, RED1_MAX, redMask1);
            Core.inRange(hsv, RED2_MIN, RED2_MAX, redMask2);
            Core.bitwise_or(redMask1, redMask2, redMask);
            fillHoles(redMask);
            Imgproc.morphologyEx(redMask, mask, Imgproc.MORPH_OPEN, kernel3, new Point(-1, -1), 3);
        } else {
            return input;
        }

//        Core.inRange(hsv, BLUE_MIN, BLUE_MAX, blueMask);

//        Core.inRange(hsv, RED1_MIN, RED1_MAX, redMask1);
//        Core.inRange(hsv, RED2_MIN, RED2_MAX, redMask2);
//        Core.bitwise_or(redMask1, redMask2, redMask);

//        Core.inRange(hsv, YELLOW_MIN, YELLOW_MAX, yellowMask);

//        fillHoles(blueMask);
//        fillHoles(redMask);
//        fillHoles(yellowMask);
//        Mat mask = yellowMask;
//        Imgproc.morphologyEx(yellowMask, mask, Imgproc.MORPH_CLOSE, kernel3, new Point(-1, -1), 1);
//        Imgproc.morphologyEx(redMask, mask, Imgproc.MORPH_OPEN, kernel3, new Point(-1, -1), 3);

        Core.bitwise_and(sharpened, sharpened, roi, mask);

        for (int i = 0; i < 7; i++) {
            Imgproc.GaussianBlur(roi, roi, new Size(3, 3), 0);
        }

        if (color == Color.YELLOW) {
            Imgproc.Canny(roi, edges, 60, 120);
        } else if (color == Color.RED) {
            Imgproc.Canny(roi, edges, 60, 120);
        } else if (color == Color.BLUE) {
            Imgproc.Canny(roi, edges, 60, 120);
        }

        Imgproc.dilate(edges, dilated, kernelDilate);

//        Mat empty = Mat.zeros(dilated.size(), CvType.CV_8UC1);
//        List<MatOfPoint> contours = new ArrayList<>();
//        Mat hierarchy = new Mat();
//        Imgproc.findContours(dilated.clone(), contours, hierarchy, Imgproc.RETR_CCOMP, Imgproc.CHAIN_APPROX_SIMPLE);
//        List<MatOfPoint> badContours = new ArrayList<>();
//        for (MatOfPoint contour : contours) {
//            if (contour.toArray().length < 20) {
//                badContours.add(contour);
//            }
//        }
//        Imgproc.drawContours(dilated, badContours, -1, new Scalar(255), -1);

//        Mat cleanedEdges = new Mat();
//        Core.bitwise_not(empty, empty);
//        Core.bitwise_and(dilated, empty, cleanedEdges);

        Core.bitwise_not(dilated, dilated);
        Core.bitwise_and(mask, dilated, resMask);

        Imgproc.morphologyEx(resMask, resMask, Imgproc.MORPH_OPEN, kernel3, new Point(-1, -1), 1);

        contours.clear();
        Imgproc.findContours(resMask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        allBoxes.clear();
        detectedRects.clear();
        dominatedRects.clear();

//        Mat areaOutput = new Mat();
//        Imgproc.cvtColor(resMask, areaOutput, Imgproc.COLOR_GRAY2BGR);
//
//        for (MatOfPoint contour : contours) {
//            double area = Imgproc.contourArea(contour);
//
//            MatOfPoint2f contour2f = new MatOfPoint2f(contour.toArray());
//
//            contour2f.fromArray(contour.toArray());
//
//            RotatedRect rect = Imgproc.minAreaRect(contour2f);
//            Size size = rect.size;
//            double w = size.width;
//            double h = size.height;
//            double ratio = Math.max(w, h) / Math.min(w, h);
//            double angle = rect.angle;
//            double areaRatio = area / (w * h);
//
//            if (w < h) {
//                angle += 90;
//            }
//            Moments M = Imgproc.moments(contour);
//            int cx, cy;
//            if (M.get_m00() != 0) {
//                cx = (int) (M.get_m10() / M.get_m00());
//                cy = (int) (M.get_m01() / M.get_m00());
//            } else {
//                Point[] pts = contour.toArray();
//                cx = (int) pts[0].x;
//                cy = (int) pts[0].y;
//            }
//
//            // Draw contour
//            Imgproc.drawContours(areaOutput, Arrays.asList(contour), -1, new Scalar(0, 255, 0), 2);
//
//            // Put text
//            Imgproc.putText(areaOutput, String.format("%.0f", w * h),
//                    new Point(cx, cy), Imgproc.FONT_HERSHEY_SIMPLEX,
//                    0.3, new Scalar(255, 0, 0), 1, Imgproc.LINE_AA);
//        }
        small.clear();
        smallAreas.clear();

        for (MatOfPoint contour : contours) {
            double area = Imgproc.contourArea(contour);
            mtelemetry.addLine(String.format("Contour area %.2f", area));

            if (area < MIN_AREA || area > MAX_AREA) {
                if (MIN_SMALL_AREA < area && area < MIN_AREA) {
                    small.add(contour);
                    smallAreas.add(area);
                }
                continue;
            }

            MatOfPoint2f contour2f = new MatOfPoint2f(contour.toArray());

            contour2f.fromArray(contour.toArray());

            RotatedRect rect = Imgproc.minAreaRect(contour2f);
            Size size = rect.size;
            double w = size.width;
            double h = size.height;
            double ratio = Math.max(w, h) / Math.min(w, h);
            double angle = rect.angle;
            double areaRatio = area / (w * h);

            if (w < h) {
                angle += 90;
            }
            mtelemetry.addLine(String.format("ratio %.2f", ratio));
            mtelemetry.addLine(String.format("areaRatio %.2f", areaRatio));

            if (1.6 < ratio && ratio < 5 && areaRatio >= 0.5 &&
                    Math.max(w, h) <= MAX_H && Math.min(w, h) <= MAX_W &&
                    w * h >= MIN_RECT_AREA && w * h <= MAX_AREA) {
                mtelemetry.addLine("yes");
                Point[] boxPoints = new Point[4];
                rect.points(boxPoints);
                allBoxes.add(new MatOfPoint(boxPoints));
                detectedRects.add(new Sample(color, rect.center.x, rect.center.y, angle));
            } else {
                if (1.6 >= ratio || ratio >= 5)mtelemetry.addLine("! ratio");
                if (areaRatio < 0.5)mtelemetry.addLine("! arearatio");
                if (Math.max(w, h) > MAX_H)mtelemetry.addLine("> MAX_H");
                if (Math.min(w, h) > MAX_W)mtelemetry.addLine("> MAX_W");
                if (w * h < MIN_RECT_AREA)mtelemetry.addLine("< MIN_RECT_AREA");
                if (w * h > MAX_AREA)mtelemetry.addLine("> MAX_AREA");
                mtelemetry.addLine("wtf");
                if (MIN_SMALL_RECT_AREA <= w * h && w * h <= MIN_RECT_AREA) {
                    small.add(contour);
                    smallAreas.add(area);
                }
            }
        }

        if (detectedRects.size() < need) {
            rectangles.clear();

            for (int i = 0; i < small.size(); i++) {
                for (int j = i + 1; j < small.size(); j++) {
                    for (int k = j; k < small.size(); k++) {

                        List<Point> mergedPoints = new ArrayList<>();
                        mergedPoints.addAll(small.get(i).toList());
                        mergedPoints.addAll(small.get(j).toList());
                        double area = smallAreas.get(i) + smallAreas.get(j);

                        int[] indices;
                        if (k != j) {
                            mergedPoints.addAll(small.get(k).toList());
                            area += smallAreas.get(k);
                            indices = new int[]{i, j, k};
                        } else {
                            indices = new int[]{i, j, j};
                        }

                        MatOfPoint2f mergedMat = new MatOfPoint2f();
                        mergedMat.fromList(mergedPoints);

                        RotatedRect rect = Imgproc.minAreaRect(mergedMat);
                        double w = rect.size.width;
                        double h = rect.size.height;

                        double ratio = Math.max(w, h) / Math.max(1.0, Math.min(w, h)); // avoid div by 0
                        double areaRatio = area / (w * h);

                        if (2 < ratio && ratio < 4 && areaRatio > 0.4 &&
                            Math.max(w, h) <= MAX_H && Math.min(w, h) <= MAX_W &&
                            w * h >= MIN_RECT_AREA && w * h <= MAX_AREA) {

                            rectangles.add(new ConsRect(
                                rect,
                                w * h,
                                indices
                            ));
                        }
                    }
                }
            }

            rectangles.sort((ConsRect a, ConsRect b) -> Double.compare(b.area, a.area));

            boolean[] used = new boolean[small.size()];

            for (ConsRect crect : rectangles) {
                RotatedRect rect = crect.rect;
                double area = crect.area;
                int[] indices = crect.indices;

                double w = rect.size.width;
                double h = rect.size.height;
                double angle = rect.angle;

                if (w < h) {
                    angle += 90;
                }

                int i = indices[0];
                int j = indices[1];
                int k = indices[2];

                if (used[i] || used[j] || used[k]) {
                    continue;
                }

                used[i] = true;
                used[j] = true;
                used[k] = true;

                Point[] boxPoints = new Point[4];
                rect.points(boxPoints);
                allBoxes.add(new MatOfPoint(boxPoints));
                detectedRects.add(new Sample(color, rect.center.x, rect.center.y, angle));
            }
        }

//        for (MatOfPoint contour : contours) {
//            double area = Imgproc.contourArea(contour);
//
//            if (area < MIN_AREA || area > MAX_AREA) continue;
//
//            MatOfPoint2f contour2f = new MatOfPoint2f(contour.toArray());
//
//            contour2f.fromArray(contour.toArray());
//
//            RotatedRect rect = Imgproc.minAreaRect(contour2f);
//            Size size = rect.size;
//            double w = size.width;
//            double h = size.height;
//            double ratio = Math.max(w, h) / Math.min(w, h);
//            double angle = rect.angle;
//
//            if (w*h > MAX_RECT_AREA) continue;
//
//            if (w < h) {
//                angle += 90;
//            }
////            telemetry.addData("angle",angle);
////            telemetry.addData("area",w*h);
//
//            if (ratio > 1.5 && ratio < 7.0) {
//                Point[] boxPoints = new Point[4];
//                rect.points(boxPoints);
//                allBoxes.add(new MatOfPoint(boxPoints));
//                detectedRects.add(new Sample(rect.center, angle));
//            }
//        }


        sharpened.copyTo(output);

        if (!allBoxes.isEmpty()) {
            Imgproc.drawContours(output, allBoxes, -1, boxColor, 2);
        }
        mtelemetry.addData("detectedrects: ",detectedRects.size());
        mtelemetry.update();

        ++cnt2;
        return output;
//        return resMask;
    }

    public List<Sample> getSamples() {
        return new ArrayList<>(detectedRects); // return a copy
    }
    public void setColor(Color color) {
        this.color = color;
    }
    public Sample getBestSample(double xMin, double xMax, double deltaX, boolean remove) {
        Sample bestSample = null;
        double bestDistance = Double.MAX_VALUE;
        if (detectedRects.isEmpty()) {
            detectedRects.addAll(dominatedRects);
            dominatedRects.clear();
        }
        List<Sample> snapshot = new ArrayList<>(detectedRects);
        if (!snapshot.isEmpty()) {
            for (Sample candidate : snapshot) {
                boolean hasBlockerAbove = false;
                for (Sample other : snapshot) {
                    if (other == candidate) continue;
                    double dx = Math.abs(candidate.getX() - other.getX());
                    double dy = other.getY() - candidate.getY();
                    if (dx <= 1.7 && dy >= 1.0 && dy <= 3.0) {
                        hasBlockerAbove = true;
                        break;
                    }
                }
                if (!hasBlockerAbove && candidate.getY() < 20 - deltaX && candidate.getY() >= 4
                    && candidate.getX() >= xMin && candidate.getX() <= xMax) {
                    double curDistance = Math.hypot(Math.abs(candidate.getX()), Math.abs(candidate.getY() - 6));
                    if (curDistance < bestDistance) {
                        bestDistance = curDistance;
                        bestSample = candidate;
                    }
                }
            }
            if (remove && bestSample != null) {
                detectedRects.remove(bestSample);
                List<Sample> toMove = new ArrayList<>();
                for (Sample candidate : new ArrayList<>(snapshot)) {
                    double dx = Math.abs(candidate.getX() - bestSample.getX());
                    double dy = bestSample.getY() - candidate.getY();
                    if (dx <= 1.7 && dy >= 1.0 && dy <= 3.0) {
                        toMove.add(candidate);
                    }
                }
                dominatedRects.addAll(toMove);
                detectedRects.removeAll(toMove);
            }
        }

        return bestSample;
    }

    public void setNeed(int need) {
        this.need = need;
    }

    public void setStartPose(Pose startPose) {
        this.startPose = startPose;
    }
    public Pose getStartPose() {
        return startPose;
    }
    public static Pose toSamplePose(Pose currentPose, Sample sample, double deltaX) {
        if (sample == null)return currentPose;
        double forwardShift = sample.getX();

        double dx = -deltaX;
        double dy = -sample.getX();

        double cosH = Math.cos(currentPose.getHeading());
        double sinH = Math.sin(currentPose.getHeading());

        double fieldDx = dx * cosH - dy * sinH;
        double fieldDy = dx * sinH + dy * cosH;

        return new Pose(
            currentPose.getX() + fieldDx,
            currentPose.getY() + fieldDy,
            currentPose.getHeading()
        );
    }

    public void forceOn() {
        alwaysOn = true;
    }
    public void clear() {
        detectedRects.clear();
        dominatedRects.clear();
        cnt = 0;
        cnt2 = 0;
    }

    public void periodic() {
        mtelemetry.addData("Vision Samples: ",detectedRects.size());
    }
}