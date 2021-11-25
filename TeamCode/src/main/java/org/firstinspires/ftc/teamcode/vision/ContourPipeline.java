package org.firstinspires.ftc.teamcode.vision;



import com.acmerobotics.dashboard.config.Config;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;

@Config
public class ContourPipeline extends OpenCvPipeline {
	private Mat alternateColorSpace = new Mat();

	public static Scalar lower = new Scalar(123.4,145.0,0);
	public static Scalar higher = new Scalar(255,255,82.3);

	public static double low1 = 90;
	public static double low2 = 140;
	public static double low3 = 100;
	public static double high1 = 109;
	public static double high2 = 190;
	public static double high3 = 110;

	private Mat maskedInputMat = new Mat();
	private Mat binaryMat      = new Mat();

	ArrayList<MatOfPoint> contours = new ArrayList<MatOfPoint>();

	position TSEPosition = position.MIDDLE;

	@Override
	public Mat processFrame(Mat input) {

		lower = new Scalar(low1,low2,low3);
		higher = new Scalar(high1, high2, high3);

		int columns = input.cols();

		int left = columns / 3;
		int right = columns / 2;


		Imgproc.cvtColor(input, alternateColorSpace, Imgproc.COLOR_RGB2YCrCb);
		Core.inRange(alternateColorSpace,lower,higher,binaryMat);
		maskedInputMat.release();
		/*
		 * Now, with our binary Mat, we perform a "bitwise and"
		 * to our input image, meaning that we will perform a mask
		 * which will include the pixels from our input Mat which
		 * are "255" in our binary Mat (meaning that they're inside
		 * the range) and will discard any other pixel outside the
		 * range (RGB 0, 0, 0. All discarded pixels will be black)
		 */
		Core.bitwise_and(input, input, maskedInputMat, binaryMat);
		contours.clear();
		Imgproc.findContours(binaryMat,contours,new Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
		double largestContourSize = 0;
		int largestContourIndex = 0;
		for (int i = 0; i < contours.size(); ++i) {

			double area = Imgproc.contourArea(contours.get(i));
			if (area > largestContourSize) {
				largestContourSize = area;
				largestContourIndex = i;
			}
		}
		try {
			MatOfPoint largestContour = contours.get(largestContourIndex);
			Rect rectangle = Imgproc.boundingRect(largestContour);
			Imgproc.drawContours(input,contours,largestContourIndex,new Scalar(255,0,0));
			Imgproc.rectangle(input,rectangle,new Scalar(0,255,255));
			int boundingBoxPosition = rectangle.x;
			if (boundingBoxPosition < left) {
				TSEPosition = position.LEFT;
			} else if (boundingBoxPosition > left && boundingBoxPosition < right) {
				TSEPosition = position.MIDDLE;
			} else {
				TSEPosition = position.RIGHT;
			}
			Imgproc.putText(input,"" + TSEPosition,new Point(rectangle.x - 30,rectangle.y - 20),2,1,new Scalar(255,0,0));
		} catch (IndexOutOfBoundsException e) {
			System.out.println("out of bounds exception was caught, try extending the detection tolerances");
		}

		return input;
	}

	public enum position {
		LEFT,
		MIDDLE,
		RIGHT
	}

	public position getTSEPosition() {
		return TSEPosition;
	}

}
