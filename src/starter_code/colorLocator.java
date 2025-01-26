package org.firstinspires.ftc.teamcode;

import android.util.Size;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.opencv.core.RotatedRect;

@TeleOp(name = "colorLocator (Blocks to Java)")
public class colorLocator extends LinearOpMode {

  /**
   * This OpMode illustrates how to use a video source (camera) to locate specifically colored regions.
   *
   * Unlike a "color sensor" which determines the color of an object in the
   * field of view, this "color locator" will search the Region Of Interest
   * (ROI) in a camera image, and find any "blobs" of color that match the
   * requested color range. These blobs can be further filtered and sorted
   * to find the one most likely to be the item the user is looking for.
   *
   * To perform this function, a VisionPortal runs a ColorBlobLocatorProcessor process.
   * The ColorBlobLocatorProcessor process is created first,
   * and then the VisionPortal is built to use this process.
   * The ColorBlobLocatorProcessor analyses the ROI and locates pixels that match the ColorRange to form a "mask".
   * The matching pixels are then collected into contiguous "blobs" of
   * pixels. The outer boundaries of these blobs are called its "contour".
   * For each blob, the process then creates the smallest possible
   * rectangle "boxFit" that will fully encase the contour.
   * The user can then call getBlobs() to retrieve the list of Blobs,
   * where each Blob contains the contour and the boxFit data.
   * Note: The default sort order for Blobs is ContourArea, in descending order, so the biggest contours are
   *   listed first.
   *
   * To aid the user, a colored boxFit rectangle is drawn on the camera preview to
   * show the location of each Blob. The original Blob contour can also be added to the
   * preview. This is helpful when configuring the ColorBlobLocatorProcessor parameters.
   */
  @Override
  public void runOpMode() {
    ColorBlobLocatorProcessor.Builder myColorBlobLocatorProcessorBuilder;
    VisionPortal.Builder myVisionPortalBuilder;
    ColorBlobLocatorProcessor myColorBlobLocatorProcessor;
    VisionPortal myVisionPortal;
    List<ColorBlobLocatorProcessor.Blob> myBlobs;
    ColorBlobLocatorProcessor.Blob myBlob;
    RotatedRect myBoxFit;

    // Build a "Color Locator" vision processor based on the ColorBlobLocatorProcessor class.
    myColorBlobLocatorProcessorBuilder = new ColorBlobLocatorProcessor.Builder();
    // - Specify the color range you are looking for.
    myColorBlobLocatorProcessorBuilder.setTargetColorRange(ColorRange.BLUE);
    // - Focus the color locator by defining a RegionOfInterest (ROI) which you want to search.
    //     This can be the entire frame, or a sub-region defined using:
    //     1) standard image coordinates or 2) a normalized +/- 1.0 coordinate system.
    //     Use one form of the ImageRegion class to define the ROI.
    // 50% width/height square centered on screen
    myColorBlobLocatorProcessorBuilder.setRoi(ImageRegion.asUnityCenterCoordinates(-1.0, 0.0, 1.0, -1.0));
    // - Define which contours are included.
    //     You can get ALL the contours, or you can skip any contours that are completely inside another contour.
    //     note: EXTERNAL_ONLY helps to avoid bright reflection spots from breaking up areas of solid color.
    myColorBlobLocatorProcessorBuilder.setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY);
    // - Turn the display of contours ON or OFF.  Turning this on helps debugging but takes up valuable CPU time.
    myColorBlobLocatorProcessorBuilder.setDrawContours(true);
    // - Include any pre-processing of the image or mask before looking for Blobs.
    //     There are some extra processing you can include to improve the formation of blobs.  Using these features requires
    //     an understanding of how they may effect the final blobs.  The "pixels" argument sets the NxN kernel size.
    //     Blurring an image helps to provide a smooth color transition between objects, and smoother contours.
    //     The higher the number of pixels, the more blurred the image becomes.
    //     Note:  Even "pixels" values will be incremented to satisfy the "odd number" requirement.
    //     Blurring too much may hide smaller features.  A "pixels" size of 5 is good for a 320x240 image.
    myColorBlobLocatorProcessorBuilder.setBlurSize(5);
    //     Erosion removes floating pixels and thin lines so that only substantive objects remain.
    //     Erosion can grow holes inside regions, and also shrink objects.
    //     "pixels" in the range of 2-4 are suitable for low res images.
    //     Dilation makes objects more visible by filling in small holes, making lines appear thicker,
    //     and making filled shapes appear larger. Dilation is useful for joining broken parts of an
    //     object, such as when removing noise from an image.
    //     "pixels" in the range of 2-4 are suitable for low res images.
    myColorBlobLocatorProcessor = myColorBlobLocatorProcessorBuilder.build();
    // Build a vision portal to run the Color Locator process.
    myVisionPortalBuilder = new VisionPortal.Builder();
    //  - Add the ColorBlobLocatorProcessor created above.
    myVisionPortalBuilder.addProcessor(myColorBlobLocatorProcessor);
    //  - Set the desired video resolution.
    //      Since a high resolution will not improve this process, choose a lower resolution that is
    //      supported by your camera. This will improve overall performance and reduce latency.
    myVisionPortalBuilder.setCameraResolution(new Size(320, 240));
    //  - Choose your video source. This may be for a webcam or for a Phone Camera.
    myVisionPortalBuilder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
    myVisionPortal = myVisionPortalBuilder.build();
    // Speed up telemetry updates, Just use for debugging.
    telemetry.setMsTransmissionInterval(50);
    telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);
    // WARNING:  To be able to view the stream preview on the Driver Station, this code runs in INIT mode.
    while (opModeIsActive() || opModeInInit()) {
      telemetry.addData("preview on/off", "... Camera Stream");
      telemetry.addLine("");
      // Read the current list of blobs.
      myBlobs = myColorBlobLocatorProcessor.getBlobs();
      // The list of Blobs can be filtered to remove unwanted Blobs.
      //   Note:  All contours will be still displayed on the Stream Preview, but only those that satisfy the filter
      //             conditions will remain in the current list of "blobs".  Multiple filters may be used.
      //
      // Use any of the following filters.
      //
      // A Blob's area is the number of pixels contained within the Contour.  Filter out any that are too big or small.
      // Start with a large range and then refine the range based on the likely size of the desired object in the viewfinder.
      ColorBlobLocatorProcessor.Util.filterByArea(50, 20000, myBlobs);
      // A blob's density is an indication of how "full" the contour is.
      // If you put a rubber band around the contour you would get the "Convex Hull" of the contour.
      // The density is the ratio of Contour-area to Convex Hull-area.
      // A blob's Aspect ratio is the ratio of boxFit long side to short side.
      // A perfect Square has an aspect ratio of 1.  All others are > 1
      // The list of Blobs can be sorted using the same Blob attributes as listed above.
      // No more than one sort call should be made.  Sorting can use ascending or descending order.
      telemetry.addLine(" Area Density Aspect  Center");
      // Display the size (area) and center location for each Blob.
      for (ColorBlobLocatorProcessor.Blob myBlob_item : myBlobs) {
        myBlob = myBlob_item;
        // Get a "best-fit" bounding box (called "boxFit", of type RotatedRect) for this blob.
        myBoxFit = myBlob.getBoxFit();
        // Get the aspect ratio of this blob, i.e. the ratio of the
        // longer side of the "boxFit" bounding box to the shorter side.
        telemetry.addLine(JavaUtil.formatNumber(myBlob.getContourArea(), 5, 0) + "  " + JavaUtil.formatNumber(myBlob.getDensity(), 4, 2) + "   " + JavaUtil.formatNumber(myBlob.getAspectRatio(), 5, 2) + "  (" + JavaUtil.formatNumber(myBoxFit.center.x, 3, 0) + "," + JavaUtil.formatNumber(myBoxFit.center.y, 3, 0) + ")");
      }
      telemetry.update();
      sleep(50);
    }
  }
}
