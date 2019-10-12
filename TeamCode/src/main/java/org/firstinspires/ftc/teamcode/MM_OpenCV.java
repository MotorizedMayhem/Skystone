package org.firstinspires.ftc.teamcode;



import android.graphics.Bitmap;

import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
//please god dont use this
public class MM_OpenCV {
    VuforiaLocalizer vuforia;

    MM_OpenCV(VuforiaLocalizer vuforia){
        this.vuforia = vuforia;
    }
    void getFrames() throws InterruptedException {
        Image rgb = null;
        VuforiaLocalizer.CloseableFrame frame = vuforia.getFrameQueue().take();
        long numImages = frame.getNumImages();
        for (int i = 0; i < numImages; i++) {
            if (frame.getImage(i).getFormat() == PIXEL_FORMAT.RGB565) {
                rgb = frame.getImage(i);
                break;
            }
        }
        if (rgb == null){return;}

        Bitmap bm = Bitmap.createBitmap(rgb.getWidth(), rgb.getHeight(), Bitmap.Config.RGB_565);
        bm.copyPixelsFromBuffer(rgb.getPixels());

    }

}
