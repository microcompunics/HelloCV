package com.martin.ads.orb_slam2_android;

import android.app.Activity;
import android.hardware.Camera;
import android.os.Bundle;
import android.os.Environment;
import android.util.Log;
import android.view.View;
import android.view.WindowManager;
import android.widget.TextView;

import org.opencv.android.CameraBridgeViewBase;
import org.opencv.android.JavaCameraView;
import org.opencv.android.OpenCVLoader;
import org.opencv.core.CvType;
import org.opencv.core.Mat;

public class MainActivity extends Activity implements CameraBridgeViewBase.CvCameraViewListener2 {

    private static final String    TAG = "MainActivity";

    private Mat                    mRgba;
    private Mat                    mIntermediateMat;
    private Mat                    mGray;

    private CameraBridgeViewBase   mOpenCvCameraView;
    private boolean initFinished;

    private FPSUpdater fpsUpdater;
    private TextView fpsText;
    private int cursor;
    // Used to load the 'native-lib' library on application startup.
    static {
        System.loadLibrary("opencv_java3");
        System.loadLibrary("opencv_java");
        System.loadLibrary("SLAM");
    }

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);

        mOpenCvCameraView = (CameraBridgeViewBase) findViewById(R.id.tutorial2_activity_surface_view);
        mOpenCvCameraView.setVisibility(CameraBridgeViewBase.VISIBLE);
        mOpenCvCameraView.setCvCameraViewListener(this);
        mOpenCvCameraView.setClickable(true);
        mOpenCvCameraView.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                Camera camera=((JavaCameraView)mOpenCvCameraView).getCamera();
                if (camera!=null) camera.autoFocus(null);
            }
        });
        initFinished=false;

        fpsText= (TextView) findViewById(R.id.fps_text);
        fpsUpdater=new FPSUpdater() {
            @Override
            public void update(int fps) {
                fpsText.setText("FPS: "+fps);
            }
        };
        cursor=0;
    }

    @Override
    public void onPause()
    {
        super.onPause();
        if (mOpenCvCameraView != null)
            mOpenCvCameraView.disableView();
    }

    @Override
    public void onResume()
    {
        super.onResume();

        Log.d(TAG, "OpenCV library found inside package. Using it!");
        mOpenCvCameraView.enableView();
        if (!initFinished) {
            initSLAM(Environment.getExternalStorageDirectory().getPath()+"/SLAM/");
            initFinished=true;
        }
    }

    public void onDestroy() {
        super.onDestroy();
        if (mOpenCvCameraView != null)
            mOpenCvCameraView.disableView();
    }

    public void onCameraViewStarted(int width, int height) {
        mRgba = new Mat(height, width, CvType.CV_8UC4);
        mIntermediateMat = new Mat(height, width, CvType.CV_8UC4);
        mGray = new Mat(height, width, CvType.CV_8UC1);
    }

    public void onCameraViewStopped() {
        mRgba.release();
        mGray.release();
        mIntermediateMat.release();
    }

    public Mat onCameraFrame(CameraBridgeViewBase.CvCameraViewFrame inputFrame) {
        mRgba = inputFrame.rgba();
        mGray = inputFrame.gray();
        cursor=(cursor+1)%11;
        if(cursor==1){
            long currentTime=System.currentTimeMillis();
            nativeProcessFrame(mGray.getNativeObjAddr(), mRgba.getNativeObjAddr());
            final int fps=(int) (1000.0f/(System.currentTimeMillis()-currentTime));
            runOnUiThread(new Runnable() {
                @Override
                public void run() {
                    fpsUpdater.update(fps);
                }
            });
        }else nativeProcessFrame(mGray.getNativeObjAddr(), mRgba.getNativeObjAddr());

        return mRgba;
    }

    public native void nativeProcessFrame(long matAddrGr, long matAddrRgba);
    public native void initSLAM(String path);


}
