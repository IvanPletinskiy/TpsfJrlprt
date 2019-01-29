package com.handen.roadhelper;

import android.Manifest;
import android.content.pm.PackageManager;
import android.os.Build;
import android.os.Bundle;
import android.speech.tts.TextToSpeech;
import android.support.v4.app.ActivityCompat;
import android.support.v7.app.AppCompatActivity;
import android.util.Log;
import android.view.SurfaceView;
import android.view.WindowManager;
import android.widget.Toast;

import org.opencv.android.BaseLoaderCallback;
import org.opencv.android.CameraBridgeViewBase;
import org.opencv.android.LoaderCallbackInterface;
import org.opencv.android.OpenCVLoader;
import org.opencv.android.Utils;
import org.opencv.core.Mat;
import org.opencv.imgcodecs.Imgcodecs;

import java.io.IOException;
import java.util.Date;
import java.util.HashMap;
import java.util.Locale;

public class MainActivity extends AppCompatActivity implements CameraBridgeViewBase.CvCameraViewListener2, TextToSpeech.OnInitListener {

    private static final String TAG = "OCVSample::Activity";
    private CameraBridgeViewBase _cameraBridgeViewBase;
    static boolean isFilterAdded = false;
    private String signsString = "";
    private HashMap<Integer, Long> lastSignEntry = new HashMap<>();
    int signCode = -1;
    int frameResult = 0;
    private TextToSpeech textToSpeech;

    // Mat code = new Mat(1, 1, 0);

    private BaseLoaderCallback _baseLoaderCallback = new BaseLoaderCallback(this) {
        @Override
        public void onManagerConnected(int status) {
            switch(status) {
                case LoaderCallbackInterface.SUCCESS: {
                    Log.i(TAG, "OpenCV loaded successfully");
                    // Load ndk built module, as specified in moduleName in build.gradle
                    // after opencv initialization
                    System.loadLibrary("native-lib");
                    initializeImages();
                    _cameraBridgeViewBase.enableView();
                }
                break;
                default: {
                    super.onManagerConnected(status);
                }
            }
        }
    };

    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);

        getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);
        setContentView(R.layout.activity_main);
        textToSpeech = new TextToSpeech(MainActivity.this, MainActivity.this);
        // Permissions for Android 6+
        ActivityCompat.requestPermissions(MainActivity.this,
                new String[]{Manifest.permission.CAMERA},
                1);
        _cameraBridgeViewBase = (CameraBridgeViewBase) findViewById(R.id.main_surface);
        _cameraBridgeViewBase.setVisibility(SurfaceView.VISIBLE);
        _cameraBridgeViewBase.setCvCameraViewListener(this);
    }

    @Override
    public void onPause() {
        super.onPause();
        disableCamera();
    }

    @Override
    public void onResume() {
        super.onResume();
        if(!OpenCVLoader.initDebug()) {
            Log.d(TAG, "Internal OpenCV library not found. Using OpenCV Manager for initialization");
            OpenCVLoader.initAsync(OpenCVLoader.OPENCV_VERSION_3_0_0, this, _baseLoaderCallback);
        }
        else {
            Log.d(TAG, "OpenCV library found inside package. Using it!");
            _baseLoaderCallback.onManagerConnected(LoaderCallbackInterface.SUCCESS);
        }
    }

    @Override
    public void onRequestPermissionsResult(int requestCode, String permissions[], int[] grantResults) {
        switch(requestCode) {
            case 1: {
                // If request is cancelled, the result arrays are empty.
                if(grantResults.length > 0
                        && grantResults[0] == PackageManager.PERMISSION_GRANTED) {
                    // permission was granted, yay! Do the
                    // contacts-related task you need to do.
                }
                else {
                    // permission denied, boo! Disable the
                    // functionality that depends on this permission.
                    Toast.makeText(MainActivity.this, "Permission denied to read your External storage", Toast.LENGTH_SHORT).show();
                }
                return;
            }
            // other 'case' lines to check for other
            // permissions this app might request
        }
    }

    private void initializeImages() {
        if(isFilterAdded) {
            return;
        }
        Mat pedastrian = null;
        try {
            pedastrian = Utils.loadResource(getApplicationContext(),
                    R.drawable.sign5162,
                    Imgcodecs.CV_LOAD_IMAGE_COLOR
            );

            addFilter(pedastrian.getNativeObjAddr(), 5612, 4);
            lastSignEntry.put(5612, -1L);
            Mat sign27 = new Mat();

            sign27 = Utils.loadResource(getApplicationContext(),
                    R.drawable.sign27,
                    Imgcodecs.CV_LOAD_IMAGE_COLOR
            );

            addFilter(sign27.getNativeObjAddr(), 27, 4);
            lastSignEntry.put(27, -1L);
            Mat sign530 = null;

            sign530 = Utils.loadResource(getApplicationContext(),
                    R.drawable.sign530,
                    Imgcodecs.CV_LOAD_IMAGE_COLOR
            );

            addFilter(sign530.getNativeObjAddr(), 530, 4);
            lastSignEntry.put(530, -1L);
            Mat sign121 = null;

            sign121 = Utils.loadResource(getApplicationContext(),
                    R.drawable.sign121,
                    Imgcodecs.CV_LOAD_IMAGE_COLOR
            );

            addFilter(sign121.getNativeObjAddr(), 121, 3);
            lastSignEntry.put(121, -1L);
            Mat sign324_60 = null;

            sign324_60 = Utils.loadResource(getApplicationContext(),
                    R.drawable.sign324_60,
                    Imgcodecs.CV_LOAD_IMAGE_COLOR
            );

            addFilter(sign324_60.getNativeObjAddr(), 32460, 0);
            lastSignEntry.put(32460, -1L);
        }
        catch(IOException e) {
            e.printStackTrace();
        }
        isFilterAdded = true;
    }

    public void onDestroy() {
        super.onDestroy();
        disableCamera();
    }

    public void disableCamera() {
        if(_cameraBridgeViewBase != null) {
            _cameraBridgeViewBase.disableView();
        }
    }

    public void onCameraViewStarted(int width, int height) {

    }

    public void onCameraViewStopped() {

    }

    public Mat onCameraFrame(CameraBridgeViewBase.CvCameraViewFrame inputFrame) {
        // signsString = "";
        //     Mat matGray = inputFrame.gray();
        Mat matRgba = inputFrame.rgba();
        frameResult = nativeOnFrame(matRgba.getNativeObjAddr());
        if(frameResult > 0) {
            long currentMillis = new Date().getTime();
            if(currentMillis - lastSignEntry.get(frameResult) > 5000) {
                //VOICE
            }
            lastSignEntry.put(frameResult, currentMillis);
        }
        //  if(frameResult > 0)
        //   Toast.makeText(MainActivity.this, "УРААААА", Toast.LENGTH_LONG).show();
        //   Mat copyMat = new Mat();
        //   matRgba.copyTo(copyMat);
        //   matRgba = null;
        //  if(code.get(0, 0)[0] > 0 ) {
        //      Toast.makeText(MainActivity.this, "УРААААА", Toast.LENGTH_LONG).show();
        //  }

        //    if(signCode > 0)
        //       Toast.makeText(MainActivity.this, "УРААААА", Toast.LENGTH_LONG).show();
        //  if(signsString.length() > 0)
        //      Toast.makeText(MainActivity.this, "УРААААА", Toast.LENGTH_LONG).show();

        return matRgba;
    }

    public native int nativeOnFrame(long matAddrGray);

    public native void addFilter(long matAddr, int code, int corners);

    @Override
    public void onInit(int status) {
        if(status == TextToSpeech.SUCCESS) {

            Locale locale = new Locale("ru");

            int result = textToSpeech.setLanguage(locale);
            //int result = mTTS.setLanguage(Locale.getDefault());

            if(result == TextToSpeech.LANG_MISSING_DATA
                    || result == TextToSpeech.LANG_NOT_SUPPORTED) {
                Log.e("TTS", "Извините, этот язык не поддерживается");
            }
            else {
                //     mButton.setEnabled(true);
                String s = "Я готов, хозяин";
                speak(s);
            }

        }
        else {
            Log.e("TTS", "Ошибка!");
        }
    }

    private void speak(String s) {
        if(Build.VERSION.SDK_INT >= Build.VERSION_CODES.LOLLIPOP) {
            textToSpeech.speak(s, TextToSpeech.QUEUE_FLUSH, null, null);
        }
        else {
            textToSpeech.speak(s, TextToSpeech.QUEUE_FLUSH, null);
        }
    }
}
