package com.handen.roadhelper;

import android.Manifest;
import android.content.pm.ActivityInfo;
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

public class MainActivity extends AppCompatActivity implements CameraBridgeViewBase.CvCameraViewListener2 {

    private static final String TAG = "OCVSample::Activity";
    private CameraBridgeViewBase cameraBridgeViewBase;
    static boolean isFilterAdded = false;
    private static HashMap<Integer, Long> lastSignEntry;
    int frameResult = 0;
    private static TextToSpeech textToSpeech;
    private static boolean isTextToSpeachLoaded = false;

    private BaseLoaderCallback baseLoaderCallback = new BaseLoaderCallback(this) {
        @Override
        public void onManagerConnected(int status) {
            switch(status) {
                case LoaderCallbackInterface.SUCCESS: {
                    Log.i(TAG, "OpenCV loaded successfully");
                    System.loadLibrary("native-lib");
                    initializeImages();
                    cameraBridgeViewBase.enableView();
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
        getWindow().addFlags(WindowManager.LayoutParams.FLAG_FULLSCREEN);
        setRequestedOrientation(ActivityInfo.SCREEN_ORIENTATION_LANDSCAPE);

        setContentView(R.layout.activity_main);
        ActivityCompat.requestPermissions(MainActivity.this,
                new String[]{Manifest.permission.CAMERA},
                1);
        cameraBridgeViewBase = findViewById(R.id.main_surface);
        cameraBridgeViewBase.setVisibility(SurfaceView.VISIBLE);
        cameraBridgeViewBase.setCvCameraViewListener(this);
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
            OpenCVLoader.initAsync(OpenCVLoader.OPENCV_VERSION_3_0_0, this, baseLoaderCallback);
        }
        else {
            baseLoaderCallback.onManagerConnected(LoaderCallbackInterface.SUCCESS);
        }
    }

    @Override
    public void onRequestPermissionsResult(int requestCode, String permissions[], int[] grantResults) {
        switch(requestCode) {
            case 1: {
                if(grantResults.length > 0
                        && grantResults[0] == PackageManager.PERMISSION_GRANTED) {
                }
                else {
                    showToast("Permission denied to read your External storage");
                }
                return;
            }
        }
    }

    private void initializeImages() {
        if(isFilterAdded) {
            return;
        }
        lastSignEntry = new HashMap<>();
        if(!isTextToSpeachLoaded) {
            textToSpeech = new TextToSpeech(getApplicationContext(), new TextToSpeech.OnInitListener() {
                @Override
                public void onInit(int status) {
                    if(status == TextToSpeech.SUCCESS) {
                        isTextToSpeachLoaded = true;

                        Locale locale = new Locale("ru");
                        int result = textToSpeech.setLanguage(locale);

                        if(result == TextToSpeech.LANG_MISSING_DATA
                                || result == TextToSpeech.LANG_NOT_SUPPORTED) {
                            showToast("Русский язык не поддерживается");
                        }
                        else {
                            showToast("Синтезатор речи загружен");
                            String s = "Я готов, хозяин";
                            //               speak(s);
                        }
                    }
                    else {
                        showToast("Ошибка");
                    }
                }
            });
        }

        try {
            Mat pedastrian = null;
            pedastrian = Utils.loadResource(getApplicationContext(),
                    R.drawable.sign5162,
                    Imgcodecs.CV_LOAD_IMAGE_COLOR
            );

            addFilter(pedastrian.getNativeObjAddr(), 5612, 4);
            lastSignEntry.put(5612, 0L);
            Mat sign27 = new Mat();

            sign27 = Utils.loadResource(getApplicationContext(),
                    R.drawable.sign27,
                    Imgcodecs.CV_LOAD_IMAGE_COLOR
            );

            addFilter(sign27.getNativeObjAddr(), 27, 4);
            lastSignEntry.put(27, 0L);
            Mat sign530 = null;

            sign530 = Utils.loadResource(getApplicationContext(),
                    R.drawable.sign530,
                    Imgcodecs.CV_LOAD_IMAGE_COLOR
            );

            addFilter(sign530.getNativeObjAddr(), 530, 4);
            lastSignEntry.put(530, 0L);
            Mat sign121 = null;

            sign121 = Utils.loadResource(getApplicationContext(),
                    R.drawable.sign121,
                    Imgcodecs.CV_LOAD_IMAGE_COLOR
            );

            addFilter(sign121.getNativeObjAddr(), 121, 3);
            lastSignEntry.put(121, 0L);
            Mat sign324_60 = null;

            sign324_60 = Utils.loadResource(getApplicationContext(),
                    R.drawable.sign324_60,
                    Imgcodecs.CV_LOAD_IMAGE_COLOR
            );

            addFilter(sign324_60.getNativeObjAddr(), 32460, 0);
            lastSignEntry.put(32460, 0L);

            Mat sign5191 = null;
            sign5191 = Utils.loadResource(getApplicationContext(),
                    R.drawable.sign5191,
                    Imgcodecs.CV_LOAD_IMAGE_COLOR
            );
            addFilter(sign5191.getNativeObjAddr(), 5191, 4);
            lastSignEntry.put(5191, 0L);


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
        if(cameraBridgeViewBase != null) {
            cameraBridgeViewBase.disableView();
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
                switch(frameResult) {
                    case 5612: {
                        speak("Пешеходный переход");
                        break;
                    }
                    case 121: {
                        speak("Осторожно, дети");
                        break;
                    }
                    case 530: {
                        speak("Минимальная скорость 50");
                        break;
                    }
                    case 27: {
                        speak("Преемущество перед встречным движением");
                        break;
                    }
                    case 32460: {
                        speak("Максимальная скорость 60");
                    }
                    case 5191: {
                        speak("Тупик");
                    }
                }
            }
            lastSignEntry.put(frameResult, currentMillis);
        }

        return matRgba;
    }

    public native int nativeOnFrame(long matAddrGray);

    public native void addFilter(long matAddr, int code, int corners);

    private void showToast(String s) {
        Toast.makeText(MainActivity.this, s, Toast.LENGTH_SHORT).show();
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