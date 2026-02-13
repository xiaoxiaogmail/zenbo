package com.example.hellozenbo;

import android.app.Activity;
import android.os.Bundle;
import android.util.Log;
import android.view.View;
import android.widget.Button;
import android.widget.TextView;

import com.asus.robotframework.API.RobotAPI;
import com.asus.robotframework.API.RobotCallback;
import com.asus.robotframework.API.RobotCmdState;
import com.asus.robotframework.API.RobotErrorCode;
import com.asus.robotframework.API.RobotFace;

import org.json.JSONObject;

public class MainActivity extends Activity {
    private static final String TAG = "HelloZenbo";
    private RobotAPI robotAPI;
    private TextView statusText;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        statusText = (TextView) findViewById(R.id.status_text);
        
        // Initialize Robot API
        robotAPI = new RobotAPI(getApplicationContext(), robotCallback);

        Button speakBtn = (Button) findViewById(R.id.btn_speak);
        speakBtn.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                if(robotAPI != null) {
                    robotAPI.robot.speak("Hello Robin Max! I am alive!");
                    robotAPI.robot.setExpression(RobotFace.HAPPY);
                }
            }
        });

        Button moveBtn = (Button) findViewById(R.id.btn_move);
        moveBtn.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                 if(robotAPI != null) {
                     // Move forward slightly
                     robotAPI.motion.moveBody(0.1f, 0, 0); 
                 }
            }
        });
    }

    @Override
    protected void onResume() {
        super.onResume();
        if (robotAPI != null) {
            // Re-connect or ensure state
        }
    }

    @Override
    protected void onDestroy() {
        super.onDestroy();
        if (robotAPI != null) {
            robotAPI.release();
        }
    }

    public static RobotCallback robotCallback = new RobotCallback() {
        @Override
        public void onResult(int cmd, int serial, RobotErrorCode err_code, Bundle result) {
            Log.d(TAG, "onResult: " + cmd + ", serial: " + serial + ", err: " + err_code);
        }

        @Override
        public void onStateChange(int cmd, int serial, RobotErrorCode err_code, RobotCmdState state) {
            Log.d(TAG, "onStateChange: " + cmd + ", serial: " + serial + ", state: " + state);
        }

        @Override
        public void initComplete() {
            super.initComplete();
            Log.d(TAG, "Robot API Init Complete");
        }
    };
}
