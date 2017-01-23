package com.martin.ads.orb_slam2_android;

import android.app.Activity;
import android.content.Context;
import android.content.Intent;
import android.os.AsyncTask;
import android.os.Bundle;
import android.os.Environment;

import java.io.File;
import java.io.IOException;

/**
 * Created by Ads on 2016/11/28.
 */

public class ModelActivity extends Activity {
    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.pending_layout);
        //"android.resource://" + getPackageName() + "/" +R.raw.clm_model
        new ExtractModelTask(this).execute();
    }

    class ExtractModelTask extends AsyncTask<Void,Void,Boolean>{
        Context context;

        public ExtractModelTask(Context context) {
            this.context = context;
        }

        @Override
        protected Boolean doInBackground(Void... params) {
            ZipHelper.saveFile(context,Environment.getExternalStorageDirectory().getPath()+"/SLAM","CameraSettings.yaml","CameraSettings.yaml");
            ZipHelper.saveFile(context,Environment.getExternalStorageDirectory().getPath()+"/SLAM","config.txt","config.txt");
            ZipHelper.saveFile(context,Environment.getExternalStorageDirectory().getPath()+"/SLAM","ORBvoc.txt.arm.bin","ORBvoc.txt.arm.bin");
//                ZipHelper.upZipFile(new File(Environment.getExternalStorageDirectory().getPath()+"/CLM/clm_model.zip"),
//                        Environment.getExternalStorageDirectory().getPath()+"/CLM");
            return true;
        }

        @Override
        protected void onPostExecute(Boolean aBoolean) {
            super.onPostExecute(aBoolean);
            Intent intent=new Intent(ModelActivity.this,MainActivity.class);
            startActivity(intent);
            finish();
        }
    }
}
