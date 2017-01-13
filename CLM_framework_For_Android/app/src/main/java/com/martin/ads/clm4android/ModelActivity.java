package com.martin.ads.clm4android;

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
            try {
                ZipHelper.saveFile(context,Environment.getExternalStorageDirectory().getPath()+"/CLM","clm_model.zip","clm_model.zip");
                ZipHelper.upZipFile(new File(Environment.getExternalStorageDirectory().getPath()+"/CLM/clm_model.zip"),
                        Environment.getExternalStorageDirectory().getPath()+"/CLM");
            } catch (IOException e) {
                e.printStackTrace();
            }
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
