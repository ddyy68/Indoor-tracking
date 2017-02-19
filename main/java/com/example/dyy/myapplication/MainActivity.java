package com.example.dyy.myapplication;

import android.content.Context;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.os.Environment;
import android.support.v7.app.AppCompatActivity;
import android.os.Bundle;
import android.util.Log;
import android.view.View;
import android.widget.TextView;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Calendar;
import java.util.Date;
import java.util.List;
import java.util.Locale;
import com.opencsv.CSVWriter;
/* distance and angle tracking algorithm, use experienced equation to predict step length
** Author:Yuanyuan Dong
 */

public class MainActivity extends AppCompatActivity implements SensorEventListener {
    File path;
    File file;
    private String baseFolder;
    float ax,ay,az,grax,gray,graz,linax,linay,linaz,gx,gy,gz,mx,my,mz,lx,mag_acc,gzo;
    double auv;
    long timer;
    FileWriter fWriter;
    public SensorManager aSensorManager;
    int step;
    private Context mContext;
    CSVWriter writer;
    private String timestampFineFormat = "dd-MM-yy_HH:mm:ss:SSS";
    boolean started_counting=false;
    private SimpleDateFormat sdfFine = new SimpleDateFormat(timestampFineFormat, Locale.US);
    float accumulate_gyroz=0;
    String nameTime,startt;
    long previous_timestamp=0;
    long current_timestamp=0;
    boolean turnstart=false;
    long turnstarttime=0;
    float beforeturngyro=0;
    long turnendtime=0;
    float offset=0;
    float[] mGravity;
    float[] mGeomagnetic;
    float azimut;
    float l=0.0f;
    Float cvalue=0.0f;
    List<String> al=new ArrayList<>();
    long start_time=0;
    int count=0;
    int summs,summs2;
    float[] lowlinearacc={0,0,0},lowgravity={0,0,0};
    int mark;
    double distance=0;
    String asx="";
    String asy="";
    String asz="";
    String sgrax="";
    String sgray="";
    String sgraz="";
    String slinax="";
    String slinay="";
    String slinaz="";
    String gsx="";
    String gsy="";
    String gsz="";
    String msx="";
    String msy="";
    String msz="";
    String lsx="";
    String auvs="";



boolean stop=true;
    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        String filename = "90cm_8steps"+nameTime+".csv";
        if(Environment.getExternalStorageState().equals(Environment.MEDIA_MOUNTED)) {
            path = Environment.getExternalStoragePublicDirectory(Environment.DIRECTORY_DOWNLOADS);
            file = new File(path, filename);
        }
        //revert to internal storage
        else {
            baseFolder = mContext.getFilesDir().getAbsolutePath();
            file = new File(baseFolder + filename);
        }
    }

    public void onAccuracyChanged(Sensor sensor, int accuracy) {

    }

    public void onSensorChanged(SensorEvent event){
        //save time,data to container
        if(started_counting==false)
        {
            start_time=event.timestamp;
            previous_timestamp=event.timestamp;
            started_counting=true;
        }


        if(started_counting==true&&event.sensor.getType()==Sensor.TYPE_GYROSCOPE)
        {
            if(1112000000<(event.timestamp-start_time)&&(event.timestamp-start_time)<1128000000)
            {
                offset=event.values[2];
            }
            if(1112000000>(event.timestamp-start_time))
            { return; }

        }

        String timestamp=sdfFine.format(new Date());
        al.add(timestamp+",");
        nameTime=timestamp;

        switch(event.sensor.getType()) {
            case Sensor.TYPE_ACCELEROMETER:
                mGravity = event.values;
                ax = event.values[0];
                asx = Float.toString(ax);
                ay = event.values[1];
                asy = Float.toString(ay);
                az = event.values[2];
                asz = Float.toString(az);
                al.add(asx);
                al.add(",");
                al.add(asy);
                al.add(",");
                al.add(asz);
                al.add(",");
                al.add(sgrax);
                al.add(",");
                al.add(sgray);
                al.add(",");
                al.add(sgraz);
                al.add(",");
                al.add(slinax);
                al.add(",");
                al.add(slinay);
                al.add(",");
                al.add(slinaz);
                al.add(",");
                al.add(auvs);
                al.add(",");
                al.add(gsx);
                al.add(",");
                al.add(gsy);
                al.add(",");
                al.add(gsz);
                al.add(",");
                al.add(msx);
                al.add(",");
                al.add(msy);
                al.add(",");
                al.add(msz);
                al.add(",");
                al.add(lsx);
                mag_acc = ax * ax + ay * ay + az * az;
                al.add(",");
                al.add(Float.toString(l));
                al.add(",");
                al.add("90");
                al.add(",");
                al.add(Integer.toString(mark));
                al.add("\n");
                String dad = "";
                for (Integer i = 0; i < al.size(); i++) {
                    dad = dad + al.get(i);
                }

                try {
                    fWriter = new FileWriter(file, true);
                    fWriter.write(dad);
                    fWriter.flush();
                    fWriter.close();
                } catch (Exception e) {
                    e.printStackTrace();
                }
                mark=0;
                break;


            case Sensor.TYPE_GRAVITY:
                lowgravity=lowPass(event.values,lowgravity);
                grax=lowgravity[0];
                gray=lowgravity[1];
                graz=lowgravity[2];
                sgrax=Float.toString(grax);
                sgray=Float.toString(gray);
                sgraz=Float.toString(graz);
                break;
            case Sensor.TYPE_LINEAR_ACCELERATION:
                double stepsize=0.0;
                lowlinearacc=lowPass(event.values,lowlinearacc);
                linax=lowlinearacc[0];
                linay=lowlinearacc[1];
                linaz=lowlinearacc[2];
                slinax=Float.toString(linax);
                slinay=Float.toString(linay);
                slinaz=Float.toString(linaz);

                double fi =Math.atan(Math.abs(graz/gray));
                auv=linaz*Math.sin(fi)+linaz*Math.cos(fi);
                auvs=Double.toString(auv);
                Log.e("AUV",Double.toString(auv));


                if(auv>2.15){
                    String thist=sdfFine.format(new  Date());
                    String[] t=thist.split(":");
                    int sec=Integer.parseInt(t[2]);
                    int ms=Integer.parseInt(t[3]);
                     summs=sec*1000+ms;
                    Log.e("summs time",thist);
                    Log.e("summs time",Integer.toString(summs));

                    if (stop==true){
                        startt=thist;
                    }
                    String[] st = startt.split(":");
                    int sec2 = Integer.parseInt(st[2]);
                    int ms2 = Integer.parseInt(st[3]);
                    summs2 = sec2 * 1000 + ms2;
                    Log.e("summs2 time",thist);
                    Log.e("summs2 time",Integer.toString(summs2));
                    stop=false;
                Log.e("summs2-1",Integer.toString(summs-summs2) );
                    startt=thist;
                    if ((summs-summs2)>200){
                        count=count+1;

                        float f=1000/(summs-summs2);
                        stepsize=0.2975*1.65*Math.sqrt(f);

                        TextView b = (TextView) findViewById(R.id.textView3);
                        b.setText(Double.toString(stepsize));

                        TextView c = (TextView) findViewById(R.id.textView4);
                        distance=stepsize+distance;
                        c.setText(Double.toString(distance));
                    }

                }

                TextView a = (TextView) findViewById(R.id.textView2);
                   a.setText(Float.toString(count));
                Log.e("StepCount",Integer.toString(count));
                break;

            case Sensor.TYPE_GYROSCOPE:
                gx = event.values[0];
                gsx = Float.toString(gx);
                gy = event.values[1];
                gsy = Float.toString(gy);
                gz = event.values[2];
                gsz = Float.toString(gz);
                current_timestamp = event.timestamp;
                gzo = gz - offset;
                if (Math.abs(gzo) >= 0.27) {
                    if (turnstart == false) {
                        turnstart = true;
                        turnstarttime = event.timestamp;
                        beforeturngyro = accumulate_gyroz;
                    }
                    gzo = gzo * 89 / 80;
                    accumulate_gyroz = Math.abs((gzo) * (current_timestamp - previous_timestamp) / 3.14f * 180 / 1000000000) + accumulate_gyroz;
                } else {
                    turnendtime = event.timestamp;
                    if (turnendtime - turnstarttime < 500000000) {
                        accumulate_gyroz = beforeturngyro;
                    }
                    turnstart = false;
                }
                previous_timestamp = current_timestamp;

                break;

            case Sensor.TYPE_MAGNETIC_FIELD:
                mGeomagnetic = event.values;
                mx = event.values[0];
                msx = Float.toString(mx);
                my = event.values[1];
                msy = Float.toString(my);
                mz = event.values[2];
                msz = Float.toString(mz);

                if (mGravity != null && mGeomagnetic != null) {
                    float R[] = new float[9];
                    float I[] = new float[9];
                    boolean success = SensorManager.getRotationMatrix(R, I, mGravity, mGeomagnetic);
                    if (success) {
                        float orientation[] = new float[3];
                        SensorManager.getOrientation(R, orientation);
                        azimut = orientation[0]; // orientation contains: azimut, pitch and roll
                        cvalue = azimut * 180 / (3.14159f);
                    }

                }


                break;
            case Sensor.TYPE_LIGHT:
                lx = event.values[0];
                lsx = Float.toString(ax);
                break;

            case Sensor.TYPE_ORIENTATION: {
                l = event.values[0];
                break;
            }
        }

        al.clear();
    }


    /*
     * time smoothing constant for low-pass filter
     * 0 ≤ alpha ≤ 1 ; a smaller value basically means more smoothing
     * See: http://en.wikipedia.org/wiki/Low-pass_filter#Discrete-time_realization
     */
    static final float ALPHA = 0.5f;

    /**
     *  http://en.wikipedia.org/wiki/Low-pass_filter#Algorithmic_implementation
     * http://developer.android.com/reference/android/hardware/SensorEvent.html#values
     */
    protected float[] lowPass( float[] input, float[] output ) {
        if ( output == null ) return input;

        for ( int i=0; i<input.length; i++ ) {
            output[i] = output[i] + ALPHA * (input[i] - output[i]);
        }
        return output;
    }


//used in step data marking
    void stepMark(View button){
//        String mark="-10000,-10000,-10000,-10000,-10000,-10000,-10000,-10000,-10000,-10000,-10000,-10000,-10000,-10000,-10000,-10000,-10000,-10000,-10000,-10000";
//        mark=mark+'\n';
         mark=1;
//        try {
//            fWriter = new FileWriter(file, true);
//            fWriter.write(mark);
//            fWriter.flush();
//            fWriter.close();
//        } catch (Exception e) {
//            e.printStackTrace();
//        }

    }

    void record(View button) {
        Log.e("started", "Good to go");
        step=0;
        timer= 0;
        aSensorManager=(SensorManager) getSystemService(SENSOR_SERVICE);
        Sensor lig = aSensorManager.getDefaultSensor(Sensor.TYPE_LIGHT);
        aSensorManager.registerListener(this, lig, SensorManager.SENSOR_DELAY_FASTEST);
        Sensor grav=aSensorManager.getDefaultSensor(Sensor.TYPE_GRAVITY);
        aSensorManager.registerListener(this, grav, SensorManager.SENSOR_DELAY_FASTEST);
        Sensor linea=aSensorManager.getDefaultSensor(Sensor.TYPE_LINEAR_ACCELERATION);
        aSensorManager.registerListener(this, linea, SensorManager.SENSOR_DELAY_FASTEST);
        Sensor mag = aSensorManager.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD);
        aSensorManager.registerListener(this, mag, SensorManager.SENSOR_DELAY_FASTEST);
        Sensor accel = aSensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);
        aSensorManager.registerListener(this, accel, SensorManager.SENSOR_DELAY_FASTEST);
        Sensor gyro = aSensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE);
        aSensorManager.registerListener(this, gyro, SensorManager.SENSOR_DELAY_FASTEST);
        aSensorManager.registerListener(this, aSensorManager.getDefaultSensor(Sensor.TYPE_ORIENTATION), SensorManager.SENSOR_DELAY_FASTEST);

        try {
            fWriter=new FileWriter(file,false);
            writer=new CSVWriter(fWriter);
            String[] datav={"Timestamp","Accel_x","Accel_y","Accel_z","GRAVITY_x","GRAVITY_y","GRAVITY_z","LINEAR_ACC_x","LINEAR_ACC_y","LINEAR_ACC_z","AUV","Gyro_x","GYRO_y","GYRO_z","Mag_x","Mag_y","Mag_z","Ligt intensity","Compass","Label","StepMark"};
            writer.writeNext(datav);
            writer.close();
        }
        catch(Exception e){
            Log.w("ExternalStorage", "Error writing " + "recording", e);
        }

    }


    boolean save(View button) throws IOException {
        aSensorManager.unregisterListener(this);
        Log.e("Stoped running","Stoped.........*******");
        super.onPause();
        return true;
    }



}
