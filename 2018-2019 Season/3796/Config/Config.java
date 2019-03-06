package org.firstinspires.ftc.teamcode.Config;

import android.os.Environment;

import com.google.gson.JsonElement;
import com.google.gson.JsonObject;
import com.google.gson.JsonParser;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.io.InputStreamReader;

/**
 * Name: Config
 * Authors: Lincoln Doney
 * Team: FTC Talons 3796 2018-2019 season
 * Date: October 6, 2018
 *
 * Sadly, this is unused. This is supposed to allow us to control all of the settings of the robot from one file
 * Due to errors with permissions and failiure to read the json file, this part of the code was scrapped and replaced
 * by static variables in their corresponding files.
 * */
public class Config {
    public static JsonObject cfg = null;
    public static String FILE_NAME = "config.json";
    public static String PATH = Environment.getExternalStorageDirectory().getAbsolutePath() + "/instinctcoder/readwrite/" ;

    public static String init() {
        String line = null;
        try {
            JsonParser js = new JsonParser();
            FileInputStream fs = new FileInputStream(new File(PATH + FILE_NAME));
            InputStreamReader is = new InputStreamReader(fs);
            BufferedReader br = new BufferedReader(is);
            StringBuilder sb = new StringBuilder();

            while((line = br.readLine()) != null)
            {
                sb.append(line + System.getProperty("line.seperator"));
            }
            fs.close();
            line = sb.toString();
            br.close();

            return line;
        }catch(FileNotFoundException f)
        {
            return "INVALID FILE";
        }
        catch(IOException e)
        {
            return "IO ERR";
        }
    }

    public static int getInt(String id)
    {
            if (cfg == null) {
                init();
            }
            if (cfg != null) {
                return cfg.get(id).getAsInt();
            }
        return -1;
    }

    public static String getAsString()
    {
        init();
        return cfg.getAsString();
    }
    public static String getStr(String id)
    {
                init();
                return cfg.get(id).getAsString();
    }

    public static JsonElement getObj(String id)
    {
            init();
            if(cfg != null) {
                return cfg.getAsJsonObject().get(id);
            }
        return null;
    }

    public static double getDouble(String id)
    {
            if(cfg == null)
            {
                init();
            }
            if(cfg != null)
            {
                return cfg.getAsJsonObject().get(id).getAsDouble();
            }
        return -1.0;
    }
}
