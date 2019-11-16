package org.opencv.android;

import android.util.Log;

import org.opencv.core.Core;

import java.util.StringTokenizer;

public class MM_StaticHelper { //made public in order to use init Libs

    public static boolean initOpenCV(boolean InitCuda)
    {
        boolean result;
        String libs = "";

        if(InitCuda)
        {
            loadLibrary("cudart");
            loadLibrary("nppc");
            loadLibrary("nppi");
            loadLibrary("npps");
            loadLibrary("cufft");
            loadLibrary("cublas");
        }

        Log.d(TAG, "Trying to get library list");

        try
        {
            System.loadLibrary("opencv_info");
            libs = getLibraryList();
        }
        catch(UnsatisfiedLinkError e)
        {
            Log.e(TAG, "OpenCV error: Cannot load info library for OpenCV");
        }

        Log.d(TAG, "Library list: \"" + libs + "\"");
        Log.d(TAG, "First attempt to load libs");
        if (initOpenCVLibs(libs))
        {
            Log.d(TAG, "First attempt to load libs is OK");
            String eol = System.getProperty("line.separator");
            for (String str : Core.getBuildInformation().split(eol))
                Log.i(TAG, str);

            result = true;
        }
        else
        {
            Log.d(TAG, "First attempt to load libs fails");
            result = false;
        }

        return result;
    }

    public static boolean initImgprocOnly()
    {
        boolean result;
        String libs = "";

        Log.d(TAG, "Trying to get library list");

        try
        {
            System.loadLibrary("opencv_info");
            //libs = "opencv_java3;opencv_core;opencv_imgproc"; //no idea if this works
            libs = "opencv_java3";
        }
        catch(UnsatisfiedLinkError e)
        {
            Log.e(TAG, "OpenCV error: Cannot load info library for OpenCV");
        }

        Log.d(TAG, "Library list: \"" + libs + "\"");
        Log.d(TAG, "First attempt to load libs");
        if (initOpenCVLibs(libs))
        {
            Log.d(TAG, "First attempt to load libs is OK");
            String eol = System.getProperty("line.separator");
            for (String str : Core.getBuildInformation().split(eol))
                Log.i(TAG, str);

            result = true;
        }
        else
        {
            Log.d(TAG, "First attempt to load libs fails");
            result = false;
        }

        return result;
    }


    private static boolean loadLibrary(String Name)
    {
        boolean result = true;

        Log.d(TAG, "Trying to load library " + Name);
        try
        {
            System.loadLibrary(Name);
            Log.d(TAG, "Library " + Name + " loaded");
        }
        catch(UnsatisfiedLinkError e)
        {
            Log.d(TAG, "Cannot load library \"" + Name + "\"");
            e.printStackTrace();
            result = false;
        }

        return result;
    }

    private static boolean initOpenCVLibs(String Libs)
    {
        Log.d(TAG, "Trying to init OpenCV libs");

        boolean result = true;

        if ((null != Libs) && (Libs.length() != 0)) //basically did we actually get a string
        {
            Log.d(TAG, "Trying to load libs by dependency list");
            StringTokenizer splitter = new StringTokenizer(Libs, ";"); //basically makes an array from string split by ';'
            while(splitter.hasMoreTokens()) //for each string in string_array
            {
                result &= loadLibrary(splitter.nextToken()); //get the token, iterate, and load lib
            } //uses logical and for each load --> starts true and stays true until a false is seen
        }
        else
        {
            // If dependencies list is not defined or empty.
            result = loadLibrary("opencv_java3");
        }

        return result;
    }

    private static final String TAG = "OpenCV/MM_StaticHelper";

    private static native String getLibraryList(); //TODO was private, find return then fix it

    public static String publicGetLibraryList(){
        System.loadLibrary("opencv_info");
        return getLibraryList();
    }
}
