package net.sf.openrocket.simulation.extension.impl;

import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.ObjectInputStream;
import java.io.ObjectOutputStream;
import java.io.OutputStream;
import java.util.ArrayList;
import java.util.HashMap;

/*
https://www.java2novice.com/java-file-io-operations/read-write-object-from-file/
 */

public class RLMyObjectFileStore {
    private static String episodeFileName = "episodesData.txt";
    private static String actionValueFunctionFileName = "actionValue.txt";

    public ArrayList readEpisodesData(){
        return (ArrayList) readObjects(episodeFileName);
    }

    public void storeEpisodesData(ArrayList episodesData){
        storeObject(episodeFileName, episodesData);
    }

    public HashMap readActionValueFunction(){
        return (HashMap) readObjects(actionValueFunctionFileName);
    }

    public void storeActionValueFunction(HashMap actionValueFunction){
        storeObject(actionValueFunctionFileName, actionValueFunction);
    }

    /*
    Private implementation.  Details.
     */

    private void storeObject(String fileName, Object data) {
        OutputStream ops = null;
        ObjectOutputStream objOps = null;
        try {
            ops = new FileOutputStream(fileName);
            objOps = new ObjectOutputStream(ops);
            objOps.writeObject(data);
            objOps.flush();
        } catch (FileNotFoundException e) {
            e.printStackTrace();
        } catch (IOException e) {
            e.printStackTrace();
        } finally {
            try {
                if (objOps != null) objOps.close();
            } catch (Exception ex) {
            }
        }
    }

    private Object readObjects(String fileName) {
        InputStream fileIs = null;
        ObjectInputStream objIs = null;
        Object data = new Object();
        try {
            fileIs = new FileInputStream(fileName);
            objIs = new ObjectInputStream(fileIs);
            data = objIs.readObject();
        } catch (FileNotFoundException e) {
            e.printStackTrace();
        } catch (IOException e) {
            e.printStackTrace();
        } catch (ClassNotFoundException e) {
            e.printStackTrace();
        } finally {
            try {
                if (objIs != null) objIs.close();
            } catch (Exception ex) {
            }
        }
        return data;
    }
}

