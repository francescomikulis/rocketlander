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
import java.util.concurrent.ConcurrentHashMap;

/*
https://www.java2novice.com/java-file-io-operations/read-write-object-from-file/
 */

public class RLObjectFileStore {
    private static String episodeDataFileName = "episodesData.txt";
    private static String actionValueFunctionFileName = "actionValue";

    private static class InstanceHolder {
        private static final RLObjectFileStore instance = new RLObjectFileStore();
    }

    public static RLObjectFileStore getInstance() {
        return InstanceHolder.instance;
    }

    private RLObjectFileStore(){}


    public ArrayList readEpisodesData(){
        return (ArrayList) readObjects(episodeDataFileName);
    }

    public HashMap<StateActionTuple, Double> readActionValueFunction(String filenameExtension){
        return (HashMap<StateActionTuple, Double>) readObjects(actionValueFunctionFileName + filenameExtension);
    }

    public void storeActionValueFunction(HashMap actionValueFunction, String filenameExtension){
        storeObject(actionValueFunctionFileName + filenameExtension, actionValueFunction);
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
            objOps.close();
        } catch (FileNotFoundException e) {
            e.printStackTrace();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    private Object readObjects(String fileName) {
        InputStream fileIs = null;
        ObjectInputStream objIs = null;
        Object data = null;
        try {
            fileIs = new FileInputStream(fileName);
            objIs = new ObjectInputStream(fileIs);
            data = objIs.readObject();
            objIs.close();
        } catch (FileNotFoundException e) {
            e.printStackTrace();
        } catch (IOException e) {
            e.printStackTrace();
        } catch (ClassNotFoundException e) {
            e.printStackTrace();
        }
        return data;
    }
}

