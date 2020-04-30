package net.sf.openrocket.simulation.extension.impl;

import net.sf.openrocket.simulation.extension.impl.methods.ModelBaseImplementation;

import java.io.*;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.Map;

/*
https://www.java2novice.com/java-file-io-operations/read-write-object-from-file/
 */

public class RLObjectFileStore {
    private static String actionValueFunctionFileName = "actionValue";

    private static class InstanceHolder {
        private static final RLObjectFileStore instance = new RLObjectFileStore();
    }

    public static RLObjectFileStore getInstance() {
        return InstanceHolder.instance;
    }

    private RLObjectFileStore(){}

    public boolean tryToReadActionValueFunctionFromDefinition(MDPDefinition definition) {
        String fileName = actionValueFunctionFileName + definition.name + ".txt";
        boolean exists = false;
        try {
            File tempFile = new File(fileName);
            exists = tempFile.exists();
        } catch (Exception e) {
            exists = false;
        }
        if (exists) {
            float[] valueFunction = (float[]) readObjects(fileName);
            if (valueFunction.length == definition.indexProduct)
                definition.valueFunction = valueFunction;
            else
                exists = false;  // sizes are different so must re-allocate
        }
        return exists;
    }

    // starting to attempt to move the actionValueFunction to the MDP Definition

    public static void storeDefinition(MDPDefinition definition, String fileName) {
        storeObject(MDPDefinition.toJsonString(definition), fileName);
    }

    public MDPDefinition readDefinition(String fileName){
        return MDPDefinition.buildFromJsonString((String)readObjects(fileName));
    }

    public static void storeActionValueFunctions(){
        for (Map.Entry<String, MDPDefinition> entry: RLModel.getInstance().getMethods().entrySet()) {
            float[] actionValueFunction = entry.getValue().valueFunction;
            storeObject(actionValueFunction, actionValueFunctionFileName + entry.getKey() + ".txt");
        }
    }

    /*
    Private implementation.  Details.
     */

    private static void storeObject(Object data, String fileName) {
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

    private static Object readObjects(String fileName) {
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

