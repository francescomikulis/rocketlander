package net.sf.openrocket.simulation.extension.impl;

import net.sf.openrocket.simulation.extension.impl.methods.ModelBaseImplementation;

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

    public OptimizedMap readActionValueFunctionFromMethods(LinkedHashMap<String, ModelBaseImplementation> methods) {
        for (Map.Entry<String, ModelBaseImplementation> entry: methods.entrySet()) {
            float[] actionValueFunction = (float[]) readObjects(actionValueFunctionFileName + entry.getKey() + ".txt");
            entry.getValue().definition.valueFunction = actionValueFunction;
        }
        return new OptimizedMap(methods);
    }

    // starting to attempt to move the actionValueFunction to the MDP Definition

    public static void storeDefinition(HashMap<String, LinkedHashMap> definition, String fileName) {
        storeObject(definition, fileName);
    }

    public HashMap<String, LinkedHashMap> readDefinition(String fileName){
        HashMap<String, LinkedHashMap> definition = (HashMap<String, LinkedHashMap>) readObjects(fileName);
        return definition;
        // return new OptimizedMap(null, null, null);
    }

    public static void storeActionValueFunctions(){
        OptimizedMap optimizedMap = RLModel.getInstance().getValueFunctionTable();
        for (Map.Entry<String, float[]> entry: optimizedMap.valueFunctionTables.entrySet()) {
            float[] actionValueFunction = entry.getValue();
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

