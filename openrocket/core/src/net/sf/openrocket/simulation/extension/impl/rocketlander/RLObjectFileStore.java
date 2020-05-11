package net.sf.openrocket.simulation.extension.impl.rocketlander;

import java.io.*;
import java.util.HashMap;
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
            try {
                try {
                    float[] valueFunctionTable = (float[]) readObjects(fileName);
                    if (valueFunctionTable.length == definition.indexProduct)
                        definition.setValueFunction(new RLValueFunction(valueFunctionTable));
                    else
                        return false;  // sizes are different so must re-allocate
                } catch (Exception e) {
                    HashMap<Integer, Float> valueFunctionMap = (HashMap<Integer, Float>) readObjects(fileName);
                    definition.setValueFunction(new RLValueFunction(valueFunctionMap));
                    exists = true;
                }
            } catch (Exception e) {
                exists = false;
            }
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
        for (Map.Entry<String, MDPDefinition> entry: RLModelSingleton.getInstance().getMethods().entrySet()) {
            if (entry.getValue().valueFunction.isTable) {
                float[] actionValueFunctionTable = entry.getValue().valueFunction.getValueFunctionTable();
                storeObject(actionValueFunctionTable, actionValueFunctionFileName + entry.getKey() + ".txt");
            } else {
                HashMap<Integer, Float> actionValueFunctionMap = entry.getValue().valueFunction.getValueFunctionMap();
                storeObject(actionValueFunctionMap, actionValueFunctionFileName + entry.getKey() + ".txt");
            }
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

