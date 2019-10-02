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

public class MyObjectFileStore {
    private static String fileName = "data.txt";

    public void storeObject(ArrayList data) {
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

    public ArrayList readObjects() {
        InputStream fileIs = null;
        ObjectInputStream objIs = null;
        ArrayList data = new ArrayList();
        try {
            fileIs = new FileInputStream(fileName);
            objIs = new ObjectInputStream(fileIs);
            data = (ArrayList) objIs.readObject();
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

    public static void main(String a[]) {
        MyObjectFileStore mof = new MyObjectFileStore();
        ArrayList tester = new ArrayList();
        tester.add(new Integer(5));
        mof.storeObject(tester);
        mof.readObjects();
    }
}

