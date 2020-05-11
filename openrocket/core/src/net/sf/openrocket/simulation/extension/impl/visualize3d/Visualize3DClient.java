package net.sf.openrocket.simulation.extension.impl.visualize3d;

// A Java program for a Visualize3DClient
import java.net.*;
import java.io.*;

public class Visualize3DClient {
    // initialize socket and output streams
    private Socket socket            = null;
    private DataOutputStream out     = null;
    private String connectionString = "10.0.0.226:8080"; // 10.22.0.180:8080  // "127.0.0.1:5000"
    private int port;
    private String address;

    private static volatile Visualize3DClient instance;

    private Visualize3DClient(){
        setConnectionString(this.connectionString);
    }

    public static Visualize3DClient getInstance() {
        if (instance == null) { // first time lock
            synchronized (Visualize3DClient.class) {
                if (instance == null) {  // second time lock
                    instance = new Visualize3DClient();
                }
            }
        }
        return instance;
    }


    public void setConnectionString(String connectionString) {
        this.connectionString = connectionString;
        String[] parts = connectionString.split(":");
        this.address = parts[0];
        this.port = Integer.parseInt(parts[1]);
    }

    public boolean Connect(){
        // establish a connection
        try {
            try { socket = new Socket(this.address, this.port); } catch (Exception e) {
                System.out.println("Blender not running!");
                System.out.println(e);
                return false;
            }

            // sends output to the socket
            out = new DataOutputStream(socket.getOutputStream());
            return true;
        } catch (Exception e) {
            System.out.println(e);
            return false;
        }
    }

    boolean Connected(){ return this.socket != null; }

    public void write(String data) {
        try {
            out.writeBytes(data);
        } catch (IOException e){
            e.printStackTrace();
        }

    }

    public void write(byte[] b, int off, int len) {
        try {
            out.write(b, off, len);
        } catch (IOException e){
            e.printStackTrace();
        }
    }

    public void close() {
        try { out.close(); } catch (Exception e) {}
    }

    private void killAll() {
        System.out.println("CALLED KILL ALL!");

        try { out.close(); } catch (Exception e) {}
        //try { socket.shutdownInput(); } catch (Exception e) {}
        //try { socket.shutdownOutput(); } catch (Exception e) {}
        try { socket.close(); } catch (Exception e) {}
        out = null;
        socket = null;
    }

    protected void finalize() throws Throwable {
        killAll();
    }

}
