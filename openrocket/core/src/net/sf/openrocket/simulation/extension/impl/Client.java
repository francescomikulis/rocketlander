package net.sf.openrocket.simulation.extension.impl;

// A Java program for a Client
import java.net.*;
import java.io.*;

public class Client {
    // initialize socket and output streams
    private Socket socket            = null;
    private DataOutputStream out     = null;
    private String connectionString = "127.0.0.1:5000"; // 10.22.0.180:8080
    private int port;
    private String address;

    private static volatile Client instance;

    private Client(){
        setConnectionString(this.connectionString);
    }

    public static Client getInstance() {
        if (instance == null) { // first time lock
            synchronized (Client.class) {
                if (instance == null) {  // second time lock
                    instance = new Client();
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

    public void Connect(){
        // establish a connection
        try {
            try { socket = new Socket(this.address, this.port); } catch (Exception e) {
                System.out.println("BAD BAD BAD");
                System.out.println(e);
            }

            // sends output to the socket
            out = new DataOutputStream(socket.getOutputStream());
        } catch (Exception e) {
            System.out.println(e);
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
