package net.sf.openrocket.simulation.extension.impl;

// A Java program for a Client
import java.net.*;
import java.io.*;

public class Client {
    // initialize socket and output streams
    private Socket socket            = null;
    private DataOutputStream out     = null;
    int port = 8080;  // 5000;
    String address = "10.22.0.180";// 127.0.0.1  // 10.22.0.252

    private static class InstanceHolder {
        private static final Client instance = new Client();
    }

    public static Client getInstance() {
        return InstanceHolder.instance;
    }

    private Client(){
        try { socket = new Socket(this.address, this.port); } catch (Exception e) {}
        System.out.println("Connected");
    }

    // constructor to put ip address and port
    private Client(String address, int port) {
        this.address=address;
        this.port=port;
        try { socket = new Socket(this.address, this.port); } catch (Exception e) {}
        System.out.println("Connected");

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
