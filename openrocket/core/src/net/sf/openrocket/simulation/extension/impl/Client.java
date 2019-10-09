package net.sf.openrocket.simulation.extension.impl;

// A Java program for a Client
import java.net.*;
import java.io.*;

public class Client {
    // initialize socket and output streams
    private Socket socket            = null;
    private DataOutputStream out     = null;
    int port;
    String address;

    // constructor to put ip address and port
    public Client(String address, int port) {
        this.address=address;
        this.port=port;
        Connect();

    }
    public void Connect(){
        // establish a connection
        try {
            socket = new Socket(this.address, this.port);
            System.out.println("Connected");

            // sends output to the socket
            out = new DataOutputStream(socket.getOutputStream());
        } catch (UnknownHostException u) {
            System.out.println(u);
        } catch (IOException i) {
            System.out.println(i);
        }
    }
    boolean Connected(){ return this.socket != null; }
    // string to read message from input
    public String read() {
        String line = "";

        // keep reading until "Over" is input
        while (!line.equals("Over")) {
            try {
                out.writeUTF(line);
            } catch (IOException i) {
                System.out.println(i);
            }
        }

        // close the connection
        try {
            out.close();
            socket.close();
        } catch (IOException i) {
            System.out.println(i);
        }
        return line;
    }
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

    public void killAll() {
        try { out.close(); } catch (Exception e) {}
        //try { socket.shutdownInput(); } catch (Exception e) {}
        //try { socket.shutdownOutput(); } catch (Exception e) {}
        try { socket.close(); } catch (Exception e) {}
        out = null;
        socket = null;
    }

}
