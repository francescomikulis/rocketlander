package net.sf.openrocket.simulation.extension.impl;

import net.sf.openrocket.simulation.SimulationConditions;
import net.sf.openrocket.simulation.SimulationStatus;
import net.sf.openrocket.simulation.exception.SimulationException;
import net.sf.openrocket.simulation.extension.AbstractSimulationExtension;
import net.sf.openrocket.simulation.listeners.AbstractSimulationListener;

import java.io.IOException;
import java.lang.Double.*;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;

public class Visualize3DListener extends AbstractSimulationListener {
	Visualize3D visualize3D;
	Client client;
	long curTime;

	Visualize3DListener(Visualize3D visualize3D) {
		this.visualize3D = visualize3D;
		this.client = new Client("127.0.0.1",5000);
	}

	@Override
	public void startSimulation(SimulationStatus status) throws SimulationException {
		client.Connect();
	}

	@Override
	public void postStep(SimulationStatus status) throws SimulationException{
		if (!client.Connected()){
			client.Connect();
		} else{
			double x = status.getRocketPosition().x;
			double y = status.getRocketPosition().y;
			double z = status.getRocketPosition().z;
			double X = status.getRocketOrientationQuaternion().getX();
			double Y = status.getRocketOrientationQuaternion().getY();
			double Z = status.getRocketOrientationQuaternion().getZ();
			double W = status.getRocketOrientationQuaternion().getW();
			client.write("x");
			client.writeDouble(x);
			client.write("y");
			client.writeDouble(y);
			client.write("z");
			client.writeDouble(z);
			client.write("X");
			client.writeDouble(X);
			client.write("Y");
			client.writeDouble(Y);
			client.write("Z");
			client.writeDouble(Z);
			client.write("W");
			client.writeDouble(W);
		}
		waitdt(status);

	}

	@Override
	public void endSimulation(SimulationStatus status, SimulationException exception) {
		client.killAll();
	}

	private void waitdt(SimulationStatus status){
		int timeStep;
		try {
			double val = 1000*visualize3D.getTimeRate()*status.getPreviousTimeStep();
			timeStep = (int) Math.floor(val);
			long realTimeStep = (System.currentTimeMillis()-curTime);
			if (realTimeStep<timeStep)
				Thread.sleep(timeStep-realTimeStep);
			curTime = System.currentTimeMillis();

		} catch (InterruptedException e) {
			e.printStackTrace();
		}

	}
}

