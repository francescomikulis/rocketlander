package net.sf.openrocket.simulation.extension.impl;

import net.sf.openrocket.document.Simulation;
import net.sf.openrocket.gui.adaptors.BooleanModel;
import net.sf.openrocket.plugin.Plugin;
import net.sf.openrocket.simulation.extension.AbstractSwingSimulationExtensionConfigurator;
import net.sf.openrocket.simulation.extension.impl.rrt.RRTExtension;

import javax.swing.*;

@Plugin
public class RRTExtensionConfigurator extends AbstractSwingSimulationExtensionConfigurator<RRTExtension> {

	public RRTExtensionConfigurator() {
		super(RRTExtension.class);
	}
	
	@Override
	protected JComponent getConfigurationComponent(RRTExtension extension, Simulation simulation, JPanel panel) {
		panel.add(new JLabel("UsingLateralObjective"));
		BooleanModel m = new BooleanModel(extension,"UsingLateralObjective");
		m.setValue(extension.getUsingLateralObjective());
		panel.add(new JCheckBox(m), "w 100lp!");
		return panel;
	}
	
}
