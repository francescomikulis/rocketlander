package net.sf.openrocket.gui.simulation;


import net.miginfocom.swing.MigLayout;
import net.sf.openrocket.document.OpenRocketDocument;
import net.sf.openrocket.document.Simulation;
import net.sf.openrocket.gui.components.ConfigurationComboBox;
import net.sf.openrocket.gui.util.GUIUtil;
import net.sf.openrocket.l10n.Translator;
import net.sf.openrocket.rocketcomponent.FlightConfiguration;
import net.sf.openrocket.rocketcomponent.FlightConfigurationId;
import net.sf.openrocket.rocketcomponent.Rocket;
import net.sf.openrocket.simulation.extension.SimulationExtension;
import net.sf.openrocket.simulation.extension.impl.MDPDefinition;
import net.sf.openrocket.simulation.extension.impl.RLModel;
import net.sf.openrocket.startup.Application;

import javax.swing.*;
import javax.swing.event.ChangeEvent;
import javax.swing.event.ChangeListener;
import javax.swing.event.DocumentEvent;
import javax.swing.event.DocumentListener;
import java.awt.*;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;


public class MDPDefinitionEditDialog extends JDialog {
	private static final long serialVersionUID = -4468157685542912716L;
	private final Window parentWindow;
	private final Simulation[] simulationList;
	private final OpenRocketDocument document;
	private static final Translator trans = Application.getTranslator();

	JPanel cards;
	private final static String EDITMODE = "EDIT";

	public MDPDefinitionEditDialog(Window parent, final OpenRocketDocument document, Simulation... sims) {
		//// Edit simulation
		super(parent, "Edit MDP Definition", ModalityType.DOCUMENT_MODAL);
		this.document = document;
		this.parentWindow = parent;
		this.simulationList = sims;
		
		this.cards = new JPanel(new CardLayout());
		this.add(cards);
		buildEditCard();
		
		this.validate();
		this.pack();
		
		this.setLocationByPlatform(true);
		
		GUIUtil.setDisposableDialogOptions(this, null);
	}
	
	private boolean isSingleEdit() {
		return simulationList.length == 1;
	}

	private String getSimplifiedName(String jsonDefinition) {
		return jsonDefinition.substring(jsonDefinition.indexOf("\""), jsonDefinition.indexOf(","));
	}
	
	private void buildEditCard() {
		JPanel simEditPanel = new JPanel(new MigLayout("fill"));

		if (isSingleEdit()) {
			JPanel panel = new JPanel(new MigLayout("fill, ins 0"));

			//// MDPConfiguration name:
			panel.add(new JLabel("Configuration Name"), "growx 0, gapright para");
			final JTextField field = new JTextField(getSimplifiedName(simulationList[0].getName()));
			field.setEditable(false);
			panel.add(field, "growx, wrap");
			panel.add(new JPanel(), "growx, wrap");

			simEditPanel.add(panel, "growx, wrap");
		}
		JTabbedPane tabbedPane = new JTabbedPane();

		//// Simulation options
		tabbedPane.addTab("MDPDefinition", new MDPDefinitionPanel(simulationList[0]));
		
		tabbedPane.setSelectedIndex(0);
		
		simEditPanel.add(tabbedPane, "spanx, grow, wrap");

		//// Close button 
		JButton close = new JButton(trans.get("dlg.but.close"));
		close.addActionListener(new ActionListener() {
			@Override
			public void actionPerformed(ActionEvent e) {
				MDPDefinitionEditDialog.this.dispose();
			}
		});
		simEditPanel.add(close, "tag ok");
		
		cards.add(simEditPanel, EDITMODE);
	}



	private static class MDPDefinitionPanel extends JPanel {
		private static final Translator trans = Application.getTranslator();


		MDPDefinitionPanel(final Simulation simulation) {
			super(new MigLayout("fill"));

			String definition = simulation.getName();

			JTextPane jtp = new JTextPane();
			jtp.setEditable(true);
			jtp.setContentType("text/plain");
			jtp.setText(definition);

			JScrollPane scrollPane = new JScrollPane(jtp);
			this.add(scrollPane, "growx, growy, split 2, aligny 0, flowy, gapright para");

			//// Close button
			JButton save = new JButton("Save");
			save.addActionListener(new ActionListener() {
				@Override
				public void actionPerformed(ActionEvent e) {
					String newDefinition = jtp.getText().replaceAll("\n", "");
					newDefinition = newDefinition.replaceAll("\t", "");
					newDefinition = newDefinition.replaceAll(" ", "");

					String storeDefinitionString = newDefinition.replace("IGNORE", "");
					MDPDefinition theDefinition = MDPDefinition.buildFromJsonString(storeDefinitionString);
					storeDefinitionString = MDPDefinition.toJsonString(theDefinition);

					if (newDefinition.contains("IGNORE")) {
						storeDefinitionString = "IGNORE" + "\n" + storeDefinitionString;
					}

					simulation.setName(storeDefinitionString);
				}
			});
			this.add(save, "tag ok");

		}
	}
}
