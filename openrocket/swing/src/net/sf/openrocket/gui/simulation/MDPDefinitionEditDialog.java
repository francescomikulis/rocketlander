package net.sf.openrocket.gui.simulation;


import net.miginfocom.swing.MigLayout;
import net.sf.openrocket.document.Definition;
import net.sf.openrocket.document.OpenRocketDocument;
import net.sf.openrocket.gui.util.GUIUtil;
import net.sf.openrocket.l10n.Translator;
import net.sf.openrocket.simulation.extension.impl.rocketlander.MDPDefinition;
import net.sf.openrocket.startup.Application;
import org.fife.ui.rsyntaxtextarea.RSyntaxTextArea;
import org.fife.ui.rsyntaxtextarea.SyntaxConstants;
import org.fife.ui.rtextarea.RTextScrollPane;

import javax.swing.*;
import java.awt.*;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.util.regex.Matcher;
import java.util.regex.Pattern;


public class MDPDefinitionEditDialog extends JDialog {
	private static final long serialVersionUID = -4468157685542912716L;
	private final Window parentWindow;
	private final Definition[] definitionList;
	private final OpenRocketDocument document;
	private static final Translator trans = Application.getTranslator();

	JPanel cards;
	private final static String EDITMODE = "EDIT";

	public MDPDefinitionEditDialog(Window parent, final OpenRocketDocument document, Definition... definitions) {
		//// Edit simulation
		super(parent, "Edit MDP Definition", ModalityType.DOCUMENT_MODAL);
		this.document = document;
		this.parentWindow = parent;
		this.definitionList = definitions;
		
		this.cards = new JPanel(new CardLayout());
		this.add(cards);
		buildEditCard();
		
		this.validate();
		this.pack();
		
		this.setLocationByPlatform(true);
		
		GUIUtil.setDisposableDialogOptions(this, null);
	}
	
	private boolean isSingleEdit() {
		return definitionList.length == 1;
	}


	
	private void buildEditCard() {
		JPanel simEditPanel = new JPanel(new MigLayout("fill"));

		if (isSingleEdit()) {
			JPanel panel = new JPanel(new MigLayout("fill, ins 0"));

			//// Simulation name:
			panel.add(new JLabel("Configuration Name"), "growx 0, gapright para");
			final JTextField field = new JTextField(definitionList[0].getName());
			field.setEditable(false);
			panel.add(field, "growx, wrap");

			panel.add(new JPanel(), "growx, wrap");

			simEditPanel.add(panel, "growx, wrap");
		}
		JTabbedPane tabbedPane = new JTabbedPane();

		//// Simulation options
		tabbedPane.addTab("MDPDefinition", new MDPDefinitionPanel(definitionList[0]));
		
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
		private RSyntaxTextArea text;


		MDPDefinitionPanel(final Definition definition) {
			super(new MigLayout("fill"));

			text = new RSyntaxTextArea(cleanJsonStringByRemovingArraySpaces(definition.getData()), 40, 80);
			text.setEditable(true);
			text.setCaretPosition(0);
			text.setCodeFoldingEnabled(true);
			text.setLineWrap(true);
			text.setWrapStyleWord(true);

			RTextScrollPane scrollPane = new RTextScrollPane(text);
			scrollPane.setLineNumbersEnabled(true);
			this.add(scrollPane, "growx, growy, split 2, aligny 0, flowy, gapright para");

			//// Close button
			JButton save = new JButton("Save");
			final MDPDefinitionPanel thisPanel = this;
			save.addActionListener(new ActionListener() {
				@Override
				public void actionPerformed(ActionEvent e) {
					String newDefinition = text.getText().replaceAll("\n", "");
					newDefinition = newDefinition.replaceAll("\t", "");
					newDefinition = newDefinition.replaceAll(" ", "");

					try {
						MDPDefinition theDefinition = MDPDefinition.buildFromJsonString(newDefinition);
						String newName = theDefinition.name;
						String storeDefinitionString = MDPDefinition.toJsonString(theDefinition);

						definition.setName(newName);
						definition.setData(storeDefinitionString);
						text.setText(cleanJsonStringByRemovingArraySpaces(definition.getData()));
					} catch (Exception exc) {

						int realLineNumber = tryAndGetRealErrorLineNumber(exc);
						String cleanMessage = "";
						if (realLineNumber != -1) {
							cleanMessage = "Malformed JSON at definition line: " + realLineNumber;
						} else {
							cleanMessage = "Probably internal GSON parsing error.  Originated because of your last change.\n";
							cleanMessage += "Check your quote formats, missing commas and square brackets.";
						}
						JOptionPane.showMessageDialog(thisPanel, new Object[] {"ERROR\n", cleanMessage, exc.getMessage()});
					}
				}
			});
			this.add(save, "tag ok");

		}

		private String cleanJsonStringByRemovingArraySpaces(String jsonString) {
			return jsonString.replaceAll("\\[\\s*(\\d|-)", "[$1")
					.replaceAll("(\\d,)\\s*(\\d|-)", "$1 $2")
					.replaceAll("(\\d|-)\\s*]", "$1]");
		}

		private int tryAndGetRealErrorLineNumber(Exception exc) {
			int realLineNumber = -1;
			if (exc.getMessage().contains("$.")) {
				String fieldAreaError = exc.getMessage().substring(exc.getMessage().lastIndexOf("$."));
				fieldAreaError = "\"" + fieldAreaError.replace("$.", "").replace(".", "") + "\"";
				String definitionText = text.getText();
				realLineNumber = 1;
				int indexOfStartNewline = 0;
				int indexOfNextNewline = definitionText.indexOf("\n");
				if (indexOfNextNewline != -1) {
					String currentLine = definitionText.substring(indexOfStartNewline, indexOfNextNewline);
					while (!currentLine.contains(fieldAreaError)) {
						indexOfStartNewline = indexOfNextNewline;
						indexOfNextNewline = definitionText.indexOf("\n", indexOfStartNewline + 1);
						if (indexOfNextNewline == -1) {
							realLineNumber = -1;
							break;
						}
						currentLine = definitionText.substring(indexOfStartNewline, indexOfNextNewline);
						realLineNumber++;
					}
				}
			}

			return realLineNumber;
		}
	}
}
