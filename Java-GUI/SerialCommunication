import gnu.io.*;
import java.awt.Color;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.util.Enumeration;
import java.util.HashMap;
import java.util.TooManyListenersException;


public class SerialCommunication implements SerialPortEventListener{
	//passed from main GUI
	GUI window = null;
	
	//for containing the ports that will be found
	private Enumeration ports = null;
	//map the port names to CommPortIdentifiers
	private HashMap portMap = new HashMap();
	
	//this is the object that contains the opened port
	private CommPortIdentifier selectedPortIdentifier = null;
	private SerialPort serialPort = null;
	
	//input and output streams for sending and receiving data
	private InputStream input = null;
	private OutputStream output = null;
	
	//just a boolean flag that i use for enabling and disabling buttons depending
	//on whether the program is connected to a serial port or not
	private boolean bConnected = false;
	
	//the timeout value for connecting with the port
	final static int TIMEOUT = 2000;
	
	//some ascii values for certain things
	final static int SPACE_ASCII = 32;
	final static int DASH_ASCII = 45;
	final static int NEW_LINE_ASCII = 10;
	
	//a string for recording what goes on in the program
	String logText = "";

	//Search for all the serial ports.
	//pre style="font-size: 11px": none
	//post: adds all the found ports to a combo box on the GUI
	public void searchForPorts(){
		ports = CommPortIdentifier.getPortIdentifiers();
		
		while(ports.hasMoreElements()){
			CommPortIdentifier curPort = (CommPortIdentifier)ports.nextElement();
			
			//get only serial ports
			if(curPort.getPortType() == CommPortIdentifier.PORT_SERIAL){
				window.cboxPorts.addItem(curPort.getName());
				portMap.put(curPort.getName(), curPort);
			}
		}
	}
	
	public void setConnected(boolean response){
		bConnected = response;
	}
	
	public void connect(){
		String selectedPort = (String)window.cboxPorts.getSelectedItem();
		selectedPortIdentifier = (CommPortIdentifier)portMap.get(selectedPort);
		
		CommPort commPort = null;
		
		try {
			//the method below returns an object of type CommPort
			commPort = selectedPortIdentifier.open("TigerControlPanel", TIMEOUT);
			//the CommPort object can be casted to a SerialPort object
			serialPort = (SerialPort)commPort;
			
			//for controlling GUI elements
			setConnected(true);
			
			//logging
			logText = selectedPort + " opened successfully.";
			window.txtLog.setForeground(Color.black);
			window.txtLog.append(logText + "n");
			
			//CODE ON SETTING BAUD RATE ETC OMITTED
			//XBEE PAIR ASSUMED TO HAVE SAME SETTING ALREADY
			serialPort.setSerialPortParams(9600, SerialPort.DATABITS_8, SerialPort.STOPBITS_1, SerialPort.PARITY_NONE);
			
			//enables the controls on the GUI if a successful connection is made
			window.keybindingController.toggleControls();
		} catch (PortInUseException e){
			logText = selectedPort + " is in use. (" + e.toString() + ")";
			
			window.txtLog.setForeground(Color.RED);
			window.txtLog.append(logText + "n");
		} catch (Exception e){
			logText = "Failed to open " + selectedPort + "(" + e.toString() + ")";
			
			window.txtLog.append(logText + "n");
			window.txtLog.setForeground(Color.RED);
		}
	}
	
	//disconnect the serial port
	//pre style="font-size: 11px;": an open serial port
	//post: closed serial port
	public void disconnect(){
		//close the serial port
		try {
			writeData(0,0);
			
			serialPort.removeEventListener();
			serialPort.close();
			input.close();
			output.close();
			setConnected(false);
			window.keybindingController.toggleControls();
			
			logText = "Disconnected.";
			window.txtLog.setForeground(Color.red);
			window.txtLog.append(logText + "n");
		} catch (Exception e){
			logText = "Failed to close" + serialPort.getName() + "(" + e.toString() + ")";
			
			window.txtLog.setForeground(Color.red);
			window.txtLog.append(logText + "n");
		}
	}
	
	public void serialEvent(SerialPortEvent evt) {
		// READ FROM ARDUINO
		if(evt.getEventType() == SerialPortEvent.DATA_AVAILABLE){
			try {
				byte singleData = (byte)input.read();
				
				if(singleData != NEW_LINE_ASCII){
					logText = new String(new byte[] {singleData});
					window.txtLog.append(logText);
				} else {
					window.txtLog.append("n");
				}
			} catch (Exception e) {
				logText = "Failed to read data. (" + e.toString() + ")";
				window.txtLog.setForeground(Color.red);
				window.txtLog.append(logText + "n");
			}
		}
	}
	
	public void writeData(int leftThrottle, int rightThrottle){
		try {
			output.write(leftThrottle);
			output.flush();
			//this is a delimiter for the data
			output.write(DASH_ASCII);
			output.flush();
			
			output.write(rightThrottle);
			output.flush();
			//will be read as a byte so it is a space key
			output.write(SPACE_ASCII);
			output.flush();
		} catch (Exception e){
			logText = "Failed to write data. (" + e.toString() + ")";
			
			window.txtLog.setForeground(Color.red);
			window.txtLog.append(logText + "n");
		}
	}
	
	private void btnConnectctionPerformed(java.awt.event.ActionEvent evt){
		communicator.connect();
		
		if(communicator.getConnected == true){
			if(communicator.initIOStream() == true){
				communicator.initListener();
			}
		}
	}
}
