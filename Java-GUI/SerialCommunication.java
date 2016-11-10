import gnu.io.*;
import java.awt.Color;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.util.Enumeration;
import java.util.HashMap;
import java.util.TooManyListenersException;
import java.util.Vector;
import java.util.Scanner;
import javax.swing.*;

public class SerialCommunication implements SerialPortEventListener{
	//passed from main GUI
	//GUI window = null;
	
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
	private boolean handShake = false;
	
	//the timeout value for connecting with the port
	final static int TIMEOUT = 2000;
	
	//some ascii values for certain things
	final static int SPACE_ASCII = 32;
	final static int DASH_ASCII = 45;
	final static int NEW_LINE_ASCII = 10;
	
	//a string for recording what goes on in the program
	String logText = "";
	
	//Additional variables created by me
	private Scanner scan = new Scanner(System.in);
	private Vector serialPorts = null;
	String lastResponse = "";
	boolean firstResponse = true;
	
	//Search for all the serial ports.
	//pre style="font-size: 11px": none
	//post: adds all the found ports to a combo box on the GUI
	public Vector searchForPorts(){
		ports = CommPortIdentifier.getPortIdentifiers();
		serialPorts = new Vector(0);
		
		while(ports.hasMoreElements()){
			CommPortIdentifier curPort = (CommPortIdentifier)ports.nextElement();
			
			//get only serial ports
			if(curPort.getPortType() == CommPortIdentifier.PORT_SERIAL){
				//Adds found ports to a dropdown box
//				window.cboxPorts.addItem(curPort.getName());
				serialPorts.add(curPort.getName());
								
				portMap.put(curPort.getName(), curPort);
			}
		}
		return serialPorts;
	}
	
	public void setConnected(boolean response){
		bConnected = response;
	}
	
	public boolean getConnected(){
		return bConnected;
	}
	
	public void setHandShake(boolean response){
		if(response == true){
			System.out.println("Handshake made.");
		}
		handShake = response;
	}
	
	public boolean getHandShake(){
		return handShake;
	}

	
	public void connect(){
		int response;
		System.out.println("Here are the ports we found.");

		for(int i = 0; i<serialPorts.size(); i++){
			System.out.println(i + ". " + serialPorts.elementAt(i));
		}
		System.out.println("Which port would you like to connect to? ");
		
		while(!scan.hasNextInt()){
			System.out.println("Please enter the integer associated with your desired port.");
		}

		response = scan.nextInt();
		
		try{
			serialPorts.elementAt(response);
		} catch (ArrayIndexOutOfBoundsException e) {
			System.out.println("You did not enter a usable Serial Port.");
		}
				
/*		String selectedPort = (String)window.cboxPorts.getSelectedItem();
*/		String selectedPort = (String)serialPorts.elementAt(response);
		
		selectedPortIdentifier = (CommPortIdentifier)portMap.get(selectedPort);
		
		CommPort commPort = null;
		
		try {
			//the method below returns an object of type CommPort
			commPort = selectedPortIdentifier.open("BackendDesign", TIMEOUT);
			//the CommPort object can be casted to a SerialPort object
			serialPort = (SerialPort)commPort;
			serialPort.setSerialPortParams(9600, SerialPort.DATABITS_8, SerialPort.STOPBITS_1, SerialPort.PARITY_NONE);
			
			//for controlling GUI elements
			setConnected(true);
			
			//logging
			logText = selectedPort + " opened successfully.";
			System.out.println(logText);
			/*window.txtLog.setForeground(Color.black);
			window.txtLog.append(logText + "n");
			
			//enables the controls on the GUI if a successful connection is made
			window.keybindingController.toggleControls();
			
			
			*/
			
			this.initIOStream();
			this.initListener();
			
		} catch (PortInUseException e){
			logText = selectedPort + " is in use. (" + e.toString() + ")";
			
			System.out.println(logText);
//			window.txtLog.setForeground(Color.RED);
//			window.txtLog.append(logText + "n");
		} catch (Exception e){
			logText = "Failed to open " + selectedPort + "(" + e.toString() + ")";
			
			System.out.println(logText);
//			window.txtLog.append(logText + "n");
	//		window.txtLog.setForeground(Color.RED);
		}
	}

	
	//open the input and output streams
	//pre style = "font-size: 11px;": an open port 
	//post:initialized input and output streams for use to communicate data
	public boolean initIOStream() {
		//return value for whether opening the streams is successful or not
		boolean successful = false;
		
		try{
			input = serialPort.getInputStream();
			output = serialPort.getOutputStream();
			//TO DO:::: WRITE DATA (0,0);
			successful = true;
			return successful;
		} catch (IOException e ) {
			logText = "I/O Streams failed to open. (" + e.toString() + ")";
			System.out.println(logText);
			
			return successful;
		}
		
	}
	
	
	//starts the event listener that knows whenever data is available to be read
	//pre style ="font-size: 11px;": an open serial port
	//post: an event listener for the serial port that knows when data is received
	public void serialEvent(SerialPortEvent evt) {
		
		if(evt.getEventType() == SerialPortEvent.DATA_AVAILABLE){
			try {				
				byte singleData = (byte)input.read();
				
				
				if(singleData != NEW_LINE_ASCII){
					logText = new String(new byte[] {singleData});
				}
				
				if(!handShake && logText.equals("~")){
					writeData('~');
					this.setHandShake(true);
				}
				
				if(logText.equals("~")){
					;
					lastResponse = "";
				} else {
					if(logText.equals(";")){
						System.out.print("Incoming: ");
						System.out.println(lastResponse);
						lastResponse = "";
					} else if (handShake) {
						lastResponse+=logText;
					}
				}
			} catch (Exception e) {
				logText = "Failed to read data. (" + e.toString() + ")";
				System.out.println(logText);
				System.exit(-1);
			}
		}
	}
	
	
	//method that can be called to send data
	//pre style="font-size: 11px;": open serial port
	//post: data sent to the other device
	public void writeData(int message){
		try {
			output.write(message);
			output.flush();
		} catch (Exception e) {
			logText = "Failed to write data. (" + e.toString() + ")";
		}
	}
	
	
	public void initListener(){		
		try{
			serialPort.addEventListener(this);
			serialPort.notifyOnDataAvailable(true);
		} catch (TooManyListenersException e){
			logText = "Too many listeners. (" + e.toString() + ")";
			System.out.println(logText);
			//window.txtLog.setForeground(Color.red);
			//window.txtLog.append(logText + "n");
		}
	}
	
	//disconnect the serial port
	//pre style="font-size: 11px;": an open serial port
	//post: closed serial port
	public void disconnect(){
		//close the serial port
		try {
			writeData('C');
			
			serialPort.removeEventListener();
			serialPort.close();
			input.close();
			output.close();
			setConnected(false);
			//window.keybindingController.toggleControls();
			
			logText = "Disconnected.";
			//window.txtLog.setForeground(Color.red);
			//window.txtLog.append(logText + "n");
			System.out.println(logText);
		} catch (Exception e){
			logText = "Failed to close" + serialPort.getName() + "(" + e.toString() + ")";

			System.out.println(logText);
//			window.txtLog.setForeground(Color.red);
//			window.txtLog.append(logText + "n");
		}
	}

	
	/*private void btnConnectctionPerformed(java.awt.event.ActionEvent evt){
		communicator.connect();
		
		if(communicator.getConnected == true){
			if(communicator.initIOStream() == true){
				communicator.initListener();
			}
		}
	}
	*/
}
