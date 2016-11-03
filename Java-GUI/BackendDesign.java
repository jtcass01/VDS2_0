import java.util.Vector;

public class BackendDesign {
	private SerialCommunication test;
	private Vector serialPorts;

	public BackendDesign(){
		test = new SerialCommunication();
		Vector serialPorts = new Vector();
		
		serialPorts = test.searchForPorts();
		test.connect();		
	}
	
	public static void main(String args[]){
		
	}
	
	public void wData(char newData){
		test.writeData(newData);
	}
	
	public boolean isConnected(){
		return test.getHandShake();
	}
}
