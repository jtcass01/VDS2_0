import java.util.Vector;

public class BackendDesign {
	public static void main(String args[]){
		SerialCommunication test = new SerialCommunication();
		Vector serialPorts = new Vector();
		
		serialPorts = test.searchForPorts();
		test.connect();

		
	}
}
