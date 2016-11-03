import java.util.Vector;

public class BackendDesign {
	public static void main(String args[]){
		SerialCommunication test = new SerialCommunication();
		Vector serialPorts = new Vector();
		
		serialPorts = test.searchForPorts();
		
		for(int i = 0; i<serialPorts.size(); i++){
			System.out.println(i + ". " + serialPorts.elementAt(i));
		}
		
		test.connect();
		test.initListener();
	}
}
