public class BackendDesign extends SerialCommunication {
	private SerialCommunication test;

	public BackendDesign(){
		test = new SerialCommunication();
		test.searchForPorts();
		test.connect();		
	}
	
	public static void main(String args[]){
		
	}
	
	public void wData(char newData){
		test.writeData(newData);
		test.writeData('T');
		test.writeData('R');
		test.writeData('A');
		test.writeData('S');
		test.writeData('H');
		}
	
	public boolean isConnected(){
		return test.getHandShake();
	}
}
