
public class FrontEndPrototype {

	public static void main(String[] args){
		BackendDesign backend = new BackendDesign();
		
		while(!backend.isConnected()){
			System.out.println("YOLO");
			try {
				Thread.sleep(3000);
			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}

		}
		System.out.println("Writing B");
		backend.wData('B');
		backend.wData('B');
		backend.wData('B');
		backend.wData('B');
		backend.wData('B');
/*		System.out.println("Writing C");
		backend.wData('C');
		System.out.println("Writing D");
		backend.wData('D');
		System.out.println("Writing E");
		backend.wData('E');
		System.out.println("Writing F");
		backend.wData('F');
		System.out.println("Writing G");
		backend.wData('G');
		System.out.println("Writing H");
		backend.wData('H');*/
	}
}
