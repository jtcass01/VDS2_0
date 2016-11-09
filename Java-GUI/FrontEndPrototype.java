import java.util.Scanner;
import java.util.InputMismatchException;

public class FrontEndPrototype {

	public static void main(String[] args){
		BackendDesign backend = new BackendDesign();
		Scanner scan = new Scanner(System.in);
		int nextFunction = 0;
		boolean cont = true;
		boolean validResponse = false;
		
		while(!backend.isConnected()){
			System.out.println("Disconnected.");
			try {
				Thread.sleep(3000);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}

		}
		
		while(cont){
			while(!validResponse){
				printMenu();
				try{
					nextFunction = scan.nextInt();
					validResponse = true;
				} catch (InputMismatchException e){
					System.out.println("You did not input an integer.  Please try again.");
				}
			}
			validResponse = false;
			
			switch(nextFunction){
			case 1:
				System.out.println("Writing B");
				backend.wData('B');
				break;
			case 2:
				System.out.println("Writing E");
				backend.wData('E');
				break;
			case 3:
				System.out.println("Get a current altitude reading");
				backend.wData('P');
				break;
			case 4:
				System.out.println("Get a current acceleration reading");
				backend.wData('S');
			case 5:
				backend.disconnect();
				cont = false;
				System.out.println("\nGoodbye : )");
				break;
			default:
				System.out.println("Integer response is not located on menu.  Please try again.");
				break;
			}
			try {
				Thread.sleep(500);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
		}
	}
	
	public static void printMenu(){
		System.out.println("==========MENU==========");
		System.out.println("1 ) write B");
		System.out.println("2 ) write E");
		System.out.println("3 ) Get Latest Pressure Reading");
		System.out.println("4 ) Get Latest Acceleration Reading");
		System.out.println("5 ) disconnect");
		System.out.print("Enter selction: ");
	}
}
