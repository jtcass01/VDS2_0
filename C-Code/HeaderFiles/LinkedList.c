#include <stdio.h>
#include <stdlib.h>

typedef struct node {
    int val;
    struct node * next;
} node_t;

void print_list(node_t * head) {
    node_t * current = head;
    
    printf("\nPrinting list...\n");

    while (current != NULL) {
        printf("%d\n", current->val);
        current = current->next;
    }
}

int pop_top(node_t ** head) {
    int retval = -1;
    node_t * next_node = NULL;

    if (*head == NULL) {
    	return -1;
    }

    next_node = (*head)->next;
    retval = (*head)->val;
    free(*head);
    *head = next_node;

    return retval;
}

int pop_bottom(node_t ** head){
	
}

//int remove_by_value(node_t ** head, int val) {
    /* TODO: fill in your code here */
//}

void push(node_t ** head, int val) {
    node_t * new_node;
    new_node = malloc(sizeof(node_t));

    new_node->val = val;
    new_node->next = *head;
    *head = new_node;
}

int main() {
	int listSize = 0;
	const int altN = 10;           //Number of Data Points per alt array.
	int i = 0;
	node_t * tail;

    node_t * test_list = malloc(sizeof(node_t));
    
    for(i = 0; i < 20; i++){
	    if(listSize == 0){
	    	test_list->val = i;
	    	listSize++;
	    	tail = test_list; 
	    } else if (listSize > 0 && listSize < altN) {
		    push(&test_list, i);
	    	listSize++;
	    } else {
	    	
	    }
    }
    printf("%p", *tail);
    
/*    push(&test_list, 2);
    print_list(test_list);
    push(&test_list, 3);
    print_list(test_list);
    push(&test_list, 4);*/
//	push($test_list, NULL);
/*    test_list->val = 1;
    test_list->next = malloc(sizeof(node_t));
    test_list->next->val = 2;
    test_list->next->next = malloc(sizeof(node_t));
    test_list->next->next->val = 3;
    test_list->next->next->next = malloc(sizeof(node_t));
    test_list->next->next->next->val = 4;
    test_list->next->next->next->next = NULL;
*/
    //remove_by_value(&test_list, 3);

    print_list(test_list);

    pop_top(&test_list);
    pop_top(&test_list);
    print_list(test_list);

}
