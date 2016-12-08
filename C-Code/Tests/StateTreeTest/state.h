
typedef struct s_state {
  unsigned short position;
  float time;
  float alt;
  float vel;
  float accel;
  struct s_state* leftChild;
  struct s_state* rightChild;
} state;

state* createState(int pos){
  state* pState = malloc(sizeof(state));
  pState->position = pos;
  pState->alt = 0;
  pState->vel = 0;
  pState->accel = 0;
}

void deleteState(state* pState){
  free(pState);
}
