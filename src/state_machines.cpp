#include "state_machines.h"

//enum process_states {PROCESSING_MAP, PROCESSING_MAP_DONE};
//enum error_states {NONE, ESCAPE_STRATEGY};

Process_Machine::Process_Machine() {
    this->processState = INIT;
}
void Process_Machine::transition_processMap() {
    switch (processState) {
    case PROCESSING_MAP_DONE:
    case INIT: {
        printf("\tMap will be processed\n");
        processState = PROCESSING_MAP;

        processMap();

        break;
    }
    default: printf("\tMap will NOT be processed\n");
    }
}

void Process_Machine::transition_finishProcessingOfMap() {
    switch (processState) {
    case PROCESSING_MAP: processState = PROCESSING_MAP_DONE; break;
    }
}
