#include <stdio.h>

class Process_Machine {
public:
    Process_Machine();
    void transition_processMap();
    void transition_finishProcessingOfMap();

private:
    enum process_states {INIT, PROCESSING_MAP, PROCESSING_MAP_DONE};
    enum error_states {NONE, ESCAPE_STRATEGY};
    process_states processState;
    error_states errorState;

};
