#ifndef CALCULATIONS_INTEFACE_H_56E2593E52DF46C588173A04C05D90FC
#define CALCULATIONS_INTEFACE_H_56E2593E52DF46C588173A04C05D90FC

#include "state_input_struct.h" //found in rocket-specific folders
/*Classes--------------------------------------------------------------*/

class ICalculator {
public:
    /**
     * @brief Performs all relevant calculations. Note that the Calculator class contains pointers to the sensors it uses that get passed in during construction (i.e. dependency injection).
     * @param StateId Current state.
     * @param out_state_input StateInput out-variable that gets filled in during calculation.
     */
    virtual void calculateValues(StateId state, StateInput &out_state_input, uint32_t t_ms) = 0;
};

#endif