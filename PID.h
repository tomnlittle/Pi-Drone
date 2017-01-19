#include "defines.h"

#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))




class PID {
    public:
        PID();
        ~PID();

        void init(double, double, double, double);

        void updatePID(double target, double current);
        double getPID();
        
    private:
        void PrintValues();

        double P,I,D;
        clock_t previousPIDTime; // the last time the PID was updated
        clock_t current_time;
        double integratedError;
        double lastError;
        double windupGuard;

        double PIDValue;
};

