#include "defines.h"

#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

#define PID_PROPORTIONAL                1.0
#define PID_INTEGRAL                    0.0
#define PID_DERIVATIVE                  0.0
#define PID_WINDUPGUARD                 0.375


class PID {
    public:
        PID();
        ~PID();

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

