/*
 * Created on Thu Jan 26 2017 
 *
 * Copyright (c) 2017 Thomas Northall-Little
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
 
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

