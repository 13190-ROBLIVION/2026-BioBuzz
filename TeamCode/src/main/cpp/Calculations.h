#ifndef INC_2026_BIOBUZZ_CALCULATIONS_H
#define INC_2026_BIOBUZZ_CALCULATIONS_H


class Calculations {
    public:
        double RPMtoTPS(double RPM, double countsPerRev) { return ((RPM/60) * countsPerRev); }
        double TPStoRPM(double TPS, double countsPerRev) { return ((TPS / countsPerRev) * 60); }
};


#endif //INC_2026_BIOBUZZ_CALCULATIONS_H
