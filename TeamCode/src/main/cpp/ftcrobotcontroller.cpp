#include <jni.h>
#include "Calculations.h"

// JNI bridge: Java calls these, they delegate to the C++ Calculations class.
//
// Symbol names MUST match: Java_<package_with_underscores>_<Class>_<method>
// Java side: org.firstinspires.ftc.teamcode.ShootingSubsystem.Shooting
//            with `private static native double RPMtoTPS(double, double)` etc.
// static methods -> second param is jclass (jobject if you make them non-static).

extern "C"
JNIEXPORT jdouble JNICALL
Java_org_firstinspires_ftc_teamcode_ShootingSubsystem_Shooting_RPMtoTPS(
        JNIEnv* env, jclass clazz, jdouble RPM, jdouble countsPerRev) {
    return Calculations::RPMtoTPS(RPM, countsPerRev);
}

extern "C"
JNIEXPORT jdouble JNICALL
Java_org_firstinspires_ftc_teamcode_ShootingSubsystem_Shooting_TPStoRPM(
        JNIEnv* env, jclass clazz, jdouble TPS, jdouble countsPerRev) {
    return Calculations::TPStoRPM(TPS, countsPerRev);
}
