#include <jni.h>
#include <string>
#include <opencv2/core/core.hpp>

extern "C" JNIEXPORT jstring

JNICALL
Java_com_handen_roadhelper_MainActivity_stringFromJNI(JNIEnv *env, jobject /* this */) {
    std::string hello = "Ky";
    return env->NewStringUTF(hello.c_str());
}
