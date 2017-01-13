# CLM_framework_For_Android
CLM-framework (a.k.a Cambridge Face Tracker) is a framework for various Constrained Local Model based face tracking and landmark detection algorithms and their extensions/applications. Includes CLM-Z and CLNF. I moved CLM to Android platform, deleted DLib,tbb,Boost, only using embedded OpenCV.（Android Studio 2.2 or above required）

**This Project only includes armeabi-v7a**

##Deployment
* Cmake + Android Studio 2.2 or higher
* OpenCV for Android can be found [HERE](https://github.com/Martin20150405/OpenCV4AndroidWithCmake)

##Works
* moved to Android
* removed tbb（parallel for）
* removed Boost(File I/O)
* removed DLib（HOG SVM-struct based face detector）
* removed OpenCV Manager

* FPS: about 0.5-2,only using one CPU core.

##Preview
![ScreenShot](https://github.com/Martin20150405/HelloCV/tree/master/CLM_framework_For_Android/screenshots/S61128-223114.jpg)
