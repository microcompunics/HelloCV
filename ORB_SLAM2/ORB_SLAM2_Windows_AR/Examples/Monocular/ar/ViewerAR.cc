/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#include "ViewerAR.h"


using namespace std;

const float eps = 1e-4;
void checkGlDieOnError()
{
    GLenum glError = glGetError();
    if( glError != GL_NO_ERROR ) {
        printf("OpenGL Error: %s (%d)\n", gluErrorString(glError),glError);
    }
}
cv::Mat ExpSO3(const float &x, const float &y, const float &z)
{
    cv::Mat I = cv::Mat::eye(3,3,CV_32F);
    const float d2 = x*x+y*y+z*z;
    const float d = sqrt(d2);
    cv::Mat W = (cv::Mat_<float>(3,3) << 0, -z, y,
                 z, 0, -x,
                 -y,  x, 0);
    if(d<eps)
        return (I + W + 0.5f*W*W);
    else
        return (I + W*sin(d)/d + W*W*(1.0f-cos(d))/d2);
}

cv::Mat ExpSO3(const cv::Mat &v)
{
    return ExpSO3(v.at<float>(0),v.at<float>(1),v.at<float>(2));
}

ViewerAR::ViewerAR(){}

void ViewerAR::glDrawColouredCube(GLfloat axis_min, GLfloat axis_max)
{
    const GLfloat l = axis_min;
    const GLfloat h = axis_max;

    const GLfloat verts[] = {
            l,l,h,  h,l,h,  l,h,h,  h,h,h,  // FRONT
            l,l,l,  l,h,l,  h,l,l,  h,h,l,  // BACK
            l,l,h,  l,h,h,  l,l,l,  l,h,l,  // LEFT
            h,l,l,  h,h,l,  h,l,h,  h,h,h,  // RIGHT
            l,h,h,  h,h,h,  l,h,l,  h,h,l,  // TOP
            l,l,h,  l,l,l,  h,l,h,  h,l,l   // BOTTOM
    };

    glVertexPointer(3, GL_FLOAT, 0, verts);
    glEnableClientState(GL_VERTEX_ARRAY);

    glColor4f(1.0f, 0.0f, 0.0f, 1.0f);
    glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
    glDrawArrays(GL_TRIANGLE_STRIP, 4, 4);

    glColor4f(0.0f, 1.0f, 0.0f, 1.0f);
    glDrawArrays(GL_TRIANGLE_STRIP, 8, 4);
    glDrawArrays(GL_TRIANGLE_STRIP, 12, 4);

    glColor4f(0.0f, 0.0f, 1.0f, 1.0f);
    glDrawArrays(GL_TRIANGLE_STRIP, 16, 4);
    glDrawArrays(GL_TRIANGLE_STRIP, 20, 4);

    glDisableClientState(GL_VERTEX_ARRAY);
}

void drawMat(void)
{
    glFlush();
}

void ViewerAR::initViewMatrix(int w, int h, double fu, double fv, double u0, double v0, double zNear, double zFar ){
    // http://www.songho.ca/opengl/gl_projectionmatrix.html
    const double L = -(u0) * zNear / fu;
    const double R = +(w-u0) * zNear / fu;
    const double T = -(v0) * zNear / fv;
    const double B = +(h-v0) * zNear / fv;

    std::fill_n(P,4*4,0);

    P[0*4+0] = 2 * zNear / (R-L);
    P[1*4+1] = 2 * zNear / (T-B);
    P[2*4+0] = (R+L)/(L-R);
    P[2*4+1] = (T+B)/(B-T);
    P[2*4+2] = (zFar +zNear) / (zFar - zNear);
    P[2*4+3] = 1.0;
    P[3*4+2] =  (2*zFar*zNear)/(zNear - zFar);
    glLoadMatrixf(P);
}



void ViewerAR::reinitialise(GLint w, GLint h, GLint int_format, bool sampling_linear, int border, GLenum glformat, GLenum gltype, GLvoid* data )
{
    if(tid!=0) {
        glDeleteTextures(1,&tid);
    }

    internal_format = int_format;
    width = w;
    height = h;

    glEnable(GL_TEXTURE_2D);
    glGenTextures(1,&tid);
    glBindTexture(GL_TEXTURE_2D, tid);
    puts("glBindTexture");
    checkGlDieOnError();
    // GL_LUMINANCE and GL_FLOAT don't seem to actually affect buffer, but some values are required
    // for call to succeed.
    glTexImage2D(GL_TEXTURE_2D, 0, internal_format, width, height, border, glformat, gltype, data);
    puts("glTexImage2D");
    checkGlDieOnError();
    glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR );
    glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR );
//    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
//    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
    puts("glTexParameterf");
    checkGlDieOnError();
}

void ViewerAR::initGLMat(GLint width, GLint height, GLint internal_format, bool sampling_linear, int border, GLenum glformat, GLenum gltype, GLvoid* data){
    internal_format=0, tid=0;
    reinitialise(width,height,GL_RGB,sampling_linear,border,GL_RGB,GL_UNSIGNED_BYTE,data);
}
void ViewerAR::Run()
{

    int w,h;

    cv::Mat im, Tcw;
    int status;
    vector<cv::KeyPoint> vKeys;
    vector<ORB_SLAM2::MapPoint*> vMPs;

    drawPoints= true;
    while(1)
    {
        GetImagePose(im,Tcw,status,vKeys,vMPs);
        if(im.empty())
            cv::waitKey(mT);
        else
        {
            w = im.cols;
            h = im.rows;
            break;
        }
    }

    glutInitWindowSize(640,480);
    glutInitWindowPosition(40,40);
    int c=0; char **s;
    glutInit(&c,s);
    glutCreateWindow("Slam AR");

    glEnable(GL_DEPTH_TEST);
    glEnable (GL_BLEND);
    initGLMat(w,h,GL_RGB,false,0,GL_RGB,GL_UNSIGNED_BYTE);

    glutDisplayFunc(drawMat);

    vector<Plane*> vpPlane;
    int cnt=0;

    while(1){
        glutMainLoopEvent();
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        glColor3f(1.0,1.0,1.0);
        // Get last image and its computed pose from SLAM
        GetImagePose(im,Tcw,status,vKeys,vMPs);

        // Add text to image
        PrintStatus(status,false,im);

        if(drawPoints)
            DrawTrackedPoints(vKeys,vMPs,im);

        //cv::imshow("Mono SLAM",im);
        // Draw image
        DrawImageTexture(im);
        //cv::waitKey(1);
        glClear(GL_DEPTH_BUFFER_BIT);

        // Load camera projection
        glMatrixMode(GL_PROJECTION);
        initViewMatrix(w,h,fx,fy,cx,cy,0.001,1000);
        glMatrixMode(GL_MODELVIEW);

        // Load camera pose
        LoadCameraPose(Tcw);

        // Draw virtual things
        if(status==2)
        {
            if((cnt++)%360==1)
            {
                cout<<cnt<<endl;
                Plane* pPlane = DetectPlane(Tcw,vMPs,50);
                if(pPlane)
                {
                    cout << "New virtual cube inserted!" << endl;
                    vpPlane.push_back(pPlane);
                }
                else
                {
                    cout << "No plane detected. Point the camera to a planar region." << endl;
                }
            }

            if(!vpPlane.empty())
            {
                // Recompute plane if there has been a loop closure or global BA
                // In localization mode, map is not updated so we do not need to recompute
                bool bRecompute = false;
                if(mpSystem->MapChanged())
                {
                    cout << "Map changed. All virtual elements are recomputed!" << endl;
                    bRecompute = true;
                }

                for(size_t i=0; i<vpPlane.size(); i++)
                {
                    Plane* pPlane = vpPlane[i];

                    if(pPlane)
                    {
                        if(bRecompute)
                        {
                            pPlane->Recompute();
                        }
                        glPushMatrix();
                        glMultMatrixf(pPlane->glTpw);

                        // Draw cube
                        DrawCube(0.05);

                        // Draw grid plane
                        //menu_ngrid,menu_sizegrid
                        DrawPlane(3,0.05);

                        glPopMatrix();
                    }
                }
            }


        }


        glutIdleFunc(drawMat);
        glutPostRedisplay();
    }
}

void ViewerAR::SetImagePose(const cv::Mat &im, const cv::Mat &Tcw, const int &status, const vector<cv::KeyPoint> &vKeys, const vector<ORB_SLAM2::MapPoint*> &vMPs)
{
    unique_lock<mutex> lock(mMutexPoseImage);
    mImage = im.clone();
    mTcw = Tcw.clone();
    mStatus = status;
    mvKeys = vKeys;
    mvMPs = vMPs;
}

void ViewerAR::GetImagePose(cv::Mat &im, cv::Mat &Tcw, int &status, std::vector<cv::KeyPoint> &vKeys,  std::vector<ORB_SLAM2::MapPoint*> &vMPs)
{
    unique_lock<mutex> lock(mMutexPoseImage);
    im = mImage.clone();
    Tcw = mTcw.clone();
    status = mStatus;
    vKeys = mvKeys;
    vMPs = mvMPs;
}

void ViewerAR::LoadCameraPose(const cv::Mat &Tcw)
{
    if(!Tcw.empty())
    {
        camPose[0] = Tcw.at<float>(0,0);
        camPose[1] = Tcw.at<float>(1,0);
        camPose[2] = Tcw.at<float>(2,0);
        camPose[3]  = 0.0;
        camPose[4] = Tcw.at<float>(0,1);
        camPose[5] = Tcw.at<float>(1,1);
        camPose[6] = Tcw.at<float>(2,1);
        camPose[7]  = 0.0;
        camPose[8] = Tcw.at<float>(0,2);
        camPose[9] = Tcw.at<float>(1,2);
        camPose[10] = Tcw.at<float>(2,2);
        camPose[11]  = 0.0;
        camPose[12] = Tcw.at<float>(0,3);
        camPose[13] = Tcw.at<float>(1,3);
        camPose[14] = Tcw.at<float>(2,3);
        camPose[15]  = 1.0;

        glLoadMatrixf(camPose);
    }
}

void ViewerAR::PrintStatus(const int &status, const bool &bLocMode, cv::Mat &im)
{
    if(!bLocMode)
    {
        switch(status)
        {
        case 1:  {AddTextToImage("SLAM NOT INITIALIZED",im,255,0,0); break;}
        case 2:  {AddTextToImage("SLAM ON",im,0,255,0); break;}
        case 3:  {AddTextToImage("SLAM LOST",im,255,0,0); break;}
        }
    }
    else
    {
        switch(status)
        {
        case 1:  {AddTextToImage("SLAM NOT INITIALIZED",im,255,0,0); break;}
        case 2:  {AddTextToImage("LOCALIZATION ON",im,0,255,0); break;}
        case 3:  {AddTextToImage("LOCALIZATION LOST",im,255,0,0); break;}
        }
    }
}

void ViewerAR::AddTextToImage(const string &s, cv::Mat &im, const int r, const int g, const int b)
{
    int l = 10;
    //imText.rowRange(im.rows-imText.rows,imText.rows) = cv::Mat::zeros(textSize.height+10,im.cols,im.type());
    cv::putText(im,s,cv::Point(l,im.rows-l),cv::FONT_HERSHEY_PLAIN,1.5,cv::Scalar(255,255,255),2,8);
    cv::putText(im,s,cv::Point(l-1,im.rows-l),cv::FONT_HERSHEY_PLAIN,1.5,cv::Scalar(255,255,255),2,8);
    cv::putText(im,s,cv::Point(l+1,im.rows-l),cv::FONT_HERSHEY_PLAIN,1.5,cv::Scalar(255,255,255),2,8);
    cv::putText(im,s,cv::Point(l-1,im.rows-(l-1)),cv::FONT_HERSHEY_PLAIN,1.5,cv::Scalar(255,255,255),2,8);
    cv::putText(im,s,cv::Point(l,im.rows-(l-1)),cv::FONT_HERSHEY_PLAIN,1.5,cv::Scalar(255,255,255),2,8);
    cv::putText(im,s,cv::Point(l+1,im.rows-(l-1)),cv::FONT_HERSHEY_PLAIN,1.5,cv::Scalar(255,255,255),2,8);
    cv::putText(im,s,cv::Point(l-1,im.rows-(l+1)),cv::FONT_HERSHEY_PLAIN,1.5,cv::Scalar(255,255,255),2,8);
    cv::putText(im,s,cv::Point(l,im.rows-(l+1)),cv::FONT_HERSHEY_PLAIN,1.5,cv::Scalar(255,255,255),2,8);
    cv::putText(im,s,cv::Point(l+1,im.rows-(l+1)),cv::FONT_HERSHEY_PLAIN,1.5,cv::Scalar(255,255,255),2,8);

    cv::putText(im,s,cv::Point(l,im.rows-l),cv::FONT_HERSHEY_PLAIN,1.5,cv::Scalar(r,g,b),2,8);
}

void ViewerAR::upload(const uchar* data,
        GLenum data_format, GLenum data_type) {
    glBindTexture(GL_TEXTURE_2D, tid);
    glTexSubImage2D(GL_TEXTURE_2D,0,0,0,width,height,data_format,data_type,data);
    checkGlDieOnError();
    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();

    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadIdentity();
    GLfloat sq_vert[] = { -1,-1,0,  1,-1,0,  1, 1,0,  -1, 1,0 };
    glVertexPointer(3, GL_FLOAT, 0, sq_vert);
    glEnableClientState(GL_VERTEX_ARRAY);

    GLfloat sq_tex[]  = { 0,1,  1,1,  1,0,  0,0  };
    glTexCoordPointer(2, GL_FLOAT, 0, sq_tex);
    glEnableClientState(GL_TEXTURE_COORD_ARRAY);

    glEnable(GL_TEXTURE_2D);
    glBindTexture(GL_TEXTURE_2D, tid);

    glDrawArrays(GL_TRIANGLE_FAN, 0, 4);

    glDisableClientState(GL_VERTEX_ARRAY);
    glDisableClientState(GL_TEXTURE_COORD_ARRAY);

    glDisable(GL_TEXTURE_2D);

    glMatrixMode(GL_PROJECTION);
    glPopMatrix();
    glMatrixMode(GL_MODELVIEW);
    glPopMatrix();

}

void ViewerAR::DrawImageTexture(cv::Mat &im)
{
    if(!im.empty())
    {
        glViewport(0,0,width,height);
        upload(im.data,GL_RGB,GL_UNSIGNED_BYTE);
    }
}

void setIdentity(float m[])
{
    m[0] = 1.0f;  m[1] = 0.0f;  m[2] = 0.0f;  m[3] = 0.0f;
    m[4] = 0.0f;  m[5] = 1.0f;  m[6] = 0.0f;  m[7] = 0.0f;
    m[8] = 0.0f;  m[9] = 0.0f; m[10] = 1.0f; m[11] = 0.0f;
    m[12] = 0.0f; m[13] = 0.0f; m[14] = 0.0f; m[15] = 1.0f;
}

void ViewerAR::DrawCube(const float &size,const float x, const float y, const float z)
{
    glPushMatrix();
    float M[16];
    setIdentity(M);
    M[12] = -x;
    M[13] = -size-y;
    M[14] = -z;
    glMultMatrixf(M);
    glDrawColouredCube(-size,size);
    glPopMatrix();
}

void ViewerAR::DrawPlane(Plane *pPlane, int ndivs, float ndivsize)
{
    glPushMatrix();
    glMultMatrixf(pPlane->glTpw);
    DrawPlane(ndivs,ndivsize);
    glPopMatrix();
}

void ViewerAR::DrawPlane(int ndivs, float ndivsize)
{
    // Plane parallel to x-z at origin with normal -y
    const float minx = -ndivs*ndivsize;
    const float minz = -ndivs*ndivsize;
    const float maxx = ndivs*ndivsize;
    const float maxz = ndivs*ndivsize;


    glLineWidth(2);
    glColor3f(0.7f,0.7f,1.0f);
    glBegin(GL_LINES);

    for(int n = 0; n<=2*ndivs; n++)
    {
        glVertex3f(minx+ndivsize*n,0,minz);
        glVertex3f(minx+ndivsize*n,0,maxz);
        glVertex3f(minx,0,minz+ndivsize*n);
        glVertex3f(maxx,0,minz+ndivsize*n);
    }

    glEnd();

}

void ViewerAR::DrawTrackedPoints(const std::vector<cv::KeyPoint> &vKeys, const std::vector<ORB_SLAM2::MapPoint *> &vMPs, cv::Mat &im)
{
    const int N = vKeys.size();
    for(int i=0; i<N; i++)
    {
        if(vMPs[i])
        {
            cv::circle(im,vKeys[i].pt,2,cv::Scalar(0,255,255),-1);
        }
    }
}

Plane* ViewerAR::DetectPlane(const cv::Mat Tcw, const std::vector<ORB_SLAM2::MapPoint*> &vMPs, const int iterations)
{
    // Retrieve 3D points
    vector<cv::Mat> vPoints;
    vPoints.reserve(vMPs.size());
    vector<ORB_SLAM2::MapPoint*> vPointMP;
    vPointMP.reserve(vMPs.size());

    for(size_t i=0; i<vMPs.size(); i++)
    {
        ORB_SLAM2::MapPoint* pMP=vMPs[i];
        if(pMP)
        {
            if(pMP->Observations()>5)
            {
                vPoints.push_back(pMP->GetWorldPos());
                vPointMP.push_back(pMP);
            }
        }
    }

    const int N = vPoints.size();

    if(N<50)
        return NULL;


    // Indices for minimum set selection
    vector<size_t> vAllIndices;
    vAllIndices.reserve(N);
    vector<size_t> vAvailableIndices;

    for(int i=0; i<N; i++)
    {
        vAllIndices.push_back(i);
    }

    float bestDist = 1e10;
    vector<float> bestvDist;

    //RANSAC
    for(int n=0; n<iterations; n++)
    {
        vAvailableIndices = vAllIndices;

        cv::Mat A(3,4,CV_32F);
        A.col(3) = cv::Mat::ones(3,1,CV_32F);

        // Get min set of points
        for(short i = 0; i < 3; ++i)
        {
            int randi = DUtils::Random::RandomInt(0, vAvailableIndices.size()-1);

            int idx = vAvailableIndices[randi];

            A.row(i).colRange(0,3) = vPoints[idx].t();

            vAvailableIndices[randi] = vAvailableIndices.back();
            vAvailableIndices.pop_back();
        }

        cv::Mat u,w,vt;
        cv::SVDecomp(A,w,u,vt,cv::SVD::MODIFY_A | cv::SVD::FULL_UV);

        const float a = vt.at<float>(3,0);
        const float b = vt.at<float>(3,1);
        const float c = vt.at<float>(3,2);
        const float d = vt.at<float>(3,3);

        vector<float> vDistances(N,0);

        const float f = 1.0f/sqrt(a*a+b*b+c*c+d*d);

        for(int i=0; i<N; i++)
        {
            vDistances[i] = fabs(vPoints[i].at<float>(0)*a+vPoints[i].at<float>(1)*b+vPoints[i].at<float>(2)*c+d)*f;
        }

        vector<float> vSorted = vDistances;
        sort(vSorted.begin(),vSorted.end());

        int nth = max((int)(0.2*N),20);
        const float medianDist = vSorted[nth];

        if(medianDist<bestDist)
        {
            bestDist = medianDist;
            bestvDist = vDistances;
        }
    }

    // Compute threshold inlier/outlier
    const float th = 1.4*bestDist;
    vector<bool> vbInliers(N,false);
    int nInliers = 0;
    for(int i=0; i<N; i++)
    {
        if(bestvDist[i]<th)
        {
            nInliers++;
            vbInliers[i]=true;
        }
    }

    vector<ORB_SLAM2::MapPoint*> vInlierMPs(nInliers,NULL);
    int nin = 0;
    for(int i=0; i<N; i++)
    {
        if(vbInliers[i])
        {
            vInlierMPs[nin] = vPointMP[i];
            nin++;
        }
    }

    return new Plane(vInlierMPs,Tcw);
}

Plane::Plane(const std::vector<ORB_SLAM2::MapPoint *> &vMPs, const cv::Mat &Tcw):mvMPs(vMPs),mTcw(Tcw.clone())
{
    rang = -3.14f/2+((float)rand()/RAND_MAX)*3.14f;
    Recompute();
}

void Plane::Recompute()
{
    const int N = mvMPs.size();

    // Recompute plane with all points
    cv::Mat A = cv::Mat(N,4,CV_32F);
    A.col(3) = cv::Mat::ones(N,1,CV_32F);

    o = cv::Mat::zeros(3,1,CV_32F);

    int nPoints = 0;
    for(int i=0; i<N; i++)
    {
        ORB_SLAM2::MapPoint* pMP = mvMPs[i];
        if(!pMP->isBad())
        {
            cv::Mat Xw = pMP->GetWorldPos();
            o+=Xw;
            A.row(nPoints).colRange(0,3) = Xw.t();
            nPoints++;
        }
    }
    A.resize(nPoints);

    cv::Mat u,w,vt;
    cv::SVDecomp(A,w,u,vt,cv::SVD::MODIFY_A | cv::SVD::FULL_UV);

    float a = vt.at<float>(3,0);
    float b = vt.at<float>(3,1);
    float c = vt.at<float>(3,2);

    o = o*(1.0f/nPoints);
    const float f = 1.0f/sqrt(a*a+b*b+c*c);

    // Compute XC just the first time
    if(XC.empty())
    {
        cv::Mat Oc = -mTcw.colRange(0,3).rowRange(0,3).t()*mTcw.rowRange(0,3).col(3);
        XC = Oc-o;
    }

    if((XC.at<float>(0)*a+XC.at<float>(1)*b+XC.at<float>(2)*c)>0)
    {
        a=-a;
        b=-b;
        c=-c;
    }

    const float nx = a*f;
    const float ny = b*f;
    const float nz = c*f;

    n = (cv::Mat_<float>(3,1)<<nx,ny,nz);

    cv::Mat up = (cv::Mat_<float>(3,1) << 0.0f, 1.0f, 0.0f);

    cv::Mat v = up.cross(n);
    const float sa = cv::norm(v);
    const float ca = up.dot(n);
    const float ang = atan2(sa,ca);
    Tpw = cv::Mat::eye(4,4,CV_32F);


    Tpw.rowRange(0,3).colRange(0,3) = ExpSO3(v*ang/sa)*ExpSO3(up*rang);
    o.copyTo(Tpw.col(3).rowRange(0,3));

    glTpw[0] = Tpw.at<float>(0,0);
    glTpw[1] = Tpw.at<float>(1,0);
    glTpw[2] = Tpw.at<float>(2,0);
    glTpw[3]  = 0.0;
    glTpw[4] = Tpw.at<float>(0,1);
    glTpw[5] = Tpw.at<float>(1,1);
    glTpw[6] = Tpw.at<float>(2,1);
    glTpw[7]  = 0.0;
    glTpw[8] = Tpw.at<float>(0,2);
    glTpw[9] = Tpw.at<float>(1,2);
    glTpw[10] = Tpw.at<float>(2,2);
    glTpw[11]  = 0.0;
    glTpw[12] = Tpw.at<float>(0,3);
    glTpw[13] = Tpw.at<float>(1,3);
    glTpw[14] = Tpw.at<float>(2,3);
    glTpw[15]  = 1.0;

}

Plane::Plane(const float &nx, const float &ny, const float &nz, const float &ox, const float &oy, const float &oz)
{
    n = (cv::Mat_<float>(3,1)<<nx,ny,nz);
    o = (cv::Mat_<float>(3,1)<<ox,oy,oz);

    cv::Mat up = (cv::Mat_<float>(3,1) << 0.0f, 1.0f, 0.0f);

    cv::Mat v = up.cross(n);
    const float s = cv::norm(v);
    const float c = up.dot(n);
    const float a = atan2(s,c);
    Tpw = cv::Mat::eye(4,4,CV_32F);
    const float rang = -3.14f/2+((float)rand()/RAND_MAX)*3.14f;
    cout << rang;
    Tpw.rowRange(0,3).colRange(0,3) = ExpSO3(v*a/s)*ExpSO3(up*rang);
    o.copyTo(Tpw.col(3).rowRange(0,3));

    glTpw[0] = Tpw.at<float>(0,0);
    glTpw[1] = Tpw.at<float>(1,0);
    glTpw[2] = Tpw.at<float>(2,0);
    glTpw[3]  = 0.0;
    glTpw[4] = Tpw.at<float>(0,1);
    glTpw[5] = Tpw.at<float>(1,1);
    glTpw[6] = Tpw.at<float>(2,1);
    glTpw[7]  = 0.0;
    glTpw[8] = Tpw.at<float>(0,2);
    glTpw[9] = Tpw.at<float>(1,2);
    glTpw[10] = Tpw.at<float>(2,2);
    glTpw[11]  = 0.0;
    glTpw[12] = Tpw.at<float>(0,3);
    glTpw[13] = Tpw.at<float>(1,3);
    glTpw[14] = Tpw.at<float>(2,3);
    glTpw[15]  = 1.0;
}
