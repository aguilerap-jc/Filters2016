/* Copyright
 * Copyright (c) 2012-2014 Aldebaran Robotics. All rights reserved.
 * Use of this source code is governed by a BSD-style license that can be
 * found in the COPYING file.
 */

/* Coautores
   Juan Carlos Aguilera Perez
   Jonathan
   Margarita Villarreal
   Ivan Cruz
*/

/* Aspectos de revision
 * 1) Ver la manera de sacar las funciones del circulo HSV
 * 2) Definir los objetos que se utilizaran para las pruebas
 * 3) Revisar los parametros para el cafe y el blanco
 * 4) Ver en funcionamiento los tres filtros
*/

/* Reglas
 * Materiales facilitados por el equipo
 * Metal        lata aluminio 355 ml rojo
 * Plastico     botella pet   255 ml blanco
 * papel        tetrapack     255 ml cafe
 *
 * Se seleccionara al azar el material a identificar y clasificar
 * Robot en marca de inicio
 * Se presentara en una ocasion el objeto a reconocer(MENCIONAR NOMBRE DE MANERA CLARA)
 * Si se acierta se coloca el objeto en la canasta (proporcionada por equipo) mano derecha del robot
 *Con la canasta en mano el robot se desplazara hacia las zonas de clasificacion
 * Se utilizara medio de activacion que equipo desee
 * El robot podra identificar cesto por medio de Nao Mark
 *          Metal     64
 *          Plastico  80
 *          Papel     108
 * Se considerara clasificado cuando el robot ponga sus dos pies dentro de la zona frente a al cesto simbolico
 * Tiempo de etapa de clasificacion sera de 2 mins
 *
 * --------PRINCIPAL CRITERIO DE EVALUACION--------
 *  Tiempo de ejecucuion sera el principal criterio a evaluar
*/

//------------------OpenCV--------------------------
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <alvision/alimage.h>
//----------------------NAO-------------------------
#include <alproxies/almotionproxy.h>
#include <alproxies/alrobotpostureproxy.h>
#include <alproxies/alvideodeviceproxy.h>
#include <alvision/alvisiondefinitions.h>
//-------------------Nao Marks----------------------
#include <alproxies/allandmarkdetectionproxy.h>
#include <alproxies/altexttospeechproxy.h>
#include <alproxies/almemoryproxy.h>
//-------------------Nao Marks----------------------
//----------------------NAO-------------------------
#include <iostream>
//#include "NaoVision.h"
//#include "NaoMovement.h"
//void buscarNaomark(AL::ALMotionProxy movimiento, AL::ALMemoryProxy memoria, AL::ALLandMarkDetectionProxy naoMark, AL::ALTextToSpeechProxy say)

using namespace std;
using namespace AL;
using namespace cv;
//-----------------Filter Variables-------------------
double areaColorDetectionR;
double areaColorDetectionW;
double areaColorDetectionB;
//-----------------RED FILTER----------------
int RiLowH = 160;
int RiHighH = 179;
int RiLowS = 100;
int RiHighS = 255;
int RiLowV = 100;
int RiHighV = 255;
//-----------------RED FILTER----------------

//---------------WHITE FILTER----------------
int WiLowH = 89;        // 15
int WiHighH = 179;      // 179
int WiLowS = 0;         // 0
int WiHighS = 72;      // 255
int WiLowV = 115;       // 255
int WiHighV = 252;      // 255
//---------------WHITE FILTER----------------

//-------------------BROWN FILTER---------------------
                    //Claro   //obscuro        //Azul
int BiLowH = 118;   //15      //015             121
int BiHighH = 156; //179     //179             179
int BiLowS = 0;    //181     //255              0
int BiHighS = 255; //255     //255             255
int BiLowV = 26;   //90      //060              0
int BiHighV = 61; //255     //255             255
//-----------------BROWN FILTER----------------------

//-------------End of Filter Variables---------------

//-------------VARIABLES PRUEBA----------------------
#define HUEMAX 179
#define SATMAX 255
#define VALMAX 255

Mat HSV;
int H =170;
int S=200;
int V =200;
int R=0;
int G=0;
int B=0;

int MAX_H=179;
int MAX_S=255;
int MAX_V=255;
int mouse_x=0;
int mouse_y=0;
char window_name[20] = "HSV Color Plot";

//Global variable for hsv color wheel plot
int max_hue_range=179;
int max_step=3; //nuber of pixel for each hue color
int wheel_width=max_hue_range*max_step;
int wheel_hight=50;
int wheel_x=50; //x-position of wheel
int wheel_y=5;//y-position of wheel

//Global variable plot for satuarion-value plot
int S_V_Width=MAX_S;
int S_V_Height=MAX_S;
int S_V_x=10;
int S_V_y=wheel_y+wheel_hight+20;

//Global variable for HSV ploat
int HSV_Width=150;
int HSV_Height=150;
int HSV_x=S_V_x+S_V_Width+30;
int HSV_y=S_V_y+50;
//-------------VARIABLES PRUEBA-------------------

//----------FUNCIONES-----------------
void RedFilter(Mat originalImage,   bool calibrateR);
void WhiteFilter(Mat originalImage, bool calibrateW);
void BrownFilter(Mat originalImage, bool calibrateB);
//--------PRUEBAS---------------------
void HSVColorPlot();
static void onMouse( int event, int x, int y, int f, void* );
void onTrackbar_changed(int, void*);
void drawPointers();
//--------PRUEBAS--------------------

int main(int argc, char *argv[]) {
    //const int port = 9559;
    //string ip = argv[1];        // NAO ip
    //cout << "IP: " << ip << endl;

    //-----Calibration bools ---------------
    bool calibrateB = true;
    bool calibrateR = true;
    bool calibrateW = true;
    //-----Calibration bools ---------------

    bool LOCAL = false;         // Flap for the kind of execution (local or remote).
    bool NAO = false;
    bool finish = false;
    char key = 'x';
    //double angleToBlackLine;    // Angle of the detected line.

    Mat src;
    //NaoVision naoVision(ip, port, LOCAL);
    //NaoMovement naoMovement(ip, port, LOCAL);
    VideoCapture cap(0);        // Class for video capturing from video files or cameras.

    while(key != 27){
        if(!cap.isOpened())
            return -1;

        Mat frame;
        cap >> frame;
        //HSVColorPlot();
        //RedFilter(frame,calibrateR);
        BrownFilter(frame,calibrateB);
        //WhiteFilter(frame,calibrateW);
        //imshow("img",frame);
        if(areaColorDetectionR > 25)
             cout << "Metal " << areaColorDetectionR <<endl;
        if(areaColorDetectionB > 25)
              cout << "Carton " << endl;
        if(areaColorDetectionW > 25)
              cout << "Plastico " << areaColorDetectionW << endl;
        if (waitKey(3)== 'x')
            break;
   }


    return 0;
}

//----------------------COLOR FILTERS-------------------
void RedFilter(Mat originalImage  ,bool calibrateR) {
    Mat src_gray;
    Mat imgHSV;
    Mat imgThresholded;

    cvtColor(originalImage, imgHSV, COLOR_BGR2HSV);       // Convert the captured frame from BGR to HSV.
    inRange(imgHSV, Scalar(RiLowH, RiLowS, RiLowV), Scalar(RiHighH, RiHighS, RiHighV), imgThresholded); // Threshold the image.

    // Morphological opening (remove small objects from the foreground).
    erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
    dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );

    // Morphological closing (fill small holes in the foreground).
    dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
    erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );

    // Get the moments.
    Moments oMoments = moments(imgThresholded);

    // Receive the centroid area.
    double dArea = oMoments.m00;
    areaColorDetectionR = dArea / 100000;

    // Cloned the modified image to calculate the points.
    src_gray = imgThresholded.clone();

    //if (!local) {
        imshow("RThresholded Image", imgThresholded);      // Show the thresholded image.
        //imshow("ROriginal", originalImage);                // Show the original image.
    //}
    if(calibrateR){
       //namedWindow("Control", CV_WINDOW_AUTOSIZE);         // Create a window called "Control".
       // Create trackbars in "Control" window.
       cvCreateTrackbar("LowH" , "RThresholded Image", &RiLowH, 179);   // Hue (0 - 179).
       cvCreateTrackbar("HighH", "RThresholded Image", &RiHighH, 179);
       cvCreateTrackbar("LowS" , "RThresholded Image", &RiLowS, 255);   // Saturation (0 - 255).
       cvCreateTrackbar("HighS", "RThresholded Image", &RiHighS, 255);
       cvCreateTrackbar("LowV" , "RThresholded Image", &RiLowV, 255);   // Value (0 - 255).
       cvCreateTrackbar("HighV", "RThresholded Image", &RiHighV, 255);
    }

    // Blur to soften the image points.
    blur(src_gray, src_gray, Size(3,3));
}

void WhiteFilter(Mat originalImage,bool calibrateW){
    Mat src_gray;
    Mat imgHSV;
    Mat imgThresholded;

    cvtColor(originalImage, imgHSV, COLOR_BGR2HSV);       // Convert the captured frame from BGR to HSV.
    inRange(imgHSV, Scalar(WiLowH, WiLowS, WiLowV), Scalar(WiHighH, WiHighS, WiHighV), imgThresholded); // Threshold the image.

    // Morphological opening (remove small objects from the foreground).
    erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
    dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );

    // Morphological closing (fill small holes in the foreground).
    dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
    erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );

    // Get the moments.
    Moments oMoments = moments(imgThresholded);

    // Receive the centroid area.
    double dArea = oMoments.m00;
    areaColorDetectionW = dArea / 100000;

    // Cloned the modified image to calculate the points.
    src_gray = imgThresholded.clone();

    //if (!local) {
        imshow("WThresholded Image", imgThresholded);      // Show the thresholded image.
        //imshow("WOriginal", originalImage);                // Show the original image.
    //}

     if(calibrateW){
        //namedWindow("Control", CV_WINDOW_AUTOSIZE);         // Create a window called "Control".
        // Create trackbars in "Control" window.
        cvCreateTrackbar("LowH" , "WThresholded Image", &WiLowH, 179);   // Hue (0 - 179).
        cvCreateTrackbar("HighH", "WThresholded Image", &WiHighH, 179);
        cvCreateTrackbar("LowS" , "WThresholded Image", &WiLowS, 255);   // Saturation (0 - 255).
        cvCreateTrackbar("HighS", "WThresholded Image", &WiHighS, 255);
        cvCreateTrackbar("LowV" , "WThresholded Image", &WiLowV, 255);   // Value (0 - 255).
        cvCreateTrackbar("HighV", "WThresholded Image", &WiHighV, 255);
     }
    // Blur to soften the image points.
    blur(src_gray, src_gray, Size(3,3));
}

void BrownFilter(Mat originalImage,bool calibrateB){
    Mat src_gray;
    Mat imgHSV;
    Mat imgThresholded;

    cvtColor(originalImage, imgHSV, COLOR_BGR2HSV);       // Convert the captured frame from BGR to HSV.
    inRange(imgHSV, Scalar(BiLowH, BiLowS, BiLowV), Scalar(BiHighH, BiHighS, BiHighV), imgThresholded); // Threshold the image.

    // Morphological opening (remove small objects from the foreground).
    erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
    dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );

    // Morphological closing (fill small holes in the foreground).
    dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
    erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );

    // Get the moments.
    Moments oMoments = moments(imgThresholded);

    // Receive the centroid area.
    double dArea = oMoments.m00;
    areaColorDetectionB = dArea / 100000;

    // Cloned the modified image to calculate the points.
    src_gray = imgThresholded.clone();

    //if (!local) {
        imshow("BThresholded Image", imgThresholded);      // Show the thresholded image.
        //imshow("BOriginal", originalImage);                // Show the original image.
    //}
    if(calibrateB){
       //namedWindow("Control", CV_WINDOW_AUTOSIZE);         // Create a window called "Control".
       // Create trackbars in "Control" window.
       cvCreateTrackbar("LowH" , "BThresholded Image", &BiLowH, 179);   // Hue (0 - 179).
       cvCreateTrackbar("HighH", "BThresholded Image", &BiHighH, 179);
       cvCreateTrackbar("LowS" , "BThresholded Image", &BiLowS, 255);   // Saturation (0 - 255).
       cvCreateTrackbar("HighS", "BThresholded Image", &BiHighS, 255);
       cvCreateTrackbar("LowV" , "BThresholded Image", &BiLowV, 255);   // Value (0 - 255).
       cvCreateTrackbar("HighV", "BThresholded Image", &BiHighV, 255);
    }

    // Blur to soften the image points.
    blur(src_gray, src_gray, Size(3,3));
}

//----------------------COLOR FILTERS-------------------
//--------PRUEBAS--------------

void HSVColorPlot(){
    HSV.create(390,640,CV_8UC3); //Mat to store clock image
    HSV.setTo(Scalar(200,0,200));

    namedWindow(window_name);
    createTrackbar( "Hue",window_name, &H, HUEMAX, onTrackbar_changed );
    createTrackbar( "Saturation",window_name, &S, SATMAX,onTrackbar_changed );
    createTrackbar( "Value",window_name, &V, VALMAX,onTrackbar_changed);
    onTrackbar_changed(0,0); //initialoze window

    setMouseCallback( window_name, onMouse, 0 );
     while(true)
      {
        int c;
        c = waitKey( 20 );
        if( (char)c == 27 )
          { break; }
      }
}

static void onMouse( int event, int x, int y, int f, void* ){
    if(f&CV_EVENT_FLAG_LBUTTON){
        mouse_x=x;
        mouse_y=y;
        if(((wheel_x<=x)&&(x<=wheel_x+wheel_width))&&((wheel_y<=y)&&(y<=wheel_y+wheel_hight))){
            H=(x-wheel_x)/ max_step;
            cvSetTrackbarPos("Hue", window_name, H);
        }
        else if(((S_V_x<=x)&&(x<=S_V_x+S_V_Width))&&((S_V_y<=y)&&(y<=S_V_y+S_V_Height))){
            S=x-S_V_x;
            y=y-S_V_y;
            V=255-y;

            cvSetTrackbarPos("Saturation", window_name, S);
            cvSetTrackbarPos("Value", window_name, V);
        }
    }
}

void onTrackbar_changed(int, void*){
    //Plot color wheel.
    int hue_range=0;
    int step=1;
    for(int i=wheel_y;i<wheel_hight+wheel_y;i++){
        hue_range=0;
        for(int j=wheel_x;j<wheel_width+wheel_x;j++){
            if(hue_range>=max_hue_range) hue_range=0;
                if(step++==max_step){
                    hue_range++;
                    step=1;
                }
            Vec3b pix;
            pix.val[0]=hue_range;
            pix.val[1]=255;
            pix.val[2]=255;
            HSV.at<Vec3b>(i,j)=pix;
        }
   }


    //Plot for saturation and value
    int sat_range=0;
    int value_range=255;
    for(int i=S_V_y;i<S_V_Height+S_V_y;i++){
        value_range--;
        sat_range=0;
        for(int j=S_V_x;j<S_V_Width+S_V_x;j++){
            Vec3b pix;
            pix.val[0]=H;
            pix.val[1]=sat_range++;
            pix.val[2]=value_range;
            HSV.at<Vec3b>(i,j)=pix;
        }
    }

    //Ploaxt for HSV
    Mat roi1(HSV,Rect(HSV_x,HSV_y,HSV_Width,HSV_Height));
    roi1=Scalar(H,S,V);
    //drawPointers();

    Mat RGB;
    cvtColor(HSV, RGB,CV_HSV2BGR);

    imshow(window_name,RGB);
    imwrite("hsv.jpg",RGB);
}
void drawPointers(){
   // Point p(S_V_x+S,S_V_y+(255-V));
    Point p(S,255-V);

    int index=10;
    Point p1,p2;
    p1.x=p.x-index;
    p1.y=p.y;
    p2.x=p.x+index;
    p2.y=p.y;

    Mat roi1(HSV,Rect(S_V_x,S_V_y,S_V_Width,S_V_Height));
    line(roi1, p1, p2,Scalar(255,255,255),1,CV_AA,0);
    p1.x=p.x;
    p1.y=p.y-index;
    p2.x=p.x;
    p2.y=p.y+index;
    line(roi1, p1, p2,Scalar(255,255,255),1,CV_AA,0);

    int x_index=wheel_x+H*max_step;
    if(x_index>=wheel_x+wheel_width) x_index=wheel_x+wheel_width-2;
    if(x_index<=wheel_x) x_index=wheel_x+2;

    p1.x=x_index;
    p1.y=wheel_y+1;
    p2.x=x_index;
    p2.y=wheel_y+20;
    line(HSV, p1, p2,Scalar(255,255,255),2,CV_AA,0);

    Mat RGB(1,1,CV_8UC3);
    Mat temp;
    RGB=Scalar(H,S,V);
    cvtColor(RGB, temp,CV_HSV2BGR);
    Vec3b rgb=temp.at<Vec3b>(0,0);
    B=rgb.val[0];
    G=rgb.val[1];
    R=rgb.val[2];

    Mat roi2(HSV,Rect(450,130,175,175));
    roi2=Scalar(200,0,200);

    char name[30];
    sprintf(name,"R=%d",R);
    putText(HSV,name, Point(460,155) , FONT_HERSHEY_SIMPLEX, .7, Scalar(5,255,255), 2,8,false );

    sprintf(name,"G=%d",G);
    putText(HSV,name, Point(460,180) , FONT_HERSHEY_SIMPLEX, .7, Scalar(5,255,255), 2,8,false );

    sprintf(name,"B=%d",B);
    putText(HSV,name, Point(460,205) , FONT_HERSHEY_SIMPLEX, .7, Scalar(5,255,255), 2,8,false );


    sprintf(name,"H=%d",H);
    putText(HSV,name, Point(545,155) , FONT_HERSHEY_SIMPLEX, .7, Scalar(5,255,255), 2,8,false );

    sprintf(name,"S=%d",S);
    putText(HSV,name, Point(545,180) , FONT_HERSHEY_SIMPLEX, .7, Scalar(5,255,255), 2,8,false );

    sprintf(name,"V=%d",V);
    putText(HSV,name, Point(545,205) , FONT_HERSHEY_SIMPLEX, .7, Scalar(5,255,255), 2,8,false );
}
