#pragma once

#include "ofMain.h"
#include "ofxOpenCv.h"
#include "ofxKinect.h"


//------------ GUI --------------
//Slider class
class Slider {
public:
	string title;		//Title
	ofRectangle rect;	//Rectangle for drawing
	float *value;       //Pointer to value which the slider changes
	float minV, maxV;   //Minimum and maximum values
};

//Interface class, which manages sliders
class Interface {
public:
	void setup();
	void addSlider(string title, float *value, float minV, float maxV);
	void draw();

	void save(int index);		//Save preset
	void load(int index);		//Load preset

	void mousePressed(int x, int y);
	void mouseDragged(int x, int y);
	void mouseReleased(int x, int y);

	vector<Slider> slider;	//Array of sliders
	int selected;			//Index of selected slider
};


class ofApp : public ofBaseApp {
public:
	
	void setup();
	void update();
	void draw();
	void exit();
	
	void atualizaContraste(ofxCvGrayscaleImage &imgGray, int contrasteDistancia);
	void atualizaFantasmaDepth(ofxCvGrayscaleImage imgAtual, float iRastro);
	void atualizaBlurFantasmaDepth(ofxCvGrayscaleImage imgAtual, float iRastro);

	void drawPointCloud();
	void desenhaCameras(bool desenhaInstrucoes);
	void desenhaFade();
	void desenhaLogo();
	void desenhaPilares();
	void desenhaPilar(int x, int y, float altura, bool deslocado);

	void keyPressed(int key);
	void mouseDragged(int x, int y, int button);
	void mousePressed(int x, int y, int button);
	void mouseReleased(int x, int y, int button);
	void mouseEntered(int x, int y);
	void mouseExited(int x, int y);
	void windowResized(int w, int h);
	
	ofxKinect kinect;
	
#ifdef USE_TWO_KINECTS
	ofxKinect kinect2;
#endif
	
	ofxCvColorImage colorImg;

	ofxCvGrayscaleImage fantasmaDepth;
	ofxCvGrayscaleImage blurDepth;
	ofxCvGrayscaleImage blurFantasmaDepth;
	ofxCvGrayscaleImage contrasteDepth; // grayscale depth exagerado
	ofxCvGrayscaleImage grayImage; // grayscale depth image
	ofxCvGrayscaleImage grayThreshNear; // the near thresholded image
	ofxCvGrayscaleImage grayThreshFar; // the far thresholded image
	
	ofxCvContourFinder contourFinder;

	ofColor cor1[6], cor2[6], corborda;
	int iCor;

	float r1, g1, b1, r2, b2, g2;
	float qtdColW, qtdColH;
	float indiceAltura;
	int intervaloWKinect, intervaloHKinect;
	float larguraPilar;

	bool bThreshWithOpenCV;
	bool bDrawPointCloud;
	bool bDesenhaCameras;
	bool bFadeLogo, bFadePreto, flagFadeFinish;

	float inicioFade, fimFade, inicioLogo;

	ofImage logoSesc;

	int nearThreshold;
	int farThreshold;
	
	float tempoBrisa;

	int angle;
	
	// used for viewing the point cloud
	ofEasyCam easyCam;

	//GUI
	Interface interf;
	bool drawInterface;
};
