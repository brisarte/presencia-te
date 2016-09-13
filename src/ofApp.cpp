#include "ofApp.h"

/*
    If you are struggling to get the device to connect ( especially Windows Users )
    please look at the ReadMe: in addons/ofxKinect/README.md
*/

//--------------------------------------------------------------
void ofApp::setup() {


	ofSetLogLevel(OF_LOG_VERBOSE);
	
	// enable depth->video image calibration
	kinect.setRegistration(true);
    
	kinect.init();
	//kinect.init(true); // shows infrared instead of RGB video image
	//kinect.init(false, false); // disable video image (faster fps)
	
	kinect.open();		// opens first available kinect
	//kinect.open(1);	// open a kinect by id, starting with 0 (sorted by serial # lexicographically))
	//kinect.open("A00362A08602047A");	// open a kinect using it's unique serial #
	
	// print the intrinsic IR sensor values
	if(kinect.isConnected()) {
		ofLogNotice() << "sensor-emitter dist: " << kinect.getSensorEmitterDistance() << "cm";
		ofLogNotice() << "sensor-camera dist:  " << kinect.getSensorCameraDistance() << "cm";
		ofLogNotice() << "zero plane pixel size: " << kinect.getZeroPlanePixelSize() << "mm";
		ofLogNotice() << "zero plane dist: " << kinect.getZeroPlaneDistance() << "mm";
	}
	
#ifdef USE_TWO_KINECTS
	kinect2.init();
	kinect2.open();
#endif

	fantasmaDepth.allocate(kinect.width, kinect.height);
	blurDepth.allocate(kinect.width, kinect.height);
	blurFantasmaDepth.allocate(kinect.width, kinect.height);
	contrasteDepth.allocate(kinect.width, kinect.height);

	colorImg.allocate(kinect.width, kinect.height);
	grayImage.allocate(kinect.width, kinect.height);
	grayThreshNear.allocate(kinect.width, kinect.height);
	grayThreshFar.allocate(kinect.width, kinect.height);
	
	nearThreshold = 230;
	farThreshold = 70;
	bThreshWithOpenCV = true;
	
	ofSetFrameRate(60);
	
	// zero the tilt on startup
	angle = 0;
	kinect.setCameraTiltAngle(angle);
	
	// start from the front
	bDrawPointCloud = false;

	r1 = 8, g1 = 222, b1 = 39;
	r2 = 168, g2 = 0, b2 = 41;

	corborda = ofColor(0, 0, 0);

	qtdColW = 20;
	indiceAltura = 1.5;
	//GUI
	interf.setup();
	interf.addSlider("qtdColW", &qtdColW, 5, 40);
	interf.addSlider("altura", &indiceAltura, 0.5, 5);
	interf.addSlider("tempo", &tempoBrisa, 2, 600);
	interf.addSlider("contraste", &indiceContraste, 0, 1024);
	drawInterface = true;

	// Fade e logo
	bFadeLogo = false;
	inicioLogo = ofGetElapsedTimef();
	inicioFade = ofGetElapsedTimef();
	fimFade = ofGetElapsedTimef();
	logoSesc.load("../../data/brabuletas.png");

	ofEnableBlendMode(OF_BLENDMODE_ADD);

	iCor = 0;
	tempoBrisa = 50;
	indiceContraste = 558;
}

//--------------------------------------------------------------
void ofApp::update() {

	cor1[0] = ofColor(38, 208, 255); 
	cor2[0] = ofColor(255, 89, 3);

	cor1[1] = ofColor(154, 13, 158);
	cor2[1] = ofColor(30, 158, 25);

	cor1[2] = ofColor(61, 0, 168);
	cor2[2] = ofColor(245, 234, 12);

	cor1[3] = ofColor(19, 245, 201);
	cor2[3] = ofColor(245, 36, 28);

	cor1[4] = ofColor(61, 0, 168);
	cor2[4] = ofColor(245, 234, 12);

	cor1[5] = ofColor(r1, g1, b1);
	cor2[5] = ofColor(r2, g2, b2);

	ofBackground(0, 0, 0);

	// calcula qtd de colunas na altura de acordo com a proporção
	qtdColH = qtdColW*((float)ofGetHeight() / ofGetWidth())*1.2;

	// Calcula intervalo entre pilares (in)
	intervaloWKinect = 640 / qtdColW;
	intervaloHKinect = 480 / qtdColH;


	larguraPilar = ofGetWidth() / qtdColW;

	kinect.update();
	
	// there is a new frame and we are connected
	if(kinect.isFrameNew()) {
		
		// load grayscale depth image from the kinect source
		grayImage.setFromPixels(kinect.getDepthPixels());
		// we do two thresholds - one for the far plane and one for the near plane
		// we then do a cvAnd to get the pixels which are a union of the two thresholds
		if(bThreshWithOpenCV) {
			grayThreshNear = grayImage;
			grayThreshFar = grayImage;
			grayThreshNear.threshold(nearThreshold, true);
			grayThreshFar.threshold(farThreshold);
			cvAnd(grayThreshNear.getCvImage(), grayThreshFar.getCvImage(), grayImage.getCvImage(), NULL);
		} else {
			
			// or we do it ourselves - show people how they can work with the pixels
			ofPixels & pix = grayImage.getPixels();
			int numPixels = pix.size();
			for(int i = 0; i < numPixels; i++) {
				if(pix[i] < nearThreshold && pix[i] > farThreshold) {
					pix[i] = 255;
				} else {
					pix[i] = 0;
				}
			}
		}
		
		// update the cv images
		grayImage.flagImageChanged();
		
		// find contours which are between the size of 20 pixels and 1/3 the w*h pixels.
		// also, find holes is set to true so we will get interior contours as well....
		contourFinder.findContours(grayImage, 10, (kinect.width*kinect.height)/2, 20, false);

		// Pega imagem de profundidade e aumenta o contraste das distancias
		contrasteDepth.setFromPixels(kinect.getDepthPixels());
		atualizaContraste(contrasteDepth, indiceContraste);

		atualizaFantasmaDepth(contrasteDepth, 0.99);
		atualizaBlurFantasmaDepth(contrasteDepth, 0.97);

		blurDepth.setFromPixels(kinect.getDepthPixels());
		blurDepth.blur(51);

	}
	
#ifdef USE_TWO_KINECTS
	kinect2.update();
#endif

	// Se passou mais que X+6 segundos que acabou o fade (e nao ta no fade) entra no fade
	if (ofGetElapsedTimef() - fimFade > 6 + tempoBrisa && !bFadePreto) {
		bFadePreto = true;
		flagFadeFinish = false;
		inicioFade = ofGetElapsedTimef();
	}
	// fade completo
	if (!flagFadeFinish && ofGetElapsedTimef() - inicioFade > 2) {
		iCor = (iCor + 1) % 6;
		flagFadeFinish = true;
	}
	//se ja acabou
	if (bFadePreto && ofGetElapsedTimef() - inicioFade > 6) {
		bFadePreto = false;
		fimFade = ofGetElapsedTimef();
	}

	// Se passou mais que 8 segundos que começou o logo, fadeOut, fadeIn e reposiciona Logo
	if (ofGetElapsedTimef() - inicioLogo > 12 + tempoBrisa) {
		inicioLogo = ofGetElapsedTimef();
	}

}

void ofApp::atualizaContraste(ofxCvGrayscaleImage &imgGray, int contrasteDistancia) {

	ofPixels & pixNoise = imgGray.getPixels();
	int numPixelsNoise = pixNoise.size();
	for (int i = 0; i < numPixelsNoise; i++) {
		pixNoise[i] = ofClamp(ofMap(pixNoise[i], 0, 255, -contrasteDistancia, 255), 0, 255); // Aumenta contraste de distancia
	}
	imgGray.flagImageChanged();
}

void ofApp::atualizaFantasmaDepth(ofxCvGrayscaleImage imgAtual, float iRastro) {

	ofPixels & pixF = fantasmaDepth.getPixels();
	ofPixels & pixA = imgAtual.getPixels();
	int numPixels = pixF.size();
	for (int i = 0; i < numPixels; i++) {
		pixF[i] = pixF[i] * iRastro + pixA[i] * (1 - iRastro);// Aumenta contraste de distancia
	}
	fantasmaDepth.flagImageChanged();
}
void ofApp::atualizaBlurFantasmaDepth(ofxCvGrayscaleImage imgAtual, float iRastro) {

	blurFantasmaDepth.blur(11);
	blurFantasmaDepth.blur(11);
	blurFantasmaDepth.erode();
	blurFantasmaDepth.dilate();
	blurFantasmaDepth.erode();
	ofPixels & pixF = blurFantasmaDepth.getPixels();
	ofPixels & pixA = imgAtual.getPixels();
	int numPixels = pixF.size();
	for (int i = 0; i < numPixels; i++) {
		pixF[i] =ofClamp(pixF[i] * iRastro + pixA[i] * (1.2 - iRastro),0,255); // Aumenta contraste de distancia
	}
	blurFantasmaDepth.flagImageChanged();
	blurFantasmaDepth.blur(11);
	blurFantasmaDepth.blur(21);
}

//--------------------------------------------------------------
void ofApp::draw() {

	desenhaPilares();
	if (bFadePreto) {
		desenhaFade();
	}
	else {
		ofSetColor(255, 255, 255);
	}
	if(bDrawPointCloud) {
		easyCam.begin();
		drawPointCloud();
		easyCam.end();
	} else {
		ofSetColor(255, 255, 255);
		if(bDesenhaCameras)
			desenhaCameras(false);
	}
	

	//GUI
	if (drawInterface) {
		//Draw text
		ofSetColor(0, 0, 0);
		//Draw sliders
		interf.draw();

	}

	desenhaLogo();
}


void ofApp::desenhaLogo() {
	ofFill();
	int alphaLogo = 255;
	float scaleLogo = 0.4;
	//logo no canto direito inf.
	int xLogo = ofGetWidth() - 400;
	int yLogo = ofGetHeight() - 200;

	if (ofGetElapsedTimef() - inicioLogo > tempoBrisa) {
		alphaLogo = ofMap(ofGetElapsedTimef(), inicioLogo + tempoBrisa, inicioLogo + tempoBrisa+ 7.5, 255, 0);
	}
	if (ofGetElapsedTimef() - inicioLogo > 8 + tempoBrisa) {
		alphaLogo = ofMap(ofGetElapsedTimef(), inicioLogo + 8 + tempoBrisa, inicioLogo + tempoBrisa + 10, 0, 255);
		//logo no centro
		xLogo = ofGetWidth()/2 - 465;
		yLogo = ofGetHeight()/2 - 188;
		scaleLogo = 1;
	}
	if (ofGetElapsedTimef() - inicioLogo > 11 + tempoBrisa) {
		//move logo no canto direito inf.
		xLogo = ofMap(ofGetElapsedTimef(), inicioLogo + 11 + tempoBrisa, inicioLogo + 12 + tempoBrisa, ofGetWidth() / 2 - 465, ofGetWidth() - 400);
		yLogo = ofMap(ofGetElapsedTimef(), inicioLogo + 11 + tempoBrisa, inicioLogo + 12 + tempoBrisa, ofGetHeight() / 2 - 188, ofGetHeight() - 200);
		scaleLogo = ofMap(ofGetElapsedTimef(), inicioLogo + 11 + tempoBrisa, inicioLogo + 12 + tempoBrisa, 1, 0.4);
	}
	
	ofEnableAlphaBlending();
	ofPushMatrix();
	ofSetColor(255, 255, 255, alphaLogo);
	ofTranslate(xLogo, yLogo);

	ofScale(scaleLogo, scaleLogo);
	logoSesc.draw(0, 0);
	ofPopMatrix();
	ofDisableAlphaBlending();
}

void ofApp::desenhaFade() {
	ofFill();
	int nivelPreto = 0;
	if (ofGetElapsedTimef() - inicioFade > 2) {
		nivelPreto = ofMap(ofGetElapsedTimef(), inicioFade + 4, inicioFade + 6, 255, 0);
	}
	else {
		nivelPreto = ofMap(ofGetElapsedTimef(), inicioFade, inicioFade + 2, 0, 255);
	}
	ofEnableAlphaBlending();
	ofSetColor(0, 0, 0, nivelPreto);
	ofDrawRectangle(0, 0, ofGetWidth(), ofGetHeight());
	ofDisableAlphaBlending();
}
//----- Desenha Pilares que ocupam toda a tela -----------------
void ofApp::desenhaPilares() {

	// Desenha pilares
	ofPixels & pixNoise = blurFantasmaDepth.getPixels();
	int numPixelsNoise = pixNoise.size();
	int altura = 0;
	for (int iY = 0; iY < qtdColH; iY++) {

		for (int iX = 0; iX < qtdColW; iX++) {

			// Lê a posição do Kinect
			int xKinect = iX * intervaloWKinect;
			int yKinect = iY * intervaloHKinect;

			int i = xKinect + yKinect * 640; //Pega o indice da coluna na img corrida
			altura = pixNoise[i];

			// Desenha a posição lida
			desenhaPilar(iX, iY, altura, false);

		}
		// Desenha uma coluna extra
		desenhaPilar(qtdColW, iY, altura / 2, false);

		for (int iX = 0; iX < qtdColW; iX++) {
			//Desenha fileira intermediaria se existir
			int xKinect = iX * intervaloWKinect + intervaloWKinect / 2;
			int yKinect = iY * intervaloHKinect + intervaloHKinect / 2;
			if (yKinect < 480 && xKinect < 640) {
				int i = xKinect + yKinect * 640; //Pega o indice da coluna na img corrida
				altura = pixNoise[i];

				// Desenha a posição lida
				desenhaPilar(iX, iY, altura, true);
			}
		}
	}

	// Coloca linhas extras pra baixo
	for (int iY = 0; iY < qtdColH/4; iY++) {
		for (int iX = 0; iX < qtdColW; iX++) {

			int xKinect = iX * intervaloWKinect;
			int yKinect = (qtdColH - 1) * intervaloHKinect;
			if (yKinect < 480 && xKinect < 640) {
				int i = xKinect + yKinect * 640;
				altura = pixNoise[i];
				desenhaPilar(iX, iY + qtdColH, altura-5*iY, false);

			}
		}
		for (int iX = 0; iX < qtdColW; iX++) {

			int xKinect = iX * intervaloWKinect + intervaloWKinect / 2;
			int yKinect = (qtdColH - 1) * intervaloHKinect + intervaloHKinect / 2;
			if (yKinect < 480 && xKinect < 640) {
				int i = xKinect + yKinect * 640;
				altura = pixNoise[i];
				desenhaPilar(iX, iY + qtdColH, altura - 5*iY, true);
			}
		}
	}

}

//--------------------------------------------------------------
void ofApp::desenhaPilar(int x, int y, float altura, bool deslocado) {
	
	altura = indiceAltura*altura;
	int tamanhoCol = larguraPilar*0.72; //Ajuste de proporções
	// Modo cores brisas
	ofColor corQuadrado = cor1[iCor];
	ofColor corBorda = corborda;
	ofColor corPilar = cor2[iCor];

	ofColor corPilarEscuro = corPilar;
	corPilarEscuro.setBrightness(100); // Escurece o lado do pilar
	float saturation = ofMap(altura, 0, 400, 0, 20);
	//corQuadrado.setHue(saturation);

	glPushMatrix();
	if (deslocado) {
		glTranslatef(x*larguraPilar + larguraPilar / 2, y*larguraPilar*0.81 + larguraPilar*0.4 - altura, 0);
	}
	else {
		glTranslatef(x*larguraPilar, y*larguraPilar*0.81 - altura, 0);
	}

	ofSetColor(corPilar);
	ofFill();
	ofDrawRectangle(-tamanhoCol*0.7, tamanhoCol*0.57, tamanhoCol*0.7, tamanhoCol * 5);
	ofSetColor(corBorda);
	ofNoFill();
	ofDrawRectangle(-tamanhoCol*0.7, tamanhoCol*0.57, tamanhoCol*0.7, tamanhoCol * 5);

	ofSetColor(corPilarEscuro);
	ofFill();
	ofDrawRectangle(0, tamanhoCol*0.57, tamanhoCol*0.7, tamanhoCol * 5);
	ofSetColor(corBorda);
	ofNoFill();
	ofDrawRectangle(0, tamanhoCol*0.57, tamanhoCol*0.7, tamanhoCol * 5);

	glScalef(1, 0.8, 1);
	glRotatef(45, 0, 0, 1);
	ofSetColor(corQuadrado);
	ofFill();
	ofDrawRectangle(0, 0, tamanhoCol, tamanhoCol);
	ofSetColor(corBorda);
	ofNoFill();
	ofDrawRectangle(0, 0, tamanhoCol, tamanhoCol);
	glPopMatrix();
}


void ofApp::desenhaCameras(bool desenhaInstrucoes) {
	// draw from the live kinect
	kinect.drawDepth(10, 10, 300, 225);
	kinect.draw(320, 10, 300, 225);

	grayImage.draw(630, 10, 300, 225);
	contourFinder.draw(630, 10, 300, 225);

	contrasteDepth.draw(10, 245, 300, 225);
	fantasmaDepth.draw(320, 245, 300, 225);
	blurDepth.draw(630, 245, 300, 225);

	blurFantasmaDepth.draw(320, 480, 300, 225);

	#ifdef USE_TWO_KINECTS
		kinect2.draw(420, 320, 400, 300);
	#endif

	
	if (desenhaInstrucoes) {
		// draw instructions
		ofSetColor(255, 255, 255);
		stringstream reportStream;

		if (kinect.hasAccelControl()) {
			reportStream << "accel is: " << ofToString(kinect.getMksAccel().x, 2) << " / "
				<< ofToString(kinect.getMksAccel().y, 2) << " / "
				<< ofToString(kinect.getMksAccel().z, 2) << endl;
		}
		else {
			reportStream << "Note: this is a newer Xbox Kinect or Kinect For Windows device," << endl
				<< "motor / led / accel controls are not currently supported" << endl << endl;
		}

		reportStream << "press p to switch between images and point cloud, rotate the point cloud with the mouse" << endl
			<< "using opencv threshold = " << bThreshWithOpenCV << " (press spacebar)" << endl
			<< "set near threshold " << nearThreshold << " (press: + -)" << endl
			<< "set far threshold " << farThreshold << " (press: < >) num blobs found " << contourFinder.nBlobs
			<< ", fps: " << ofGetFrameRate() << endl
			<< "press c to close the connection and o to open it again, connection is: " << kinect.isConnected() << endl;

		if (kinect.hasCamTiltControl()) {
			reportStream << "press UP and DOWN to change the tilt angle: " << angle << " degrees" << endl
				<< "press 1-5 & 0 to change the led mode" << endl;
		}

		ofDrawBitmapString(reportStream.str(), 20, 652);
	}
}

void ofApp::drawPointCloud() {
	int w = 640;
	int h = 480;
	ofMesh mesh;
	mesh.setMode(OF_PRIMITIVE_POINTS);
	int step = 2;
	for(int y = 0; y < h; y += step) {
		for(int x = 0; x < w; x += step) {
			if(kinect.getDistanceAt(x, y) > 0) {
				mesh.addColor(kinect.getColorAt(x,y));
				mesh.addVertex(kinect.getWorldCoordinateAt(x, y));
			}
		}
	}
	glPointSize(3);
	ofPushMatrix();
	// the projected points are 'upside down' and 'backwards' 
	ofScale(1, -1, -1);
	ofTranslate(0, 0, -1000); // center the points a bit
	ofEnableDepthTest();
	mesh.drawVertices();
	ofDisableDepthTest();
	ofPopMatrix();
}

//--------------------------------------------------------------
void ofApp::exit() {
	kinect.setCameraTiltAngle(0); // zero the tilt on exit
	kinect.close();
	
#ifdef USE_TWO_KINECTS
	kinect2.close();
#endif
}

//--------------------------------------------------------------
void ofApp::keyPressed (int key) {
	switch (key) {
		case ' ':
			bThreshWithOpenCV = !bThreshWithOpenCV;
			break;
			
		case'p':
			bDrawPointCloud = !bDrawPointCloud;
			break;

		case 'c':
			bDesenhaCameras = !bDesenhaCameras;
			break;
			
		case '>':
		case '.':
			farThreshold ++;
			if (farThreshold > 255) farThreshold = 255;
			break;
			
		case '<':
		case ',':
			farThreshold --;
			if (farThreshold < 0) farThreshold = 0;
			break;
			
		case '+':
		case '=':
			nearThreshold ++;
			if (nearThreshold > 255) nearThreshold = 255;
			break;
			
		case '-':
			nearThreshold --;
			if (nearThreshold < 0) nearThreshold = 0;
			break;
			
		case 'w':
			kinect.enableDepthNearValueWhite(!kinect.isDepthNearValueWhite());
			break;
			
		case 'o':
			kinect.setCameraTiltAngle(angle); // go back to prev tilt
			kinect.open();
			break;
			
		case 'x':
			kinect.setCameraTiltAngle(0); // zero the tilt
			kinect.close();
			break;
			
		case '1':
			kinect.setLed(ofxKinect::LED_GREEN);
			break;
			
		case '2':
			kinect.setLed(ofxKinect::LED_YELLOW);
			break;
			
		case '3':
			kinect.setLed(ofxKinect::LED_RED);
			break;
			
		case '4':
			kinect.setLed(ofxKinect::LED_BLINK_GREEN);
			break;
			
		case '5':
			kinect.setLed(ofxKinect::LED_BLINK_YELLOW_RED);
			break;
			
		case '0':
			kinect.setLed(ofxKinect::LED_OFF);
			break;
			
		case OF_KEY_UP:
			angle++;
			if(angle>30) angle=30;
			kinect.setCameraTiltAngle(angle);
			break;
			
		case OF_KEY_DOWN:
			angle--;
			if(angle<-30) angle=-30;
			kinect.setCameraTiltAngle(angle);
			break;

		case OF_KEY_RETURN:	//Hide/show GUI
			drawInterface = !drawInterface;
			break;
		case ';':
			
			break;
	}
}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button)
{
	if (drawInterface) {
		interf.mouseDragged(x, y);
	}
}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button)
{
	if (drawInterface) {
		interf.mousePressed(x, y);
	}
}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button)
{
	interf.mouseReleased(x, y);
}

//--------------------------------------------------------------
void ofApp::mouseEntered(int x, int y){

}

//--------------------------------------------------------------
void ofApp::mouseExited(int x, int y){

}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h)
{

}


//--------------------------------------------------------------
//----------------------  GUI ----------------------------------
//--------------------------------------------------------------
void Interface::setup() {
	selected = -1;
}

void Interface::addSlider(string title, float *value, float minV, float maxV) {
	Slider s;
	s.title = title;
	s.rect = ofRectangle(20, 60 + slider.size() * 40, 150, 30);
	s.value = value;
	s.minV = minV;
	s.maxV = maxV;
	slider.push_back(s);
}

void Interface::draw() {
	for (int i = 0; i<slider.size(); i++) {
		Slider &s = slider[i];
		ofRectangle r = s.rect;
		ofFill();
		ofSetColor(255, 255, 255);
		ofRect(r);
		ofSetColor(0, 0, 0);
		ofNoFill();
		ofRect(r);
		ofFill();
		float w = ofMap(*s.value, s.minV, s.maxV, 0, r.width);
		ofRect(r.x, r.y + 15, w, r.height - 15);
		ofDrawBitmapString(s.title + " " + ofToString(*s.value, 2), r.x + 5, r.y + 12);
	}
}

void Interface::mousePressed(int x, int y) {
	for (int i = 0; i<slider.size(); i++) {
		Slider &s = slider[i];
		ofRectangle r = s.rect;
		if (ofInRange(x, r.x, r.x + r.width) && ofInRange(y, r.y, r.y + r.height)) {
			selected = i;
			*s.value = ofMap(x, r.x, r.x + r.width, s.minV, s.maxV, true);
		}
	}
}

void Interface::mouseDragged(int x, int y) {
	if (selected >= 0) {
		Slider &s = slider[selected];
		ofRectangle r = s.rect;
		*s.value = ofMap(x, r.x, r.x + r.width, s.minV, s.maxV, true);
	}
}

void Interface::mouseReleased(int x, int y) {
	selected = -1;
}