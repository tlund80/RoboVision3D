/*
 * ProjectorUI.cpp
 *
 *  Created on: Mar 18, 2013
 *      Author: thomas
 */

#include <structured_light_scanner/Projector_controller/ProjectorUI.h>

namespace structured_light_scanner {

/*ProjectorUI::ProjectorUI() {
	// TODO Auto-generated constructor stub
}
*/

ProjectorUI::ProjectorUI(){
	// TODO Auto-generated constructor stub
	width = 0;
	height = 0;
	isStarted = false;

	pos_cnt = 0;
	neg_cnt = 0;
	pattern_cnt_pos = 0;
	pattern_cnt_neg = 0;

}


ProjectorUI::~ProjectorUI() {
	// TODO Auto-generated destructor stub
}

void ProjectorUI::select_screen(QApplication& a, int &screen_width, int &screen_height){

	int screens = a.desktop()->numScreens();
	std::cout << "Number of screens: " << screens << std::endl;

	//int width, height;

	if(screens  > 1) {
		QRect re =a.desktop()->screenGeometry(1);
		screen_width = re.width();
		screen_height = re.height();
		width = screen_width;
		height = screen_height;

		myLabel.setGeometry(QApplication::desktop()->availableGeometry(1));

	}else{
		QRect re =a.desktop()->screenGeometry(2);
		screen_width = re.width();
		screen_height = re.height();
		width = screen_width;
		height = screen_height;

		myLabel.setGeometry(QApplication::desktop()->availableGeometry(2));
	}

	std::cout << "Screen Resolution: w: " << width << " h: " << height << std::endl;
}

QImage ProjectorUI::getNextPattern_Negative(void)
{
	if(pattern_cnt_neg == neg_cnt) neg_cnt = 0;
	neg_cnt++;
	return negative_pattern_list->at(neg_cnt-1);
}

QImage ProjectorUI::getNextPattern_Positive(void)
{
	if(pattern_cnt_pos == pos_cnt) pos_cnt = 0;
	pos_cnt++;
	return positive_pattern_list->at(pos_cnt-1);
}

void ProjectorUI::showPattern(QImage pattern)
{
	myLabel.setFocus(Qt::ActiveWindowFocusReason);
	myLabel.setFrameStyle(QFrame::NoFrame);
	myLabel.setPixmap(QPixmap::fromImage(pattern));
	myLabel.raise();
	myLabel.showFullScreen();

}
void ProjectorUI::createGrayPatternSerie_Positive(int numbers)
{
	pattern_cnt_pos = numbers;
	positive_pattern_list = new QList<QImage>();

	QImage black, white;
	create_black(black, width, height);
	positive_pattern_list->append(black);
	create_white(white, width, height);
	positive_pattern_list->append(white);

	for(int j = 1; j<= numbers; j++){
		QImage temp;
		create_gray_code_positive(temp,width,height,pow(2,j));//create_binary_code_negative(temp,width,height,pow(2,j));
		positive_pattern_list->append(temp);
	}
}

void ProjectorUI::createGrayPatternSerie_Negative(int numbers)
{
	pattern_cnt_neg = numbers;
	negative_pattern_list = new QList<QImage>();

	QImage black, white;
	create_black(black, width, height);
	negative_pattern_list->append(black);
	create_white(white, width, height);
	negative_pattern_list->append(white);

	for(int j = 1; j<= numbers; j++){
		QImage temp;
		create_gray_code_negative(temp,width,height,pow(2,j));//create_binary_code_negative(temp,width,height,pow(2,j));
		negative_pattern_list->append(temp);
	}

}

void ProjectorUI::loadGrayPatterns(std::string path)
{
	std::cout << "count: " << CountFiles(QString::fromStdString(path)) << std::endl;
	QList<QImage> img_list_false;
	QList<QImage> img_list_true;

		for(int i = 0; i<=(CountFiles(QString::fromStdString(path))-1)/2; i++)
		{
			QString name_true;
			name_true.append(QString::fromStdString(path));
			name_true.append("level-");
			name_true.append(QString("%1").arg(i+1));
			name_true.append("-true.png");

			QString name_false;
			name_false.append(QString::fromStdString(path));
			name_false.append("level-");
			name_false.append(QString("%1").arg(i+1));
			name_false.append("-false.png");

			//std::cout << name_true.toStdString() << std::endl;
			//std::cout << name_false.toStdString() << std::endl;
			QImage true_Image;
			QImage false_Image;
			true_Image.load(name_true);
			false_Image.load(name_false);
			if(true_Image.isNull() || false_Image.isNull()){
				std::cout << "Could not read the image!!"<< std::endl;
			}else{
				img_list_true.append(true_Image);
				img_list_false.append(false_Image);
			}
		}
}

void ProjectorUI::create_white(QImage &img, int width, int height)
{
	QImage *image = new QImage(width,height, QImage::Format_RGB16);

	QRgb white = qRgb(255,255,255);

	for(int u = 0; u<= width-1; u++){
		for(int v = 0; v<= height-1; v++){
			image->setPixel(u,v,white);
			}
		}

	img = image->copy(0,0,width, height);
	delete image;
}

void ProjectorUI::create_black(QImage &img, int width, int height)
{
	QImage *image = new QImage(width,height, QImage::Format_RGB16);

	QRgb black = qRgb(0,0,0);

	for(int u = 0; u<= width-1; u++){
		for(int v = 0; v<= height-1; v++){
			image->setPixel(u,v,black);
			}
		}

	img = image->copy(0,0,width, height);
	delete image;
}

void ProjectorUI::create_gray_code_positive(QImage &img, int width, int height, int code)
{
	QImage *image = new QImage(width,height, QImage::Format_RGB16);

	QRgb black = qRgb(0,0,0);
	QRgb white = qRgb(255,255,255);

	int bar_width = width/code;
	int bar_count = 0;
	int pair_cnt = 0;
	bool WB = false;

	//std::cout << "bar_width: " << bar_width << std::endl;
	//std::cout << "bar_number: " << bar_number << std::endl;
	//std::cout << "bar_count: " << bar_count << std::endl;

	for(int u = 0; u<= width-1; u++){
	for(int v = 0; v<= height-1; v++){

		if(WB){
			//Draw white black pair
			if(u >= bar_width*bar_count && bar_count %2 == 0){
				image->setPixel(u,v,white);
			}else
			{
			image->setPixel(u,v,black);
			}

		}else{
			//Draw black white pair
			if(u >= bar_width*bar_count && bar_count %2 == 0){
				image->setPixel(u,v,black);
			}else
			{
			image->setPixel(u,v,white);
			}
		}


		if(u == bar_width*(bar_count+1)){
		 //Counter is on a edge
		//std::cout << " bar_count++;" << std::endl;

		if(bar_count %2 != 0 )
		{
			pair_cnt++;

		}

		if(pair_cnt %2 == 0){
			WB = false;
		}else{
			WB = true;
		}

		bar_count++;

		}

	 }
	}


	img = image->copy(0,0,width, height);
	delete image;
}

void ProjectorUI::create_gray_code_negative(QImage &img, int width, int height, int code)
{
	QImage *image = new QImage(width,height, QImage::Format_RGB16);

	QRgb black = qRgb(0,0,0);
	QRgb white = qRgb(255,255,255);

	int bar_width = width/code;
	int bar_count = 0;
	int pair_cnt = 0;
	bool WB = false;

	//std::cout << "bar_width: " << bar_width << std::endl;
	//std::cout << "bar_number: " << bar_number << std::endl;
	//std::cout << "bar_count: " << bar_count << std::endl;

	for(int u = 0; u<= width-1; u++){
	for(int v = 0; v<= height-1; v++){

		if(!WB){
			//Draw white black pair
			if(u >= bar_width*bar_count && bar_count %2 == 0){
				image->setPixel(u,v,white);
			}else
			{
			image->setPixel(u,v,black);
			}

		}else{
			//Draw black white pair
			if(u >= bar_width*bar_count && bar_count %2 == 0){
				image->setPixel(u,v,black);
			}else
			{
			image->setPixel(u,v,white);
			}
		}


		if(u == bar_width*(bar_count+1)){
		 //Counter is on a edge
		//std::cout << " bar_count++;" << std::endl;

		if(bar_count %2 != 0 )
		{
			pair_cnt++;

		}

		if(pair_cnt %2 == 0){
			WB = false;
		}else{
			WB = true;
		}

		bar_count++;

		}

	 }
	}


	img = image->copy(0,0,width, height);
	delete image;
}

void ProjectorUI::create_binary_code_positive(QImage &img, int width, int height, int code)
{
	QImage *image = new QImage(width,height, QImage::Format_RGB16);

	QRgb black = qRgb(0,0,0);
	QRgb white = qRgb(255,255,255);

	int bar_width = width/code;
	//int bar_number = width/ bar_width;
	int bar_count = 0;

	//std::cout << "bar_width: " << bar_width << std::endl;
	//std::cout << "bar_number: " << bar_number << std::endl;
	//std::cout << "bar_count: " << bar_count << std::endl;

	for(int u = 0; u<= width-1; u++){
	for(int v = 0; v<= height-1; v++){

				if(u >= bar_width*bar_count && bar_count %2 == 0 ){
					image->setPixel(u,v,white);
				}else{
					image->setPixel(u,v,black);
				}

				if(u == bar_width*(bar_count+1)){
					//std::cout << " bar_count++;" << std::endl;
					 bar_count++;
				}

		}

	}
	img = image->copy(0,0,width, height);
	delete image;
}

void ProjectorUI::create_binary_code_negative(QImage &img, int width, int height, int code)
{
	QImage *image = new QImage(width,height, QImage::Format_RGB16);

	QRgb black = qRgb(0,0,0);
	QRgb white = qRgb(255,255,255);

	int bar_width = width/code;
	//int bar_number = width/ bar_width;
	int bar_count = 0;

	//std::cout << "bar_width: " << bar_width << std::endl;
	//std::cout << "bar_number: " << bar_number << std::endl;
	//std::cout << "bar_count: " << bar_count << std::endl;

	for(int u = 0; u<= width-1; u++){
	for(int v = 0; v<= height-1; v++){


				if(u >= bar_width*bar_count && bar_count %2 == 0 ){
					image->setPixel(u,v,black);
				}else{
					image->setPixel(u,v,white);
				}

				if(u == bar_width*(bar_count+1)){
					//std::cout << " bar_count++;" << std::endl;
					 bar_count++;
				}

		}

	}
	img = image->copy(0,0,width, height);
	delete image;
}

void ProjectorUI::create_random_dot( QImage &img, int width, int height, int std_deviation)
{
	unsigned int *data = new unsigned int[width*height];

	int zeros = 0;
	int ones = 0;

	for(int i = 0; i<= width*height-1; i ++)
	{
		int nr = rand() % 255;
		if(nr > std_deviation){
			data[i] = 255;
			ones++;
		}else{
			data[i]= 0;
			zeros++;
		}

		//std::cout << "nr: " << (int)data[i] << std::endl;
	}
	//std::cout << "Number of zeros: " << zeros << std::endl;
	//std::cout << "Number of ones: " << ones << std::endl;

	QImage *image = new QImage(width,height, QImage::Format_RGB16);

	QRgb black = qRgb(0,0,0);
	QRgb white = qRgb(255,255,255);
	for(int u = 0; u<= width-1; u++){
		for(int v = 0; v<= height-1; v++){

			if(data[u*v] == 0){
				image->setPixel(u,v,black);
			}else {
				image->setPixel(u,v,white);
			}

		}
	}

	img = image->copy(0,0,width, height);
	delete image;

}

void ProjectorUI::delay(int sec)
{
    QTime dieTime= QTime::currentTime().addSecs(sec);
    while( QTime::currentTime() < dieTime )
    QCoreApplication::processEvents(QEventLoop::AllEvents, 100);
}

void ProjectorUI::delayMSec(int msec)
{
    QTime dieTime= QTime::currentTime().addMSecs(msec);
    while( QTime::currentTime() < dieTime )
    QCoreApplication::processEvents(QEventLoop::AllEvents, 100);
}

int ProjectorUI::CountFiles(QString path)
{
	int sum = 0;
	QDir dir(path);

	dir.setFilter(QDir::AllDirs | QDir::NoDotAndDotDot);

	QFileInfoList lst = dir.entryInfoList();
	for(int i = 0; i < lst.size(); i++)
	{
		sum = sum + CountFiles(lst.at(i).canonicalPath());
	}
	dir.setFilter(QDir::Files);
	return dir.entryInfoList().size() + sum;
}

} /* namespace structured_light_scanner */
