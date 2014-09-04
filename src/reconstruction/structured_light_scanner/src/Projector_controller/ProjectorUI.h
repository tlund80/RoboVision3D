/*
 * ProjectorUI.h
 *
 *  Created on: Mar 18, 2013
 *      Author: thomas
 */

#ifndef PROJECTORUI_H_
#define PROJECTORUI_H_


#include <QApplication>
#include <QObject>
#include <QtGui>


#include <iostream>
#include <cmath>

namespace structured_light_scanner {

class ProjectorUI
{
public:

	ProjectorUI();
	virtual ~ProjectorUI();

	void init();
	void run();
	void loadGrayPatterns(std::string path);
	void create_black(QImage &img, int width, int height);
	void create_white(QImage &img, int width, int height);
	void create_gray_code_positive(QImage &img, int width, int height, int code);
	void create_gray_code_negative(QImage &img, int width, int height, int code);
	void create_binary_code_positive(QImage &img, int width, int height, int code);
	void create_binary_code_negative(QImage &img, int width, int height, int code);
	void create_random_dot( QImage &img, int width, int height, int std_deviation);
	void showPattern(QImage pattern);
	bool threadRunning(){return isStarted;};

	int get_image_height(){return height;};
	int get_image_width(){return width;};

	void createGrayPatternSerie_Positive(int numbers);
	void createGrayPatternSerie_Negative(int numbers);
	void select_screen(QApplication &a, int &screen_width, int &screen_height);
	void delay(int sec);
	void delayMSec(int msec);

	QImage getNextPattern_Negative(void);
	QImage getNextPattern_Positive(void);

private:

	void start_thread();

	QLabel myLabel;
	int width, height;
	bool isStarted;

	int pattern_cnt_pos, pattern_cnt_neg;
	int pos_cnt, neg_cnt;
	QList<QImage> *positive_pattern_list;
	QList<QImage> *negative_pattern_list;

	int CountFiles(QString path);


};

} /* namespace structured_light_scanner */
#endif /* PROJECTORUI_H_ */
