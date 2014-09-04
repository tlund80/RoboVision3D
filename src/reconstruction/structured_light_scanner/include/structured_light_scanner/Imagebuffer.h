/********************************************************************************************************************
 *
 * @file		Imagebuffer.h
 * @author		Thomas Sølund (thso@teknologisk.dk)
 * @date		2012-06-27
 * @version		1.0
 * @brief		Circular Image Buffer
 *
*********************************************************************************************************************/
#ifndef CIRC_BUFFER_H_
#define CIRC_BUFFER_H_

//Thread safe implementation of circular buffer
#include <boost/thread/condition.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>
#include <boost/circular_buffer.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace cv;
using namespace std;

namespace structured_light_scanner {
// Circular buffer for cv::Mat objects.
// This (hopefully) should be thread safe.
class circ_buffer : private boost::noncopyable
{
public:
    typedef boost::mutex::scoped_lock lock;
    circ_buffer() {}
    circ_buffer(int n) {cb.set_capacity(n);}

    void send (Mat image)
    {
        lock lk(monitor);
        cb.push_back((Mat)image);
      //  printf("cb size: %d",cb.size());
        buffer_not_empty.notify_one();
    }
    Mat receive()//(Mat* imgOut)
    {
        lock lk(monitor);
        while (cb.empty())
            buffer_not_empty.wait(lk);
        Mat image = cb.front();
       // imgOut->create(image.rows,image.cols,image.type());
       // imgOut->data = image.data;
        cb.pop_front();
        return image;
    }
    void clear()
    {
        lock lk(monitor);
        cb.clear();
    }
    int size() {
        lock lk(monitor);
        return cb.size();
    }
    void set_capacity(int capacity) {
         lock lk(monitor);
         cb.set_capacity(capacity);

    }

    bool isEmpty() {
         lock lk(monitor);
         return cb.empty();
      }

    bool isFull() {
              lock lk(monitor);
              return cb.full();
         }
private:
    boost::condition buffer_not_empty;
    boost::mutex monitor;
    boost::circular_buffer<Mat> cb;

};
};


#endif /* CIRC_BUFFER_H_ */
