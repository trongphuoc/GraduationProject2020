
#include <errno.h>

/* Verification Test Environment Include Files */
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <asm/types.h>
#include <linux/videodev2.h>
#include <sys/mman.h>
#include <string.h>
#include <malloc.h>

#include <sys/time.h>
#include <sys/types.h>
#include <sys/select.h>


#include "objdetect.hpp"
#include "highgui.hpp"
#include "imgproc.hpp"
#include "videoio.hpp"

extern "C"
{
#include "/opt/libjpeg-turbo/include/jpeglib.h"
}
#include "/opt/libjpeg-turbo/include/jconfig.h"
#include "/opt/libjpeg-turbo/include/jerror.h"
#include "/opt/libjpeg-turbo/include/jmorecfg.h"
#include "/opt/libjpeg-turbo/include/turbojpeg.h"

/*
#include "opencv2imgprocimgproc_c.h"
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/opencv.hpp>
#include "opencv2/objdetect.hpp"
#include <iostream>
#include "hps_0.h"
*/
#include <iostream>

using namespace std;
using namespace cv;

#define BUFFER_TEST_NUM 4
#define SAT(c)       \
    if (c & (~255))  \
    {                \
        if (c < 0)   \
            c = 0;   \
        else         \
            c = 255; \
    }
#define SDRAM_BASE_ADDR 0
#define ALT_VIP_SOFTWARE_RESET_N_BASE 0x00000200 //

typedef struct
{
    void *start;
    size_t length;
} v4l2_buffer_t;

typedef struct
{
    const char *dev_name;
    int fd;

    int mmap_flag;
    struct v4l2_capability cap;
    struct v4l2_format fmt;
    struct v4l2_requestbuffers request_buff;
    struct v4l2_buffer buf;

    v4l2_buffer_t *buffer;

    int time_out;

    int pFrame;
} src_v4l2_t;

src_v4l2_t uvc_src_v4l2;
int video_out_width = 640;
int video_out_heigh = 480;
int video_out_fmt = V4L2_PIX_FMT_YUYV;
char v4l2_device_file[50] = "/dev/video2";
char jpeg_output[50] = "Capture.jpg";

static void help()
{
    cout << "\nThis program demonstrates the use of cv::CascadeClassifier class to detect objects (Face + eyes). You can use Haar or LBP features.\n"
            "This classifier can recognize many kinds of rigid objects, once the appropriate classifier is trained.\n"
            "It's most known use is for faces.\n"
            "Usage:\n"
            "./facedetect [--cascade=<cascade_path> this is the primary trained classifier such as frontal face]\n"
            "   [--nested-cascade[=nested_cascade_path this an optional secondary classifier such as eyes]]\n"
            "   [--scale=<image scale greater or equal to 1, try 1.3 for example>]\n"
            "   [--try-flip]\n"
            "   [filename|camera_index]\n\n"
            "see facedetect.cmd for one call:\n"
            "./facedetect --cascade=\"data/haarcascades/haarcascade_frontalface_alt.xml\" --nested-cascade=\"data/haarcascades/haarcascade_eye_tree_eyeglasses.xml\" --scale=1.3\n\n"
            "During execution:\n\tHit any key to quit.\n"
            "\tUsing OpenCV version "
         << CV_VERSION << "\n"
         << endl;
}

void detectAndDraw(Mat &img, CascadeClassifier &cascade,
                   CascadeClassifier &nestedCascade,
                   double scale, bool tryflip);

string cascadeName;
string nestedCascadeName;

static int uvc_get_capability(src_v4l2_t *src_v4l2)
{
    if (ioctl(src_v4l2->fd, VIDIOC_QUERYCAP, &src_v4l2->cap) < 0)
    {
        printf("Device is not support v4l2 \n");
        return -1;
    }

    printf("cap.driver: \"%s\" \n", src_v4l2->cap.driver);
    printf("cap.card: \"%s\"\n", src_v4l2->cap.card);
    printf("cap.bus_info: \"%s\"\n", src_v4l2->cap.bus_info);
    printf("cap.capabilities=0x%08X \n", src_v4l2->cap.capabilities);
    if (src_v4l2->cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)
        printf("- VIDEO_CAPTURE");
    if (src_v4l2->cap.capabilities & V4L2_CAP_VIDEO_OUTPUT)
        printf("- VIDEO_OUTPUT");
    if (src_v4l2->cap.capabilities & V4L2_CAP_VIDEO_OVERLAY)
        printf("- VIDEO_OVERLAY");
    if (src_v4l2->cap.capabilities & V4L2_CAP_VBI_CAPTURE)
        printf("- VBI_CAPTURE");
    if (src_v4l2->cap.capabilities & V4L2_CAP_VBI_OUTPUT)
        printf("- VBI_OUTPUT");
    if (src_v4l2->cap.capabilities & V4L2_CAP_RDS_CAPTURE)
        printf("- RDS_CAPTURE");
    if (src_v4l2->cap.capabilities & V4L2_CAP_TUNER)
        printf("- TUNER");
    if (src_v4l2->cap.capabilities & V4L2_CAP_AUDIO)
        printf("- AUDIO");
    if (src_v4l2->cap.capabilities & V4L2_CAP_RADIO)
        printf("- RADIO");
    if (src_v4l2->cap.capabilities & V4L2_CAP_READWRITE)
        printf("- READWRITE");
    if (src_v4l2->cap.capabilities & V4L2_CAP_ASYNCIO)
        printf("- ASYNCIO");
    if (src_v4l2->cap.capabilities & V4L2_CAP_STREAMING)
        printf("- STREAMING");
    if (src_v4l2->cap.capabilities & V4L2_CAP_TIMEPERFRAME)
        printf("- TIMEPERFRAME");

    if (!src_v4l2->cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)
    {
        printf("device not support capture \n");
        return -1;
    }

    return 0;
}

static int uvc_set_input(src_v4l2_t *src_v4l2)
{
    struct v4l2_input input;
    memset(&input, 0, sizeof(input));
    if (ioctl(src_v4l2->fd, VIDIOC_ENUMINPUT, &input) < 0)
    {
        printf("Unable to query input %d \n", input.index);
        return -1;
    }
    printf(" Input %i information:", input.index);
    printf("name = \"%s\"", input.name);
    printf("type = %08X", input.type);

    if (ioctl(src_v4l2->fd, VIDIOC_S_INPUT, &input.index) < 0)
    {
        printf("Selecting input failed \n");
        return -1;
    }
    return 0;
}

static int uvc_set_pix_format(src_v4l2_t *src_v4l2)
{
    //struct v4l2_fmtdesc fmt;
    src_v4l2->fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    src_v4l2->fmt.fmt.pix.width = video_out_width;
    src_v4l2->fmt.fmt.pix.height = video_out_heigh;
    src_v4l2->fmt.fmt.pix.pixelformat = video_out_fmt;
    if (ioctl(src_v4l2->fd, VIDIOC_S_FMT, &src_v4l2->fmt) < 0)
    {
        printf("Setting format failed \n");
        return -1;
    }
    return 0;
}

int uvc_set_fps(src_v4l2_t *src_v4l2)
{
    struct v4l2_streamparm setfps;

    memset(&setfps, 0, sizeof(setfps));

    setfps.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    setfps.parm.capture.timeperframe.numerator = 1;
    //setfps.parm.capture.timeperframe.denominator = src_v4l2->fps;
    if (ioctl(src_v4l2->fd, VIDIOC_S_PARM, &setfps) == -1)
    {
        /* Not fatal - just warn about it */
        printf("Error setting frame rate:");
        printf("VIDIOC_S_PARM: %s", strerror(errno));
        return (-1);
    }

    return (0);
}

static int uvc_free_mmap(src_v4l2_t *src)
{
    int i;
    for (i = 0; i < src->request_buff.count; i++)
        munmap(src->buffer[i].start, src->buffer[i].length);

    return (0);
}

static int uvc_set_mmap(src_v4l2_t *src)
{
    enum v4l2_buf_type type;
    int index;
    struct v4l2_buffer buf;
    if (~src->cap.capabilities & V4L2_CAP_STREAMING)
    {
        printf("device not support capturing \n");
        return -1;
    }

    memset(&src->request_buff, 0, sizeof(src->request_buff));
    src->request_buff.count = 4;
    src->request_buff.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    src->request_buff.memory = V4L2_MEMORY_MMAP;

    if (ioctl(src->fd, VIDIOC_REQBUFS, &src->request_buff) < 0)
    {
        printf("Error requesting buffer for mmemory map \n");
        return -1;
    }

    printf("frames = %d \n", src->request_buff.count);
    if (src->request_buff.count < 2)
    {
        printf("Insufficient memory \n");
        return -1;
    }

    src->buffer = (v4l2_buffer_t *)calloc(src->request_buff.count, sizeof(v4l2_buffer_t));
    if (src->buffer == NULL)
    {
        printf("Can not allocate for memory \n");
        return -1;
    }

    for (index = 0; index < src->request_buff.count; index++)
    {

        memset(&buf, 0, sizeof(buf));
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = index;

        if (ioctl(src->fd, VIDIOC_QUERYBUF, &buf) < 0)
        {
            printf("Queue buffer failed \n");
            return -1;
        }

        src->buffer[index].length = buf.length;
        src->buffer[index].start = mmap(NULL, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, src->fd,
                                        buf.m.offset);
        if (src->buffer[index].start == MAP_FAILED)
        {
            printf("Mapping memory %d failed \n", index);
            src->request_buff.count = index;
            uvc_free_mmap(src);
            free(src->buffer);
            src->buffer = NULL;
            return -1;
        }
        printf("%d length = %d \n", index, src->buffer[index].length);
    }

    for (index = 0; index < src->request_buff.count; index++)
    {
        memset(&src->buf, 0, sizeof(src->buf));
        src->buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        src->buf.memory = V4L2_MEMORY_MMAP;
        src->buf.index = index;
        if (ioctl(src->fd, VIDIOC_QBUF, &src->buf) == -1)
        {
            printf("VIDIOC_QBUF: %s \n", strerror(errno));
            uvc_free_mmap(src);
            free(src->buffer);
            src->buffer = NULL;
            return (-1);
        }
    }

    type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(src->fd, VIDIOC_STREAMON, &type) < 0)
    {
        printf("Start capturing failed \n");
        uvc_free_mmap(src);
        free(src->buffer);
        src->buffer = NULL;
        return -1;
    }
    return 0;
}

static int compress_yuyv_to_jpeg(unsigned char *yuyv, FILE *file)
{
    struct jpeg_compress_struct cinfo;
    struct jpeg_error_mgr jerr;
    JSAMPROW row_pointer[1];
    unsigned char *line_buffer;
    int z;
    static int written;
    int quality = 90;
    unsigned long size = 0;

    line_buffer = (unsigned char *)calloc(video_out_width * 3, 1);

    cinfo.err = jpeg_std_error(&jerr);
    jpeg_create_compress(&cinfo);
    jpeg_stdio_dest(&cinfo, file);
    //dest_buffer(&cinfo, buffer, size, &written);

    cinfo.image_width = video_out_width;
    cinfo.image_height = video_out_heigh;
    cinfo.input_components = 3;
    cinfo.in_color_space = JCS_RGB;

    jpeg_set_defaults(&cinfo);
    jpeg_set_quality(&cinfo, quality, TRUE);

    jpeg_start_compress(&cinfo, TRUE);

    z = 0;
    while (cinfo.next_scanline < video_out_heigh)
    {
        int x;
        unsigned char *ptr = line_buffer;

        for (x = 0; x < video_out_width; x++)
        {
            int r, g, b;
            int y, u, v;

            if (!z)
                y = yuyv[0];
            else
                y = yuyv[2];

            u = yuyv[1];
            v = yuyv[3];

            // r = (y + (359 * v)) >> 8;
            // g = (y - (88 * u) - (183 * v)) >> 8;
            // b = (y + (454 * u)) >> 8;
            r = y + 1.13983 * (v - 128);
            g = y - 0.39465 * (u - 128) - 0.58060 * (v - 128);
            b = y + 2.03211 * (u - 128);

            *(ptr++) = (r > 255) ? 255 : ((r < 0) ? 0 : r);
            *(ptr++) = (g > 255) ? 255 : ((g < 0) ? 0 : g);
            *(ptr++) = (b > 255) ? 255 : ((b < 0) ? 0 : b);

            if (z++)
            {
                z = 0;
                yuyv += 4;
            }
        }

        row_pointer[0] = line_buffer;
        //yuyv+= video_out_width*2;
        jpeg_write_scanlines(&cinfo, row_pointer, 1);
    }

    jpeg_finish_compress(&cinfo);
    jpeg_destroy_compress(&cinfo);

    free(line_buffer);

    return 0;
}

static void yuyv_to_rgb32(int width, int height, char *src, long *dst)
{
    char *s;
    long *d;
    int l, c, alpha = 0x0;
    int r, g, b, cr, cg, cb, y1, y2;

    l = height;
    s = src;
    d = dst;
    while (l--)
    {
        c = width >> 1;
        while (c--)
        {
            y1 = *s++;
            cb = ((*s - 128) * 454) >> 8;
            cg = (*s++ - 128) * 88;
            y2 = *s++;
            cr = ((*s - 128) * 359) >> 8;
            cg = (cg + (*s++ - 128) * 183) >> 8;

            r = y1 + cr;
            b = y1 + cb;
            g = y1 - cg;
            SAT(r);
            SAT(g);
            SAT(b);

            *dst++ = ((unsigned int)alpha) << 24 | (r << 16) | (g << 8) | b;

            r = y2 + cr;
            b = y2 + cb;
            g = y2 - cg;
            SAT(r);
            SAT(g);
            SAT(b);
            *dst++ = ((unsigned int)alpha) << 24 | (r << 16) | (g << 8) | b;
        }
    }
}

static int uvc_capture(src_v4l2_t *src, const char *file , CascadeClassifier cascade, CascadeClassifier nestedCascade )
{
    if (src->time_out)
    {
        fd_set fds;
        struct timeval tv;
        int result;

        FD_ZERO(&fds);
        FD_SET(src->fd, &fds);

        tv.tv_sec = src->time_out;
        tv.tv_usec = 0;

        result = select((src->fd) + 1, &fds, NULL, NULL, &tv);

        if (result == -1)
        {
            printf("Select() function failed \n");
            return -1;
        }
        if (result == 0)
        {
            printf("Time out \n");
            return -1;
        }
    }

    if (src->mmap_flag == 1)
    {

        FILE *file_fd;
        long *bgr_buff;
        bgr_buff = (long *) malloc (sizeof(long) * video_out_width * video_out_heigh * 4);

        file_fd = fopen(file, "wb");
        if (file_fd == NULL)
        {
            printf("open file %s failed \n", file);
            return -1;
        }
        if (src->pFrame > 0)
        {
            if (ioctl(src->fd, VIDIOC_QBUF, &src->buf) < 0)
            {
                printf("VIDIOC_QBUF: %s \n", strerror(errno));
                return -1;
            }
        }

        memset(&src->buf, 0, sizeof(src->buf));

        src->buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        src->buf.memory = V4L2_MEMORY_MMAP;

        if (ioctl(src->fd, VIDIOC_DQBUF, &src->buf) < 0)
        {
            printf("VIDIOC_DQBUF: %s \n", strerror(errno));
        }
	
        Size szSize(video_out_width, video_out_heigh);
        Mat mSrc(szSize,CV_8UC2, src->buffer[src->buf.index].start);
        Mat mSrc_BGR(szSize, CV_8UC3);
        //yuyv_to_rgb32(video_out_width, video_out_heigh, (char*)src->buffer[src->buf.index].start, bgr_buff);
        // Mat mSrc(szSize,CV_8UC3, bgr_buff);
        cvtColor(mSrc, mSrc_BGR, COLOR_YUV2BGR_YUYV);
        detectAndDraw(mSrc_BGR,  cascade, nestedCascade, 1, false );
        //compress_yuyv_to_jpeg((unsigned char *)src->buffer[src->buf.index].start, file_fd);
         //Mat image_src = Mat(video_out_width, video_out_heigh, CV_8UC3, src->buffer[src->buf.index].start);
        //img = cvDecodeImage(&image_src, 1);
        // Mat image_dst = Mat(video_out_width, video_out_heigh,CV_8UC3 );
         //cvtColor(image_src, image_src, COLOR_YUV2RGB );
        // Mat image;
        // image = imread("Capture.jpg", IMREAD_COLOR);
        //  yuyv_to_rgb32(video_out_width, video_out_heigh, (char*)src->buffer[src->buf.index].start, bgr_buff);
        //  Mat image = Mat(video_out_width, video_out_heigh, CV_8UC3, bgr_buff);
        //cvtColor(image, image, COLOR_RGB2BGR);

        // detectAndDraw(image,  cascade, nestedCascade, 1, false );
        //compress_yuyv_to_jpeg(mSrc_BGR.data, file_fd);
        //image = imread("Capture.jpg", IMREAD_COLOR);
        imshow("image", mSrc_BGR);
        //compress_yuyv_to_jpeg(image.data, file_fd);
        // detectAndDraw(image, cascade, nestedCascade, 1, false);
        // compress_yuyv_to_jpeg(image_dst.data, file_fd);
        //imshow("image", image);
        waitKey(0); 
	
	//compress_yuyv_to_jpeg((unsigned char *)src->buffer[src->buf.index].start, file_fd);

        fclose(file_fd);
        printf("Frame saved in File %s\n", file);
    }
}

static int uvc_open(src_v4l2_t *src)
{
    int fd_ret;
    src->fd = open(src->dev_name, O_RDWR, 0);
    if (src->fd < 0)
    {
        printf("Open device %s failed \n", *(src->dev_name));
        return -1;
    }
    src->pFrame = -1;
    fd_ret = src->fd;
    return fd_ret;
}

static int uvc_close(src_v4l2_t *src)
{
    if (src->buffer != NULL)
    {
        uvc_free_mmap(src);
        free(src->buffer);
    }
    close(src->fd);
    return 0;
}

static void srcv4l2_init(src_v4l2_t *src)
{
    src->dev_name = v4l2_device_file;
    src->mmap_flag = 1;
    src->time_out = 5;
}

int main(int argc, const char **argv)
{

//    VideoCapture capture;
    Mat frame, image;
    string inputName;
    bool tryflip;
    CascadeClassifier cascade, nestedCascade;
    double scale;

    cv::CommandLineParser parser(argc, argv,
                                 "{help h||}"
                                 "{cascade|/home/trongphuoc/openCV/opencv_linux/opencv/data/haarcascades/haarcascade_frontalface_alt.xml|}"
                                 "{nested-cascade|/home/trongphuoc/openCV/opencv_linux/opencv/data/haarcascades/haarcascade_eye_tree_eyeglasses.xml|}"
                                 "{scale|1|}{try-flip||}{@filename||}");
    if (parser.has("help"))
    {
        help();
        return 0;
    }
    cascadeName = parser.get<string>("cascade");
    nestedCascadeName = parser.get<string>("nested-cascade");
    scale = parser.get<double>("scale");
    if (scale < 1)
        scale = 1;
    tryflip = parser.has("try-flip");
    inputName = parser.get<string>("@filename");
    if (!parser.check())
    {
        parser.printErrors();
        return 0;
    }
    if (!nestedCascade.load(samples::findFileOrKeep(nestedCascadeName)))
        cerr << "WARNING: Could not load classifier cascade for nested objects" << endl;
    if (!cascade.load(samples::findFile(cascadeName)))
    {
        cerr << "ERROR: Could not load classifier cascade" << endl;
        help();
        return -1;
    }
    srcv4l2_init(&uvc_src_v4l2);
    uvc_open(&uvc_src_v4l2);
    printf("start \n");
    uvc_get_capability(&uvc_src_v4l2);
    uvc_set_input(&uvc_src_v4l2);
    uvc_set_pix_format(&uvc_src_v4l2);
    uvc_set_mmap(&uvc_src_v4l2);
    uvc_capture(&uvc_src_v4l2, jpeg_output, cascade, nestedCascade);
 //   uvc_capture(&uvc_src_v4l2, jpeg_output);
    return 0;
}

void detectAndDraw(Mat &img, CascadeClassifier &cascade,
                   CascadeClassifier &nestedCascade,
                   double scale, bool tryflip)
{
    double t = 0;
    vector<Rect> faces, faces2;
    const static Scalar colors[] =
        {
            Scalar(255, 0, 0),
            Scalar(255, 128, 0),
            Scalar(255, 255, 0),
            Scalar(0, 255, 0),
            Scalar(0, 128, 255),
            Scalar(0, 255, 255),
            Scalar(0, 0, 255),
            Scalar(255, 0, 255)};
    Mat gray, smallImg;

    cvtColor(img, gray, COLOR_BGR2GRAY);
    double fx = 1 / scale;
    resize(gray, smallImg, Size(), fx, fx, INTER_LINEAR_EXACT);
    equalizeHist(smallImg, smallImg);

    t = (double)getTickCount();
    cascade.detectMultiScale(smallImg, faces,
                             1.1, 2, 0
                                         //|CASCADE_FIND_BIGGEST_OBJECT
                                         //|CASCADE_DO_ROUGH_SEARCH
                                         | CASCADE_SCALE_IMAGE,
                             Size(30, 30));
    if (tryflip)
    {
        flip(smallImg, smallImg, 1);
        cascade.detectMultiScale(smallImg, faces2,
                                 1.1, 2, 0
                                             //|CASCADE_FIND_BIGGEST_OBJECT
                                             //|CASCADE_DO_ROUGH_SEARCH
                                             | CASCADE_SCALE_IMAGE,
                                 Size(30, 30));
        for (vector<Rect>::const_iterator r = faces2.begin(); r != faces2.end(); ++r)
        {
            faces.push_back(Rect(smallImg.cols - r->x - r->width, r->y, r->width, r->height));
        }
    }
    t = (double)getTickCount() - t;
    printf("detection time = %g ms\n", t * 1000 / getTickFrequency());
    for (size_t i = 0; i < faces.size(); i++)
    {
        Rect r = faces[i];
        Mat smallImgROI;
        vector<Rect> nestedObjects;
        Point center;
        Scalar color = colors[i % 8];
        int radius;

        double aspect_ratio = (double)r.width / r.height;
        if (0.75 < aspect_ratio && aspect_ratio < 1.3)
        {
            center.x = cvRound((r.x + r.width * 0.5) * scale);
            center.y = cvRound((r.y + r.height * 0.5) * scale);
            radius = cvRound((r.width + r.height) * 0.25 * scale);
            circle(img, center, radius, color, 3, 8, 0);
        }
        else
            rectangle(img, Point(cvRound(r.x * scale), cvRound(r.y * scale)),
                      Point(cvRound((r.x + r.width - 1) * scale), cvRound((r.y + r.height - 1) * scale)),
                      color, 3, 8, 0);
        if (nestedCascade.empty())
            continue;
        smallImgROI = smallImg(r);
        nestedCascade.detectMultiScale(smallImgROI, nestedObjects,
                                       1.1, 2, 0
                                                   //|CASCADE_FIND_BIGGEST_OBJECT
                                                   //|CASCADE_DO_ROUGH_SEARCH
                                                   //|CASCADE_DO_CANNY_PRUNING
                                                   | CASCADE_SCALE_IMAGE,
                                       Size(30, 30));
        for (size_t j = 0; j < nestedObjects.size(); j++)
        {
            Rect nr = nestedObjects[j];
            center.x = cvRound((r.x + nr.x + nr.width * 0.5) * scale);
            center.y = cvRound((r.y + nr.y + nr.height * 0.5) * scale);
            radius = cvRound((nr.width + nr.height) * 0.25 * scale);
            circle(img, center, radius, color, 3, 8, 0);
        }
    }
   // imshow("result", img);
}

