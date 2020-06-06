
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

//extern "C"
//{
#include "/opt/libjpeg-turbo/include/jpeglib.h"
//#include <SDL_image.h>
//}
//#include <config.h>
// #include "./opencv/include/jerror.h"
// #include "./opencv/include/jmorecfg.h"
#include "/opt/libjpeg-turbo/include/turbojpeg.h"

#include <iostream>
#include "hps_0.h"

#define BUFFER_TEST_NUM 4
#define SAT(c)       \
    if (c & (~255))  \
    {                \
        if (c < 0)   \
            c = 0;   \
        else         \
            c = 255; \
    }
#define ALT_STM_OFST (0xfc000000)
#define ALT_LWFPGASLVS_OFST (0xff200000)  // axi_lw_master

#define ALT_AXI_FPGASLVS_OFST (0xC0000000)  // axi_master
#define HW_FPGA_AXI_SPAN (0x40000000)  // Bridge span
#define HW_FPGA_AXI_MASK ( HW_FPGA_AXI_SPAN - 1 )


#define HW_REGS_BASE ( ALT_STM_OFST )
#define HW_REGS_SPAN (0x04000000 )
#define HW_REGS_MASK ( HW_REGS_SPAN - 1 )

#define ALT_GPIO1_BASE_OFST   (0xFF709000)




#define DEMO_VGA_FRAME0_ADDR					                0x00000000//0x00080000 //0x00100000  //on chip memory base
#define FR0_FRAME0_OFFSET							(0x00000000)
#define FR0_FRAME1_OFFSET	

//base addr
static volatile unsigned long *h2p_lw_axi_addr=NULL;
static volatile unsigned char  *h2p_vip_frame_reader0_addr=NULL;
static volatile unsigned long *h2p_vip_frame_reader1_addr=NULL;
static  unsigned long *h2p_memory_addr=NULL;
static volatile unsigned long *h2p_onchip_memory_addr=NULL;
static volatile unsigned long *h2p_vip_mix_addr=NULL;



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

char *v4l2_device_file = "/dev/video0";
char *jpeg_output = "Capture.jpg";


/////////////////////////////////////////////////////////
// VIP Frame Reader: Select active frame

void VIP_FR0_SetActiveFrame(int nActiveFrame){
	
	// select active frame
	h2p_vip_frame_reader0_addr[3]=nActiveFrame; // active frame 0 was set
	
}

/////////////////////////////////////////////////////////
// VIP Frame Reader: configure

void VIP_FR_Config(int Width, int Height){
	int word = Width*Height;
	int cycle = Width*Height;
	int interlace = 0;
	printf("start config \n");	
	// stop
	h2p_vip_frame_reader0_addr[0]=0x00; // stop
	printf("Width=%d\r\n",Width);
	printf("Width=%d\r\n",Height);

	// configure frame 0
	h2p_vip_frame_reader0_addr[4]=DEMO_VGA_FRAME0_ADDR+FR0_FRAME0_OFFSET; // // frame0 base address
	h2p_vip_frame_reader0_addr[5]=word; // frame0 word
	h2p_vip_frame_reader0_addr[6]=cycle; //  The number of single-cycle color patterns to read for the frame
	h2p_vip_frame_reader0_addr[8]=Width; // frame0 width  
	h2p_vip_frame_reader0_addr[9]=Height; // frame0 height
	h2p_vip_frame_reader0_addr[10]=interlace; // frame0 interlace

	h2p_vip_frame_reader0_addr[0]=0x01; //start

	// select active frame
	h2p_vip_frame_reader0_addr[3]=0; // active frame 0 was set

		
}

/////////////////////////////////////////////////////////
// VIP MIX
void    VIP_MIX_Config(void){
 	h2p_vip_mix_addr[0]=0x00; //stop   	
	printf("Config \n"); 	
 	// din0 is layer 0, background, fixed
	/*		
	// layer 2 (log)
	h2p_vip_mix_addr[2]=130; 
	h2p_vip_mix_addr[3]=770;
	h2p_vip_mix_addr[4]=0x01;
	
	h2p_vip_mix_addr[5]=0;//(SCREEN_WIDTH-VIDEO_WIDTH)/2;//layer1 x offset
	h2p_vip_mix_addr[6]=0;//(SCREEN_HEIGHT-VIDEO_HEIGHT)/2;//layer1 y offset
	h2p_vip_mix_addr[7]=0x01;//set layer 1 active	
	
    h2p_vip_mix_addr[0]=0x01; //start
*/
}


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
    printf("==> Input %i information:", input.index);
    printf("==> name = \"%s\"\n", input.name);
    printf("==> Type = %08X\n", input.type);

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
    }else{
        printf("==> Setting format: (%d, %d)\n", video_out_width, video_out_heigh);
    }
    return 0;
}

int uvc_set_fps(src_v4l2_t *src_v4l2)
{
    struct v4l2_streamparm setfps;

    memset(&setfps, 0, sizeof(setfps));

    setfps.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    setfps.parm.capture.timeperframe.numerator = 8;
    //setfps.parm.capture.timeperframe.denominator = src_v4l2->fps;
    if (ioctl(src_v4l2->fd, VIDIOC_S_PARM, &setfps) == -1)
    {
        /* Not fatal - just warn about it */
        printf("Error setting frame rate:");
        printf("VIDIOC_S_PARM: %s", strerror(errno));
        return (-1);
    }else{
        printf("'==> Set stream parameter successfully\n");
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
	memset(src->buffer[index].start, 0xFF, src->buffer[index].length);

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

static void yuyv_to_rgb32 (int width, int height, char *src, long *dst)
{
    char *s;
    long *d;
    int l, c, alpha = 0x0;
    int r, g, b, cr, cg, cb, y1, y2;

    l = height;
    s = src;
    d = dst;
    while (l--) {
        c = width >> 1;
        while (c--) {
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

            *dst++ = ((unsigned int) alpha) << 24 |  (r << 16) | (g << 8) | b;

            r = y2 + cr;
            b = y2 + cb;
            g = y2 - cg;
            SAT(r);
            SAT(g);
            SAT(b);
            *dst++ = ((unsigned int) alpha) << 24 |  (r << 16) | (g << 8) | b;

        }
    }
}

static int uvc_stop_capturing(src_v4l2_t *src)
{
    enum v4l2_buf_type type;
    type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if(ioctl(src->fd, VIDIOC_STREAMOFF, &type) < 0)
    {
        printf("Stream off failed \n");
        return -1;
    }
    return 0;
}


static int uvc_capture(src_v4l2_t *src, const char* file)
{
    printf("start capturing \n");
    if(src->time_out)
    {
        fd_set fds;
        struct timeval tv;
        int result;

        FD_ZERO(&fds);
        FD_SET(src->fd, &fds);

        tv.tv_sec = src->time_out;
        tv.tv_usec = 0;

        result = select((src->fd) + 1, &fds, NULL, NULL, &tv);

        if(result == -1)
        {
            printf("Select() function failed \n");
            return -1;
        }
        if(result == 0)
        {
            printf("Time out \n");
            return -1;
        }
    }

    if(src->mmap_flag == 1)   {
	long *rgb_buff;
   	rgb_buff = (long *) malloc (sizeof(long) * video_out_width * video_out_heigh * 4);
	if(rgb_buff == NULL)
	{
		printf("Allocate rgb_buff failed \n");
		return -1;
	}
        printf("mmap in processing \n");
        FILE* file_fd;
        file_fd = fopen(file, "wb");
        if(file_fd == NULL)
        {
            printf("open file %s failed \n", file);
            return -1;
        }
        if(src->pFrame > 0)
        {
            if(ioctl(src->fd, VIDIOC_QBUF, &src->buf) < 0)
            {
                printf("VIDIOC_QBUF: %s \n", strerror(errno));
                return -1;
            }
        }

        memset(&src->buf, 0, sizeof(src->buf));

        src->buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        src->buf.memory = V4L2_MEMORY_MMAP;

        if(ioctl(src->fd, VIDIOC_DQBUF, &src->buf) < 0)
        {
            printf("VIDIOC_DQBUF: %s \n", strerror(errno));
        }
        else{
            printf("==>Dequeue buffer successfully\n");
        }
//        compress_yuyv_to_jpeg((unsigned char*)src->buffer[src->buf.index].start, file_fd);
	yuyv_to_rgb32(video_out_width, video_out_heigh,( char*)src->buffer[src->buf.index].start,rgb_buff);
	memcpy(h2p_memory_addr, rgb_buff, sizeof(rgb_buff));
        fclose(file_fd);
	free(rgb_buff);
        printf("Frame saved in File %s\n", file);
    }
}
static int uvc_streaming(src_v4l2_t *src)
{
    long *rgb_buff;
    rgb_buff = (long *) malloc (sizeof(long) * video_out_width * video_out_heigh * 4);

   while(1)
   {  
	printf("Getting frame from camera \n");        
        src->buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        src->buf.memory = V4L2_MEMORY_MMAP;
        if (ioctl (src->fd, VIDIOC_DQBUF, &src->buf) < 0)	
        {
            printf("VIDIOC_DQBUF failed.\n");
            break;
        }
        yuyv_to_rgb32(video_out_width, video_out_heigh, (char*)src->buffer[src->buf.index].start,rgb_buff);
         // copy rgb data to sdram address
       // memcpy(rgb_buff, src->buffer[src->buf.index].start, (video_out_heigh*video_out_width*4));
        memcpy(h2p_memory_addr, rgb_buff, (video_out_heigh*video_out_width*4));
		
        if(ioctl(src->fd, VIDIOC_QBUF, &src->buf) < 0)
        {
            printf("STREAMING: VIDIOC_QBUF failed \n");
        }
    }

    uvc_stop_capturing(src);
    //free rgb buffer
    free(rgb_buff);
    uvc_free_mmap(src);
    // off
}

static int uvc_open(src_v4l2_t *src)
{
    int fd_ret;
    src->fd = open(src->dev_name, O_RDWR, 0);
    if(src->fd < 0)
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
    if(src->buffer != NULL)
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


int main()
{

    void *virtual_base;
    void *axi_virtual_base;
    int fd;


	if( ( fd = open( "/dev/mem", ( O_RDWR | O_SYNC ) ) ) == -1 ) {
		printf( "ERROR: could not open \"/dev/mem\"...\n" );
		return( 1 );
	}

	// lw
	virtual_base = mmap( NULL, HW_REGS_SPAN, ( PROT_READ | PROT_WRITE ), MAP_SHARED, fd, HW_REGS_BASE );	
	if( virtual_base == MAP_FAILED ) {
		printf( "ERROR: mmap() failed...\n" );
		printf("%d %s \n", errno, strerror(errno));
		close( fd );
		return( -1 );
	}

	axi_virtual_base  = mmap( NULL, HW_FPGA_AXI_SPAN, ( PROT_READ | PROT_WRITE ), MAP_SHARED, fd,ALT_AXI_FPGASLVS_OFST  );	
	if( axi_virtual_base == MAP_FAILED ) {
		printf( "ERROR: axi mmap() failed...\n" );
        printf("%s \n", strerror(errno));
		close( fd );
		return( -1 );
	}

   // h2p_memory_addr=(unsigned long*)axi_virtual_base;
    h2p_memory_addr =(unsigned long*)axi_virtual_base + ( ( unsigned long  )( DEMO_VGA_FRAME0_ADDR) & ( unsigned long)( HW_FPGA_AXI_MASK ) );
   // h2p_vip_mix_addr=(unsigned long*)virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + ALT_VIP_MIX_0_BASE ) & ( unsigned long)( HW_REGS_MASK ) );
    h2p_vip_frame_reader0_addr= (unsigned char*)virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + ALT_VIP_VFR_0_BASE ) & ( unsigned long)( HW_REGS_MASK ) );		
    h2p_vip_mix_addr=(unsigned long*) virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + ALT_VIP_MIX_0_BASE ) & ( unsigned long)( HW_REGS_MASK ) );		

	
    //Configure Mixer IP core
//    VIP_MIX_Config();

    // Configure Frame Reader core
    VIP_FR_Config(video_out_width, video_out_heigh);
    usleep(500*1000);
    
    srcv4l2_init(&uvc_src_v4l2);
    uvc_open(&uvc_src_v4l2);
    uvc_get_capability(&uvc_src_v4l2);
    uvc_set_input(&uvc_src_v4l2);
    uvc_set_pix_format(&uvc_src_v4l2);
    uvc_set_fps(&uvc_src_v4l2);
    uvc_set_mmap(&uvc_src_v4l2);
//  uvc_capture(&uvc_src_v4l2, jpeg_output);
    uvc_streaming(&uvc_src_v4l2);
    uvc_close(&uvc_src_v4l2);
    return 0;
}
