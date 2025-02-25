/*
 * This is a modified file of the OpenKinect Project. Some code is from the original file
 * However, the code for the LED Kinect Mirror exists here as well
 * This file is part of the OpenKinect Project. http://www.openkinect.org
 *
 * Copyright (c) 2010 individual OpenKinect contributors. See the CONTRIB file
 * for details.
 *
 * This code is licensed to you under the terms of the Apache License, version
 * 2.0, or, at your option, the terms of the GNU General Public License,
 * version 2.0. See the APACHE20 and GPL2 files for the text of the licenses,
 * or the following URLs:
 * http://www.apache.org/licenses/LICENSE-2.0
 * http://www.gnu.org/licenses/gpl-2.0.txt
 *
 * If you redistribute this file in source form, modified or unmodified, you
 * may:
 *   1) Leave this header intact and distribute it under the same terms,
 *      accompanying it with the APACHE20 and GPL20 files, or
 *   2) Delete the Apache 2.0 clause and accompany it with the GPL2 file, or
 *   3) Delete the GPL v2 clause and accompany it with the APACHE20 file
 * In all cases you must keep the copyright notice intact and include a copy
 * of the CONTRIB file.
 *
 * Binary distributions must follow the binary distribution requirements of
 * either License.
 */
#include "../rpi_ws281x/ws2811.h"
#include "../rpi_ws281x/clk.h"
#include "../rpi_ws281x/gpio.h"
#include "../rpi_ws281x/dma.h"
#include "../rpi_ws281x/pwm.h"
#include "../rpi_ws281x/version.h"
#include <stdio.h>
#include <signal.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <signal.h>
#include <stdarg.h>
#include <getopt.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/mman.h>

#include "libfreenect.h"


// LED MATRIX SECTION

#define ARRAY_SIZE(stuff)       (sizeof(stuff) / sizeof(stuff[0]))

// defaults for cmdline options
#define TARGET_FREQ             WS2811_TARGET_FREQ
#define GPIO_PIN                18
#define DMA                     10
//#define STRIP_TYPE            WS2811_STRIP_RGB		// WS2812/SK6812RGB integrated chip+leds
#define STRIP_TYPE              WS2811_STRIP_GBR		// WS2812/SK6812RGB integrated chip+leds
//#define STRIP_TYPE            SK6812_STRIP_RGBW		// SK6812RGBW (NOT SK6812RGB)

#define WIDTH                   24
#define HEIGHT                  32
#define LED_COUNT               (WIDTH * HEIGHT)

int width = WIDTH;
int height = HEIGHT;
int led_count = LED_COUNT;

int clear_on_exit = 0;

ws2811_t ledstring =
{
    .freq = TARGET_FREQ,
    .dmanum = DMA,
    .channel =
    {
        [0] =
        {
            .gpionum = GPIO_PIN,
            .invert = 0,
            .count = LED_COUNT,
            .strip_type = STRIP_TYPE,
            .brightness = 255,
        },
        [1] =
        {
            .gpionum = 0,
            .invert = 0,
            .count = 0,
            .brightness = 0,
        },
    },
};

ws2811_led_t *matrix;

static uint8_t running = 1;
ws2811_led_t dotcolors[] =
{
    0x00200000,  // red
    0x00201000,  // orange
    0x00202000,  // yellow
    0x00002000,  // green
    0x00002020,  // lightblue
    0x00000020,  // blue
    0x00100010,  // purple
    0x00200010,  // pink
};
ws2811_led_t dotcolors_rgbw[] =
{
    0x00200000,  // red
    0x10200000,  // red + W
    0x00002000,  // green
    0x10002000,  // green + W
    0x00000020,  // blue
    0x10000020,  // blue + W
    0x00101010,  // white
    0x10101010,  // white + W

};
// END



#ifdef _MSC_VER
#define HAVE_STRUCT_TIMESPEC
#endif
#include <pthread.h>

#if defined(__APPLE__)
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif

#define _USE_MATH_DEFINES
#include <math.h>

pthread_t freenect_thread;
volatile int die = 0;

int g_argc;
char **g_argv;

int window;


//~ struct RGBLedMatrixOptions options;
//~ struct RGBLedRuntimeOptions rt_options;
//~ struct RGBLedMatrix *matrix;
//~ struct LedCanvas *offscreen_canvas;


pthread_mutex_t gl_backbuf_mutex = PTHREAD_MUTEX_INITIALIZER;

// back: owned by libfreenect (implicit for depth)
// mid: owned by callbacks, "latest frame ready"
// front: owned by GL, "currently being drawn"
uint8_t *depth_mid, *depth_front;
uint8_t *rgb_back, *rgb_mid, *rgb_front;
int big_matrix[480][640];
int small_matrix[48][64];
ws2811_led_t crunched_matrix[width][height];
char small_matrix_debug[32][64];

GLuint gl_depth_tex;
GLuint gl_rgb_tex;
GLfloat camera_angle = 0.0;
int camera_rotate = 0;
int tilt_changed = 0;

freenect_context *f_ctx;
freenect_device *f_dev;
int freenect_angle = 0;
int freenect_led;

freenect_video_format requested_format = FREENECT_VIDEO_RGB;
freenect_video_format current_format = FREENECT_VIDEO_RGB;

pthread_cond_t gl_frame_cond = PTHREAD_COND_INITIALIZER;
int got_rgb = 0;
int got_depth = 0;



uint16_t t_gamma[2048];
int convert_to_appropiate_plot(int x, int y){
	// How this matrix works, one matrix is 256 pixels 8 x 32 where it is an array that zigzags
	// The zero point is at the bottom left and last pixel is top right
	// 
	//check direction, if even - left to right, odd is right to left
	
	// separation is needed to know which strip we want to access

	int separation = (-1*((x/8)-2)) * 256  ;
	int buffer_x= x - 23;
	x = buffer_x * -1; 
	if(x/8 == 1){
		int buffer= y - 31;
		y = buffer * -1; 
		
		if(y%2 != 0){
			return ((8 * y) +(x % 8)) + separation;
			}
		else{
			return (((y+1) * 8) -1) - (x%8) +separation;
				
		}
	}
	else{
		if(y%2 == 0){
			return ((8 * y) +(x % 8)) + separation;
			}
		else{
			return (((y+1) * 8) -1) - (x%8) +separation;
				
		}
	}

	
	return 0;
	}
void convert_to_matrix(void *depth_data){
	uint16_t *depth = (uint16_t*)depth_data;
	for(int i = 0; i < 480 ; i++) {
		for(int j = 0 ; j < 640 ; j++){
			int sum = j + (i * 640);
			int pval = t_gamma[depth[sum]];
			switch (pval>>8) {
				case 0:
					big_matrix[i][j] = 'o';
					break;
				case 1:
					big_matrix[i][j] = '@';
					break;
				case 2:
					big_matrix[i][j] = '*';
					break;

				default:
					big_matrix[i][j] = '.';
					break;
			}
		}
	}
}	
void print_big_matrix(){
	//~ led_canvas_clear(offscreen_canvas);
	printf("\n -------------------------------------\n");
	for(int i = 0; i < width ; i++) {
		for(int j = 0 ; j < height ; j++){
			ledstring.channel[0].leds[convert_to_appropiate_plot(i,j)] = crunched_matrix[i][j];
			if(crunched_matrix[i][j] == 0){
				printf("-");
				}
			else{
				printf("#");
				}
			
			}
			printf("\n");
		}
	ws2811_render(&ledstring);
	//~ offscreen_canvas = led_matrix_swap_on_vsync(matrix, offscreen_canvas);

	}
	
void clear_matrix(){
	//~ led_canvas_clear(offscreen_canvas);
	printf("\n -------------------------------------\n");
	for(int i = 0; i < width ; i++) {
		for(int j = 0 ; j < height ; j++){
			ledstring.channel[0].leds[convert_to_appropiate_plot(i,j)] =0;

		}
	ws2811_render(&ledstring);
	//~ offscreen_canvas = led_matrix_swap_on_vsync(matrix, offscreen_canvas);

	}	
}
/*
 * For testin purposes, we shall print out this matrix
 * */
 int gather_area_data(int x, int y, int x_param, int y_param){
	 int sum = 0;
	 
	 for(int i = x * x_param ; i < (x+1)*x_param; i++){
		 for(int j = y*y_param; j < (y+1)*y_param ; j++ ){
			int tmp = sum + big_matrix[i][j];
			sum = tmp;
		 }
	 }
	 if(sum != 300 && sum !=0){
		 return 1;
		 }
	 else{
		 return 0;
		 }
}

int crunch_matrix_16_32(int x, int y, int x_param, int y_param){	
	/* The purpose of this is to transfer the 32 x 64 matrix into a 16 x 32
	 * by sampling a 2 x 2 area and if it contains a 1 , we return 1
	 * if not, return 0
	 * */
	 int sum = 0;
	 for(int i = x * x_param ; i < (x+1)*x_param; i++){
		 for(int j = y*y_param; j < (y+1)*y_param ; j++ ){
			if(small_matrix[i][j] == 1){
				sum +=1;
				}
				
			if(sum  == 2){
				return 1;
				}
		 }
	 }
	 return 0;
}


 void downscale_matrix(){
	 /*
	  * I do not feel like making this variable so i will hardcode with the conditions i need
	  * Remember to change the small matrix size
	  * */
	  
	  /* These would be flipped when I rotate the main matrix or this one since it will be less costly*/
	  int x_target_small = 48;
	  int y_target_small = 64;
	  
	  /*This will be the area where I downsize the image to*/
	  int x_sample = 480 / x_target_small;
	  int y_sample = 640 / y_target_small;
	  
	  for(int i = 0 ; i < 48 ; i++){
		  for(int j = 0 ; j<64; j++){
			  small_matrix[i][j] = gather_area_data(i ,j, x_sample, y_sample);
			  }
		  }
		  
	  for(int i = 0; i < width ; i++){
		for(int j = 0 ; j <height ; j++){
			if(crunch_matrix_16_32(i, j , 2 , 2)){
				crunched_matrix[i][j] =  dotcolors[0];
				}
			else{
				crunched_matrix[i][j] = 0;
				}
				
					
			}
		}
	print_big_matrix();
	 }




/*
 * This is to convert depth data into colors
 * */
void depth_cb(freenect_device *dev, void *v_depth, uint32_t timestamp)
{
	int i;
	uint16_t *depth = (uint16_t*)v_depth;
	
	pthread_mutex_lock(&gl_backbuf_mutex);
	for (i=0; i<640*480; i++) {
		
		int x_val = i / 640;
		int y_val = i % 640;
		
		int pval = t_gamma[depth[i]];
		int lb = pval & 0xff;
		switch (pval>>8) {
			case 0:
				big_matrix[x_val][y_val] = 3;
				depth_mid[3*i+0] = 255;
				depth_mid[3*i+1] = 255-lb;
				depth_mid[3*i+2] = 255-lb;
				break;
			case 1:
				big_matrix[x_val][y_val] = 3;
				depth_mid[3*i+0] = 255;
				depth_mid[3*i+1] = lb;
				depth_mid[3*i+2] = 0;
				break;
			case 2:
				big_matrix[x_val][y_val] = 3;
				depth_mid[3*i+0] = 255-lb;
				depth_mid[3*i+1] = 255;
				depth_mid[3*i+2] = 0;
				break;

			default:
				big_matrix[x_val][y_val] = 0;
				depth_mid[3*i+0] = 0;
				depth_mid[3*i+1] = 0;
				depth_mid[3*i+2] = 0;
				break;
		}
	}
	got_depth++;
	pthread_cond_signal(&gl_frame_cond);
	pthread_mutex_unlock(&gl_backbuf_mutex);
}

void rgb_cb(freenect_device *dev, void *rgb, uint32_t timestamp)
{
	pthread_mutex_lock(&gl_backbuf_mutex);

	// swap buffers
	assert (rgb_back == rgb);
	rgb_back = rgb_mid;
	freenect_set_video_buffer(dev, rgb_back);
	rgb_mid = (uint8_t*)rgb;

	got_rgb++;
	pthread_cond_signal(&gl_frame_cond);
	pthread_mutex_unlock(&gl_backbuf_mutex);
}

void *freenect_threadfunc(void *arg)
{
	int accelCount = 0;
/*
	freenect_set_tilt_degs(f_dev,freenect_angle);
	freenect_set_led(f_dev,LED_RED);
	freenect_set_video_callback(f_dev, rgb_cb);
	freenect_set_video_mode(f_dev, freenect_find_video_mode(FREENECT_RESOLUTION_MEDIUM, current_format));
	freenect_set_video_buffer(f_dev, rgb_back);
	freenect_start_video(f_dev);
	*/
	freenect_set_depth_callback(f_dev, depth_cb);
	freenect_set_depth_mode(f_dev, freenect_find_depth_mode(FREENECT_RESOLUTION_MEDIUM, FREENECT_DEPTH_11BIT));
	freenect_start_depth(f_dev);

	printf("'w' - tilt up, 's' - level, 'x' - tilt down, '0'-'6' - select LED mode, '+' & '-' - change IR intensity \n");
	printf("'f' - change video format, 'm' - mirror video, 'o' - rotate video with accelerometer \n");
	printf("'e' - auto exposure, 'b' - white balance, 'r' - raw color, 'n' - near mode (K4W only) \n");

	while (running && freenect_process_events(f_ctx) >= 0) {
		//Throttle the text output
		if (accelCount++ >= 2000)
		{
			accelCount = 0;
			freenect_raw_tilt_state* state;
			freenect_update_tilt_state(f_dev);
			state = freenect_get_tilt_state(f_dev);
			double dx,dy,dz;
			freenect_get_mks_accel(state, &dx, &dy, &dz);
			fflush(stdout);
		}

		if (requested_format != current_format) {
			freenect_stop_video(f_dev);
			freenect_set_video_mode(f_dev, freenect_find_video_mode(FREENECT_RESOLUTION_MEDIUM, requested_format));
			freenect_start_video(f_dev);
			current_format = requested_format;
		}
		downscale_matrix();
		 usleep(30 / 60);
				
		
	}

	printf("\nshutting down streams...\n");

	freenect_stop_depth(f_dev);
	//freenect_stop_video(f_dev);
	clear_matrix();
	freenect_close_device(f_dev);
	freenect_shutdown(f_ctx);

	printf("-- done!\n");
	return NULL;
}

static void ctrl_c_handler(int signum)
{
    running = 0;
}

static void setup_handlers(void)
{
    struct sigaction sa =
    {
        .sa_handler = ctrl_c_handler,
    };

    sigaction(SIGINT, &sa, NULL);
    sigaction(SIGTERM, &sa, NULL);
}
void parseargs(int argc, char **argv, ws2811_t *ws2811)
{
	int index;
	int c;

	static struct option longopts[] =
	{
		{"help", no_argument, 0, 'h'},
		{"dma", required_argument, 0, 'd'},
		{"gpio", required_argument, 0, 'g'},
		{"invert", no_argument, 0, 'i'},
		{"clear", no_argument, 0, 'c'},
		{"strip", required_argument, 0, 's'},
		{"height", required_argument, 0, 'y'},
		{"width", required_argument, 0, 'x'},
		{"version", no_argument, 0, 'v'},
		{0, 0, 0, 0}
	};

	while (1)
	{

		index = 0;
		c = getopt_long(argc, argv, "cd:g:his:vx:y:", longopts, &index);

		if (c == -1)
			break;

		switch (c)
		{
		case 0:
			/* handle flag options (array's 3rd field non-0) */
			break;

		case 'h':
			fprintf(stderr, "Usage: %s \n"
				"-h (--help)    - this information\n"
				"-s (--strip)   - strip type - rgb, grb, gbr, rgbw\n"
				"-x (--width)   - matrix width (default 8)\n"
				"-y (--height)  - matrix height (default 8)\n"
				"-d (--dma)     - dma channel to use (default 10)\n"
				"-g (--gpio)    - GPIO to use\n"
				"                 If omitted, default is 18 (PWM0)\n"
				"-i (--invert)  - invert pin output (pulse LOW)\n"
				"-c (--clear)   - clear matrix on exit.\n"
				"-v (--version) - version information\n"
				, argv[0]);
			exit(-1);

		case 'D':
			break;

		case 'g':
			if (optarg) {
				int gpio = atoi(optarg);
/*
	PWM0, which can be set to use GPIOs 12, 18, 40, and 52.
	Only 12 (pin 32) and 18 (pin 12) are available on the B+/2B/3B
	PWM1 which can be set to use GPIOs 13, 19, 41, 45 and 53.
	Only 13 is available on the B+/2B/PiZero/3B, on pin 33
	PCM_DOUT, which can be set to use GPIOs 21 and 31.
	Only 21 is available on the B+/2B/PiZero/3B, on pin 40.
	SPI0-MOSI is available on GPIOs 10 and 38.
	Only GPIO 10 is available on all models.

	The library checks if the specified gpio is available
	on the specific model (from model B rev 1 till 3B)

*/
				ws2811->channel[0].gpionum = gpio;
			}
			break;

		case 'i':
			ws2811->channel[0].invert=1;
			break;

		case 'c':
			clear_on_exit=1;
			break;

		case 'd':
			if (optarg) {
				int dma = atoi(optarg);
				if (dma < 14) {
					ws2811->dmanum = dma;
				} else {
					printf ("invalid dma %d\n", dma);
					exit (-1);
				}
			}
			break;

		case 'y':
			if (optarg) {
				height = atoi(optarg);
				if (height > 0) {
					ws2811->channel[0].count = height * width;
				} else {
					printf ("invalid height %d\n", height);
					exit (-1);
				}
			}
			break;

		case 'x':
			if (optarg) {
				width = atoi(optarg);
				if (width > 0) {
					ws2811->channel[0].count = height * width;
				} else {
					printf ("invalid width %d\n", width);
					exit (-1);
				}
			}
			break;

		case 's':
			if (optarg) {
				if (!strncasecmp("rgb", optarg, 4)) {
					ws2811->channel[0].strip_type = WS2811_STRIP_RGB;
				}
				else if (!strncasecmp("rbg", optarg, 4)) {
					ws2811->channel[0].strip_type = WS2811_STRIP_RBG;
				}
				else if (!strncasecmp("grb", optarg, 4)) {
					ws2811->channel[0].strip_type = WS2811_STRIP_GRB;
				}
				else if (!strncasecmp("gbr", optarg, 4)) {
					ws2811->channel[0].strip_type = WS2811_STRIP_GBR;
				}
				else if (!strncasecmp("brg", optarg, 4)) {
					ws2811->channel[0].strip_type = WS2811_STRIP_BRG;
				}
				else if (!strncasecmp("bgr", optarg, 4)) {
					ws2811->channel[0].strip_type = WS2811_STRIP_BGR;
				}
				else if (!strncasecmp("rgbw", optarg, 4)) {
					ws2811->channel[0].strip_type = SK6812_STRIP_RGBW;
				}
				else if (!strncasecmp("grbw", optarg, 4)) {
					ws2811->channel[0].strip_type = SK6812_STRIP_GRBW;
				}
				else {
					printf ("invalid strip %s\n", optarg);
					exit (-1);
				}
			}
			break;

		case 'v':
			exit(-1);

		case '?':
			/* getopt_long already reported error? */
			exit(-1);

		default:
			exit(-1);
		}
	}
}



int main(int argc, char **argv)
{
	ws2811_return_t ret;


    matrix = malloc(sizeof(ws2811_led_t) * width * height);


    if ((ret = ws2811_init(&ledstring)) != WS2811_SUCCESS)
    {
        fprintf(stderr, "ws2811_init failed: %s\n", ws2811_get_return_t_str(ret));
        return ret;
    }
    ledstring.channel[0].leds[0] = dotcolors[0];
    signal(SIGINT, ctrl_c_handler);
	int res;
	
	//teest
	ws2811_led_t test_matrix[width][height];
	for(int i= 0 ; i< width;i++){
		for(int j = 0; j < height ; j++){
			if(i == 7){
				ledstring.channel[0].leds[convert_to_appropiate_plot(i,j)] = dotcolors[0];
				}
			}		
		}
	ws2811_render(&ledstring);
	
	printf("lol\n");
	// Handle signals gracefully.
	
	//~ memset(&options, 0, sizeof(options));
    //~ memset(&rt_options, 0, sizeof(rt_options));
    //~ options.rows = 32;
    //~ options.cols = 64;
    //~ options.chain_length = 1;
    //~ matrix = led_matrix_create_from_options_and_rt_options(&options, &rt_options);
    //~ if (matrix == NULL) {
       //~ return 1;
    //~ }
    
    //~ offscreen_canvas = led_matrix_create_offscreen_canvas(matrix);
	//~ led_canvas_clear(offscreen_canvas);
	//~ offscreen_canvas = led_matrix_swap_on_vsync(matrix, offscreen_canvas);
	
	
	depth_mid = (uint8_t*)malloc(640*480*3);
	depth_front = (uint8_t*)malloc(640*480*3);
	rgb_back = (uint8_t*)malloc(640*480*3);
	rgb_mid = (uint8_t*)malloc(640*480*3);
	rgb_front = (uint8_t*)malloc(640*480*3);

	printf("Kinect camera test\n");

	int i;
	for (i=0; i<2048; i++) {
		float v = i/2048.0;
		v = powf(v, 3)* 6;
		t_gamma[i] = v*6*256;
	}

	g_argc = argc;
	g_argv = argv;

	if (freenect_init(&f_ctx, NULL) < 0) {
		printf("freenect_init() failed\n");
		return 1;
	}

	freenect_set_log_level(f_ctx, FREENECT_LOG_DEBUG);
	freenect_select_subdevices(f_ctx, (freenect_device_flags)(FREENECT_DEVICE_MOTOR | FREENECT_DEVICE_CAMERA));

	int nr_devices = freenect_num_devices (f_ctx);
	printf ("Number of devices found: %d\n", nr_devices);

	int user_device_number = 0;
	if (argc > 1)
		user_device_number = atoi(argv[1]);

	if (nr_devices < 1) {
		freenect_shutdown(f_ctx);
		return 1;
	}

	if (freenect_open_device(f_ctx, &f_dev, user_device_number) < 0) {
		printf("Could not open device\n");
		freenect_shutdown(f_ctx);
		return 1;
	}

	res = pthread_create(&freenect_thread, NULL, freenect_threadfunc, NULL);
	if (res) {
		printf("pthread_create failed\n");
		freenect_shutdown(f_ctx);
		return 1;
	}
	
	while (running){
		printf("");
		}
		
	
	printf("done");
	//~ led_matrix_delete(matrix);
	
	return 0;
}
