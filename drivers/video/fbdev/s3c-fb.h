/*******************************************************************************
* Copyright (C), 2000-2017,  Electronic Technology Co., Ltd.
*                
* @filename: s3c-fb.h 
*                
* @author: Clarence.Zhou <zhou_chenz@163.com> 
*                
* @version:
*                
* @date: 2017-12-2    
*                
* @brief:          
*                  
*                  
* @details:        
*                 
*    
*    
* @comment           
*******************************************************************************/
#ifndef _S3C_FB_H_
#define _S3C_FB_H_


struct s3c_fb_user_plane_alpha {
	int		channel;
	unsigned char	red;
	unsigned char	green;
	unsigned char	blue;
};
/*
struct s3c_fb_user_chroma {
	int		enabled;
	unsigned char	red;
	unsigned char	green;
	unsigned char	blue;
};
*/
struct s3c_fb_user_ion_client {
	int	fd;
	int	offset;
};

enum s3c_fb_pixel_format {
	S3C_FB_PIXEL_FORMAT_RGBA_8888 = 0,
	S3C_FB_PIXEL_FORMAT_RGBX_8888 = 1,
	S3C_FB_PIXEL_FORMAT_RGBA_5551 = 2,
	S3C_FB_PIXEL_FORMAT_RGB_565 = 3,
	S3C_FB_PIXEL_FORMAT_BGRA_8888 = 4,
	S3C_FB_PIXEL_FORMAT_MAX = 5,
};

enum s3c_fb_blending {
	S3C_FB_BLENDING_NONE = 0,
	S3C_FB_BLENDING_PREMULT = 1,
	S3C_FB_BLENDING_COVERAGE = 2,
	S3C_FB_BLENDING_MAX = 3,
};

struct s3c_fb_win_config {
	enum {
		S3C_FB_WIN_STATE_DISABLED = 0,
		S3C_FB_WIN_STATE_COLOR,
		S3C_FB_WIN_STATE_BUFFER,
	} state;

	union {
		__u32 color;
		struct {
			int				fd;
			__u32				offset;
			__u32				stride;
			enum s3c_fb_pixel_format	format;
			enum s3c_fb_blending		blending;
			int				fence_fd;
		};
	};

	int	x;
	int	y;
	__u32	w;
	__u32	h;
};

/* S3C_FB_MAX_WIN
 * Set to the maximum number of windows that any of the supported hardware
 * can use. Since the platform data uses this for an array size, having it
 * set to the maximum of any version of the hardware can do is safe.
 */
#define S3C_FB_MAX_WIN	(5)

struct s3c_fb_win_config_data {
	int	fence;
	struct s3c_fb_win_config config[S3C_FB_MAX_WIN];
};

/* IOCTL commands */
#define S3CFB_WIN_POSITION		_IOW('F', 203, \
						struct s3c_fb_user_window)
#define S3CFB_WIN_SET_PLANE_ALPHA	_IOW('F', 204, \
						struct s3c_fb_user_plane_alpha)
#define S3CFB_WIN_SET_CHROMA		_IOW('F', 205, \
						struct s3c_fb_user_chroma)
#define S3CFB_SET_VSYNC_INT		_IOW('F', 206, __u32)

#define S3CFB_GET_ION_USER_HANDLE	_IOWR('F', 208, \
						struct s3c_fb_user_ion_client)
#define S3CFB_WIN_CONFIG		_IOW('F', 209, \
						struct s3c_fb_win_config_data)

/*************************Tiny4412 LCD *****************************************/
/*
 * struct s3cfb_lcd_polarity
 * @rise_vclk:	if 1, video data is fetched at rising edge
 * @inv_hsync:	if HSYNC polarity is inversed
 * @inv_vsync:	if VSYNC polarity is inversed
 * @inv_vden:	if VDEN polarity is inversed
 */
struct s3cfb_lcd_polarity {
	int	rise_vclk;
	int	inv_hsync;
	int	inv_vsync;
	int	inv_vden;
};

/*
 * struct s3cfb_lcd_timing
 * @h_fp:	horizontal front porch
 * @h_bp:	horizontal back porch
 * @h_sw:	horizontal sync width
 * @v_fp:	vertical front porch
 * @v_fpe:	vertical front porch for even field
 * @v_bp:	vertical back porch
 * @v_bpe:	vertical back porch for even field
 */
struct s3cfb_lcd_timing {
	int	h_fp;
	int	h_bp;
	int	h_sw;
	int	v_fp;
	int	v_fpe;
	int	v_bp;
	int	v_bpe;
	int	v_sw;
};

/*
 * struct s3cfb_lcd
 * @width:		horizontal resolution
 * @height:		vertical resolution
 * @p_width:	width of lcd in mm
 * @p_height:	height of lcd in mm
 * @bpp:		bits per pixel
 * @freq:		vframe frequency
 * @timing:		timing values
 * @polarity:	polarity settings
 * @init_ldi:	pointer to LDI init function
 *
 */
struct s3cfb_lcd {
	int	width;
	int	height;
	int	p_width;
	int	p_height;
	int	bpp;
	int	freq;
	struct	s3cfb_lcd_timing timing;
	struct	s3cfb_lcd_polarity polarity;
};

#endif /* ifndef _S3C-FB_H_.2017-12-2 11:36:37 zcz */

