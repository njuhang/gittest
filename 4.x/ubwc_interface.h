
#ifndef UBWC_INTERFACE_H
#define UBWC_INTERFACE_H
typedef enum
{
	IONBUFF_DEFAULT = 0,
	IONBUFF_UNCACHED,
}ionbuff_flag_t;

int get_ubwcsize(int height, int width);
int ubwc2nv12_dsp(const unsigned char *src_buff, unsigned char *dst_buff, int height, int width, int ubwc_size, int dsp_size);
int nv122ubwc_dsp(const unsigned char *src_buff, unsigned char *dst_buff, int height, int width, int ubwc_size, int dsp_size);
int nv122ubwc_dsp_dequant(const unsigned char *src_buff, unsigned char *dst_buff, int height, int width, int ubwc_size, int dsp_size, float scale, int zero_point);
int nv122ubwc_dsp_dequant_resize(const unsigned char *src_buff, const unsigned char *uv_buff, unsigned char *dst_buff, int height, int width, int ubwc_size, int dsp_size, int uv_size, float scale, int zero_point);
int nv122ubwc_dsp_final(const unsigned char *src_buff, const unsigned char *uv_buff, unsigned char *dst_buff, int height, int width, int ubwc_size, int dsp_size, int uv_size, float scale, int zero_point);
void* alloc_ionbuff(int size, ionbuff_flag_t flag);
void release_ionbuff(void *buff);
void register_ionbuff(void *buff, int size, int fd);
int ionbuff_to_fd(void *buff);
void set_powersave_level(unsigned int level, unsigned int set_case);
int nv12_dsp_dequant_resize(unsigned char *nv12_buff, unsigned char *pad_buff, int height, int width, int nv12_size, int pad_size, float scale, int zero_point);
int depth_to_space_d16b4(const unsigned char *in_buff, unsigned char *out_buff, int height, int width);
//int convolution(const unsigned char *input, int input_size, char *weight, int weight_size, unsigned char *output, int output_size, int *recip, int recip_size, int height, int width);
//int memcpy_dsp(const unsigned char *src_buff, unsigned char *dst_buff, int src_size, int dst_size);
int ubwc_resizebilinear_dsp(const unsigned char* ubwc_in, int ubwcin_size, unsigned char *ubwc_out, int ubwcout_size,
											unsigned char* nv12_in, int nv12in_size, unsigned char* nv12_out, int nv12out_size,
											int height, int width);
int ubwc_gaussian5x5_dsp(const unsigned char* nv12_in, int nv12in_size, 
								unsigned char* nv12_out, int nv12out_size,
								unsigned char* table, int table_size,
								int height, int width);
#endif

