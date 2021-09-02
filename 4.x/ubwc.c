#include "AEEStdErr.h"
#include "aiboostubwc.h"
#include "rpcmem.h"
#include "remote.h"
#include <stdlib.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <dlfcn.h>
#include <unistd.h>
//#include <sys/time.h>
#include <string.h>

#include <android/log.h>
#include <jni.h>

#include "ubwc_interface.h"

#define ALIGNUP(a, b) (((a) + ((b) - 1)) & ~((b) - 1))
#define UBWC_ERR_OFFSET (0x70000000)

#define LOG_TAG "aiboost"
#define UBWC_LOGI(...) ((void)__android_log_print(ANDROID_LOG_INFO, LOG_TAG, __VA_ARGS__))
#define UBWC_LOGD(...) ((void)__android_log_print(ANDROID_LOG_DEBUG, LOG_TAG, __VA_ARGS__))
#define UBWC_LOGE(...) ((void)__android_log_print(ANDROID_LOG_ERROR, LOG_TAG, __VA_ARGS__))

int get_ubwcsize(int height, int width)
{
	int ubwc_height = ALIGNUP(height, 32);
    int ubwc_width = ALIGNUP(width, 128);
	
	int ret = aiboostubwc_getFrameSize(ubwc_height, ubwc_width, 1);
	if(ret > UBWC_ERR_OFFSET)
	{
		UBWC_LOGE("UBWCDMA Get frame size error, errno = 0x%x.\n", ret);
	}

	return ret;
}

int ubwc2nv12_dsp(const unsigned char *src_buff, unsigned char *dst_buff, int height, int width, int ubwc_size, int dsp_size)
{
	const uint8_t *ubwc_buff = src_buff;
	uint8_t *dsp_buff = dst_buff;
	int ret = 0;

	//UBWC_LOGI("ubwc2nv12_dsp height = %d width = %d pad_height = %d pad_width= %d.\n", height & 0xffff, width & 0xffff, pad_height, pad_width);

	if(ubwc_size < 0 || dsp_size < 0)
	{
		UBWC_LOGE("ubwc2nv12_dsp : ubwc_size(%d) or dsp_size(%d) error.\n", ubwc_size, dsp_size);
		return -1;
	}

	if(ubwc_buff == NULL || dsp_buff == NULL)
	{
		UBWC_LOGE("ubwc2nv12_dsp input buffer is NULL.\n");
		return -1;		
	}
	
	if((ret = aiboostubwc_readBuffer(ubwc_buff, ubwc_size, dsp_buff, dsp_size, height, width)))
	{
		UBWC_LOGE("UBWCDMA Read error, errno = 0x%x.\n", ret);
		return -1;
	}
	
	return 0;
}

int nv122ubwc_dsp(const unsigned char *src_buff, unsigned char *dst_buff, int height, int width, int ubwc_size, int dsp_size)
{
	uint8_t *ubwc_buff = dst_buff;
	const uint8_t *dsp_buff = src_buff;
	int ret = 0;

	if(ubwc_size < 0 || dsp_size < 0)
	{
		UBWC_LOGE("nv122ubwc_dsp ubwc_size(%d) or dsp_size(%d) error.\n", ubwc_size, dsp_size);
		return -1;
	}

	if(ubwc_buff == NULL || dsp_buff == NULL)
	{
		UBWC_LOGE("nv122ubwc_dsp input buffer is NULL.\n");
		return -1;		
	}

	if((ret = aiboostubwc_writeBuffer(dsp_buff, dsp_size, ubwc_buff, ubwc_size, height, width)))
	{
		UBWC_LOGE("UBWCDMA Write error, errno = 0x%x.\n", ret);
		return -1;
	}

	return 0;	
}

int nv122ubwc_dsp_dequant(const unsigned char *src_buff, unsigned char *dst_buff, int height, int width, int ubwc_size, int dsp_size, float scale, int zero_point)
{
	uint8_t *ubwc_buff = dst_buff;
	const uint8_t *dsp_buff = src_buff;
	int ret = 0;
	
	if(ubwc_size < 0 || dsp_size < 0)
	{
		UBWC_LOGE("nv122ubwc_dsp_dequant ubwc_size(%d) or dsp_size(%d) error.\n", ubwc_size, dsp_size);
		return -1;
	}

	if(ubwc_buff == NULL || dsp_buff == NULL)
	{
		UBWC_LOGE("nv122ubwc_dsp_dequant input buffer is NULL.\n");
		return -1;		
	}

	if((ret = aiboostubwc_writeBufferDequant(dsp_buff, dsp_size, ubwc_buff, ubwc_size, height, width, scale, zero_point)))
	{
		UBWC_LOGE("UBWCDMA Write Dequant error, errno = 0x%x.\n", ret);
		return -1;
	}

	return 0;	
}

int nv122ubwc_dsp_dequant_resize(const unsigned char *src_buff, const unsigned char *uv_buff, unsigned char *dst_buff, int height, int width, int ubwc_size, int dsp_size, int uv_size, float scale, int zero_point)
{
	uint8_t *ubwc_buff = dst_buff;
	const uint8_t *dsp_buff = src_buff;
	int ret = 0;
	
	if(ubwc_size < 0 || dsp_size < 0)
	{
		UBWC_LOGE("nv122ubwc_dsp_dequant_resize ubwc_size(%d) or dsp_size(%d) error.\n", ubwc_size, dsp_size);
		return -1;
	}

	if(ubwc_buff == NULL || dsp_buff == NULL || uv_buff == NULL)
	{
		UBWC_LOGE("nv122ubwc_dsp_dequant_resize input buffer is NULL.\n");
		return -1;		
	}

	if((ret = aiboostubwc_writeBufferDequantResize(dsp_buff, dsp_size, uv_buff, uv_size, ubwc_buff, ubwc_size, height, width, scale, zero_point)))
	{
		UBWC_LOGE("UBWCDMA Write Dequant Resize error, errno = 0x%x.\n", ret);
		return -1;
	}
	
	return 0;	
}

int nv122ubwc_dsp_final(const unsigned char *src_buff, const unsigned char *uv_buff, unsigned char *dst_buff, int height, int width, int ubwc_size, int dsp_size, int uv_size, float scale, int zero_point)
{
	uint8_t *ubwc_buff = dst_buff;
	const uint8_t *dsp_buff = src_buff;
	int ret = 0;

	//UBWC_LOGI("nv122ubwc_dsp_final height = %d width = %d pad_height = %d pad_width= %d.\n", height & 0xffff, width & 0xffff, pad_height, pad_width);
	
	if(ubwc_size < 0 || dsp_size < 0)
	{
		UBWC_LOGE("nv122ubwc_dsp_final ubwc_size(%d) or dsp_size(%d) error.\n", ubwc_size, dsp_size);
		return -1;
	}
	
	if(ubwc_buff == NULL || dsp_buff == NULL || uv_buff == NULL)
	{
		UBWC_LOGE("nv122ubwc_dsp_final input buffer is NULL.\n");
		return -1;		
	}

	if((ret = aiboostubwc_writeBufferFinal(dsp_buff, dsp_size, uv_buff, uv_size, ubwc_buff, ubwc_size, height, width, scale, zero_point)))
	{
		UBWC_LOGE("UBWCDMA Write Final error, errno = 0x%x.\n", ret);
		return -1;
	}

	//printf("Resize cost %d us\n", *(int *)uv_buff);
	
	return 0;	
}

int nv12_dsp_dequant_resize(unsigned char *nv12_buff, unsigned char *pad_buff, int height, int width, int nv12_size, int pad_size, float scale, int zero_point)
{
	int ret = 0;
	if((nv12_buff == NULL) || ((pad_buff == NULL) && pad_size))
	{
		UBWC_LOGE("nv12_dsp_dequant_resize input buffer is NULL.\n");
		return -1;
	}
		
	if((ret = aiboostubwc_dequantResize(nv12_buff, nv12_size, pad_buff, pad_size, height, width, scale, zero_point)))
	{
		UBWC_LOGE("NV12 Dequant Resize error, errno = 0x%x.\n", ret);
		return -1;
	}

	return 0;	
}

int depth_to_space_d16b4(const unsigned char *in_buff, unsigned char *out_buff, int height, int width)
{
	int ret = 0;
	int size = height * width * 16;

	if(in_buff == NULL || out_buff == NULL)
	{
		UBWC_LOGE("depth_to_space_d16b4 input buffer is NULL.\n");
		return -1;		
	}

	if((ret = aiboostubwc_depthToSpaceD16B4(in_buff, size, out_buff, size, height, width)))
	{
		UBWC_LOGE("DepthToSpace error, errno = 0x%x.\n", ret);
		return -1;
	}

	return 0;
}

#if 0
int convolution(const unsigned char *input, int input_size, char *weight, int weight_size, unsigned char *output, int output_size, int *recip, int recip_size, int height, int width)
{
	int ret = 0;

	if((ret = aiboostubwc_convolution(input, input_size, (const signed char *)weight, weight_size, output, output_size, recip, recip_size, height, width)))
	{
		UBWC_LOGE("Convolution error, errno = 0x%x.\n", ret);
		return -1;
	}

	return 0;
}
#endif

#if 0
int memcpy_dsp(const unsigned char *src_buff, unsigned char *dst_buff, int src_size, int dst_size)
{
	int ret = 0;
	
	if(src_buff == NULL || dst_buff == NULL)
	{
		UBWC_LOGE("MemcpyDsp buffer is NULL.\n");
		return -1;		
	}

	if((ret = aiboostubwc_memcpy(src_buff, src_size, dst_buff, dst_size)))
	{
		UBWC_LOGE("MemcpyDsp error, errno = 0x%x.\n", ret);
		return -1;
	}
	
	return 0;	
}
#endif

int ubwc_resizebilinear_dsp(const unsigned char* ubwc_in, int ubwcin_size, unsigned char *ubwc_out, int ubwcout_size,
											unsigned char* nv12_in, int nv12in_size, unsigned char* nv12_out, int nv12out_size,
											int height, int width)
{
	int ret = 0;

	/*if(ubwc_in == NULL || ubwc_out == NULL || nv12_in == NULL || nv12_out == NULL)
	{
		UBWC_LOGE("Ubwc_resizebilinear_dsp buffer is NULL.\n");
		return -1;		
	}*/
	
	if((ret = aiboostubwc_resizeBilinear(ubwc_in, ubwcin_size, ubwc_out, ubwcout_size, nv12_in, nv12in_size, nv12_out, nv12out_size, height, width)))
	{
		UBWC_LOGE("Ubwc_resizebilinear_dsp error, errno = 0x%x.\n", ret);
		return -1;
	}

    return ret;
}

#if 0
int ubwc_gaussian5x5_dsp(const unsigned char* nv12_in, int nv12in_size, 
								unsigned char* nv12_out, int nv12out_size,
								unsigned char* table, int table_size,
								int height, int width)
{
	int ret = 0;

	if((ret = aiboostubwc_gaussian5x5(nv12_in, nv12in_size, nv12_out, nv12out_size, table, table_size, height, width)))
	{
		UBWC_LOGE("ubwc_gaussian5x5 error, errno = 0x%x.\n", ret);
		return -1;
	}

    return ret;
}
int ubwc_osie_dsp(const unsigned char* ubwc_in, int ubwcin_size, unsigned char *ubwc_out, int ubwcout_size,
				  unsigned char *nv12_in, int nv12_in_size,
				  unsigned char *nv12_out, int nv12_out_size,
				  unsigned char *lut_buff, int lut_buff_size,
				  int height, int width)
{
	int ret = 0;

	if(ubwc_in == NULL || ubwc_out == NULL || nv12_in == NULL || nv12_out == NULL)
	{
		UBWC_LOGE("ubwc_osie_dsp buffer is NULL.\n");
		return -1;		
	}
	
	if((ret = aiboostubwc_osie(ubwc_in, ubwcin_size, ubwc_out, ubwcout_size, 
			nv12_in, nv12_in_size, nv12_out, nv12_out_size, lut_buff, lut_buff_size, height, width)))
	{
		UBWC_LOGE("ubwc_osie_dsp error, errno = 0x%x.\n", ret);
		return -1;
	}

    return ret;
}
#endif

#pragma weak remote_handle_control  // Declare it as a weak symbol
void dsp_global_init() 
{
	
	rpcmem_init();
	// Non-domains QoS invocation
	struct remote_rpc_control_latency data;
	data.enable = 1;
	if (remote_handle_control) {  // Check if API is available before invoking
	 remote_handle_control(DSPRPC_CONTROL_LATENCY, (void*)&data, sizeof(data));
	}
}
 
void dsp_global_teardown() 
{ 
	rpcmem_deinit(); 
}

void* alloc_ionbuff(int size, ionbuff_flag_t flag)
{
	unsigned char *buff = NULL;
	if(flag == IONBUFF_UNCACHED) buff = (uint8_t*)rpcmem_alloc(RPCMEM_HEAP_ID_SYSTEM, RPCMEM_FLAG_UNCACHED, size);
	else buff = (uint8_t*)rpcmem_alloc(RPCMEM_HEAP_ID_SYSTEM, RPCMEM_DEFAULT_FLAGS, size);
	if (buff == NULL)
	{
	    UBWC_LOGE("rpcmem_alloc error.\n");
		return NULL;
	}

	return (void *)buff;
}

void release_ionbuff(void *buff)
{
	rpcmem_free(buff);
}

extern void remote_register_buf(void* buf, int size, int fd);
#pragma weak  remote_register_buf

void register_ionbuff(void *buff, int size, int fd)
{
	if(size < 0)
	{
		UBWC_LOGE("register_ionbuff input fd or size error.\n");
		return;
	}

	if(buff == NULL)
	{
		UBWC_LOGE("register_ionbuff input buffer is NULL.\n");
		return;
	}

	return remote_register_buf(buff, size, fd);
}

int ionbuff_to_fd(void *buff)
{
	return rpcmem_to_fd(buff);
}

void set_powersave_level(unsigned int level, unsigned int set_case)
{
	aiboostubwc_setPowerSaveLevel(level, set_case);
}

int reset_powersave_level()
{
	return aiboostubwc_resetPowerSaveLevel();
}

