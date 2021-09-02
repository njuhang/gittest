#include "AEEStdErr.h"
#include "AEEStdErr.h"
//#include "ubwc.h"
#include "rpcmem.h"
#include <stdlib.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <dlfcn.h>
#include <unistd.h>
//#include "dspCV.h"
#include <sys/time.h>
#include <string.h>

#include <sys/mman.h>


#include "ubwc_interface.h"

#define ALIGNUP(a, b) (((a) + ((b) - 1)) & ~((b) - 1))

#if 0
#include "remote.h"
#pragma weak remote_session_control

int hexnn_controller_request_unsigned_pd() 
{
	int ret = -1;
	if (remote_session_control) {
		struct remote_rpc_control_unsigned_module data;
		data.enable = 1;
		data.domain = CDSP_DOMAIN_ID;
		ret = remote_session_control(DSPRPC_CONTROL_UNSIGNED_MODULE,
		(void *) &data,
		sizeof(data));
	}
	return ret;
}
#endif

unsigned long long GetTime(void)
{
    struct timeval tv;

    gettimeofday(&tv, NULL);

    return tv.tv_sec * 1000000ULL + tv.tv_usec;
}

#if 0
int main(int argc, char **argv)
{
	uint8_t *ubwc_buff = NULL;
	uint8_t *dsp_buff = NULL;

	if(argc < 4)
	{
		printf("%s widht height do_read[0/1]\n", argv[0]);
		return -1;
	}
	
	int width = atoi(argv[1]);
	int height = atoi(argv[2]);
	int do_read = atoi(argv[3]);

	unsigned long long t1 = 0, t2 = 0;
	
	rpcmem_init();
	//hexnn_controller_request_unsigned_pd();

#ifndef __hexagon__
    int retVal; 
    dspCV_Attribute attrib[] =
    {
        {DSP_TOTAL_MCPS, 1000},
        {DSP_MCPS_PER_THREAD, 500},
        {PEAK_BUS_BANDWIDTH_MBPS, 12000},
        {BUS_USAGE_PERCENT, 100},
                                          
    };

    retVal = dspCV_initQ6_with_attributes(attrib, sizeof(attrib)/sizeof(attrib[0]));
	
    if (retVal != 0)
    {
        printf("Error initializing the Q6 for this application\n");
       	return -1;
    }
#endif
	int dsp_size = ubwc_getFrameSize(height, width, 0);
	int ubwc_size = ubwc_getFrameSize(height, width, 1);
	if(ubwc_size < 0 || dsp_size < 0)
	{
		printf("height or width error.\n");
		return -1;
	}
	
	printf("ubwc_size = %d\n", ubwc_size);

	ubwc_buff = (uint8_t*)rpcmem_alloc(RPCMEM_HEAP_ID_SYSTEM, RPCMEM_FLAG_UNCACHED, ubwc_size);
	if (ubwc_buff == NULL)
	{
	    printf("rpcmem_alloc failed!\n");
		return -1;
	}

	dsp_buff = (uint8_t*)rpcmem_alloc(RPCMEM_HEAP_ID_SYSTEM, RPCMEM_FLAG_UNCACHED, dsp_size);
	if (dsp_buff == NULL)
	{
	    printf("rpcmem_alloc failed!\n");
		return -1;
	}


	if(do_read)
	{
	#if 0	
		srand(0x2345678);
		for(int i = 0; i < ubwc_size; i++)
		{
			ubwc_buff[i] = rand() % 256;
		}
	#else
		FILE *fp_w = fopen("ubwc_input.bin", "r");
		if(fp_w == NULL)
		{
			printf("open ubwc_input.bin failed.\n");
			return -1;
		}
		fread(ubwc_buff, 1, ubwc_size, fp_w);
		fclose(fp_w);
	#endif
		t1 = GetTime();

		if(ubwc_readBuffer(ubwc_buff, ubwc_size, dsp_buff, dsp_size, height, width) < 0)
		//if(ubwc_transBuffer(ubwc_buff, ubwc_size, dsp_buff, dsp_size, height, width, 1) < 0)
		{
			printf("UBWCDMA Read failed!\n");
			return -1;
		}

		t2 = GetTime();

		if(dsp_size == ubwc_size)
		{
			int j = 0;
			for(j = 0; j < ubwc_size; j++)
			{
				if(ubwc_buff[j] != dsp_buff[j])
				{
					printf("Result compare error.\n");
					break;
				}
			}
			if(j == ubwc_size)
			{
				printf("Result compare success.\n");
			}
		}

	#if 0
		FILE *fp_w = fopen("ubwc_buff.bin", "w");
		if(fp_w == NULL)
		{
			printf("open ubwc_buff.bin failed.\n");
			return -1;
		}
		fwrite(ubwc_buff, 1, ubwc_size, fp_w);
		fclose(fp_w);
	#endif
		fp_w = fopen("dsp_buff.bin", "w");
		if(fp_w == NULL)
		{
			printf("open dsp_buff.bin failed.\n");
			return -1;
		}
		fwrite(dsp_buff, 1, dsp_size, fp_w);
		fclose(fp_w);
	}

	else
	{
		FILE *fp_w = fopen("dsp_buff.bin", "r");
		if(fp_w == NULL)
		{
			printf("open dsp_buff.bin failed.\n");
			return -1;
		}
		fread(dsp_buff, 1, dsp_size, fp_w);
		fclose(fp_w);

		t1 = GetTime();
		
		//if(ubwc_writeBuffer(dsp_buff, dsp_size, ubwc_buff, ubwc_size, height, width) < 0)
		if(ubwc_transBuffer(dsp_buff, dsp_size, ubwc_buff, ubwc_size, height, width, 0) < 0)
		{
			printf("UBWCDMA Read failed!\n");
			return -1;
		}

		t2 = GetTime();

		fp_w = fopen("ubwc_output.bin", "w");
		if(fp_w == NULL)
		{
			printf("open ubwc_output.bin failed.\n");
			return -1;
		}
		fwrite(ubwc_buff, 1, ubwc_size, fp_w);
		fclose(fp_w);
	}

	printf("UBWC cost time %lldus\n", t2 - t1);
	
	rpcmem_free(ubwc_buff);
	rpcmem_free(dsp_buff);
	
	return 0;
}
#endif

int depth_to_space_local(const unsigned char *input_data, unsigned char *output_data, int height, int width)
{
	const int depth = 16;
	const int block_size = 4;

	for(int h = 0; h < height; h++)
	{
		const unsigned char *input_ptr = input_data + h * width * depth;
		for(int b = 0; b < block_size; b++)
		{
			const unsigned char *src = input_ptr;
			for(int w = 0; w < width; w++)
			{
				memcpy(output_data, src, block_size);
				output_data += block_size;
				src += depth;
			}
			input_ptr += block_size;
		}
	}

	return 0;
}

#if 1
void convolution_local(unsigned char *input, char *weight, unsigned char *output, int* recip, int height, int width)
{
	int outw = width / 2;
	int outh = height / 2;

	for(int oy = 0; oy < outh; oy++)
	{
		for(int ox = 0; ox < outw; ox++)
		{		
			for(int oc = 0; oc < 32; oc++)
			{
				int sum = 0;
				for(int kh = 0; kh < 3; kh++)
				{
					for(int kw = 0; kw < 3; kw++)
					{
						int iy = oy * 2 + kh;
						int ix = ox * 2 + kw;
						if(((unsigned int)ix < (unsigned int)width) && ((unsigned int)iy < (unsigned int)height)) 
						{
							int indata = input[iy * width + ix];
							int weightdata = weight[(kh * 3 + kw) * 32 + oc];
							if(weightdata & 0x80) weightdata |= 0xffffff00;
							
							sum += indata * weightdata;
						}
					}
				}
				
				sum = ((sum << recip[oc + 32]) * recip[oc]);//+ (1<<15));
				//sum >>= 16;
		    	//if (sum < 0) sum = 0;
	            //if (sum > 255) sum = 255;
				
				output[(oy * outw + ox) * 32 + oc] = (uint8)sum;
			}
		}
	}
}
#endif

#if 0
static const int GAUSS_5x5[5*5] = {
   13,   63,  103,  63,  13,
   63,  305,  499, 305,  63,
   103, 499,  815, 499,  103,
   63,  305,  499, 305,  63,
   13,   63,  103,  63,  13,
};

static void Gaussian5x5_ref(
    unsigned char *src,
    int            width,
    int            height,
    int            stride,
    unsigned char *dst,
    int            dstStride
    )
{

    int x, y, s, t;
    int sum, out;

    for (y = 2; y < height - 2; y++)
    {
        for (x = 2; x < width - 2; x++)
        {
            sum = 0;

            for (t = -2; t <= 2; t++)
            {
                for (s = -2; s <= 2; s++)
                {
                    sum += src[(y+t)*stride + x+s] * GAUSS_5x5[((t+2)*5)+(s+2)];
                }
            }

            out  = sum >> 12;

            out = out < 0 ? 0 : out > 255 ? 255 : out;

            dst[y*dstStride + x] = (unsigned char)out;
        }
    }
}
#endif

int main(int argc, char**argv)
{
	if(argc < 3)
	{
		printf("%s widht height [loops]\n", argv[0]);
		return -1;
	}

	int loops = 1;
	
	int width = atoi(argv[1]);
	int height = atoi(argv[2]);

	if(argc > 3) loops = atoi(argv[3]);

#if 0
	(void)width; (void)height;
	set_powersave_level(102, 1);
	printf("getchar......\n");
	getchar();

	set_powersave_level(255, 0);
	printf("getchar......\n");
	getchar();
	
	set_powersave_level(0x7fffffff, 1);
	printf("getchar......\n");
	getchar();

	return 0;

#if 0
	int input_size = width * height;
	int weight_size = 3 * 3 * 32;
	int output_size =  width * height / 4 * 32;

	unsigned char *input = alloc_ionbuff(input_size, IONBUFF_DEFAULT);
	char *weight = alloc_ionbuff(weight_size, IONBUFF_DEFAULT);
	unsigned char *output = alloc_ionbuff(output_size, IONBUFF_DEFAULT);
	unsigned char *output_local = alloc_ionbuff(output_size, IONBUFF_DEFAULT);
	int *recip = alloc_ionbuff(65 * sizeof(int), IONBUFF_DEFAULT);

	for(int i = 0; i < input_size; i++) input[i] = rand() & 0xff;
	for(int i = 0; i < weight_size; i++) weight[i] = rand() & 0xff;
	for(int i = 0; i < 32; i++) {recip[i] = 0; recip[i + 32] = 14626804;}
	recip[64] = 0;

	convolution_local(input, weight, output_local, recip, height, width);

	for(int i = 0; i < loops; i++)
	{
		unsigned long long t1 = GetTime();
		if(convolution(input, input_size, weight, weight_size, output, output_size, recip, 64, height, width) < 0)
		{
			return -1;
		}
		unsigned long long t2 = GetTime();
		printf("[ROUND %d]convolution cost time %lldus.\n", i, t2 - t1);
	}
	
	printf("output[0] = %d, output[%d] = %d\n", output[0], output_size - 1, output[output_size - 1]);

#if 1
	for(int i = 0; i < output_size; i++)
	{
		if(output[i] != output_local[i])
		{
			printf("Result check failed, output[%d] = %d, output_local[%d] = %d\n", i, output[i], i, output_local[i]);
			return -1;
		}
	}

	printf("Result check success!\n");
#endif
#endif
#else
#if 0
#if 0
	srand(0x2345678);

	int size = width * height * 16;
	
	unsigned char *in_buff = alloc_ionbuff(size, IONBUFF_DEFAULT);
	unsigned char *out_buff = alloc_ionbuff(size, IONBUFF_DEFAULT);
	unsigned char *out_buff_local = alloc_ionbuff(size, IONBUFF_DEFAULT);

	for(int i = 0; i < size; i++) in_buff[i] = rand() & 0xff;
	
	depth_to_space_local(in_buff, out_buff_local, height, width);

	for(int i = 0; i < loops; i++)
	{
		memcpy(out_buff, in_buff, size);
		unsigned long long t1 = GetTime();
		depth_to_space_d16b4(out_buff, out_buff, height, width);
		unsigned long long t2 = GetTime();

		//unsigned long long t3 = GetTime();
		//depth_to_space_local(in_buff, out_buff_local, height, width);
		//unsigned long long t4 = GetTime();

		//printf("[ROUND %d]Depthtospace dsp cost time %lldus, local cost time %lldus.\n", i, t2 - t1, t4 - t3);
		printf("[ROUND %d]Depthtospace dsp cost time %lldus.\n", i, t2 - t1);
	}
	
	for(int i = 0; i < size; i++)
	{
		if(out_buff_local[i] != out_buff[i])
		{
			printf("Result check failed, out_buff[%d] = %d, out_buff_local[%d] = %d\n", i, out_buff[i], i, out_buff_local[i]);
			return -1;
		}
	}

	printf("Result check success!\n");
	return 0;
#else
	int fd = open("ubwc_input.bin", O_RDWR);
	if(fd < 0) return -1;

	int size = width * height;
	unsigned char *pbuff = (unsigned char *)mmap(NULL, size, PROT_READ|PROT_WRITE, MAP_SHARED, fd, 0); 
	unsigned char *ubwc_buff = alloc_ionbuff(size, IONBUFF_UNCACHED);

	printf("pbuff = %p\n", pbuff);
	
	if(pbuff == NULL || ubwc_buff == NULL) return -1;

	for(int i = 0; i < loops; i++)
	{
		unsigned long long t1 = GetTime();
		memcpy(ubwc_buff, pbuff, size);
		unsigned long long t2 = GetTime();

		printf("[ROUND %d]memcpy cost time %lldus.\n", i, t2 - t1);
	}

	munmap(pbuff, size);
	close(fd);
	
	return 0;
#endif
#else
#if 0
	//int loop_num = 0;
	#define CLAMP(x, min, max)	(((x) < (min)) ? (min) : ( ((x) > (max)) ? (max) : (x) ))

	FILE *fp = fopen("dsp_buff0.bin", "r");
	if(fp == NULL)
	{
		printf("No dsp_buff0.bin.\n");
		return -1;
	}

	int size = (height * width * 3) >> 1;
	
	unsigned char *src_buff = alloc_ionbuff(size, IONBUFF_DEFAULT);
	unsigned char *dst_buff = alloc_ionbuff(size, IONBUFF_DEFAULT);
	unsigned char *table = alloc_ionbuff(256, IONBUFF_DEFAULT);
	fread(src_buff, 1, size, fp);
	fclose(fp);

	for (int i = 0; i < 256;i++) 
	{
		float value = (i * 1.0) / 255.0;
		float tmp_2;
		if((i<128)&&(i>=98)) {
		   tmp_2 =	(((value - 0.275) * (3.5 * (value - 0.275) + 0.35)) / ((value - 0.275) * (2.2 * (value - 0.275) + 1.9) + 0.5)) + 0.275; ; 
		} else if(i>=128) {
		   tmp_2 = value + 0.02;   
		} else {
		   tmp_2 = value;
		}
		tmp_2 = CLAMP((tmp_2 - 0.01),0.0,1.0);	 

		unsigned char tmp = (unsigned char)(tmp_2 * 255.0);
		table[i] = tmp;
	} 

	for(int i = 0; i < loops; i++)
	{
		unsigned long long t1 = GetTime();
		ubwc_gaussian5x5_dsp((const unsigned char *)src_buff, size, dst_buff, size, table, 256, height, width);
		(void)Gaussian5x5_ref;
		//Gaussian5x5_ref(src_buff, width, height, width, dst_buff, width);
		unsigned long long t2 = GetTime();

		printf("[ROUND %d]ubwc_gaussian5x5_dsp cost time %lldus.\n", i, t2 - t1);
	}

	//memcpy(dst_buff + height * width, src_buff+ height * width, height * width / 2);

	fp = fopen("dsp_buff.bin", "w");
	if(fp == NULL)
	{
		printf("No dsp_buff.bin.\n");
		return -1;
	}

	fwrite(dst_buff, 1, size, fp);
	fclose(fp);

	return 0;

#if 0
	FILE *fp = fopen("ubwc_input.bin", "r");
	if(fp == NULL)
	{
		return -1;
	}
	
	int ubwcin_size = get_ubwcsize(height, width);
	printf("ubwcin_size = %d\n", ubwcin_size);
	int outw = width << 1;
	int outh = height << 1;
	int ubwcout_size = get_ubwcsize(height << 1, width << 1);
	int dsp_size = outw * outh * 3 / 2;
	unsigned char *ubwcin_buff = alloc_ionbuff(ubwcin_size, IONBUFF_UNCACHED);
	unsigned char *ubwcout_buff = alloc_ionbuff(ubwcout_size, IONBUFF_UNCACHED);
	unsigned char *dsp_buff = alloc_ionbuff(dsp_size, IONBUFF_DEFAULT);

	fread(ubwcin_buff, 1, ubwcin_size, fp);
	ubwc_resizebilinear_dsp(ubwcin_buff, ubwcin_size, ubwcout_buff, ubwcout_size, NULL, 0, NULL, 0, height, width);
	
	fp = fopen("dsp_buff.bin", "w");
	if(fp == NULL)
	{
		return -1;
	}
	
	nv122ubwc_dsp(dsp_buff, ubwcout_buff, outh, outw, ubwcout_size, dsp_size);

	fwrite(dsp_buff, 1, dsp_size, fp);
	fclose(fp);

	release_ionbuff(dsp_buff);
	release_ionbuff(ubwcin_buff);
	release_ionbuff(ubwcout_buff);
#endif
#else
	rpcmem_init();
	FILE *fp = fopen("ubwc_input.bin", "r");
	if(fp == NULL)
	{
		fp = fopen("dsp_buff.bin", "r");
		if(fp == NULL)
		{
			printf("No ubwc_input.bin or dsp_buff.bin.\n");
			return -1;
		}

		printf("Generate ubwc by nv12...\n");
		fseek(fp, 0, SEEK_END);
		long file_size = ftell(fp);
		rewind(fp);

		if(file_size != (height * width * 3) >> 1)
		{
			printf("File size error.\n");
			return -1;			
		}

		unsigned char *dsp_buff = alloc_ionbuff(file_size, IONBUFF_DEFAULT);
		fread(dsp_buff, 1, file_size, fp);
		fclose(fp);

		int ubwc_size = get_ubwcsize(height, width);
		printf("ubwc_size = %d\n", ubwc_size);
		unsigned char *ubwc_buff = alloc_ionbuff(ubwc_size, IONBUFF_UNCACHED);

		nv122ubwc_dsp(dsp_buff, ubwc_buff, height, width, ubwc_size, file_size);

		fp = fopen("ubwc_input.bin", "w");
		fwrite(ubwc_buff, 1, ubwc_size, fp);
		fclose(fp);

		release_ionbuff(dsp_buff);
		release_ionbuff(ubwc_buff);

		return 0;
	}

	fseek(fp, 0, SEEK_END);
	long file_size = ftell(fp);
	rewind(fp);

	unsigned char *buff = malloc(file_size);
	if(buff == NULL)
	{
	  printf("malloc failed.\n");
	  return -1;
	}
	
	fread(buff, 1, file_size, fp);
	fclose(fp);

	//int dsp_size = (ALIGNUP(height, 32) * ALIGNUP(width, 128) * 3) >> 1;
	int dsp_size = (height * width * 3) >> 1;

	
	unsigned char *dsp_buff = alloc_ionbuff(dsp_size, IONBUFF_DEFAULT);
	int ubwc_size = get_ubwcsize(height, width);
	unsigned char *ubwc_buff = alloc_ionbuff(ubwc_size, IONBUFF_UNCACHED);

	printf("ubwc_size = %d\n", ubwc_size);

	for(int i = 0; i < loops; i++)
	{
		memcpy(ubwc_buff, buff, ubwc_size);
		
		unsigned long long t1 = GetTime();
		ubwc2nv12_dsp(ubwc_buff, dsp_buff, height, width, ubwc_size, dsp_size);
		unsigned long long t2 = GetTime();

		fp = fopen("dsp_buff.bin", "w");
		if(fp == NULL)
		{
			printf("open dsp_buff.bin failed.\n");
			return -1;
		}
		
		fwrite(dsp_buff, 1, dsp_size, fp);
		fclose(fp);

    	unsigned long long t3 = GetTime();
	    nv122ubwc_dsp(dsp_buff, ubwc_buff, height, width, ubwc_size, dsp_size);
	    //nv122ubwc_dsp_dequant(dsp_buff, ubwc_buff, height, width, ubwc_size, 0.004, 0);
		unsigned long long t4 = GetTime();
			
		fp = fopen("ubwc_output.bin", "w");
		if(fp == NULL)
		{
			printf("open ubwc_output.bin failed.\n");
			return -1;
		}
		
		fwrite(ubwc_buff, 1, ubwc_size, fp);
		fclose(fp);
		
		printf("[ROUND %d]ubwc2nv12_dsp cost time %lldus\n", i, t2 - t1);
		printf("[ROUND %d]nv122ubwc_dsp cost time %lldus\n", i, t4 - t3);
	}
	
	release_ionbuff(dsp_buff);
	release_ionbuff(ubwc_buff);
	free(buff);

	//set_powersave_level(0x7fffffff);

	//printf("getchar......\n");
	//getchar();
#endif
#endif
#endif
	return 0;
}

