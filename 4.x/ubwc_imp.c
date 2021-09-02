#include "aiboostubwc.h"
#include "dma_def.h"
#include "dma_types.h"
#include "dmaWrapper.h"
#include "vqf16.h"
#include "HAP_power.h"
#include "dsp_internal.h"

#if (defined DEQUANT_MULTI_THREADS) || (defined RESIZE_MULTI_THREADS) || (defined UBWC_MULTI_THREADS)
#include "worker_pool.h"
#endif

#define MAXTHREADS (4)
#define ENABLE_OSIE

#ifdef DEQUANT_MULTI_THREADS
typedef struct
{
    worker_synctoken_t *token;
	const unsigned char *dsp_buff;
	unsigned char *dequant_buff;
	uint32 buff_size;
	float scale;
	int zero_point;
	unsigned int jobCount;
	unsigned int sizePerJob;
}dequant_callback_t;
#endif

#ifdef RESIZE_MULTI_THREADS
typedef struct
{
    worker_synctoken_t *token;
	unsigned char *uv_input;
	unsigned char *uv_output;
	unsigned char *uv_buff;
	int uv_in_height;
	int uv_in_width;
	int uv_out_width;
	unsigned int jobCount;
	unsigned int h_in_block;
	unsigned int h_out_block;
}resize_callback_t;

typedef struct
{
    worker_synctoken_t *token;
	unsigned char *nv12_input;
	unsigned char *nv12_output;
	int width;
	int height;
	unsigned int jobCount;
	unsigned int sizePerJob;
	int numWorkers;
	int nRet[MAXTHREADS];
}resize_callback2_t;
#endif

#ifdef UBWC_MULTI_THREADS
typedef struct
{
    worker_synctoken_t *token;
	unsigned char *ubwc_buff;
	unsigned char *dsp_buff;
	int nFrameHeight;
	int nFrameWidth;
	int totalHeight;
	unsigned int jobCount;
	int numThreads;
	t_DmaWrapper_DmaEngineHandle handle[MAXTHREADS];
	int nRet[MAXTHREADS];
	float scale;
	int zero_point;
	unsigned char *dsp_buff2;
}ubwc_callback_t;
#endif

#define ISUBWC (TRUE)
#define PAD16BIT (FALSE)

typedef struct iRoiInfo
{
	int nRoiWalkHeight;
	int nRoiWalkWidth;
	int nFrameHeight;
	int nFrameWidth;
	qurt_addr_t tcm_buf_vaddr;
	qurt_size_t ChromaOffset;
	qurt_size_t tcm_buf_size;
}iRoiInfo_t;

int depth_to_space_d16b4(const unsigned char *input_data, unsigned char *output_data, int height, int width);
int depth_to_space_d16b4_dequant(const unsigned char *input_data, unsigned char *output_data, int height, int width, float scale, int zero_point);
void depthtospaceToTcmBuffer(void *tcm_buffer, const uint8* dsp_buff, 
							int h, int w, int hb, int wb, 
							int nColIdx, int nRowIdx, int stride_dst, int stride_src, 
							int offset, float scale, int zero_point);
int conv3x3x32x1S2(const unsigned char* input, const char *weight,  unsigned char *output, const int *recip, int height, int width);

static void writeDspBuffer(void *tcm_buffer, uint8* dsp_buff, int h, int w, int hb, int wb, int nColIdx, int nRowIdx, int stride_src, int stride_dst, int offset)
{	
	if(((stride_dst | w | (int)dsp_buff | (int)tcm_buffer | offset) & 127) == 0)
	{
		for(int i = 0; i < h; i++)
		{
		    vmemcpy_128(dsp_buff + offset + (nRowIdx * hb + i) * stride_dst + nColIdx * wb, (uint8 *)tcm_buffer + i * stride_src, w);
		}
	}
	else
	{
		for(int i = 0; i < h; i++)
		{
		    vmemcpy_asm(dsp_buff + offset + (nRowIdx * hb + i) * stride_dst + nColIdx * wb, (uint8 *)tcm_buffer + i * stride_src, w);
		    //memcpy(dsp_buff + offset + (nRowIdx * h + i) * stride_dst + nColIdx * wb, (uint8 *)tcm_buffer + i * stride_src, w);
		}	
	}
}

static void writeTcmBuffer(void *tcm_buffer, const uint8* dsp_buff, int h, int w, int hb, int wb, int nColIdx, int nRowIdx, int stride_dst, int stride_src, int offset)
{	
	if(((stride_src | w | (int)dsp_buff | (int)tcm_buffer | offset) & 127) == 0)
	{
		for(int i = 0; i < h; i++)
		{
		    vmemcpy_128((uint8 *)tcm_buffer + i * stride_dst, dsp_buff + offset + (nRowIdx * hb + i) * stride_src + nColIdx * wb, w);

		}
	}
	else
	{
		for(int i = 0; i < h; i++)
		{
		    vmemcpy_asm((uint8 *)tcm_buffer + i * stride_dst, dsp_buff + offset + (nRowIdx * hb + i) * stride_src + nColIdx * wb, w);

		    //memcpy((uint8 *)tcm_buffer + i * stride_dst, dsp_buff + offset + (nRowIdx * h + i) * stride_src + nColIdx * wb, w);

		}
	}		
}

static void padDspBUffer(uint8* dsp_buff, int h, int w, int padh, int padw)
{
	#define PAD_SIZE (8)

	int uvpad_flag = padh & 0x8000;
	padh &= 0x7fff; 

	if(padh < h) padh = h;
	if(padw < w) padw = w;

	const int ph = MIN(padh - h, PAD_SIZE);
	const int pw = MIN(padw - w, PAD_SIZE);

	uint8* dsp_buff_bak = dsp_buff;
	
	for(int i = 0; i < h; i++)
	{

		uint8* buff = dsp_buff + i * padw + w;

		uint8 value = *(buff - 1);
		memset(buff, value, pw);
	}

	dsp_buff += h * padw;
	uint8 *padh_buff = dsp_buff - padw;

	for(int i = 0; i < ph; i++)
	{
		vmemcpy_asm(dsp_buff, padh_buff, w + pw);
		dsp_buff += padw;
	}
	
	if(uvpad_flag)
	{
		dsp_buff = dsp_buff_bak + h * padw;
		for(int i = 0; i < h >> 1; i++)
		{
			uint8* buff = dsp_buff + i * padw + w;
			uint8 valueU = *(buff - 2);
			uint8 valueV = *(buff - 1);
			for(int j = 0; j < (pw >> 1); j++)
			{
				buff[j * 2] = valueU;
				buff[j * 2 + 1] = valueV;
			}
		}
	}

	#undef PAD_SIZE
}

static void updateRoiInfo(int nRowIdx, int nRow,
        				int nColIdx, int nCol, int nIdx,
        				iRoiInfo_t *iRoi,
        				t_StDmaWrapper_UpdateParm* stWrapUpdateParm)
{
	for (int j = 0; j < 2; j++)
    {
        stWrapUpdateParm[j].u.stPixData.stRoi.u16Y = iRoi->nRoiWalkHeight * nRowIdx;
        if (nRowIdx == (nRow - 1))
        {
            stWrapUpdateParm[j].u.stPixData.stRoi.u16H = iRoi->nFrameHeight - (iRoi->nRoiWalkHeight * nRowIdx);
        }
        else
        {
            stWrapUpdateParm[j].u.stPixData.stRoi.u16H = iRoi->nRoiWalkHeight;
        }

        stWrapUpdateParm[j].u.stPixData.stRoi.u16X = iRoi->nRoiWalkWidth * nColIdx;
        if (nColIdx == (nCol - 1))
        {
            stWrapUpdateParm[j].u.stPixData.stRoi.u16W = iRoi->nFrameWidth - (iRoi->nRoiWalkWidth * nColIdx);
        }
        else
        {
            stWrapUpdateParm[j].u.stPixData.stRoi.u16W = iRoi->nRoiWalkWidth;
        }

        stWrapUpdateParm[j].aCacheAddr = qurt_lookup_physaddr(iRoi->tcm_buf_vaddr) + 
															(nIdx % 2 ? iRoi->tcm_buf_size : 0) + 
															j * iRoi->ChromaOffset;
	}
}

#ifdef UBWC_MULTI_THREADS
static void updateRoiInfoThreads(int nRowIdx, int nRow,
        				int nColIdx, int nCol, int nIdx,
        				iRoiInfo_t *iRoi,
        				t_StDmaWrapper_UpdateParm* stWrapUpdateParm,
        				int yOff)
{
	for (int j = 0; j < 2; j++)
    {
        stWrapUpdateParm[j].u.stPixData.stRoi.u16Y = iRoi->nRoiWalkHeight * nRowIdx + yOff;
        if (nRowIdx == (nRow - 1))
        {
            stWrapUpdateParm[j].u.stPixData.stRoi.u16H = iRoi->nFrameHeight - (iRoi->nRoiWalkHeight * nRowIdx);
        }
        else
        {
            stWrapUpdateParm[j].u.stPixData.stRoi.u16H = iRoi->nRoiWalkHeight;
        }

        stWrapUpdateParm[j].u.stPixData.stRoi.u16X = iRoi->nRoiWalkWidth * nColIdx;
        if (nColIdx == (nCol - 1))
        {
            stWrapUpdateParm[j].u.stPixData.stRoi.u16W = iRoi->nFrameWidth - (iRoi->nRoiWalkWidth * nColIdx);
        }
        else
        {
            stWrapUpdateParm[j].u.stPixData.stRoi.u16W = iRoi->nRoiWalkWidth;
        }

        stWrapUpdateParm[j].aCacheAddr = qurt_lookup_physaddr(iRoi->tcm_buf_vaddr) + 
															(nIdx % 2 ? iRoi->tcm_buf_size : 0) + 
															j * iRoi->ChromaOffset;
	}
}
#endif

static int g_power_save_case0 = 0;
static int g_power_save_case1 = 0;

int aiboostubwc_setPowerSaveLevel(uint32 level, uint32 set_case)
{
	HAP_power_request_t request;
	memset(&request, 0, sizeof(HAP_power_request_t));
	request.type = HAP_power_set_DCVS_v2;
	request.dcvs_v2.dcvs_enable = FALSE;

	const unsigned int RELEASE_VOTE = 0x7FFFFFFF;
	const unsigned int TURBO_PLUS_UPPER_BOUNDS = 25;
	const unsigned int TURBO_UPPER_BOUNDS = 51;
	const unsigned int NOMINAL_PLUS_UPPER_BOUNDS = 102;
	const unsigned int NOMINAL_UPPER_BOUNDS = 153;
	const unsigned int SVS_PLUS_UPPER_BOUNDS = 204;
	const unsigned int SVS_UPPER_BOUNDS = 255;

	const int LOW_LATENCY = 100;
	const int MEDIUM_LATENCY = 500;
	const int HIGH_LATENCY = 1000;

	if (level == RELEASE_VOTE){
		request.dcvs_v2.dcvs_option = HAP_DCVS_V2_POWER_SAVER_MODE;
	}
	else {
		request.dcvs_v2.dcvs_option = HAP_DCVS_V2_PERFORMANCE_MODE;
		request.dcvs_v2.set_latency = TRUE;
		request.dcvs_v2.set_dcvs_params = TRUE;
		if (level < TURBO_PLUS_UPPER_BOUNDS) {
			request.dcvs_v2.latency = LOW_LATENCY;
			request.dcvs_v2.dcvs_params.max_corner = HAP_DCVS_VCORNER_TURBO_PLUS;
			request.dcvs_v2.dcvs_params.min_corner = HAP_DCVS_VCORNER_TURBO_PLUS;
			request.dcvs_v2.dcvs_params.target_corner = HAP_DCVS_VCORNER_TURBO_PLUS;
		} else if (level < TURBO_UPPER_BOUNDS){
			request.dcvs_v2.latency = LOW_LATENCY;
			request.dcvs_v2.dcvs_params.max_corner = HAP_DCVS_VCORNER_TURBO;
			request.dcvs_v2.dcvs_params.min_corner = HAP_DCVS_VCORNER_TURBO;
			request.dcvs_v2.dcvs_params.target_corner = HAP_DCVS_VCORNER_TURBO;
		} else if (level < NOMINAL_PLUS_UPPER_BOUNDS){
			request.dcvs_v2.latency = LOW_LATENCY;
			request.dcvs_v2.dcvs_params.max_corner = HAP_DCVS_VCORNER_NOMPLUS;
			request.dcvs_v2.dcvs_params.min_corner = HAP_DCVS_VCORNER_NOMPLUS;
			request.dcvs_v2.dcvs_params.target_corner = HAP_DCVS_VCORNER_NOMPLUS;
		} else if (level < NOMINAL_UPPER_BOUNDS){
			request.dcvs_v2.latency = LOW_LATENCY;
			request.dcvs_v2.dcvs_params.max_corner = HAP_DCVS_VCORNER_NOM;
			request.dcvs_v2.dcvs_params.min_corner = HAP_DCVS_VCORNER_NOM;
			request.dcvs_v2.dcvs_params.target_corner = HAP_DCVS_VCORNER_NOM;
		} else if (level < SVS_PLUS_UPPER_BOUNDS){
			request.dcvs_v2.latency = MEDIUM_LATENCY;
			request.dcvs_v2.dcvs_params.max_corner = HAP_DCVS_VCORNER_SVSPLUS;
			request.dcvs_v2.dcvs_params.min_corner = HAP_DCVS_VCORNER_SVSPLUS;
			request.dcvs_v2.dcvs_params.target_corner = HAP_DCVS_VCORNER_SVSPLUS;
		} else if (level < SVS_UPPER_BOUNDS){
			request.dcvs_v2.latency = HIGH_LATENCY;
			request.dcvs_v2.dcvs_params.max_corner = HAP_DCVS_VCORNER_SVS;
			request.dcvs_v2.dcvs_params.min_corner = HAP_DCVS_VCORNER_SVS;
			request.dcvs_v2.dcvs_params.target_corner = HAP_DCVS_VCORNER_SVS;
		} else {
			request.dcvs_v2.set_latency = FALSE;
			request.dcvs_v2.dcvs_params.max_corner = HAP_DCVS_VCORNER_SVS2;
			request.dcvs_v2.dcvs_params.min_corner = HAP_DCVS_VCORNER_SVS2;
			request.dcvs_v2.dcvs_params.target_corner = HAP_DCVS_VCORNER_SVS2;
		}
	}

#if 1
	switch(set_case)
	{
		case 0:
			return HAP_power_set(NULL, &request);
			break;
		case 1:
			return HAP_power_set(&g_power_save_case0, &request);
			break;
		case 2:
			return HAP_power_set(&g_power_save_case1, &request);
			break;
		default:
			return HAP_power_set(NULL, &request);
			break;
	}
#else
	(void)set_case;(void)g_power_save_case0;(void)g_power_save_case1;
#endif

	return HAP_power_set(NULL, &request);
}

int aiboostubwc_resetPowerSaveLevel()
{
	const int LOW_LATENCY = 100;
	
	HAP_power_request_t request;
	memset(&request, 0, sizeof(HAP_power_request_t)); //Remove all votes for NULL context
	request.type = HAP_power_set_DCVS_v2;
	request.dcvs_v2.dcvs_enable = TRUE;
	request.dcvs_v2.dcvs_option = HAP_DCVS_V2_POWER_SAVER_MODE;
	request.dcvs_v2.latency = LOW_LATENCY;
	return HAP_power_set(NULL, &request); //Remove all votes for NULL context
}

int aiboostubwc_getFrameSize(int32 nFrameHeight, int32 nFrameWidth, int32 ubwc_flag)
{
	t_StDmaWrapper_FrameProp stFrameProp;
	t_StDmaWrapper_RoiAlignInfo stAlignInfo;
	int nRet = UBWC_OK;
	int src_buf_size = 0;
	bool isUbwc = ubwc_flag ? TRUE : FALSE;

	nRet = nDmaWrapper_GetFmtAlignment(eDmaFmt_NV12 + 1, isUbwc, &stAlignInfo);
	if(nRet)
	{
		nRet = UBWC_ERR_DMAGETALIGN;
		LOGE("nDmaWrapper_GetFmtAlignment error.");
	    return nRet;
	}

    if(nFrameHeight % (stAlignInfo.u16H) != 0 || nFrameWidth % (stAlignInfo.u16W) != 0)
    {
    	LOGE("nFrameHeight or nFrameWidth error:");
		LOGE("nFrameHeight = %d, nFrameWidth = %d, stAlignInfo.u16H = %d, stAlignInfo.u16W = %d.",
			nFrameHeight, nFrameWidth, stAlignInfo.u16H, stAlignInfo.u16W);
		nRet = UBWC_ERR_INPUTSIZE;
		return nRet;
    }
	
	stFrameProp.aAddr = 0;
    stFrameProp.u16H = nFrameHeight;
    stFrameProp.u16W = nFrameWidth;
    stFrameProp.u16Stride = nFrameWidth;	
	src_buf_size = nDmaWrapper_GetFramesize(eDmaFmt_NV12 + 1, &stFrameProp, isUbwc);
	nRet = nDmaWrapper_GetFmtAlignment(eDmaFmt_NV12 + 2, isUbwc, &stAlignInfo);
	if(nRet)
	{
		nRet = UBWC_ERR_DMAGETALIGN;
		LOGE("nDmaWrapper_GetFmtAlignment error.");
	    return nRet;
	}
	
	stFrameProp.u16H = ALIGNUP(stFrameProp.u16H, stAlignInfo.u16H);
	src_buf_size += nDmaWrapper_GetFramesize(eDmaFmt_NV12 + 2, &stFrameProp, isUbwc);
	
	return src_buf_size;
}

#ifndef UBWC_MULTI_THREADS
int aiboostubwc_readBuffer(const unsigned char *ubwc_buff, int ubwc_size, unsigned char *dsp_buff, int dsp_size, int32 nFrameHeight, int32 nFrameWidth)
{
#ifdef PROFILING_ON
	uint64 startTime = HAP_perf_get_time_us();
	uint64 startCycles = HAP_perf_get_pcycles();
#endif
	t_DmaWrapper_DmaEngineHandle handle;
	t_EDma_WaitType eWaitType = eDmaWaitType_Polling;
	t_eDmaFmt eFmtLuma = eDmaFmt_NV12_Y;
    t_eDmaFmt eFmtChroma = eDmaFmt_NV12_UV;

	t_eDmaFmt efmtLumaChroma[2] = {eFmtLuma, eFmtChroma};

    int nRet = UBWC_OK;
	t_StDmaWrapper_Roi stRoi;
	t_StDmaWrapper_RoiAlignInfo stAlignInfo;
	int lumaStride, chromaStride;
	qurt_addr_t tcm_buf_vaddr, tcm_desc_vaddr;
	unsigned long long tcm_buf_paddr, tcm_desc_paddr;
	qurt_size_t tcm_buf_size, ChromaOffset, tcm_desc_size, region_tcm_size;
	t_StDmaWrapper_PrepareParm stWrapPrepParm;
	t_StDmaWrapper_WorkDescrip staWorkDesc[2];
    t_StDmaWrapper_FrameProp walkRoi;
    t_StDmaWrapper_FrameProp frameProp;
	int nIdx = 0;
    int nRow, nCol;
    int nRowIdx = 0, nColIdx = 0;
	t_StDmaWrapper_UpdateParm stWrapUpdateParm[2];

	int nPadHeight = nFrameHeight >> 16;
	int nPadWidth = nFrameWidth >> 16;
	nFrameHeight &= 0xffff;
	nFrameWidth &= 0xffff;
	int ubwcHeight = ALIGNUP(nFrameHeight, 32);
	int ubwcWidth = ALIGNUP(nFrameWidth, 128);

	qurt_mem_cache_clean((qurt_addr_t)ubwc_buff, ubwc_size, QURT_MEM_CACHE_FLUSH_INVALIDATE, QURT_MEM_DCACHE);
	
	stRoi.u16W = 256;
    stRoi.u16H = 32;
	nRet = nDmaWrapper_GetRecommendedRoi(eDmaFmt_NV12, ISUBWC, &stRoi);
	if(nRet)
	{
		nRet = UBWC_ERR_DMAGETROI;
		LOGE("nDmaWrapper_GetRecommendedRoi error.");
	    return nRet;
	}

	nRow = ROUNDDIV(ubwcHeight, stRoi.u16H);
	nCol = ROUNDDIV(ubwcWidth,  stRoi.u16W);

	LOGI("stRoi.u16W = %d, stRoi.u16H = %d, nRow = %d, nCol = %d", stRoi.u16W, stRoi.u16H, nRow, nCol);

	stAlignInfo.u16W = stRoi.u16W;
    stAlignInfo.u16H = stRoi.u16H;

	lumaStride = nDmaWrapper_GetRecommendedIntermBufStride(eFmtLuma, &stAlignInfo, ISUBWC);
	chromaStride = nDmaWrapper_GetRecommendedIntermBufStride(eFmtChroma, &stAlignInfo, ISUBWC);

	LOGI("lumaStride = %d, chromaStride = %d", lumaStride, chromaStride);

	tcm_buf_size = nDmaWrapper_GetRecommendedIntermBufSize(eFmtLuma, PAD16BIT, &stAlignInfo, ISUBWC, lumaStride);
	ChromaOffset = tcm_buf_size;
	tcm_buf_size += nDmaWrapper_GetRecommendedIntermBufSize(eFmtChroma, PAD16BIT, &stAlignInfo, ISUBWC, chromaStride);
	region_tcm_size = ALIGNUP(tcm_buf_size * 2, 0x1000);
	tcm_desc_size = ALIGNUP(nDmaWrapper_GetDescbuffsize(efmtLumaChroma, 2) * 2 * 2, 0x1000);


	LOGI("tcm_buf_size = %d, region_tcm_size = %d, tcm_desc_size = %d, ChromaOffset = %d",
		  tcm_buf_size, region_tcm_size, tcm_desc_size, ChromaOffset);
		
	tcm_desc_vaddr = (addr_t)HAP_cache_lock((unsigned int)tcm_desc_size, &tcm_desc_paddr);
	tcm_buf_vaddr = (addr_t)HAP_cache_lock((unsigned int)region_tcm_size, &tcm_buf_paddr);
	if(!tcm_desc_vaddr || !tcm_buf_vaddr)
	{
		LOGE("HAP_cache_lock error.");
		nRet = UBWC_ERR_ALLOCTCM;
		return nRet;
	}

	LOGI("tcm_desc_vaddr = 0x%x, tcm_buf_vaddr = 0x%x, tcm_desc_paddr = 0x%x, tcm_buf_paddr = 0x%x", 
			tcm_desc_vaddr, tcm_buf_vaddr, qurt_lookup_physaddr(tcm_desc_vaddr), qurt_lookup_physaddr(tcm_buf_vaddr));

	handle = hDmaWrapper_AllocDmaSpecifyWaitType(eWaitType);
	if(!handle) 
	{
		nRet = UBWC_ERR_ALLOCDMA;
		LOGE("hDmaWrapper_AllocDmaSpecifyWaitType error.");
		goto error;
	}

#if 0
	nRet = nDmaWrapper_PowerVoting(PW_SVS2);
	if(nRet)
	{
		LOGE("nDmaWrapper_PowerVoting error.");
		goto error;
	}
#endif

	stWrapPrepParm.u32NumOfWorkDesc = 2;
    stWrapPrepParm.staWorkDesc = staWorkDesc;
	for (int i = 0; i < 1; i++)
    {
        for (int j = 0; j < 2; j++)
        {
            int descIndex = i * 2 + j;
            frameProp.aAddr = (addr_t)qurt_lookup_physaddr((qurt_addr_t)ubwc_buff);
            frameProp.u16W = ubwcWidth;
            frameProp.u16H = ubwcHeight;
            frameProp.u16Stride = ubwcWidth;
            walkRoi.aAddr = 0;
            walkRoi.u16W = stRoi.u16W;
            walkRoi.u16H = stRoi.u16H;
            walkRoi.u16Stride = (j) ? chromaStride : lumaStride;
            nRet = nDmaWrapper_WorkDescrip_populate(&staWorkDesc[descIndex],
                                                    efmtLumaChroma[j],
                                                    eDmaWrapper_DdrToL2,
                                                    &walkRoi, &frameProp,
                                                    ISUBWC,
                                                    PAD16BIT,
                                                    NULL);
        }
    }
    stWrapPrepParm.u32DescBufSize = nDmaWrapper_GetDescbuffsize(efmtLumaChroma, 2);
	stWrapPrepParm.pPingDescBuf = (void *)tcm_desc_vaddr;
	stWrapPrepParm.pPongDescBuf = (void *)(tcm_desc_vaddr + stWrapPrepParm.u32DescBufSize);
	LOGI("stWrapPrepParm.u32DescBufSize = %d", stWrapPrepParm.u32DescBufSize);
	nRet |= nDmaWrapper_Prepare(handle, &stWrapPrepParm);
	if(nRet)
	{
		nRet = UBWC_ERR_DMAPREPARE;
		LOGE("nDmaWrapper_Prepare error.");
		goto error;
	}

	iRoiInfo_t iRoi;
	iRoi.nFrameHeight = ubwcHeight;
	iRoi.nFrameWidth = ubwcWidth;
	iRoi.nRoiWalkHeight = stRoi.u16H;
	iRoi.nRoiWalkWidth = stRoi.u16W;
	iRoi.tcm_buf_vaddr = tcm_buf_vaddr;
	iRoi.tcm_buf_size = tcm_buf_size;
	iRoi.ChromaOffset = ChromaOffset;
	
	updateRoiInfo(nRowIdx, nRow, nColIdx, nCol, nIdx, &iRoi, &stWrapUpdateParm[0]);
	nRet = nDmaWrapper_Update(handle, &stWrapUpdateParm[0]);
	if(nRet)
	{
		nRet = UBWC_ERR_DMAUPDATE;
		LOGE("nDmaWrapper_Update error.");
		goto error;
	}

	nRet = nDmaWrapper_Move(handle);
	if(nRet)
	{
		nRet = UBWC_ERR_DMAMOVE;
		LOGE("nDmaWrapper_Move error.");
		goto error;
	}
    nIdx = (nIdx + 1) % 2;

	int offsetC = MAX(nFrameHeight, nPadHeight) * MAX(nFrameWidth, nPadWidth); 
	int sizeH = (nRow == 1 && MARGIN(nFrameHeight, stRoi.u16H) != 0) ? MARGIN(nFrameHeight, stRoi.u16H) : stWrapUpdateParm[0].u.stPixData.stRoi.u16H;
	int sizeW = (nCol == 1 && MARGIN(nFrameWidth, stRoi.u16W) != 0) ? MARGIN(nFrameWidth, stRoi.u16W) : stWrapUpdateParm[0].u.stPixData.stRoi.u16W;
	LOGI("nDmaWrapper_Move size = %d\n", sizeH * sizeW);
	
    for (nRowIdx = 0; nRowIdx < nRow; nRowIdx++)
    {
        for (nColIdx = (nRowIdx)? 0:1; nColIdx < nCol; nColIdx++)
        {
			updateRoiInfo(nRowIdx, nRow, nColIdx, nCol, nIdx, &iRoi, &stWrapUpdateParm[0]);
            nRet = nDmaWrapper_Update(handle, &stWrapUpdateParm[0]);
            nRet |= nDmaWrapper_Wait(handle);
			writeDspBuffer((void *)(tcm_buf_vaddr + (nIdx % 2 ? 0 : tcm_buf_size)), dsp_buff, 
				            sizeH, sizeW, stRoi.u16H, stRoi.u16W,
							nColIdx == 0 ? nCol - 1 : nColIdx - 1, 
							nColIdx == 0 ? nRowIdx - 1 : nRowIdx, 
							stRoi.u16W, MAX(nFrameWidth, nPadWidth), 0);
			writeDspBuffer((void *)(tcm_buf_vaddr + (nIdx % 2 ? 0 : tcm_buf_size) + iRoi.ChromaOffset), dsp_buff, 
				            sizeH >> 1, sizeW, stRoi.u16H >> 1, stRoi.u16W,
							nColIdx == 0 ? nCol - 1 : nColIdx - 1, 
							nColIdx == 0 ? nRowIdx - 1 : nRowIdx, 
							stRoi.u16W, nFrameWidth, offsetC);
            nRet |= nDmaWrapper_Move(handle);
			if(nRet)
			{
				nRet = UBWC_ERR_DMAMOVE | UBWC_ERR_DMAUPDATE | UBWC_ERR_DMAWAIT;
				LOGE("nDmaWrapper_Move or nDmaWrapper_Update or nDmaWrapper_Wait error.");
				goto error;
			}		
            nIdx = (nIdx + 1) % 2;
			sizeH = (nRowIdx == (nRow - 1) && MARGIN(nFrameHeight, stRoi.u16H) != 0) ? MARGIN(nFrameHeight, stRoi.u16H) : stWrapUpdateParm[0].u.stPixData.stRoi.u16H;
	        sizeW = (nColIdx == (nCol - 1) && MARGIN(nFrameWidth, stRoi.u16W) != 0) ? MARGIN(nFrameWidth, stRoi.u16W) : stWrapUpdateParm[0].u.stPixData.stRoi.u16W;
			LOGI("nDmaWrapper_Move size = %d", sizeH * sizeW);
        }
    }
	nRet = nDmaWrapper_Wait(handle);
	if(nRet)
	{
		nRet = UBWC_ERR_DMAWAIT;
		LOGE("nDmaWrapper_Wait error.");
		goto error;
	}
	writeDspBuffer((void *)(tcm_buf_vaddr + (nIdx % 2 ? 0 : tcm_buf_size)), dsp_buff, sizeH, sizeW, stRoi.u16H, stRoi.u16W, 
					nCol - 1, nRow - 1, stRoi.u16W, MAX(nFrameWidth, nPadWidth), 0);
	writeDspBuffer((void *)(tcm_buf_vaddr + (nIdx % 2 ? 0 : tcm_buf_size) + iRoi.ChromaOffset), dsp_buff, sizeH >> 1, sizeW, stRoi.u16H >> 1, stRoi.u16W, 
					nCol - 1, nRow - 1, stRoi.u16W, nFrameWidth, offsetC);
	
	nRet = nDmaWrapper_FinishFrame(handle);
	if(nRet)
	{
		nRet = UBWC_ERR_DMAFINISH;
		LOGE("nDmaWrapper_FinishFrame error.");
		goto error;
	}

	nRet = nDmaWrapper_FreeDma(handle);
	if(nRet)
	{
		nRet = UBWC_ERR_DMAFREE;
		LOGE("nDmaWrapper_FreeDma error.");
		goto error;
	}
error:	
	HAP_cache_unlock((void*)tcm_buf_vaddr);
	HAP_cache_unlock((void*)tcm_desc_vaddr);

	padDspBUffer(dsp_buff, nFrameHeight, nFrameWidth, nPadHeight, nPadWidth);
	
#ifdef PROFILING_ON
   	uint64 endCycles = HAP_perf_get_pcycles();
	uint64 endTime = HAP_perf_get_time_us();
	FARF(ALWAYS, "[UBWCREAD]profiling: %d PCycles, %d us. Observed clock rate %d MHz",
		(int)(endCycles - startCycles), (int)(endTime - startTime), (int)((endCycles - startCycles) / (endTime - startTime)));
#endif

	return nRet;
}
#else
static void ubwc_readBuffer_callback(void* data)
{
	int nRet = UBWC_OK;
	unsigned int jobCount = 0;
	ubwc_callback_t *dptr = (ubwc_callback_t *)data;

	jobCount = worker_pool_atomic_inc_return(&(dptr->jobCount)) - 1; 
	
	int nFrameHeight = dptr->nFrameHeight;
	int nFrameWidth = dptr->nFrameWidth & 0xffff;
	int nPadWidth = dptr->nFrameWidth >> 16;
	int totalHeight = dptr->totalHeight & 0xffff;
	int totalPadHeight = (dptr->totalHeight >> 16) & 0x7fff;
	unsigned char *ubwc_buff = dptr->ubwc_buff;
	unsigned char *dsp_buff = dptr->dsp_buff;
	int uvpad_flag = dptr->totalHeight & 0x80000000;

	if(jobCount * nFrameHeight >= totalHeight) goto out;

	if((jobCount == dptr->numThreads - 1) || (totalHeight - jobCount * dptr->nFrameHeight < dptr->nFrameHeight)) 
	{
		nFrameHeight = totalHeight - jobCount * dptr->nFrameHeight;
	}

	t_DmaWrapper_DmaEngineHandle handle = dptr->handle[jobCount];
	//t_EDma_WaitType eWaitType = eDmaWaitType_Polling;
	t_eDmaFmt eFmtLuma = eDmaFmt_NV12_Y;
    t_eDmaFmt eFmtChroma = eDmaFmt_NV12_UV;

	t_eDmaFmt efmtLumaChroma[2] = {eFmtLuma, eFmtChroma};

	t_StDmaWrapper_Roi stRoi;
	t_StDmaWrapper_RoiAlignInfo stAlignInfo;
	int lumaStride, chromaStride;
	qurt_addr_t tcm_buf_vaddr, tcm_desc_vaddr;
	unsigned long long tcm_buf_paddr, tcm_desc_paddr;
	qurt_size_t tcm_buf_size, ChromaOffset, tcm_desc_size, region_tcm_size;
	t_StDmaWrapper_PrepareParm stWrapPrepParm;
	t_StDmaWrapper_WorkDescrip staWorkDesc[2];
    t_StDmaWrapper_FrameProp walkRoi;
    t_StDmaWrapper_FrameProp frameProp;
	int nIdx = 0;
    int nRow, nCol;
    int nRowIdx = 0, nColIdx = 0;
	t_StDmaWrapper_UpdateParm stWrapUpdateParm[2];

	int ubwcHeight = ALIGNUP(nFrameHeight, 32);
	int ubwcWidth = ALIGNUP(nFrameWidth, 128);
	
	stRoi.u16W = 256;
    stRoi.u16H = 32;
	nRet = nDmaWrapper_GetRecommendedRoi(eDmaFmt_NV12, ISUBWC, &stRoi);
	if(nRet)
	{
		nRet = UBWC_ERR_DMAGETROI;
		LOGE("nDmaWrapper_GetRecommendedRoi error.");
	    goto out;
	}

	nRow = ROUNDDIV(ubwcHeight, stRoi.u16H);
	nCol = ROUNDDIV(ubwcWidth,  stRoi.u16W);

	LOGI("stRoi.u16W = %d, stRoi.u16H = %d, nRow = %d, nCol = %d", stRoi.u16W, stRoi.u16H, nRow, nCol);

	stAlignInfo.u16W = stRoi.u16W;
    stAlignInfo.u16H = stRoi.u16H;

	lumaStride = nDmaWrapper_GetRecommendedIntermBufStride(eFmtLuma, &stAlignInfo, ISUBWC);
	chromaStride = nDmaWrapper_GetRecommendedIntermBufStride(eFmtChroma, &stAlignInfo, ISUBWC);

	LOGI("lumaStride = %d, chromaStride = %d", lumaStride, chromaStride);

	tcm_buf_size = nDmaWrapper_GetRecommendedIntermBufSize(eFmtLuma, PAD16BIT, &stAlignInfo, ISUBWC, lumaStride);
	ChromaOffset = tcm_buf_size;
	tcm_buf_size += nDmaWrapper_GetRecommendedIntermBufSize(eFmtChroma, PAD16BIT, &stAlignInfo, ISUBWC, chromaStride);
	region_tcm_size = ALIGNUP(tcm_buf_size * 2, 0x1000);
	tcm_desc_size = ALIGNUP(nDmaWrapper_GetDescbuffsize(efmtLumaChroma, 2) * 2 * 2, 0x1000);


	LOGI("tcm_buf_size = %d, region_tcm_size = %d, tcm_desc_size = %d, ChromaOffset = %d",
		  tcm_buf_size, region_tcm_size, tcm_desc_size, ChromaOffset);
		
	tcm_desc_vaddr = (addr_t)HAP_cache_lock((unsigned int)tcm_desc_size, &tcm_desc_paddr);
	tcm_buf_vaddr = (addr_t)HAP_cache_lock((unsigned int)region_tcm_size, &tcm_buf_paddr);
	if(!tcm_desc_vaddr || !tcm_buf_vaddr)
	{
		nRet = UBWC_ERR_ALLOCTCM;
		LOGE("HAP_cache_lock error.");
		goto out;
	}

	LOGI("tcm_desc_vaddr = 0x%x, tcm_buf_vaddr = 0x%x, tcm_desc_paddr = 0x%x, tcm_buf_paddr = 0x%x", 
			tcm_desc_vaddr, tcm_buf_vaddr, qurt_lookup_physaddr(tcm_desc_vaddr), qurt_lookup_physaddr(tcm_buf_vaddr));
#if 0
	handle = hDmaWrapper_AllocDmaSpecifyWaitType(eWaitType);
#endif
	if(!handle) 
	{
		nRet = UBWC_ERR_ALLOCDMA;
		LOGE("hDmaWrapper_AllocDmaSpecifyWaitType error.");
		goto error;
	}

#if 0
	nRet = nDmaWrapper_PowerVoting(PW_SVS2);
	if(nRet)
	{
		LOGE("nDmaWrapper_PowerVoting error.");
		goto error;
	}
#endif
	stWrapPrepParm.u32NumOfWorkDesc = 2;
    stWrapPrepParm.staWorkDesc = staWorkDesc;
	for (int i = 0; i < 1; i++)
    {
        for (int j = 0; j < 2; j++)
        {
            int descIndex = i * 2 + j;
            frameProp.aAddr = (addr_t)qurt_lookup_physaddr((qurt_addr_t)ubwc_buff);
            frameProp.u16W = ubwcWidth;
            frameProp.u16H = ALIGNUP(totalHeight, 32);//ubwcHeight;
            frameProp.u16Stride = ubwcWidth;
            walkRoi.aAddr = 0;
            walkRoi.u16W = stRoi.u16W;
            walkRoi.u16H = stRoi.u16H;
            walkRoi.u16Stride = (j) ? chromaStride : lumaStride;
            nRet = nDmaWrapper_WorkDescrip_populate(&staWorkDesc[descIndex],
                                                    efmtLumaChroma[j],
                                                    eDmaWrapper_DdrToL2,
                                                    &walkRoi, &frameProp,
                                                    ISUBWC,
                                                    PAD16BIT,
                                                    NULL);
        }
    }
    stWrapPrepParm.u32DescBufSize = nDmaWrapper_GetDescbuffsize(efmtLumaChroma, 2);
	stWrapPrepParm.pPingDescBuf = (void *)tcm_desc_vaddr;
	stWrapPrepParm.pPongDescBuf = (void *)(tcm_desc_vaddr + stWrapPrepParm.u32DescBufSize);
	LOGI("stWrapPrepParm.u32DescBufSize = %d", stWrapPrepParm.u32DescBufSize);
	nRet |= nDmaWrapper_Prepare(handle, &stWrapPrepParm);
	if(nRet)
	{
		nRet = UBWC_ERR_DMAPREPARE;
		LOGE("nDmaWrapper_Prepare error.");
		goto error;
	}
	
	iRoiInfo_t iRoi;
	iRoi.nFrameHeight = ubwcHeight;
	iRoi.nFrameWidth = ubwcWidth;
	iRoi.nRoiWalkHeight = stRoi.u16H;
	iRoi.nRoiWalkWidth = stRoi.u16W;
	iRoi.tcm_buf_vaddr = tcm_buf_vaddr;
	iRoi.tcm_buf_size = tcm_buf_size;
	iRoi.ChromaOffset = ChromaOffset;
	
	updateRoiInfoThreads(nRowIdx, nRow, nColIdx, nCol, nIdx, &iRoi, &stWrapUpdateParm[0], jobCount * dptr->nFrameHeight);
	nRet = nDmaWrapper_Update(handle, &stWrapUpdateParm[0]);
	if(nRet)
	{
		nRet = UBWC_ERR_DMAUPDATE;
		LOGE("nDmaWrapper_Update error.");
		goto error;
	}

	nRet = nDmaWrapper_Move(handle);
	if(nRet)
	{
		nRet = UBWC_ERR_DMAMOVE;
		LOGE("nDmaWrapper_Move error.");
		goto error;
	}
    nIdx = (nIdx + 1) % 2;

	int offsetC = MAX(totalHeight, totalPadHeight) * MAX(nFrameWidth, nPadWidth) + jobCount * (dptr->nFrameHeight >> 1) * (uvpad_flag ? MAX(nFrameWidth, nPadWidth) : nFrameWidth); 
	int sizeH = (nRow == 1 && MARGIN(nFrameHeight, stRoi.u16H) != 0) ? MARGIN(nFrameHeight, stRoi.u16H) : stWrapUpdateParm[0].u.stPixData.stRoi.u16H;
	int sizeW = (nCol == 1 && MARGIN(nFrameWidth, stRoi.u16W) != 0) ? MARGIN(nFrameWidth, stRoi.u16W) : stWrapUpdateParm[0].u.stPixData.stRoi.u16W;
	LOGI("nDmaWrapper_Move size = %d\n", sizeH * sizeW);
	
    for (nRowIdx = 0; nRowIdx < nRow; nRowIdx++)
    {
        for (nColIdx = (nRowIdx)? 0:1; nColIdx < nCol; nColIdx++)
        {
			updateRoiInfoThreads(nRowIdx, nRow, nColIdx, nCol, nIdx, &iRoi, &stWrapUpdateParm[0], jobCount * dptr->nFrameHeight);
            nRet = nDmaWrapper_Update(handle, &stWrapUpdateParm[0]);
            nRet |= nDmaWrapper_Wait(handle);
			writeDspBuffer((void *)(tcm_buf_vaddr + (nIdx % 2 ? 0 : tcm_buf_size)), dsp_buff, 
				            sizeH, sizeW, stRoi.u16H, stRoi.u16W,
							nColIdx == 0 ? nCol - 1 : nColIdx - 1, 
							nColIdx == 0 ? nRowIdx - 1 : nRowIdx, 
							stRoi.u16W, MAX(nFrameWidth, nPadWidth), jobCount * dptr->nFrameHeight * MAX(nFrameWidth, nPadWidth));
			writeDspBuffer((void *)(tcm_buf_vaddr + (nIdx % 2 ? 0 : tcm_buf_size) + iRoi.ChromaOffset), dsp_buff, 
				            sizeH >> 1, sizeW, stRoi.u16H >> 1, stRoi.u16W,
							nColIdx == 0 ? nCol - 1 : nColIdx - 1, 
							nColIdx == 0 ? nRowIdx - 1 : nRowIdx, 
							stRoi.u16W, (uvpad_flag ? MAX(nFrameWidth, nPadWidth) : nFrameWidth), offsetC);
            nRet |= nDmaWrapper_Move(handle);
			if(nRet)
			{
				nRet = UBWC_ERR_DMAMOVE | UBWC_ERR_DMAUPDATE | UBWC_ERR_DMAWAIT;
				LOGE("nDmaWrapper_Move or nDmaWrapper_Update or nDmaWrapper_Wait error.");
				goto error;
			}		
            nIdx = (nIdx + 1) % 2;
			sizeH = (nRowIdx == (nRow - 1) && MARGIN(nFrameHeight, stRoi.u16H) != 0) ? MARGIN(nFrameHeight, stRoi.u16H) : stWrapUpdateParm[0].u.stPixData.stRoi.u16H;
	        sizeW = (nColIdx == (nCol - 1) && MARGIN(nFrameWidth, stRoi.u16W) != 0) ? MARGIN(nFrameWidth, stRoi.u16W) : stWrapUpdateParm[0].u.stPixData.stRoi.u16W;
			LOGI("nDmaWrapper_Move size = %d", sizeH * sizeW);
        }
    }
	nRet = nDmaWrapper_Wait(handle);
	if(nRet)
	{
		nRet = UBWC_ERR_DMAWAIT;
		LOGE("nDmaWrapper_Wait error.");
		goto error;
	}
	writeDspBuffer((void *)(tcm_buf_vaddr + (nIdx % 2 ? 0 : tcm_buf_size)), dsp_buff, sizeH, sizeW, stRoi.u16H, stRoi.u16W, 
					nCol - 1, nRow - 1, stRoi.u16W, MAX(nFrameWidth, nPadWidth), jobCount * dptr->nFrameHeight * MAX(nFrameWidth, nPadWidth));
	writeDspBuffer((void *)(tcm_buf_vaddr + (nIdx % 2 ? 0 : tcm_buf_size) + iRoi.ChromaOffset), dsp_buff, sizeH >> 1, sizeW, stRoi.u16H >> 1, stRoi.u16W, 
					nCol - 1, nRow - 1, stRoi.u16W, (uvpad_flag ? MAX(nFrameWidth, nPadWidth) : nFrameWidth), offsetC);

	nRet = nDmaWrapper_FinishFrame(handle);
	if(nRet)
	{
		nRet = UBWC_ERR_DMAFINISH;
		LOGE("nDmaWrapper_FinishFrame error.");
		goto error;
	}
	
#if 0
	nRet = nDmaWrapper_FreeDma(handle);
	if(nRet)
	{
		LOGE("nDmaWrapper_FreeDma error.");
		goto error;
	}
#endif

error:	
	HAP_cache_unlock((void*)tcm_buf_vaddr);
	HAP_cache_unlock((void*)tcm_desc_vaddr);
out:
	dptr->nRet[jobCount] = nRet;
    worker_pool_synctoken_jobdone(dptr->token);
	
	return;
}

#include <dlfcn.h>

//extern int os_thread_id();
typedef unsigned int (*get_vtcm_size_ptr)(void);
typedef void (*reset_dma_ptr)(void);

#ifdef ENABLE_OSIE
int osie_uv(unsigned char* input, int uvin_size, unsigned char *output, int uvout_size, int height, int width);
#endif

int aiboostubwc_readBuffer(const unsigned char *ubwc_buff, int ubwc_size, unsigned char *dsp_buff, int dsp_size, int32 nFrameHeight, int32 nFrameWidth)
{
#ifdef PROFILING_ON
	uint64 startTime = HAP_perf_get_time_us();
	uint64 startCycles = HAP_perf_get_pcycles();
#endif	

#if 0
	FARF(ALWAYS, "aiboostubwc_readBuffer");

	static void *handle = NULL;
	
	handle = dlopen("libQnnHtpSkel.so", RTLD_LAZY | RTLD_LOCAL);
	if(!handle)
	{
		FARF(ALWAYS, "dlopen error");
		return -1;
	}

	reset_dma_ptr reset_dma = (reset_dma_ptr)dlsym(handle, "reset_dma");
	if(!reset_dma)
	{
		FARF(ALWAYS, "reset_dma dlsym error");

		return -1;
	}

	reset_dma();
#endif

#if 0
	get_vtcm_size_ptr get_vtcm_size = (get_vtcm_size_ptr)dlsym(handle, "get_vtcm_size");
	if(!get_vtcm_size)
	{
		FARF(ALWAYS, "get_vtcm_size dlsym error");

		return -1;
	}

	FARF(ALWAYS, "%d", get_vtcm_size());
#endif

	int nRet = UBWC_OK;

	qurt_mem_cache_clean((qurt_addr_t)ubwc_buff, ubwc_size, QURT_MEM_CACHE_FLUSH_INVALIDATE, QURT_MEM_DCACHE);

	int numWorkers = MIN((qurt_hvx_get_units() >> 8) & 0xFF, MAXTHREADS);


	worker_pool_job_t   job;
    worker_synctoken_t    token;
    worker_pool_context_t context = NULL;
    //worker_pool_synctoken_init(&token, numWorkers);
		
	ubwc_callback_t dptr;
	int retry_times = 100;
	int dma_alloc_flag = 0;
	
	while(retry_times--)
	{
		for(int i = 0; i < numWorkers; i++)
		{
			dptr.nRet[i] = UBWC_OK;
			dptr.handle[i] = hDmaWrapper_AllocDmaSpecifyWaitType(eDmaWaitType_Polling);
			if(!dptr.handle[i])
			{
				numWorkers = i;
				break;
			}
		}

		if(numWorkers == 0)
		{
			numWorkers = 1;
			qurt_timer_sleep(1);
		}
		else
		{
			dma_alloc_flag = 1;
			break;
		}
	}
	if(dma_alloc_flag == 0)
	{
		nRet = UBWC_ERR_ALLOCDMA;
		LOGE("hDmaWrapper_AllocDmaSpecifyWaitType error.");
		return nRet;
	}
	
	int h = nFrameHeight & 0xffff;
	int n = ROUNDDIV(h, 32);
	int nn = ROUNDDIV(n, numWorkers);
	
	LOGI("MultiThreds:hvxInfo.numThreads = %d, numWorkers = %d", hvxInfo.numThreads, numWorkers);

    //(void)worker_pool_init(&context);
    worker_pool_synctoken_init(&token, numWorkers);

	job.fptr = ubwc_readBuffer_callback;
	dptr.token = &token;
	dptr.jobCount = 0;
	dptr.ubwc_buff = (unsigned char*)ubwc_buff;
	dptr.dsp_buff = dsp_buff;
	dptr.nFrameHeight = nn * 32;
	dptr.nFrameWidth = nFrameWidth;
	dptr.totalHeight = nFrameHeight;
	dptr.numThreads = numWorkers;
	
	job.dptr = (void *)&dptr;

	for(int i = 0; i < numWorkers; i++)
	{
	   (void)worker_pool_submit(context, job);
	}
	worker_pool_synctoken_wait(&token);

	//worker_pool_deinit(&context);

	for(int i = 0; i < numWorkers; i++)
	{
		if(dptr.handle[i]) nRet |= (nDmaWrapper_FreeDma(dptr.handle[i]) < 0 ? UBWC_ERR_DMAFREE : UBWC_OK);
		nRet |= dptr.nRet[i];
	}

#ifdef ENABLE_OSIE
	//call osie for uv
	int pad_h = (nFrameHeight >> 16);
	int pad_w = (nFrameWidth >> 16);
	int real_h = nFrameHeight & 0xffff;
	int real_w = nFrameWidth & 0xffff;
	unsigned char *uv_input = (unsigned char*)dsp_buff + MAX(pad_h, real_h) * MAX(pad_w, real_w);
	if (pad_h != 0)
	{
		int ret_osie = osie_uv(uv_input, real_h/2 * real_w, uv_input, real_h/2 * real_w, real_h/2, real_w);
		if (ret_osie != 0)
		{
			nRet |= UBWC_ERR_OSIE;
			LOGE("osie_uv error.");
			return nRet;
		}
	}
#endif	

	padDspBUffer(dsp_buff, nFrameHeight & 0xffff, nFrameWidth & 0xffff, nFrameHeight >> 16, nFrameWidth >> 16);
	
#ifdef PROFILING_ON
	uint64 endCycles = HAP_perf_get_pcycles();
	uint64 endTime = HAP_perf_get_time_us();
	FARF(ALWAYS, "[UBWCREAD]profiling: %d PCycles, %d us. Observed clock rate %d MHz",
	 (int)(endCycles - startCycles), (int)(endTime - startTime), (int)((endCycles - startCycles) / (endTime - startTime)));
#endif

	return nRet;
}
#endif

#ifndef UBWC_MULTI_THREADS
int aiboostubwc_writeBuffer(const unsigned char *dsp_buff, int dsp_size, unsigned char *ubwc_buff, int ubwc_size, int32 nFrameHeight, int32 nFrameWidth)
{
#ifdef PROFILING_ON
	uint64 startTime = HAP_perf_get_time_us();
	uint64 startCycles = HAP_perf_get_pcycles();
#endif
	t_DmaWrapper_DmaEngineHandle handle;
	t_EDma_WaitType eWaitType = eDmaWaitType_Polling;
	t_eDmaFmt eFmtLuma = eDmaFmt_NV12_Y;
	t_eDmaFmt eFmtChroma = eDmaFmt_NV12_UV;

	t_eDmaFmt efmtLumaChroma[2] = {eFmtLuma, eFmtChroma};

	int nRet = UBWC_OK;
	t_StDmaWrapper_Roi stRoi;
	t_StDmaWrapper_RoiAlignInfo stAlignInfo;
	int lumaStride, chromaStride;
	qurt_addr_t tcm_buf_vaddr, tcm_desc_vaddr;
	unsigned long long tcm_buf_paddr, tcm_desc_paddr;
	qurt_size_t tcm_buf_size, ChromaOffset, tcm_desc_size, region_tcm_size;
	t_StDmaWrapper_PrepareParm stWrapPrepParm;
	t_StDmaWrapper_WorkDescrip staWorkDesc[2];
	t_StDmaWrapper_FrameProp walkRoi;
	t_StDmaWrapper_FrameProp frameProp;
	int nIdx = 0;
	int nRow, nCol;
	int nRowIdx = 0, nColIdx = 0;
	t_StDmaWrapper_UpdateParm stWrapUpdateParm[2];
	
	int nPadHeight = nFrameHeight >> 16;
	int nPadWidth = nFrameWidth >> 16;
	nFrameHeight &= 0xffff;
	nFrameWidth &= 0xffff;
	int offsetC = MAX(nFrameHeight, nPadHeight) * MAX(nFrameWidth, nPadWidth); 
	int ubwcHeight = ALIGNUP(nFrameHeight, 32);
	int ubwcWidth = ALIGNUP(nFrameWidth, 128);
	
	stRoi.u16W = 256;
	stRoi.u16H = 32;
	nRet = nDmaWrapper_GetRecommendedRoi(eDmaFmt_NV12, ISUBWC, &stRoi);
	if(nRet)
	{
		nRet = UBWC_ERR_DMAGETROI;
		LOGE("nDmaWrapper_GetRecommendedRoi error.");
		return nRet;
	}

	nRow = ROUNDDIV(ubwcHeight, stRoi.u16H);
	nCol = ROUNDDIV(ubwcWidth,  stRoi.u16W);

	LOGI("stRoi.u16W = %d, stRoi.u16H = %d, nRow = %d, nCol = %d", stRoi.u16W, stRoi.u16H, nRow, nCol);

	stAlignInfo.u16W = stRoi.u16W;
	stAlignInfo.u16H = stRoi.u16H;

	lumaStride = nDmaWrapper_GetRecommendedIntermBufStride(eFmtLuma, &stAlignInfo, ISUBWC);
	chromaStride = nDmaWrapper_GetRecommendedIntermBufStride(eFmtChroma, &stAlignInfo, ISUBWC);

	LOGI("lumaStride = %d, chromaStride = %d", lumaStride, chromaStride);

	tcm_buf_size = nDmaWrapper_GetRecommendedIntermBufSize(eFmtLuma, PAD16BIT, &stAlignInfo, ISUBWC, lumaStride);
	ChromaOffset = tcm_buf_size;
	tcm_buf_size += nDmaWrapper_GetRecommendedIntermBufSize(eFmtChroma, PAD16BIT, &stAlignInfo, ISUBWC, chromaStride);
	region_tcm_size = ALIGNUP(tcm_buf_size * 2, 0x1000);
	tcm_desc_size = ALIGNUP(nDmaWrapper_GetDescbuffsize(efmtLumaChroma, 2) * 2 * 2, 0x1000);


	LOGI("tcm_buf_size = %d, region_tcm_size = %d, tcm_desc_size = %d, ChromaOffset = %d",
		  tcm_buf_size, region_tcm_size, tcm_desc_size, ChromaOffset);
		
	tcm_desc_vaddr = (addr_t)HAP_cache_lock((unsigned int)tcm_desc_size, &tcm_desc_paddr);
	tcm_buf_vaddr = (addr_t)HAP_cache_lock((unsigned int)region_tcm_size, &tcm_buf_paddr);
	if(!tcm_desc_vaddr || !tcm_buf_vaddr)
	{
		LOGE("HAP_cache_lock error.");
		nRet = UBWC_ERR_ALLOCTCM;
		return nRet;
	}

	LOGI("tcm_desc_vaddr = 0x%x, tcm_buf_vaddr = 0x%x, tcm_desc_paddr = 0x%x, tcm_buf_paddr = 0x%x", 
			tcm_desc_vaddr, tcm_buf_vaddr, qurt_lookup_physaddr(tcm_desc_vaddr), qurt_lookup_physaddr(tcm_buf_vaddr));

	handle = hDmaWrapper_AllocDmaSpecifyWaitType(eWaitType);
	if(!handle) 
	{
		nRet = UBWC_ERR_ALLOCDMA;
		LOGE("hDmaWrapper_AllocDmaSpecifyWaitType error.");
		goto error;
	}

#if 0
	nRet = nDmaWrapper_PowerVoting(PW_SVS2);
	if(nRet)
	{
		LOGE("nDmaWrapper_PowerVoting error.");
		goto error;
	}
#endif

	stWrapPrepParm.u32NumOfWorkDesc = 2;
	stWrapPrepParm.staWorkDesc = staWorkDesc;
	for (int i = 0; i < 1; i++)
	{
		for (int j = 0; j < 2; j++)
		{
			int descIndex = i * 2 + j;
			frameProp.aAddr = (addr_t)qurt_lookup_physaddr((qurt_addr_t)ubwc_buff);
			frameProp.u16W = ubwcWidth;
			frameProp.u16H = ubwcHeight;
			frameProp.u16Stride = ubwcWidth;
			walkRoi.aAddr = 0;
			walkRoi.u16W = stRoi.u16W;
			walkRoi.u16H = stRoi.u16H;
			walkRoi.u16Stride = (j) ? chromaStride : lumaStride;
			nRet = nDmaWrapper_WorkDescrip_populate(&staWorkDesc[descIndex],
													efmtLumaChroma[j],
													eDmaWrapper_L2ToDdr,
													&walkRoi, &frameProp,
													ISUBWC,
													PAD16BIT,
													NULL);
		}
	}
	stWrapPrepParm.u32DescBufSize = nDmaWrapper_GetDescbuffsize(efmtLumaChroma, 2);
	stWrapPrepParm.pPingDescBuf = (void *)tcm_desc_vaddr;
	stWrapPrepParm.pPongDescBuf = (void *)(tcm_desc_vaddr + stWrapPrepParm.u32DescBufSize);
	LOGI("stWrapPrepParm.u32DescBufSize = %d", stWrapPrepParm.u32DescBufSize);
	nRet |= nDmaWrapper_Prepare(handle, &stWrapPrepParm);
	if(nRet)
	{
		nRet = UBWC_ERR_DMAPREPARE;
		LOGE("nDmaWrapper_Prepare error.");
		goto error;
	}

	iRoiInfo_t iRoi;
	iRoi.nFrameHeight = ubwcHeight;
	iRoi.nFrameWidth = ubwcWidth;
	iRoi.nRoiWalkHeight = stRoi.u16H;
	iRoi.nRoiWalkWidth = stRoi.u16W;
	iRoi.tcm_buf_vaddr = tcm_buf_vaddr;
	iRoi.tcm_buf_size = tcm_buf_size;
	iRoi.ChromaOffset = ChromaOffset;
	
	updateRoiInfo(nRowIdx, nRow, nColIdx, nCol, nIdx, &iRoi, &stWrapUpdateParm[0]);
	writeTcmBuffer((void *)(tcm_buf_vaddr), dsp_buff, 
					(nRow == 1 && MARGIN(nFrameHeight, stRoi.u16H) != 0) ? MARGIN(nFrameHeight, stRoi.u16H) : stWrapUpdateParm[0].u.stPixData.stRoi.u16H, 
					(nCol == 1 && MARGIN(nFrameWidth, stRoi.u16W) != 0) ? MARGIN(nFrameWidth, stRoi.u16W) : stWrapUpdateParm[0].u.stPixData.stRoi.u16W, 
					stRoi.u16H, stRoi.u16W, 
					nColIdx, nRowIdx, stRoi.u16W, MAX(nFrameWidth, nPadWidth), 0);
	writeTcmBuffer((void *)(tcm_buf_vaddr + iRoi.ChromaOffset), dsp_buff, 
					((nRow == 1 && MARGIN(nFrameHeight, stRoi.u16H) != 0) ? MARGIN(nFrameHeight, stRoi.u16H) : stWrapUpdateParm[0].u.stPixData.stRoi.u16H) >> 1, 
					(nCol == 1 && MARGIN(nFrameWidth, stRoi.u16W) != 0) ? MARGIN(nFrameWidth, stRoi.u16W) : stWrapUpdateParm[0].u.stPixData.stRoi.u16W,  
					stRoi.u16H >> 1, stRoi.u16W,
				    nColIdx, nRowIdx,  stRoi.u16W, nFrameWidth, offsetC);
	nRet = nDmaWrapper_Update(handle, &stWrapUpdateParm[0]);
	if(nRet)
	{
		nRet = UBWC_ERR_DMAUPDATE;
		LOGE("nDmaWrapper_Update error.");
		goto error;
	}

	nRet = nDmaWrapper_Move(handle);
	if(nRet)
	{
		nRet = UBWC_ERR_DMAMOVE;
		LOGE("nDmaWrapper_Move error.");
		goto error;
	}
	nIdx = (nIdx + 1) % 2;

	LOGI("nDmaWrapper_Move size = %d\n", stWrapUpdateParm[0].u.stPixData.stRoi.u16H * stWrapUpdateParm[0].u.stPixData.stRoi.u16W);
	
	for (nRowIdx = 0; nRowIdx < nRow; nRowIdx++)
	{
		for (nColIdx = (nRowIdx)? 0:1; nColIdx < nCol; nColIdx++)
		{
			updateRoiInfo(nRowIdx, nRow, nColIdx, nCol, nIdx, &iRoi, &stWrapUpdateParm[0]);
			writeTcmBuffer((void *)(tcm_buf_vaddr + (nIdx % 2 ? tcm_buf_size : 0)), dsp_buff, 
							(nRowIdx == (nRow - 1) && MARGIN(nFrameHeight, stRoi.u16H) != 0) ? MARGIN(nFrameHeight, stRoi.u16H) : stWrapUpdateParm[0].u.stPixData.stRoi.u16H, 
					        (nColIdx == (nCol - 1) && MARGIN(nFrameWidth, stRoi.u16W) != 0) ? MARGIN(nFrameWidth, stRoi.u16W) : stWrapUpdateParm[0].u.stPixData.stRoi.u16W,  
							stRoi.u16H, stRoi.u16W, 
							nColIdx, nRowIdx, stRoi.u16W, MAX(nFrameWidth, nPadWidth), 0);
			writeTcmBuffer((void *)(tcm_buf_vaddr + (nIdx % 2 ? tcm_buf_size : 0) + iRoi.ChromaOffset), dsp_buff, 
				            ((nRowIdx == (nRow - 1) && MARGIN(nFrameHeight, stRoi.u16H) != 0) ? MARGIN(nFrameHeight, stRoi.u16H) : stWrapUpdateParm[0].u.stPixData.stRoi.u16H) >> 1, 
							(nColIdx == (nCol - 1) && MARGIN(nFrameWidth, stRoi.u16W) != 0) ? MARGIN(nFrameWidth, stRoi.u16W) : stWrapUpdateParm[0].u.stPixData.stRoi.u16W,
							stRoi.u16H >> 1, stRoi.u16W,
							nColIdx, nRowIdx,  stRoi.u16W, nFrameWidth, offsetC);
			nRet = nDmaWrapper_Update(handle, &stWrapUpdateParm[0]);
			nRet = nDmaWrapper_Wait(handle);
			nRet = nDmaWrapper_Move(handle);
			if(nRet)
			{
				nRet = UBWC_ERR_DMAMOVE | UBWC_ERR_DMAUPDATE | UBWC_ERR_DMAWAIT;
				LOGE("nDmaWrapper_Move or nDmaWrapper_Update or nDmaWrapper_Wait error.");
				goto error;
			}		
			nIdx = (nIdx + 1) % 2;
			LOGI("nDmaWrapper_Move size = %d\n", stWrapUpdateParm[0].u.stPixData.stRoi.u16H * stWrapUpdateParm[0].u.stPixData.stRoi.u16W);
		}
	}
	nRet = nDmaWrapper_Wait(handle);
	if(nRet)
	{
		nRet = UBWC_ERR_DMAWAIT;
		LOGE("nDmaWrapper_Wait error.");
		goto error;
	}
	
	nRet = nDmaWrapper_FinishFrame(handle);
	if(nRet)
	{
		nRet = UBWC_ERR_DMAFINISH;
		LOGE("nDmaWrapper_FinishFrame error.");
		goto error;
	}

	nRet = nDmaWrapper_FreeDma(handle);
	if(nRet)
	{
		nRet = UBWC_ERR_DMAFREE;
		LOGE("nDmaWrapper_FreeDma error.");
		goto error;
	}
error:
	HAP_cache_unlock((void*)tcm_buf_vaddr);
	HAP_cache_unlock((void*)tcm_desc_vaddr);
#ifdef PROFILING_ON
	uint64 endCycles = HAP_perf_get_pcycles();
	uint64 endTime = HAP_perf_get_time_us();
	FARF(ALWAYS, "[UBWCWRITE]profiling: %d PCycles, %d us. Observed clock rate %d MHz",
	 (int)(endCycles - startCycles), (int)(endTime - startTime), (int)((endCycles - startCycles) / (endTime - startTime)));
#endif

	return nRet;
}
#else
void depthtospaceToTcmBuffer2(void *tcm_buffer, const uint8* dsp_buff, 
							int h, int w, int hb, int wb, 
							int nColIdx, int nRowIdx, int stride_dst, int stride_src, 
							int offset, float scale, int zero_point);

static void ubwc_writeBuffer_callback(void *data)
{
	int nRet = UBWC_OK;
	ubwc_callback_t *dptr = (ubwc_callback_t *)data;
	unsigned int jobCount = 0;
	
	jobCount = worker_pool_atomic_inc_return(&(dptr->jobCount)) - 1;
		
	int nFrameHeight = dptr->nFrameHeight;
	int nFrameWidth = dptr->nFrameWidth & 0xffff;
	int nPadWidth = dptr->nFrameWidth >> 16;
	int totalHeight = dptr->totalHeight & 0xffff;
	int totalPadHeight = (dptr->totalHeight >> 16) & 0x7fff;
	unsigned char *ubwc_buff = dptr->ubwc_buff;
	unsigned char *dsp_buff = dptr->dsp_buff;

	int uvpad_flag = dptr->totalHeight & 0x80000000;
	
	if(jobCount * nFrameHeight >= totalHeight) goto out;

	if((jobCount == dptr->numThreads - 1) || (totalHeight - jobCount * dptr->nFrameHeight < dptr->nFrameHeight)) 
	{
		nFrameHeight = totalHeight - jobCount * dptr->nFrameHeight;
	}

	t_DmaWrapper_DmaEngineHandle handle = dptr->handle[jobCount];
	//t_EDma_WaitType eWaitType = eDmaWaitType_Polling;
	t_eDmaFmt eFmtLuma = eDmaFmt_NV12_Y;
	t_eDmaFmt eFmtChroma = eDmaFmt_NV12_UV;

	t_eDmaFmt efmtLumaChroma[2] = {eFmtLuma, eFmtChroma};

	t_StDmaWrapper_Roi stRoi;
	t_StDmaWrapper_RoiAlignInfo stAlignInfo;
	int lumaStride, chromaStride;
	qurt_addr_t tcm_buf_vaddr, tcm_desc_vaddr;
	unsigned long long tcm_buf_paddr, tcm_desc_paddr;
	qurt_size_t tcm_buf_size, ChromaOffset, tcm_desc_size, region_tcm_size;
	t_StDmaWrapper_PrepareParm stWrapPrepParm;
	t_StDmaWrapper_WorkDescrip staWorkDesc[2];
	t_StDmaWrapper_FrameProp walkRoi;
	t_StDmaWrapper_FrameProp frameProp;
	int nIdx = 0;
	int nRow, nCol;
	int nRowIdx = 0, nColIdx = 0;
	t_StDmaWrapper_UpdateParm stWrapUpdateParm[2];
	int offsetC = MAX(totalHeight, totalPadHeight) * MAX(nFrameWidth,  nPadWidth) + jobCount * (dptr->nFrameHeight >> 1) * (uvpad_flag ? MAX(nFrameWidth, nPadWidth) : nFrameWidth);

	int ubwcHeight = ALIGNUP(nFrameHeight, 32);
	int ubwcWidth = ALIGNUP(nFrameWidth, 128);
	
	stRoi.u16W = 256;
	stRoi.u16H = 32;
	nRet = nDmaWrapper_GetRecommendedRoi(eDmaFmt_NV12, ISUBWC, &stRoi);
	if(nRet)
	{	
		nRet = UBWC_ERR_DMAGETROI;
		LOGE("nDmaWrapper_GetRecommendedRoi error.");
		goto out;
	}

	nRow = ROUNDDIV(ubwcHeight, stRoi.u16H);
	nCol = ROUNDDIV(ubwcWidth,  stRoi.u16W);

	LOGI("stRoi.u16W = %d, stRoi.u16H = %d, nRow = %d, nCol = %d", stRoi.u16W, stRoi.u16H, nRow, nCol);

	stAlignInfo.u16W = stRoi.u16W;
	stAlignInfo.u16H = stRoi.u16H;

	lumaStride = nDmaWrapper_GetRecommendedIntermBufStride(eFmtLuma, &stAlignInfo, ISUBWC);
	chromaStride = nDmaWrapper_GetRecommendedIntermBufStride(eFmtChroma, &stAlignInfo, ISUBWC);

	LOGI("lumaStride = %d, chromaStride = %d", lumaStride, chromaStride);

	tcm_buf_size = nDmaWrapper_GetRecommendedIntermBufSize(eFmtLuma, PAD16BIT, &stAlignInfo, ISUBWC, lumaStride);
	ChromaOffset = tcm_buf_size;
	tcm_buf_size += nDmaWrapper_GetRecommendedIntermBufSize(eFmtChroma, PAD16BIT, &stAlignInfo, ISUBWC, chromaStride);
	region_tcm_size = ALIGNUP(tcm_buf_size * 2, 0x1000);
	tcm_desc_size = ALIGNUP(nDmaWrapper_GetDescbuffsize(efmtLumaChroma, 2) * 2 * 2, 0x1000);


	LOGI("tcm_buf_size = %d, region_tcm_size = %d, tcm_desc_size = %d, ChromaOffset = %d",
		  tcm_buf_size, region_tcm_size, tcm_desc_size, ChromaOffset);
		
	tcm_desc_vaddr = (addr_t)HAP_cache_lock((unsigned int)tcm_desc_size, &tcm_desc_paddr);
	tcm_buf_vaddr = (addr_t)HAP_cache_lock((unsigned int)region_tcm_size, &tcm_buf_paddr);
	if(!tcm_desc_vaddr || !tcm_buf_vaddr)
	{
		nRet = UBWC_ERR_ALLOCTCM;
		LOGE("HAP_cache_lock error.");
		goto out;
	}

	LOGI("tcm_desc_vaddr = 0x%x, tcm_buf_vaddr = 0x%x, tcm_desc_paddr = 0x%x, tcm_buf_paddr = 0x%x", 
			tcm_desc_vaddr, tcm_buf_vaddr, qurt_lookup_physaddr(tcm_desc_vaddr), qurt_lookup_physaddr(tcm_buf_vaddr));

#if 0
	handle = hDmaWrapper_AllocDmaSpecifyWaitType(eWaitType);
#endif
	if(!handle) 
	{
		nRet = UBWC_ERR_ALLOCDMA;
		LOGE("hDmaWrapper_AllocDmaSpecifyWaitType error.");
		goto error;
	}

#if 0
	nRet = nDmaWrapper_PowerVoting(PW_SVS2);
	if(nRet)
	{
		LOGE("nDmaWrapper_PowerVoting error.");
		goto error;
	}
#endif

	stWrapPrepParm.u32NumOfWorkDesc = 2;
	stWrapPrepParm.staWorkDesc = staWorkDesc;
	for (int i = 0; i < 1; i++)
	{
		for (int j = 0; j < 2; j++)
		{
			int descIndex = i * 2 + j;
			frameProp.aAddr = (addr_t)qurt_lookup_physaddr((qurt_addr_t)ubwc_buff);
			frameProp.u16W = ubwcWidth;
			frameProp.u16H = ALIGNUP(totalHeight, 32);//ubwcHeight;
			frameProp.u16Stride = ubwcWidth;
			walkRoi.aAddr = 0;
			walkRoi.u16W = stRoi.u16W;
			walkRoi.u16H = stRoi.u16H;
			walkRoi.u16Stride = (j) ? chromaStride : lumaStride;
			nRet = nDmaWrapper_WorkDescrip_populate(&staWorkDesc[descIndex],
													efmtLumaChroma[j],
													eDmaWrapper_L2ToDdr,
													&walkRoi, &frameProp,
													ISUBWC,
													PAD16BIT,
													NULL);
		}
	}
	stWrapPrepParm.u32DescBufSize = nDmaWrapper_GetDescbuffsize(efmtLumaChroma, 2);
	stWrapPrepParm.pPingDescBuf = (void *)tcm_desc_vaddr;
	stWrapPrepParm.pPongDescBuf = (void *)(tcm_desc_vaddr + stWrapPrepParm.u32DescBufSize);
	LOGI("stWrapPrepParm.u32DescBufSize = %d", stWrapPrepParm.u32DescBufSize);
	nRet |= nDmaWrapper_Prepare(handle, &stWrapPrepParm);
	if(nRet)
	{
		nRet = UBWC_ERR_DMAPREPARE;
		LOGE("nDmaWrapper_Prepare error.");
		goto error;
	}

	iRoiInfo_t iRoi;
	iRoi.nFrameHeight = ubwcHeight;
	iRoi.nFrameWidth = ubwcWidth;
	iRoi.nRoiWalkHeight = stRoi.u16H;
	iRoi.nRoiWalkWidth = stRoi.u16W;
	iRoi.tcm_buf_vaddr = tcm_buf_vaddr;
	iRoi.tcm_buf_size = tcm_buf_size;
	iRoi.ChromaOffset = ChromaOffset;
	
	updateRoiInfoThreads(nRowIdx, nRow, nColIdx, nCol, nIdx, &iRoi, &stWrapUpdateParm[0], jobCount * dptr->nFrameHeight);
	writeTcmBuffer((void *)(tcm_buf_vaddr), dsp_buff, 
					(nRow == 1 && MARGIN(nFrameHeight, stRoi.u16H) != 0) ? MARGIN(nFrameHeight, stRoi.u16H) : stWrapUpdateParm[0].u.stPixData.stRoi.u16H, 
					(nCol == 1 && MARGIN(nFrameWidth, stRoi.u16W) != 0) ? MARGIN(nFrameWidth, stRoi.u16W) : stWrapUpdateParm[0].u.stPixData.stRoi.u16W, 
					stRoi.u16H, stRoi.u16W, 
					nColIdx, nRowIdx, stRoi.u16W, MAX(nFrameWidth, nPadWidth), jobCount * dptr->nFrameHeight * MAX(nFrameWidth, nPadWidth));
	writeTcmBuffer((void *)(tcm_buf_vaddr + iRoi.ChromaOffset), dsp_buff, 
					((nRow == 1 && MARGIN(nFrameHeight, stRoi.u16H) != 0) ? MARGIN(nFrameHeight, stRoi.u16H) : stWrapUpdateParm[0].u.stPixData.stRoi.u16H) >> 1, 
					(nCol == 1 && MARGIN(nFrameWidth, stRoi.u16W) != 0) ? MARGIN(nFrameWidth, stRoi.u16W) : stWrapUpdateParm[0].u.stPixData.stRoi.u16W,  
					stRoi.u16H >> 1, stRoi.u16W,
				    nColIdx, nRowIdx,  stRoi.u16W, (uvpad_flag ? MAX(nFrameWidth, nPadWidth) : nFrameWidth), offsetC);
	nRet = nDmaWrapper_Update(handle, &stWrapUpdateParm[0]);
	if(nRet)
	{
		nRet = UBWC_ERR_DMAUPDATE;
		LOGE("nDmaWrapper_Update error.");
		goto error;
	}

	nRet = nDmaWrapper_Move(handle);
	if(nRet)
	{
		nRet = UBWC_ERR_DMAMOVE;
		LOGE("nDmaWrapper_Move error.");
		goto error;
	}
	nIdx = (nIdx + 1) % 2;

	LOGI("nDmaWrapper_Move size = %d\n", stWrapUpdateParm[0].u.stPixData.stRoi.u16H * stWrapUpdateParm[0].u.stPixData.stRoi.u16W);
	
	for (nRowIdx = 0; nRowIdx < nRow; nRowIdx++)
	{
		for (nColIdx = (nRowIdx)? 0:1; nColIdx < nCol; nColIdx++)
		{
			updateRoiInfoThreads(nRowIdx, nRow, nColIdx, nCol, nIdx, &iRoi, &stWrapUpdateParm[0], jobCount * dptr->nFrameHeight);
			writeTcmBuffer((void *)(tcm_buf_vaddr + (nIdx % 2 ? tcm_buf_size : 0)), dsp_buff, 
							(nRowIdx == (nRow - 1) && MARGIN(nFrameHeight, stRoi.u16H) != 0) ? MARGIN(nFrameHeight, stRoi.u16H) : stWrapUpdateParm[0].u.stPixData.stRoi.u16H, 
					        (nColIdx == (nCol - 1) && MARGIN(nFrameWidth, stRoi.u16W) != 0) ? MARGIN(nFrameWidth, stRoi.u16W) : stWrapUpdateParm[0].u.stPixData.stRoi.u16W,  
							stRoi.u16H, stRoi.u16W, 
							nColIdx, nRowIdx, stRoi.u16W, MAX(nFrameWidth, nPadWidth), jobCount * dptr->nFrameHeight * MAX(nFrameWidth, nPadWidth));
			writeTcmBuffer((void *)(tcm_buf_vaddr + (nIdx % 2 ? tcm_buf_size : 0) + iRoi.ChromaOffset), dsp_buff, 
				            ((nRowIdx == (nRow - 1) && MARGIN(nFrameHeight, stRoi.u16H) != 0) ? MARGIN(nFrameHeight, stRoi.u16H) : stWrapUpdateParm[0].u.stPixData.stRoi.u16H) >> 1, 
							(nColIdx == (nCol - 1) && MARGIN(nFrameWidth, stRoi.u16W) != 0) ? MARGIN(nFrameWidth, stRoi.u16W) : stWrapUpdateParm[0].u.stPixData.stRoi.u16W,
							stRoi.u16H >> 1, stRoi.u16W,
							nColIdx, nRowIdx,  stRoi.u16W, (uvpad_flag ? MAX(nFrameWidth, nPadWidth) : nFrameWidth), offsetC);
			nRet = nDmaWrapper_Update(handle, &stWrapUpdateParm[0]);
			nRet |= nDmaWrapper_Wait(handle);
			nRet |= nDmaWrapper_Move(handle);
			if(nRet)
			{
				nRet = UBWC_ERR_DMAMOVE | UBWC_ERR_DMAUPDATE | UBWC_ERR_DMAWAIT;
				LOGE("nDmaWrapper_Move or nDmaWrapper_Update or nDmaWrapper_Wait error.");
				goto error;
			}		
			nIdx = (nIdx + 1) % 2;
			LOGI("nDmaWrapper_Move size = %d\n", stWrapUpdateParm[0].u.stPixData.stRoi.u16H * stWrapUpdateParm[0].u.stPixData.stRoi.u16W);
		}
	}
	nRet = nDmaWrapper_Wait(handle);
	if(nRet)
	{
		nRet = UBWC_ERR_DMAWAIT; 
		LOGE("nDmaWrapper_Wait error.");
		goto error;
	}
	
	nRet = nDmaWrapper_FinishFrame(handle);
	if(nRet)
	{
		nRet = UBWC_ERR_DMAFINISH; 
		LOGE("nDmaWrapper_FinishFrame error.");
		goto error;
	}

#if 0
	nRet = nDmaWrapper_FreeDma(handle);
	if(nRet)
	{
		LOGE("nDmaWrapper_FreeDma error.");
		goto error;
	}
#endif

error:
	HAP_cache_unlock((void*)tcm_buf_vaddr);
	HAP_cache_unlock((void*)tcm_desc_vaddr);
out:
	dptr->nRet[jobCount] = nRet;
    worker_pool_synctoken_jobdone(dptr->token);
	
	return;
}

int aiboostubwc_writeBuffer(const unsigned char *dsp_buff, int dsp_size, unsigned char *ubwc_buff, int ubwc_size, int32 nFrameHeight, int32 nFrameWidth)
{
#ifdef PROFILING_ON
	uint64 startTime = HAP_perf_get_time_us();
	uint64 startCycles = HAP_perf_get_pcycles();
#endif
	
	int nRet = UBWC_OK;

	qurt_mem_cache_clean((qurt_addr_t)ubwc_buff, ubwc_size, QURT_MEM_CACHE_FLUSH_INVALIDATE, QURT_MEM_DCACHE);

	int numWorkers = MIN((qurt_hvx_get_units() >> 8) & 0xFF, MAXTHREADS);
	
	worker_pool_job_t   job;
    worker_synctoken_t    token;
    worker_pool_context_t context = NULL;
	
	ubwc_callback_t dptr;
	int retry_times = 100;
	int dma_alloc_flag = 0;

	while(retry_times--)
	{
		for(int i = 0; i < numWorkers; i++)
		{
			dptr.nRet[i] = UBWC_OK;
			dptr.handle[i] = hDmaWrapper_AllocDmaSpecifyWaitType(eDmaWaitType_Polling);
			if(!dptr.handle[i])
			{
				numWorkers = i;
				break;
			}
		}

		if(numWorkers == 0)
		{
			numWorkers = 1;
			qurt_timer_sleep(1);
		}
		else
		{
			dma_alloc_flag = 1;
			break;
		}
	}
	if(dma_alloc_flag == 0)
	{
		nRet = UBWC_ERR_ALLOCDMA;
		LOGE("hDmaWrapper_AllocDmaSpecifyWaitType error.");
		return nRet;
	}

	int h = nFrameHeight & 0xffff;
	int n = ROUNDDIV(h, 32);
	int nn = ROUNDDIV(n, numWorkers);
	
	LOGI("MultiThreds:hvxInfo.numThreads = %d, numWorkers = %d", hvxInfo.numThreads, numWorkers);
	
	job.fptr = ubwc_writeBuffer_callback;

	dptr.token = &token;
	dptr.jobCount = 0;
	dptr.ubwc_buff = ubwc_buff;
	dptr.dsp_buff =  (unsigned char*)dsp_buff;
	dptr.nFrameHeight = nn * 32;
	dptr.nFrameWidth = nFrameWidth;
	dptr.totalHeight = nFrameHeight;
	dptr.numThreads = numWorkers;

	job.dptr = (void *)&dptr;
	
	//(void)worker_pool_init(&context);
	worker_pool_synctoken_init(&token, numWorkers);

	for(int i = 0; i < numWorkers; i++)
	{
	   (void)worker_pool_submit(context, job);
	}
	worker_pool_synctoken_wait(&token);

	//worker_pool_deinit(&context);
	
	for(int i = 0; i < numWorkers; i++)
	{
		if(dptr.handle[i]) nRet |= (nDmaWrapper_FreeDma(dptr.handle[i]) < 0 ? UBWC_ERR_DMAFREE : UBWC_OK);
		nRet |= dptr.nRet[i];
	}
	
#ifdef PROFILING_ON
	uint64 endCycles = HAP_perf_get_pcycles();
	uint64 endTime = HAP_perf_get_time_us();
	FARF(ALWAYS, "[UBWCWRITE]profiling: %d PCycles, %d us. Observed clock rate %d MHz",
	 (int)(endCycles - startCycles), (int)(endTime - startTime), (int)((endCycles - startCycles) / (endTime - startTime)));
#endif

	return nRet;
}

static void ubwc_writeBuffer_callback1(void *data)
{
	int nRet = UBWC_OK;
	ubwc_callback_t *dptr = (ubwc_callback_t *)data;
	unsigned int jobCount = 0;
	
	jobCount = worker_pool_atomic_inc_return(&(dptr->jobCount)) - 1;
		
	int nFrameHeight = dptr->nFrameHeight;
	int nFrameWidth = dptr->nFrameWidth & 0xffff;
	int nPadWidth = dptr->nFrameWidth >> 16;
	int totalHeight = dptr->totalHeight & 0xffff;
	int totalPadHeight = (dptr->totalHeight >> 16) & 0x7fff;
	unsigned char *ubwc_buff = dptr->ubwc_buff;
	unsigned char *dsp_buff = dptr->dsp_buff;

	int uvpad_flag = dptr->totalHeight & 0x80000000;
	
	if(jobCount * nFrameHeight >= totalHeight) goto out;

	if((jobCount == dptr->numThreads - 1) || (totalHeight - jobCount * dptr->nFrameHeight < dptr->nFrameHeight)) 
	{
		nFrameHeight = totalHeight - jobCount * dptr->nFrameHeight;
	}

	t_DmaWrapper_DmaEngineHandle handle = dptr->handle[jobCount];
	//t_EDma_WaitType eWaitType = eDmaWaitType_Polling;
	t_eDmaFmt eFmtLuma = eDmaFmt_NV12_Y;
	t_eDmaFmt eFmtChroma = eDmaFmt_NV12_UV;

	t_eDmaFmt efmtLumaChroma[2] = {eFmtLuma, eFmtChroma};

	t_StDmaWrapper_Roi stRoi;
	t_StDmaWrapper_RoiAlignInfo stAlignInfo;
	int lumaStride, chromaStride;
	qurt_addr_t tcm_buf_vaddr, tcm_desc_vaddr;
	unsigned long long tcm_buf_paddr, tcm_desc_paddr;
	qurt_size_t tcm_buf_size, ChromaOffset, tcm_desc_size, region_tcm_size;
	t_StDmaWrapper_PrepareParm stWrapPrepParm;
	t_StDmaWrapper_WorkDescrip staWorkDesc[2];
	t_StDmaWrapper_FrameProp walkRoi;
	t_StDmaWrapper_FrameProp frameProp;
	int nIdx = 0;
	int nRow, nCol;
	int nRowIdx = 0, nColIdx = 0;
	t_StDmaWrapper_UpdateParm stWrapUpdateParm[2];
	int offsetC = MAX(totalHeight, totalPadHeight) * MAX(nFrameWidth,  nPadWidth) + jobCount * (dptr->nFrameHeight >> 1) * (uvpad_flag ? MAX(nFrameWidth, nPadWidth) : nFrameWidth);

	int ubwcHeight = ALIGNUP(nFrameHeight, 32);
	int ubwcWidth = ALIGNUP(nFrameWidth, 128);
	
	stRoi.u16W = 256;
	stRoi.u16H = 32;
	nRet = nDmaWrapper_GetRecommendedRoi(eDmaFmt_NV12, ISUBWC, &stRoi);
	if(nRet)
	{	
		nRet = UBWC_ERR_DMAGETROI;
		LOGE("nDmaWrapper_GetRecommendedRoi error.");
		goto out;
	}

	nRow = ROUNDDIV(ubwcHeight, stRoi.u16H);
	nCol = ROUNDDIV(ubwcWidth,  stRoi.u16W);

	LOGI("stRoi.u16W = %d, stRoi.u16H = %d, nRow = %d, nCol = %d", stRoi.u16W, stRoi.u16H, nRow, nCol);

	stAlignInfo.u16W = stRoi.u16W;
	stAlignInfo.u16H = stRoi.u16H;

	lumaStride = nDmaWrapper_GetRecommendedIntermBufStride(eFmtLuma, &stAlignInfo, ISUBWC);
	chromaStride = nDmaWrapper_GetRecommendedIntermBufStride(eFmtChroma, &stAlignInfo, ISUBWC);

	LOGI("lumaStride = %d, chromaStride = %d", lumaStride, chromaStride);

	tcm_buf_size = nDmaWrapper_GetRecommendedIntermBufSize(eFmtLuma, PAD16BIT, &stAlignInfo, ISUBWC, lumaStride);
	ChromaOffset = tcm_buf_size;
	tcm_buf_size += nDmaWrapper_GetRecommendedIntermBufSize(eFmtChroma, PAD16BIT, &stAlignInfo, ISUBWC, chromaStride);
	region_tcm_size = ALIGNUP(tcm_buf_size * 2, 0x1000);
	tcm_desc_size = ALIGNUP(nDmaWrapper_GetDescbuffsize(efmtLumaChroma, 2) * 2 * 2, 0x1000);


	LOGI("tcm_buf_size = %d, region_tcm_size = %d, tcm_desc_size = %d, ChromaOffset = %d",
		  tcm_buf_size, region_tcm_size, tcm_desc_size, ChromaOffset);
		
	tcm_desc_vaddr = (addr_t)HAP_cache_lock((unsigned int)tcm_desc_size, &tcm_desc_paddr);
	tcm_buf_vaddr = (addr_t)HAP_cache_lock((unsigned int)region_tcm_size, &tcm_buf_paddr);
	if(!tcm_desc_vaddr || !tcm_buf_vaddr)
	{
		nRet = UBWC_ERR_ALLOCTCM;
		LOGE("HAP_cache_lock error.");
		goto out;
	}

	LOGI("tcm_desc_vaddr = 0x%x, tcm_buf_vaddr = 0x%x, tcm_desc_paddr = 0x%x, tcm_buf_paddr = 0x%x", 
			tcm_desc_vaddr, tcm_buf_vaddr, qurt_lookup_physaddr(tcm_desc_vaddr), qurt_lookup_physaddr(tcm_buf_vaddr));

#if 0
	handle = hDmaWrapper_AllocDmaSpecifyWaitType(eWaitType);
#endif
	if(!handle) 
	{
		nRet = UBWC_ERR_ALLOCDMA;
		LOGE("hDmaWrapper_AllocDmaSpecifyWaitType error.");
		goto error;
	}

#if 0
	nRet = nDmaWrapper_PowerVoting(PW_SVS2);
	if(nRet)
	{
		LOGE("nDmaWrapper_PowerVoting error.");
		goto error;
	}
#endif

	stWrapPrepParm.u32NumOfWorkDesc = 2;
	stWrapPrepParm.staWorkDesc = staWorkDesc;
	for (int i = 0; i < 1; i++)
	{
		for (int j = 0; j < 2; j++)
		{
			int descIndex = i * 2 + j;
			frameProp.aAddr = (addr_t)qurt_lookup_physaddr((qurt_addr_t)ubwc_buff);
			frameProp.u16W = ubwcWidth;
			frameProp.u16H = ALIGNUP(totalHeight, 32);//ubwcHeight;
			frameProp.u16Stride = ubwcWidth;
			walkRoi.aAddr = 0;
			walkRoi.u16W = stRoi.u16W;
			walkRoi.u16H = stRoi.u16H;
			walkRoi.u16Stride = (j) ? chromaStride : lumaStride;
			nRet = nDmaWrapper_WorkDescrip_populate(&staWorkDesc[descIndex],
													efmtLumaChroma[j],
													eDmaWrapper_L2ToDdr,
													&walkRoi, &frameProp,
													ISUBWC,
													PAD16BIT,
													NULL);
		}
	}
	stWrapPrepParm.u32DescBufSize = nDmaWrapper_GetDescbuffsize(efmtLumaChroma, 2);
	stWrapPrepParm.pPingDescBuf = (void *)tcm_desc_vaddr;
	stWrapPrepParm.pPongDescBuf = (void *)(tcm_desc_vaddr + stWrapPrepParm.u32DescBufSize);
	LOGI("stWrapPrepParm.u32DescBufSize = %d", stWrapPrepParm.u32DescBufSize);
	nRet |= nDmaWrapper_Prepare(handle, &stWrapPrepParm);
	if(nRet)
	{
		nRet = UBWC_ERR_DMAPREPARE;
		LOGE("nDmaWrapper_Prepare error.");
		goto error;
	}

	iRoiInfo_t iRoi;
	iRoi.nFrameHeight = ubwcHeight;
	iRoi.nFrameWidth = ubwcWidth;
	iRoi.nRoiWalkHeight = stRoi.u16H;
	iRoi.nRoiWalkWidth = stRoi.u16W;
	iRoi.tcm_buf_vaddr = tcm_buf_vaddr;
	iRoi.tcm_buf_size = tcm_buf_size;
	iRoi.ChromaOffset = ChromaOffset;
	
	updateRoiInfoThreads(nRowIdx, nRow, nColIdx, nCol, nIdx, &iRoi, &stWrapUpdateParm[0], jobCount * dptr->nFrameHeight);
#if 0
	writeTcmBuffer((void *)(tcm_buf_vaddr), dsp_buff, 
					(nRow == 1 && MARGIN(nFrameHeight, stRoi.u16H) != 0) ? MARGIN(nFrameHeight, stRoi.u16H) : stWrapUpdateParm[0].u.stPixData.stRoi.u16H, 
					(nCol == 1 && MARGIN(nFrameWidth, stRoi.u16W) != 0) ? MARGIN(nFrameWidth, stRoi.u16W) : stWrapUpdateParm[0].u.stPixData.stRoi.u16W, 
					stRoi.u16H, stRoi.u16W, 
					nColIdx, nRowIdx, stRoi.u16W, MAX(nFrameWidth, nPadWidth), jobCount * dptr->nFrameHeight * MAX(nFrameWidth, nPadWidth));
#else
	depthtospaceToTcmBuffer2((void *)(tcm_buf_vaddr), dsp_buff, 
					(nRow == 1 && MARGIN(nFrameHeight, stRoi.u16H) != 0) ? MARGIN(nFrameHeight, stRoi.u16H) : stWrapUpdateParm[0].u.stPixData.stRoi.u16H, 
					(nCol == 1 && MARGIN(nFrameWidth, stRoi.u16W) != 0) ? MARGIN(nFrameWidth, stRoi.u16W) : stWrapUpdateParm[0].u.stPixData.stRoi.u16W, 
					stRoi.u16H, stRoi.u16W, 
					nColIdx, nRowIdx, stRoi.u16W, MAX(nFrameWidth, nPadWidth) * 4, (jobCount * dptr->nFrameHeight >> 2) * MAX(nFrameWidth, nPadWidth) * 4,
					dptr->scale, dptr->zero_point);
#endif
	writeTcmBuffer((void *)(tcm_buf_vaddr + iRoi.ChromaOffset), dsp_buff, 
					((nRow == 1 && MARGIN(nFrameHeight, stRoi.u16H) != 0) ? MARGIN(nFrameHeight, stRoi.u16H) : stWrapUpdateParm[0].u.stPixData.stRoi.u16H) >> 1, 
					(nCol == 1 && MARGIN(nFrameWidth, stRoi.u16W) != 0) ? MARGIN(nFrameWidth, stRoi.u16W) : stWrapUpdateParm[0].u.stPixData.stRoi.u16W,  
					stRoi.u16H >> 1, stRoi.u16W,
				    nColIdx, nRowIdx,  stRoi.u16W, (uvpad_flag ? MAX(nFrameWidth, nPadWidth) : nFrameWidth), offsetC);
	nRet = nDmaWrapper_Update(handle, &stWrapUpdateParm[0]);
	if(nRet)
	{
		nRet = UBWC_ERR_DMAUPDATE;
		LOGE("nDmaWrapper_Update error.");
		goto error;
	}

	nRet = nDmaWrapper_Move(handle);
	if(nRet)
	{
		nRet = UBWC_ERR_DMAMOVE;
		LOGE("nDmaWrapper_Move error.");
		goto error;
	}
	nIdx = (nIdx + 1) % 2;

	LOGI("nDmaWrapper_Move size = %d\n", stWrapUpdateParm[0].u.stPixData.stRoi.u16H * stWrapUpdateParm[0].u.stPixData.stRoi.u16W);
	
	for (nRowIdx = 0; nRowIdx < nRow; nRowIdx++)
	{
		for (nColIdx = (nRowIdx)? 0:1; nColIdx < nCol; nColIdx++)
		{
			updateRoiInfoThreads(nRowIdx, nRow, nColIdx, nCol, nIdx, &iRoi, &stWrapUpdateParm[0], jobCount * dptr->nFrameHeight);
		#if 0
			writeTcmBuffer((void *)(tcm_buf_vaddr + (nIdx % 2 ? tcm_buf_size : 0)), dsp_buff, 
							(nRowIdx == (nRow - 1) && MARGIN(nFrameHeight, stRoi.u16H) != 0) ? MARGIN(nFrameHeight, stRoi.u16H) : stWrapUpdateParm[0].u.stPixData.stRoi.u16H, 
					        (nColIdx == (nCol - 1) && MARGIN(nFrameWidth, stRoi.u16W) != 0) ? MARGIN(nFrameWidth, stRoi.u16W) : stWrapUpdateParm[0].u.stPixData.stRoi.u16W,  
							stRoi.u16H, stRoi.u16W, 
							nColIdx, nRowIdx, stRoi.u16W, MAX(nFrameWidth, nPadWidth), jobCount * dptr->nFrameHeight * MAX(nFrameWidth, nPadWidth));
		#else
			depthtospaceToTcmBuffer2((void *)(tcm_buf_vaddr + (nIdx % 2 ? tcm_buf_size : 0)), dsp_buff, 
							(nRow == 1 && MARGIN(nFrameHeight, stRoi.u16H) != 0) ? MARGIN(nFrameHeight, stRoi.u16H) : stWrapUpdateParm[0].u.stPixData.stRoi.u16H, 
							(nCol == 1 && MARGIN(nFrameWidth, stRoi.u16W) != 0) ? MARGIN(nFrameWidth, stRoi.u16W) : stWrapUpdateParm[0].u.stPixData.stRoi.u16W, 
							stRoi.u16H, stRoi.u16W, 
							nColIdx, nRowIdx, stRoi.u16W, MAX(nFrameWidth, nPadWidth) * 4, (jobCount * dptr->nFrameHeight >> 2) * MAX(nFrameWidth, nPadWidth) * 4,
							dptr->scale, dptr->zero_point);	
		#endif
			writeTcmBuffer((void *)(tcm_buf_vaddr + (nIdx % 2 ? tcm_buf_size : 0) + iRoi.ChromaOffset), dsp_buff, 
				            ((nRowIdx == (nRow - 1) && MARGIN(nFrameHeight, stRoi.u16H) != 0) ? MARGIN(nFrameHeight, stRoi.u16H) : stWrapUpdateParm[0].u.stPixData.stRoi.u16H) >> 1, 
							(nColIdx == (nCol - 1) && MARGIN(nFrameWidth, stRoi.u16W) != 0) ? MARGIN(nFrameWidth, stRoi.u16W) : stWrapUpdateParm[0].u.stPixData.stRoi.u16W,
							stRoi.u16H >> 1, stRoi.u16W,
							nColIdx, nRowIdx,  stRoi.u16W, (uvpad_flag ? MAX(nFrameWidth, nPadWidth) : nFrameWidth), offsetC);
			nRet = nDmaWrapper_Update(handle, &stWrapUpdateParm[0]);
			nRet |= nDmaWrapper_Wait(handle);
			nRet |= nDmaWrapper_Move(handle);
			if(nRet)
			{
				nRet = UBWC_ERR_DMAMOVE | UBWC_ERR_DMAUPDATE | UBWC_ERR_DMAWAIT;
				LOGE("nDmaWrapper_Move or nDmaWrapper_Update or nDmaWrapper_Wait error.");
				goto error;
			}		
			nIdx = (nIdx + 1) % 2;
			LOGI("nDmaWrapper_Move size = %d\n", stWrapUpdateParm[0].u.stPixData.stRoi.u16H * stWrapUpdateParm[0].u.stPixData.stRoi.u16W);
		}
	}
	nRet = nDmaWrapper_Wait(handle);
	if(nRet)
	{
		nRet = UBWC_ERR_DMAWAIT; 
		LOGE("nDmaWrapper_Wait error.");
		goto error;
	}
	
	nRet = nDmaWrapper_FinishFrame(handle);
	if(nRet)
	{
		nRet = UBWC_ERR_DMAFINISH; 
		LOGE("nDmaWrapper_FinishFrame error.");
		goto error;
	}

#if 0
	nRet = nDmaWrapper_FreeDma(handle);
	if(nRet)
	{
		LOGE("nDmaWrapper_FreeDma error.");
		goto error;
	}
#endif

error:
	HAP_cache_unlock((void*)tcm_buf_vaddr);
	HAP_cache_unlock((void*)tcm_desc_vaddr);
out:
	dptr->nRet[jobCount] = nRet;
    worker_pool_synctoken_jobdone(dptr->token);
	
	return;
}

int aiboostubwc_writeBuffer1(const unsigned char *dsp_buff, int dsp_size, unsigned char *ubwc_buff, int ubwc_size, int32 nFrameHeight, int32 nFrameWidth, float scale, int zero_point)
{
#ifdef PROFILING_ON
	uint64 startTime = HAP_perf_get_time_us();
	uint64 startCycles = HAP_perf_get_pcycles();
#endif
	
	int nRet = UBWC_OK;

	qurt_mem_cache_clean((qurt_addr_t)ubwc_buff, ubwc_size, QURT_MEM_CACHE_FLUSH_INVALIDATE, QURT_MEM_DCACHE);

	int numWorkers = MIN((qurt_hvx_get_units() >> 8) & 0xFF, MAXTHREADS);

	worker_pool_job_t   job;
    worker_synctoken_t    token;
    worker_pool_context_t context = NULL;
	
	ubwc_callback_t dptr;
	int retry_times = 100;
	int dma_alloc_flag = 0;

	while(retry_times--)
	{
		for(int i = 0; i < numWorkers; i++)
		{
			dptr.nRet[i] = UBWC_OK;
			dptr.handle[i] = hDmaWrapper_AllocDmaSpecifyWaitType(eDmaWaitType_Polling);
			if(!dptr.handle[i])
			{
				numWorkers = i;
				break;
			}
		}

		if(numWorkers == 0)
		{
			numWorkers = 1;
			qurt_timer_sleep(1);
		}
		else
		{
			dma_alloc_flag = 1;
			break;
		}
	}
	if(dma_alloc_flag == 0)
	{
		nRet = UBWC_ERR_ALLOCDMA;
		LOGE("hDmaWrapper_AllocDmaSpecifyWaitType error.");
		return nRet;
	}
	
	int h = nFrameHeight & 0xffff;
	int n = ROUNDDIV(h, 32);
	int nn = ROUNDDIV(n, numWorkers);
	
	LOGI("MultiThreds:hvxInfo.numThreads = %d, numWorkers = %d", hvxInfo.numThreads, numWorkers);

	job.fptr = ubwc_writeBuffer_callback1;

	dptr.token = &token;
	dptr.jobCount = 0;
	dptr.ubwc_buff = ubwc_buff;
	dptr.dsp_buff =  (unsigned char*)dsp_buff;
	dptr.nFrameHeight = nn * 32;
	dptr.nFrameWidth = nFrameWidth;
	dptr.totalHeight = nFrameHeight;
	dptr.numThreads = numWorkers;
	dptr.scale = scale;
	dptr.zero_point = zero_point;

	job.dptr = (void *)&dptr;
	
	//(void)worker_pool_init(&context);
    worker_pool_synctoken_init(&token, numWorkers);

	for(int i = 0; i < numWorkers; i++)
	{
	   (void)worker_pool_submit(context, job);
	}
	worker_pool_synctoken_wait(&token);

	//worker_pool_deinit(&context);
	
	for(int i = 0; i < numWorkers; i++)
	{
		if(dptr.handle[i]) nRet |= (nDmaWrapper_FreeDma(dptr.handle[i]) < 0 ? UBWC_ERR_DMAFREE : UBWC_OK);
		nRet |= dptr.nRet[i];
	}
	
#ifdef PROFILING_ON
	uint64 endCycles = HAP_perf_get_pcycles();
	uint64 endTime = HAP_perf_get_time_us();
	FARF(ALWAYS, "[UBWCWRITE]profiling: %d PCycles, %d us. Observed clock rate %d MHz",
	 (int)(endCycles - startCycles), (int)(endTime - startTime), (int)((endCycles - startCycles) / (endTime - startTime)));
#endif

	return nRet;
}

static void ubwc_writeBuffer_callback2(void *data)
{
	int nRet = UBWC_OK;
	ubwc_callback_t *dptr = (ubwc_callback_t *)data;
	unsigned int jobCount = 0;
	
	jobCount = worker_pool_atomic_inc_return(&(dptr->jobCount)) - 1;
		
	int nFrameHeight = dptr->nFrameHeight;
	int nFrameWidth = dptr->nFrameWidth & 0xffff;
	int totalHeight = dptr->totalHeight & 0xffff;
	unsigned char *ubwc_buff = dptr->ubwc_buff;
	unsigned char *dsp_buff = dptr->dsp_buff;
	unsigned char *dsp_buff2 = dptr->dsp_buff2;
	
	if(jobCount * nFrameHeight >= totalHeight) goto out;

	if((jobCount == dptr->numThreads - 1) || (totalHeight - jobCount * dptr->nFrameHeight < dptr->nFrameHeight)) 
	{
		nFrameHeight = totalHeight - jobCount * dptr->nFrameHeight;
	}

	int d32_wpad_after = ((D32_WPAD_BEFORE + (nFrameWidth >> 2) + 3) & ~3) - (D32_WPAD_BEFORE + (nFrameWidth >> 2));
	int dtos_width = (nFrameWidth >> 2) + d32_wpad_after + D32_WPAD_BEFORE;

	t_DmaWrapper_DmaEngineHandle handle = dptr->handle[jobCount];
	//t_EDma_WaitType eWaitType = eDmaWaitType_Polling;
	t_eDmaFmt eFmtLuma = eDmaFmt_NV12_Y;
	t_eDmaFmt eFmtChroma = eDmaFmt_NV12_UV;

	t_eDmaFmt efmtLumaChroma[2] = {eFmtLuma, eFmtChroma};

	t_StDmaWrapper_Roi stRoi;
	t_StDmaWrapper_RoiAlignInfo stAlignInfo;
	int lumaStride, chromaStride;
	qurt_addr_t tcm_buf_vaddr, tcm_desc_vaddr;
	unsigned long long tcm_buf_paddr, tcm_desc_paddr;
	qurt_size_t tcm_buf_size, ChromaOffset, tcm_desc_size, region_tcm_size;
	t_StDmaWrapper_PrepareParm stWrapPrepParm;
	t_StDmaWrapper_WorkDescrip staWorkDesc[2];
	t_StDmaWrapper_FrameProp walkRoi;
	t_StDmaWrapper_FrameProp frameProp;
	int nIdx = 0;
	int nRow, nCol;
	int nRowIdx = 0, nColIdx = 0;
	t_StDmaWrapper_UpdateParm stWrapUpdateParm[2];
	int offsetC = jobCount * (dptr->nFrameHeight >> 1) * ALIGNUP(nFrameWidth, 256);

	int ubwcHeight = ALIGNUP(nFrameHeight, 32);
	int ubwcWidth = ALIGNUP(nFrameWidth, 128);
	
	stRoi.u16W = 256;
	stRoi.u16H = 32;
	nRet = nDmaWrapper_GetRecommendedRoi(eDmaFmt_NV12, ISUBWC, &stRoi);
	if(nRet)
	{	
		nRet = UBWC_ERR_DMAGETROI;
		LOGE("nDmaWrapper_GetRecommendedRoi error.");
		goto out;
	}

	nRow = ROUNDDIV(ubwcHeight, stRoi.u16H);
	nCol = ROUNDDIV(ubwcWidth,  stRoi.u16W);

	LOGI("stRoi.u16W = %d, stRoi.u16H = %d, nRow = %d, nCol = %d", stRoi.u16W, stRoi.u16H, nRow, nCol);

	stAlignInfo.u16W = stRoi.u16W;
	stAlignInfo.u16H = stRoi.u16H;

	lumaStride = nDmaWrapper_GetRecommendedIntermBufStride(eFmtLuma, &stAlignInfo, ISUBWC);
	chromaStride = nDmaWrapper_GetRecommendedIntermBufStride(eFmtChroma, &stAlignInfo, ISUBWC);

	LOGI("lumaStride = %d, chromaStride = %d", lumaStride, chromaStride);

	tcm_buf_size = nDmaWrapper_GetRecommendedIntermBufSize(eFmtLuma, PAD16BIT, &stAlignInfo, ISUBWC, lumaStride);
	ChromaOffset = tcm_buf_size;
	tcm_buf_size += nDmaWrapper_GetRecommendedIntermBufSize(eFmtChroma, PAD16BIT, &stAlignInfo, ISUBWC, chromaStride);
	region_tcm_size = ALIGNUP(tcm_buf_size * 2, 0x1000);
	tcm_desc_size = ALIGNUP(nDmaWrapper_GetDescbuffsize(efmtLumaChroma, 2) * 2 * 2, 0x1000);


	LOGI("tcm_buf_size = %d, region_tcm_size = %d, tcm_desc_size = %d, ChromaOffset = %d",
		  tcm_buf_size, region_tcm_size, tcm_desc_size, ChromaOffset);
		
	tcm_desc_vaddr = (addr_t)HAP_cache_lock((unsigned int)tcm_desc_size, &tcm_desc_paddr);
	tcm_buf_vaddr = (addr_t)HAP_cache_lock((unsigned int)region_tcm_size, &tcm_buf_paddr);
	if(!tcm_desc_vaddr || !tcm_buf_vaddr)
	{
		nRet = UBWC_ERR_ALLOCTCM;
		LOGE("HAP_cache_lock error.");
		goto out;
	}

	LOGI("tcm_desc_vaddr = 0x%x, tcm_buf_vaddr = 0x%x, tcm_desc_paddr = 0x%x, tcm_buf_paddr = 0x%x", 
			tcm_desc_vaddr, tcm_buf_vaddr, qurt_lookup_physaddr(tcm_desc_vaddr), qurt_lookup_physaddr(tcm_buf_vaddr));

#if 0
	handle = hDmaWrapper_AllocDmaSpecifyWaitType(eWaitType);
#endif
	if(!handle) 
	{
		nRet = UBWC_ERR_ALLOCDMA;
		LOGE("hDmaWrapper_AllocDmaSpecifyWaitType error.");
		goto error;
	}

#if 0
	nRet = nDmaWrapper_PowerVoting(PW_SVS2);
	if(nRet)
	{
		LOGE("nDmaWrapper_PowerVoting error.");
		goto error;
	}
#endif

	stWrapPrepParm.u32NumOfWorkDesc = 2;
	stWrapPrepParm.staWorkDesc = staWorkDesc;
	for (int i = 0; i < 1; i++)
	{
		for (int j = 0; j < 2; j++)
		{
			int descIndex = i * 2 + j;
			frameProp.aAddr = (addr_t)qurt_lookup_physaddr((qurt_addr_t)ubwc_buff);
			frameProp.u16W = ubwcWidth;
			frameProp.u16H = ALIGNUP(totalHeight, 32);//ubwcHeight;
			frameProp.u16Stride = ubwcWidth;
			walkRoi.aAddr = 0;
			walkRoi.u16W = stRoi.u16W;
			walkRoi.u16H = stRoi.u16H;
			walkRoi.u16Stride = (j) ? chromaStride : lumaStride;
			nRet = nDmaWrapper_WorkDescrip_populate(&staWorkDesc[descIndex],
													efmtLumaChroma[j],
													eDmaWrapper_L2ToDdr,
													&walkRoi, &frameProp,
													ISUBWC,
													PAD16BIT,
													NULL);
		}
	}
	stWrapPrepParm.u32DescBufSize = nDmaWrapper_GetDescbuffsize(efmtLumaChroma, 2);
	stWrapPrepParm.pPingDescBuf = (void *)tcm_desc_vaddr;
	stWrapPrepParm.pPongDescBuf = (void *)(tcm_desc_vaddr + stWrapPrepParm.u32DescBufSize);
	LOGI("stWrapPrepParm.u32DescBufSize = %d", stWrapPrepParm.u32DescBufSize);
	nRet |= nDmaWrapper_Prepare(handle, &stWrapPrepParm);
	if(nRet)
	{
		nRet = UBWC_ERR_DMAPREPARE;
		LOGE("nDmaWrapper_Prepare error.");
		goto error;
	}

	iRoiInfo_t iRoi;
	iRoi.nFrameHeight = ubwcHeight;
	iRoi.nFrameWidth = ubwcWidth;
	iRoi.nRoiWalkHeight = stRoi.u16H;
	iRoi.nRoiWalkWidth = stRoi.u16W;
	iRoi.tcm_buf_vaddr = tcm_buf_vaddr;
	iRoi.tcm_buf_size = tcm_buf_size;
	iRoi.ChromaOffset = ChromaOffset;
	
	updateRoiInfoThreads(nRowIdx, nRow, nColIdx, nCol, nIdx, &iRoi, &stWrapUpdateParm[0], jobCount * dptr->nFrameHeight);
	depthtospaceToTcmBuffer((void *)(tcm_buf_vaddr), dsp_buff, 
					(nRow == 1 && MARGIN(nFrameHeight, stRoi.u16H) != 0) ? MARGIN(nFrameHeight, stRoi.u16H) : stWrapUpdateParm[0].u.stPixData.stRoi.u16H, 
					(nCol == 1 && MARGIN(nFrameWidth, stRoi.u16W) != 0) ? MARGIN(nFrameWidth, stRoi.u16W) : stWrapUpdateParm[0].u.stPixData.stRoi.u16W, 
					stRoi.u16H, stRoi.u16W, 
					nColIdx, nRowIdx, stRoi.u16W, dtos_width * 32, D32_HPAD_BEFORE * dtos_width * 32 + (jobCount * dptr->nFrameHeight >> 2) * dtos_width * 32,
					dptr->scale, dptr->zero_point);
	writeTcmBuffer((void *)(tcm_buf_vaddr + iRoi.ChromaOffset), dsp_buff2, 
					((nRow == 1 && MARGIN(nFrameHeight, stRoi.u16H) != 0) ? MARGIN(nFrameHeight, stRoi.u16H) : stWrapUpdateParm[0].u.stPixData.stRoi.u16H) >> 1, 
					(nCol == 1 && MARGIN(nFrameWidth, stRoi.u16W) != 0) ? MARGIN(nFrameWidth, stRoi.u16W) : stWrapUpdateParm[0].u.stPixData.stRoi.u16W,  
					stRoi.u16H >> 1, stRoi.u16W,
				    nColIdx, nRowIdx,  stRoi.u16W, ALIGNUP(nFrameWidth, 256), offsetC);
	nRet = nDmaWrapper_Update(handle, &stWrapUpdateParm[0]);
	if(nRet)
	{
		nRet = UBWC_ERR_DMAUPDATE;
		LOGE("nDmaWrapper_Update error.");
		goto error;
	}

	nRet = nDmaWrapper_Move(handle);
	if(nRet)
	{
		nRet = UBWC_ERR_DMAMOVE;
		LOGE("nDmaWrapper_Move error.");
		goto error;
	}
	nIdx = (nIdx + 1) % 2;

	LOGI("nDmaWrapper_Move size = %d\n", stWrapUpdateParm[0].u.stPixData.stRoi.u16H * stWrapUpdateParm[0].u.stPixData.stRoi.u16W);
	
	for (nRowIdx = 0; nRowIdx < nRow; nRowIdx++)
	{
		for (nColIdx = (nRowIdx)? 0:1; nColIdx < nCol; nColIdx++)
		{
			updateRoiInfoThreads(nRowIdx, nRow, nColIdx, nCol, nIdx, &iRoi, &stWrapUpdateParm[0], jobCount * dptr->nFrameHeight);
			depthtospaceToTcmBuffer((void *)(tcm_buf_vaddr + (nIdx % 2 ? tcm_buf_size : 0)), dsp_buff, 
							(nRowIdx == (nRow - 1) && MARGIN(nFrameHeight, stRoi.u16H) != 0) ? MARGIN(nFrameHeight, stRoi.u16H) : stWrapUpdateParm[0].u.stPixData.stRoi.u16H, 
					        (nColIdx == (nCol - 1) && MARGIN(nFrameWidth, stRoi.u16W) != 0) ? MARGIN(nFrameWidth, stRoi.u16W) : stWrapUpdateParm[0].u.stPixData.stRoi.u16W,  
							stRoi.u16H, stRoi.u16W, 
							nColIdx, nRowIdx, stRoi.u16W, dtos_width * 32, D32_HPAD_BEFORE * dtos_width * 32 + (jobCount * dptr->nFrameHeight >> 2) * dtos_width * 32,
							dptr->scale, dptr->zero_point);
			writeTcmBuffer((void *)(tcm_buf_vaddr + (nIdx % 2 ? tcm_buf_size : 0) + iRoi.ChromaOffset), dsp_buff2, 
				            ((nRowIdx == (nRow - 1) && MARGIN(nFrameHeight, stRoi.u16H) != 0) ? MARGIN(nFrameHeight, stRoi.u16H) : stWrapUpdateParm[0].u.stPixData.stRoi.u16H) >> 1, 
							(nColIdx == (nCol - 1) && MARGIN(nFrameWidth, stRoi.u16W) != 0) ? MARGIN(nFrameWidth, stRoi.u16W) : stWrapUpdateParm[0].u.stPixData.stRoi.u16W,
							stRoi.u16H >> 1, stRoi.u16W,
							nColIdx, nRowIdx,  stRoi.u16W, ALIGNUP(nFrameWidth, 256), offsetC);
			nRet = nDmaWrapper_Update(handle, &stWrapUpdateParm[0]);
			nRet |= nDmaWrapper_Wait(handle);
			nRet |= nDmaWrapper_Move(handle);
			if(nRet)
			{
				nRet = UBWC_ERR_DMAMOVE | UBWC_ERR_DMAUPDATE | UBWC_ERR_DMAWAIT;
				LOGE("nDmaWrapper_Move or nDmaWrapper_Update or nDmaWrapper_Wait error.");
				goto error;
			}		
			nIdx = (nIdx + 1) % 2;
			LOGI("nDmaWrapper_Move size = %d\n", stWrapUpdateParm[0].u.stPixData.stRoi.u16H * stWrapUpdateParm[0].u.stPixData.stRoi.u16W);
		}
	}
	nRet = nDmaWrapper_Wait(handle);
	if(nRet)
	{
		nRet = UBWC_ERR_DMAWAIT; 
		LOGE("nDmaWrapper_Wait error.");
		goto error;
	}
	
	nRet = nDmaWrapper_FinishFrame(handle);
	if(nRet)
	{
		nRet = UBWC_ERR_DMAFINISH; 
		LOGE("nDmaWrapper_FinishFrame error.");
		goto error;
	}

#if 0
	nRet = nDmaWrapper_FreeDma(handle);
	if(nRet)
	{
		LOGE("nDmaWrapper_FreeDma error.");
		goto error;
	}
#endif

error:
	HAP_cache_unlock((void*)tcm_buf_vaddr);
	HAP_cache_unlock((void*)tcm_desc_vaddr);
out:
	dptr->nRet[jobCount] = nRet;
    worker_pool_synctoken_jobdone(dptr->token);
	
	return;
}


static int aiboostubwc_writeBuffer2(const unsigned char *dsp_buff, const unsigned char *dsp_buff2, unsigned char *ubwc_buff, int ubwc_size, int32 nFrameHeight, int32 nFrameWidth, float scale, int zero_point)
{
#ifdef PROFILING_ON
	uint64 startTime = HAP_perf_get_time_us();
	uint64 startCycles = HAP_perf_get_pcycles();
#endif
	
	int nRet = UBWC_OK;

	qurt_mem_cache_clean((qurt_addr_t)ubwc_buff, ubwc_size, QURT_MEM_CACHE_FLUSH_INVALIDATE, QURT_MEM_DCACHE);

	int numWorkers = MIN((qurt_hvx_get_units() >> 8) & 0xFF, MAXTHREADS);

	worker_pool_job_t   job;
    worker_synctoken_t    token;
    worker_pool_context_t context = NULL;
	
	ubwc_callback_t dptr;
	int retry_times = 100;
	int dma_alloc_flag = 0;

	while(retry_times--)
	{
		for(int i = 0; i < numWorkers; i++)
		{
			dptr.nRet[i] = UBWC_OK;
			dptr.handle[i] = hDmaWrapper_AllocDmaSpecifyWaitType(eDmaWaitType_Polling);
			if(!dptr.handle[i])
			{
				numWorkers = i;
				break;
			}
		}

		if(numWorkers == 0)
		{
			numWorkers = 1;
			qurt_timer_sleep(1);
		}
		else
		{
			dma_alloc_flag = 1;
			break;
		}
	}
	if(dma_alloc_flag == 0)
	{
		nRet = UBWC_ERR_ALLOCDMA;
		LOGE("hDmaWrapper_AllocDmaSpecifyWaitType error.");
		return nRet;
	}

	int h = nFrameHeight & 0xffff;
	int n = ROUNDDIV(h, 32);
	int nn = ROUNDDIV(n, numWorkers);
	
	LOGI("MultiThreds:hvxInfo.numThreads = %d, numWorkers = %d", hvxInfo.numThreads, numWorkers);

	job.fptr = ubwc_writeBuffer_callback2;

	dptr.token = &token;
	dptr.jobCount = 0;
	dptr.ubwc_buff = ubwc_buff;
	dptr.dsp_buff =  (unsigned char*)dsp_buff;
	dptr.dsp_buff2 =  (unsigned char*)dsp_buff2;
	dptr.nFrameHeight = nn * 32;
	dptr.nFrameWidth = nFrameWidth;
	dptr.totalHeight = nFrameHeight;
	dptr.numThreads = numWorkers;
	dptr.scale = scale;
	dptr.zero_point = zero_point;

	job.dptr = (void *)&dptr;
	
	//(void)worker_pool_init(&context);
    worker_pool_synctoken_init(&token, numWorkers);

	for(int i = 0; i < numWorkers; i++)
	{
	   (void)worker_pool_submit(context, job);
	}
	worker_pool_synctoken_wait(&token);
	
	//worker_pool_deinit(&context);
	
	for(int i = 0; i < numWorkers; i++)
	{
		if(dptr.handle[i]) nRet |= (nDmaWrapper_FreeDma(dptr.handle[i]) < 0 ? UBWC_ERR_DMAFREE : UBWC_OK);
		nRet |= dptr.nRet[i];
	}
	
#ifdef PROFILING_ON
	uint64 endCycles = HAP_perf_get_pcycles();
	uint64 endTime = HAP_perf_get_time_us();
	FARF(ALWAYS, "[UBWCWRITE]profiling: %d PCycles, %d us. Observed clock rate %d MHz",
	 (int)(endCycles - startCycles), (int)(endTime - startTime), (int)((endCycles - startCycles) / (endTime - startTime)));
#endif

	return nRet;
}


#endif
#ifdef DEQUANT_MULTI_THREADS
static void dequant_callback(void* data)
{
	dequant_callback_t *dptr = (dequant_callback_t*)data;
	
	unsigned int jobCount = worker_pool_atomic_inc_return(&(dptr->jobCount)) - 1;
			
	LOGI("MultiThreads:jobCount = %d, sizePerJob = %d, buff_size = %d", jobCount, dptr->sizePerJob,  dptr->buff_size);

	if(jobCount * dptr->sizePerJob >= dptr->buff_size)
    {
        return;
    }

	const int size = dptr->sizePerJob;
	const int vsize = size >> 7;

	const unsigned char *dsp_buff =  dptr->dsp_buff + jobCount * size;
	unsigned char *dequant_buff = dptr->dequant_buff + jobCount * size;
	
	struct qf16 qf_scale = f2q(dptr->scale * 255);
	HVX_Vector *in_vec = (HVX_Vector *)dsp_buff;
	HVX_Vector *out_vec = (HVX_Vector *)dequant_buff;

	if(dptr->zero_point == 0)
	{
		for(int i = 0; i < vsize; i++)
		{
			HVX_Vector a_mant0, a_mant1, a_exp0, a_exp1;
			HVX_Vector b_mant0, b_mant1, b_exp0, b_exp1;
			
			vs2q(in_vec, &a_mant0, &a_exp0, &a_mant1, &a_exp1);

			vsmpyqf(a_mant0, a_exp0, qf_scale.m, qf_scale.e, &b_mant0, &b_exp0);
			vsmpyqf(a_mant1, a_exp1, qf_scale.m, qf_scale.e, &b_mant1, &b_exp1);

			vq2s(out_vec, b_mant0, b_exp0, b_mant1, b_exp1);		
			
			in_vec++;
			out_vec++;
		}
	}
	else
	{
		struct qf16 qf_zero_point;
		i2q(dptr->zero_point, &qf_zero_point);
		HVX_Vector zero_point_mant = Q6_Vh_vsplat_R(qf_zero_point.m);
		HVX_Vector zero_point_exp = Q6_Vh_vsplat_R(qf_zero_point.e);
		
		for(int i = 0; i < vsize; i++)
		{
			HVX_Vector a_mant0, a_mant1, a_exp0, a_exp1;
			HVX_Vector b_mant0, b_mant1, b_exp0, b_exp1;

			vs2q(in_vec, &a_mant0, &a_exp0, &a_mant1, &a_exp1);

			vsubqf(a_mant0, a_exp0, zero_point_mant, zero_point_exp, &b_mant0, &b_exp0);
			vsubqf(a_mant1, a_exp1, zero_point_mant, zero_point_exp, &b_mant1, &b_exp1);

			vsmpyqf(b_mant0, b_exp0, qf_scale.m, qf_scale.e, &a_mant0, &a_exp0);
			vsmpyqf(b_mant1, b_exp1, qf_scale.m, qf_scale.e, &a_mant1, &a_exp1);

			vq2s(out_vec, a_mant0, a_exp0, a_mant1, a_exp1);

			in_vec++;
			out_vec++;
		}
	}
	for(int i = (vsize << 7); i < size; i++)
	{
		int tmp =  ((int)dsp_buff[i] - dptr->zero_point) * dptr->scale * 255;
		dequant_buff[i] = (unsigned char)CLIP_INTENAL(tmp, 0, 255);		
	}
		
    worker_pool_synctoken_jobdone(dptr->token);
}
#endif

int aiboostubwc_writeBufferDequant(const unsigned char *dsp_buff, int dsp_size, 
												unsigned char *ubwc_buff, int ubwc_size, 
												int32 nFrameHeight, int32 nFrameWidth,
												float scale, int32 zero_point)
{
#ifdef PROFILING_ON
	uint64 startTime = HAP_perf_get_time_us();
	uint64 startCycles = HAP_perf_get_pcycles();
#endif
	int nRet = UBWC_OK;
	unsigned char *dequant_buff = (unsigned char *)dsp_buff;
	if((int)dequant_buff & 127)
	{
		nRet = UBWC_ERR_ADDRALIGNED;
		LOGE("Dequant buffer is not 128 bytes aligned, addr = %p", dequant_buff);
        return nRet;
	}
	
#ifndef DEQUANT_MULTI_THREADS
	const int size = (nFrameHeight & 0xffff) * MAX((nFrameWidth & 0xffff), (nFrameWidth >> 16));
	const int vsize = size >> 7;
	
	struct qf16 qf_scale = f2q(scale * 255);
	HVX_Vector *in_vec = (HVX_Vector *)dsp_buff;
	HVX_Vector *out_vec = (HVX_Vector *)dequant_buff;

	if(zero_point == 0)
	{
		for(int i = 0; i < vsize; i++)
		{
			HVX_Vector a_mant0, a_mant1, a_exp0, a_exp1;
			HVX_Vector b_mant0, b_mant1, b_exp0, b_exp1;
			
			vs2q(in_vec, &a_mant0, &a_exp0, &a_mant1, &a_exp1);

			vsmpyqf(a_mant0, a_exp0, qf_scale.m, qf_scale.e, &b_mant0, &b_exp0);
			vsmpyqf(a_mant1, a_exp1, qf_scale.m, qf_scale.e, &b_mant1, &b_exp1);

			vq2s(out_vec, b_mant0, b_exp0, b_mant1, b_exp1);		
			
			in_vec++;
			out_vec++;
		}
	}
	else
	{
		struct qf16 qf_zero_point;
		i2q(zero_point, &qf_zero_point);
		HVX_Vector zero_point_mant = Q6_Vh_vsplat_R(qf_zero_point.m);
		HVX_Vector zero_point_exp = Q6_Vh_vsplat_R(qf_zero_point.e);
		
		for(int i = 0; i < vsize; i++)
		{
			HVX_Vector a_mant0, a_mant1, a_exp0, a_exp1;
			HVX_Vector b_mant0, b_mant1, b_exp0, b_exp1;

			vs2q(in_vec, &a_mant0, &a_exp0, &a_mant1, &a_exp1);

			vsubqf(a_mant0, a_exp0, zero_point_mant, zero_point_exp, &b_mant0, &b_exp0);
			vsubqf(a_mant1, a_exp1, zero_point_mant, zero_point_exp, &b_mant1, &b_exp1);

			vsmpyqf(b_mant0, b_exp0, qf_scale.m, qf_scale.e, &a_mant0, &a_exp0);
			vsmpyqf(b_mant1, b_exp1, qf_scale.m, qf_scale.e, &a_mant1, &a_exp1);

			vq2s(out_vec, a_mant0, a_exp0, a_mant1, a_exp1);

			in_vec++;
			out_vec++;
		}
	}
	for(int i = (vsize << 7); i < size; i++)
	{
		int tmp =  ((int)dsp_buff[i] - zero_point) * scale * 255;
		dequant_buff[i] = (unsigned char)CLIP_INTENAL(tmp, 0, 255);		
	}
#else
	int numWorkers = MIN((qurt_hvx_get_units() >> 8) & 0xFF, MAXTHREADS);

	worker_pool_job_t   job;
    worker_synctoken_t    token;
    worker_pool_context_t context = NULL;

	int buff_size = (nFrameHeight & 0xffff) * MAX((nFrameWidth & 0xffff), (nFrameWidth >> 16));
	while((buff_size % numWorkers) /*|| ((buff_size / numWorkers) & 127)*/)
	{
		numWorkers--;
		if(numWorkers == 0)
		{
			numWorkers = 1;
			break;
		}
	}
	LOGI("MultiThreds:hvxInfo.numThreads = %d, numWorkers = %d", hvxInfo.numThreads, numWorkers);
	
	job.fptr = dequant_callback;
	dequant_callback_t dptr;

	dptr.token = &token;
	dptr.jobCount = 0;
	dptr.dsp_buff = dsp_buff;
	dptr.dequant_buff = dequant_buff;
	dptr.buff_size = buff_size;
	dptr.sizePerJob = buff_size / numWorkers;
	dptr.scale = scale;
	dptr.zero_point = zero_point;

	job.dptr = (void *)&dptr;

	//(void)worker_pool_init(&context);
    worker_pool_synctoken_init(&token, numWorkers);
	
    for(int i = 0; i < numWorkers; i++)
    {
       (void)worker_pool_submit(context, job);
    }
    worker_pool_synctoken_wait(&token);

	//worker_pool_deinit(&context);
	
#endif
#ifdef PROFILING_ON
	uint64 endCycles = HAP_perf_get_pcycles();
	uint64 endTime = HAP_perf_get_time_us();
	FARF(ALWAYS, "[DEQUANTIZE]profiling: %d PCycles, %d us. Observed clock rate %d MHz",
		(int)(endCycles - startCycles), (int)(endTime - startTime), (int)((endCycles - startCycles) / (endTime - startTime)));
#endif

	nRet = aiboostubwc_writeBuffer(dequant_buff, dsp_size, ubwc_buff, ubwc_size, nFrameHeight, nFrameWidth);

	return nRet;
}

#if 0
static inline void ComputeInterpolationValues(const float value, const float scale,
                                                         int32 input_size, float* scaled_value,
                                                         int32* lower_bound, int32* upper_bound)
{
  *scaled_value = value * scale;
  
  float scaled_value_floor = floor(*scaled_value);
  float scaled_value_ceil = ceil(*scaled_value);
 
  *lower_bound = MAX((int32)scaled_value_floor, 0);
  *upper_bound = MIN((int32)scaled_value_ceil, input_size - 1);
}

static void resize_bilinear(unsigned char* input, unsigned char* output, 
							   int in_height, int in_width, int channels, 
							   int out_height, int out_width)
{
    float height_scale = (float)in_height / out_height;
    float width_scale = (float)in_width / out_width;
    
    for (int y = 0; y < out_height; ++y)
    {
	    float input_y;
	    int32 y0, y1;
    
    	ComputeInterpolationValues(y, height_scale, in_height, &input_y, &y0, &y1);
		unsigned char *input_y0 = input + y0 * in_width * channels;
		unsigned char *input_y1 = input + y1 * in_width * channels;
    
	    for (int x = 0; x < out_width; ++x)
	    {
			float input_x;
			int32 x0, x1;
		    ComputeInterpolationValues(x, width_scale,in_width, &input_x, &x0, &x1);

			unsigned char *input_y0x0 = input_y0 + x0 * channels;
			unsigned char *input_y0x1 = input_y0 + x1 * channels;
			unsigned char *input_y1x0 = input_y1 + x0 * channels;
			unsigned char *input_y1x1 = input_y1 + x1 * channels;

			for (int c = 0; c < channels; ++c) 
			{
				output[(y * out_width + x) * channels + c] = (unsigned char)(input_y0x0[c] * (1 - (input_y - y0)) * (1 - (input_x - x0)) +
							                             input_y1x0[c] * (input_y - y0) * (1 - (input_x - x0)) +
							                             input_y0x1[c] * (1 - (input_y - y0)) * (input_x - x0) +
							                             input_y1x1[c] * (input_y - y0) * (input_x - x0));
            }
	    }
    }
}
#else
void resize_bilinear(unsigned char* input, unsigned char* output, 
							   int in_height, int in_width, int channels, 
							   int out_height, int out_width);
#endif

#ifdef RESIZE_MULTI_THREADS
static void resize_callback(void* data)
{
    #define CHANNELS (2)
	resize_callback_t *dptr = (resize_callback_t*)data;
	int uv_in_width = dptr->uv_in_width;
	int uv_in_height = dptr->uv_in_height;
	int uv_out_width = dptr->uv_out_width;
	unsigned char *uv_input = dptr->uv_input;
	unsigned char *uv_output = dptr->uv_output;
	unsigned char *uv_buff = dptr->uv_buff;
	unsigned int h_in_block = dptr->h_in_block;
	unsigned int h_out_block = dptr->h_out_block;
	
	unsigned int jobCount = worker_pool_atomic_inc_return(&(dptr->jobCount)) - 1;

	if(jobCount * h_in_block >= uv_in_height)
    {
        return;
    }

	uv_input += jobCount * h_in_block * uv_in_width * CHANNELS;
	uv_output += jobCount * h_out_block * uv_out_width * CHANNELS;

	if(uv_in_width & (uv_in_width - 1))
	{
		int in_width_pad = ALIGN2POWN(uv_in_width);
		int out_width_pad = in_width_pad << 1;
		unsigned char *uv_buff_pad = (unsigned char*)uv_buff + jobCount * h_in_block * in_width_pad * CHANNELS;
		unsigned char *out_buff_pad = (unsigned char*)uv_buff + in_width_pad * uv_in_height * CHANNELS + jobCount * h_out_block * out_width_pad * CHANNELS;

		resize_bilinear(uv_buff_pad, out_buff_pad, h_in_block, in_width_pad, CHANNELS, h_out_block, out_width_pad);

		for(int i = 0; i < h_out_block; i++)
		{
			vmemcpy_asm(uv_output + i * uv_out_width * CHANNELS, out_buff_pad + i * out_width_pad * CHANNELS, uv_out_width * CHANNELS);
		}
	}
	else if(((int)uv_input & 127) || ((int)uv_output & 127))
	{
		unsigned char *uv_buff_pad = (unsigned char*)uv_buff + jobCount * h_in_block * uv_in_width * CHANNELS;
		unsigned char *out_buff_pad = (unsigned char*)uv_buff + uv_in_width * uv_in_height * CHANNELS + jobCount * h_out_block * uv_out_width * CHANNELS;
		
		resize_bilinear(uv_buff_pad, out_buff_pad, h_in_block, uv_in_width, CHANNELS, h_out_block, uv_out_width);

		vmemcpy_asm(uv_output, out_buff_pad, h_out_block * uv_out_width * CHANNELS);
	}
	else
	{
		resize_bilinear(uv_input, uv_output, h_in_block, uv_in_width, CHANNELS, h_out_block, uv_out_width);
	}

    worker_pool_synctoken_jobdone(dptr->token);
}
#endif

int aiboostubwc_writeBufferDequantResize(const unsigned char *dsp_buff, int dsp_size, 
												const unsigned char *uv_buff, int uv_size, 
												unsigned char *ubwc_buff, int ubwc_size, 
												int32 nFrameHeight, int32 nFrameWidth,
												float scale, int32 zero_point)
{
#ifdef PROFILING_ON
	uint64 startTime = HAP_perf_get_time_us();
	uint64 startCycles = HAP_perf_get_pcycles();
#endif
	int nRet = UBWC_OK;

	int nFrameHeightBak = nFrameHeight;
	int nFrameWidthBak = nFrameWidth;
	int nPadHeight = nFrameHeight >> 16;
	int nPadWidth = nFrameWidth >> 16;
	nFrameHeight &= 0xffff;
	nFrameWidth &= 0xffff;

	#define CHANNELS (2)
	int uv_in_height = nFrameHeight >> 2;
	int uv_in_width = nFrameWidth >> 2;
	int uv_out_height = nFrameHeight >> 1;
	int uv_out_width = nFrameWidth >> 1;
    unsigned char *uv_input = (unsigned char*)uv_buff + ((MAX(nFrameHeight, nPadHeight) * MAX(nFrameWidth, nPadWidth)) >> 2);
	unsigned char *uv_output = (unsigned char*)dsp_buff + (MAX(nFrameHeight, nPadHeight) * MAX(nFrameWidth, nPadWidth));

#ifndef RESIZE_MULTI_THREADS
	if(uv_in_width & (uv_in_width - 1))
	{
		int in_width_pad = ALIGN2POWN(uv_in_width);
		int out_width_pad = in_width_pad << 1;
		unsigned char *uv_buff_pad = (unsigned char*)uv_buff;
		unsigned char *out_buff_pad = (unsigned char*)uv_buff + in_width_pad * uv_in_height * CHANNELS;

		for(int i = 0; i < uv_in_height; i++)
		{
			vmemcpy_asm(uv_buff_pad + i * in_width_pad * CHANNELS, uv_input + i * uv_in_width * CHANNELS, uv_in_width * CHANNELS);
			uv_buff_pad[i * in_width_pad * CHANNELS + uv_in_width * CHANNELS + 0] = uv_input[i * uv_in_width * CHANNELS + uv_in_width * CHANNELS - 2];
			uv_buff_pad[i * in_width_pad * CHANNELS + uv_in_width * CHANNELS + 1] = uv_input[i * uv_in_width * CHANNELS + uv_in_width * CHANNELS - 1];
		}

		resize_bilinear(uv_buff_pad, out_buff_pad, uv_in_height, in_width_pad, CHANNELS, uv_out_height, out_width_pad);

		for(int i = 0; i < uv_out_height; i++)
		{
			vmemcpy_asm(uv_output + i * uv_out_width * CHANNELS, out_buff_pad + i * out_width_pad * CHANNELS, uv_out_width * CHANNELS);
		}
	}
	else
	{
		resize_bilinear(uv_input, uv_output, uv_in_height, uv_in_width, CHANNELS, uv_out_height, uv_out_width);
	}
#else
	int numWorkers = MIN((qurt_hvx_get_units() >> 8) & 0xFF, MAXTHREADS);

    worker_pool_job_t   job;
    worker_synctoken_t    token;
    worker_pool_context_t context = NULL;

	while((uv_in_height % numWorkers))
	{
		numWorkers--;
		if(numWorkers == 0)
		{
			numWorkers = 1;
			break;
		}
	}
	LOGI("MultiThreds:hvxInfo.numThreads = %d, numWorkers = %d", hvxInfo.numThreads, numWorkers);
	
	job.fptr = resize_callback;
	resize_callback_t dptr;

	dptr.token = &token;
	dptr.jobCount = 0;
	dptr.h_in_block = uv_in_height / numWorkers; 
	dptr.h_out_block = uv_out_height / numWorkers;
	dptr.uv_buff = (unsigned char *)uv_buff;
	dptr.uv_input = uv_input;
	dptr.uv_output = uv_output;
	dptr.uv_in_height = uv_in_height;
	dptr.uv_in_width = uv_in_width;
	dptr.uv_out_width = uv_out_width;

	job.dptr = (void *)&dptr;
	
	if(uv_in_width & (uv_in_width - 1))
	{
		int in_width_pad = ALIGN2POWN(uv_in_width);
		unsigned char *uv_buff_pad = (unsigned char*)uv_buff;
		for(int i = 0; i < uv_in_height; i++)
		{
			vmemcpy_asm(uv_buff_pad + i * in_width_pad * CHANNELS, uv_input + i * uv_in_width * CHANNELS, uv_in_width * CHANNELS);
			uv_buff_pad[i * in_width_pad * CHANNELS + uv_in_width * CHANNELS + 0] = uv_input[i * uv_in_width * CHANNELS + uv_in_width * CHANNELS - 2];
			uv_buff_pad[i * in_width_pad * CHANNELS + uv_in_width * CHANNELS + 1] = uv_input[i * uv_in_width * CHANNELS + uv_in_width * CHANNELS - 1];
		}
	}
	
	//(void)worker_pool_init(&context);
    worker_pool_synctoken_init(&token, numWorkers);

	for(int i = 0; i < numWorkers; i++)
	{
	   (void)worker_pool_submit(context, job);
	}
	worker_pool_synctoken_wait(&token);

	//worker_pool_deinit(&context);
#endif
#ifdef PROFILING_ON
	uint64 endCycles = HAP_perf_get_pcycles();
	uint64 endTime = HAP_perf_get_time_us();
	FARF(ALWAYS, "[RESIZEBILINEAR]profiling: %d PCycles, %d us. Observed clock rate %d MHz",
		(int)(endCycles - startCycles), (int)(endTime - startTime), (int)((endCycles - startCycles) / (endTime - startTime)));
#endif
	
	nRet = aiboostubwc_writeBufferDequant(dsp_buff, dsp_size, ubwc_buff, ubwc_size, nFrameHeightBak, nFrameWidthBak, scale, zero_point);

	return nRet;
}


int aiboostubwc_transBuffer(const unsigned char *src_buff, int src_size, unsigned char *dst_buff, int dst_size, int32 nFrameHeight, int32 nFrameWidth, int32 isUbwc2nv12)
{
#ifdef PROFILING_ON
	uint64 startTime = HAP_perf_get_time_us();
	uint64 startCycles = HAP_perf_get_pcycles();
#endif
	t_DmaWrapper_DmaEngineHandle handle_rd, handle_wr;
	t_EDma_WaitType eWaitType = eDmaWaitType_Polling;
	t_eDmaFmt eFmtLuma = eDmaFmt_NV12_Y;
    t_eDmaFmt eFmtChroma = eDmaFmt_NV12_UV;

	t_eDmaFmt efmtLumaChroma[2] = {eFmtLuma, eFmtChroma};

    int nRet = UBWC_OK;
	t_StDmaWrapper_Roi stRoi;
	t_StDmaWrapper_RoiAlignInfo stAlignInfo;
	int lumaStride, chromaStride;
	qurt_addr_t tcm_buf_vaddr, tcm_desc_vaddr;
	unsigned long long tcm_buf_paddr, tcm_desc_paddr;
	qurt_size_t tcm_buf_size, ChromaOffset, tcm_desc_size, region_tcm_size;
	t_StDmaWrapper_PrepareParm stWrapPrepParm;
	t_StDmaWrapper_WorkDescrip staWorkDesc[2];
    t_StDmaWrapper_FrameProp walkRoi;
    t_StDmaWrapper_FrameProp frameProp;
	int nIdx = 0;
    int nRow, nCol;
    int nRowIdx = 0, nColIdx = 0;
	t_StDmaWrapper_UpdateParm stWrapUpdateParm[2];
	
	if((nFrameWidth & 127) != 0 || (nFrameHeight & 31) != 0)
	{	
		LOGE("input nFrameWidth(%d) or nFrameHeight(%d) wrong", nFrameWidth, nFrameHeight);
		nRet = UBWC_ERR_INPUTSIZE;
		return nRet;
	}
	
	stRoi.u16W = 256;
    stRoi.u16H = 32;
	nRet = nDmaWrapper_GetRecommendedRoi(eDmaFmt_NV12, TRUE, &stRoi);
	if(nRet)
	{
		LOGE("nDmaWrapper_GetRecommendedRoi error.");
	    return nRet;
	}

	nRow = ROUNDDIV(nFrameHeight, stRoi.u16H);
	nCol = ROUNDDIV(nFrameWidth,  stRoi.u16W);

	LOGI("stRoi.u16W = %d, stRoi.u16H = %d, nRow = %d, nCol = %d", stRoi.u16W, stRoi.u16H, nRow, nCol);

	stAlignInfo.u16W = stRoi.u16W;
    stAlignInfo.u16H = stRoi.u16H;

	lumaStride = nDmaWrapper_GetRecommendedIntermBufStride(eFmtLuma, &stAlignInfo, TRUE);
	chromaStride = nDmaWrapper_GetRecommendedIntermBufStride(eFmtChroma, &stAlignInfo, TRUE);

	LOGI("lumaStride = %d, chromaStride = %d", lumaStride, chromaStride);

	tcm_buf_size = nDmaWrapper_GetRecommendedIntermBufSize(eFmtLuma, PAD16BIT, &stAlignInfo, TRUE, lumaStride);
	ChromaOffset = tcm_buf_size;
	tcm_buf_size += nDmaWrapper_GetRecommendedIntermBufSize(eFmtChroma, PAD16BIT, &stAlignInfo, TRUE, chromaStride);
	region_tcm_size = ALIGNUP(tcm_buf_size * 2, 0x1000);
	tcm_desc_size = ALIGNUP(nDmaWrapper_GetDescbuffsize(efmtLumaChroma, 2) * 2 * 2, 0x1000);


	LOGI("tcm_buf_size = %d, region_tcm_size = %d, tcm_desc_size = %d, ChromaOffset = %d",
		  tcm_buf_size, region_tcm_size, tcm_desc_size, ChromaOffset);
		
	tcm_desc_vaddr = (addr_t)HAP_cache_lock((unsigned int)tcm_desc_size, &tcm_desc_paddr);
	tcm_buf_vaddr = (addr_t)HAP_cache_lock((unsigned int)region_tcm_size, &tcm_buf_paddr);
	if(!tcm_desc_vaddr || !tcm_buf_vaddr)
	{
		LOGE("HAP_cache_lock error.");
		nRet = UBWC_ERR_ALLOCTCM;
		return nRet;
	}

	LOGI("tcm_desc_vaddr = 0x%x, tcm_buf_vaddr = 0x%x, tcm_desc_paddr = 0x%x, tcm_buf_paddr = 0x%x", 
			tcm_desc_vaddr, tcm_buf_vaddr, qurt_lookup_physaddr(tcm_desc_vaddr), qurt_lookup_physaddr(tcm_buf_vaddr));

	handle_rd = hDmaWrapper_AllocDmaSpecifyWaitType(eWaitType);
	if(!handle_rd) 
	{
		nRet = UBWC_ERR_ALLOCDMA;
		LOGE("hDmaWrapper_AllocDmaSpecifyWaitType error.");
		goto error;
	}

	handle_wr = hDmaWrapper_AllocDmaSpecifyWaitType(eWaitType);
	if(!handle_wr) 
	{
		nRet = UBWC_ERR_ALLOCDMA;
		LOGE("hDmaWrapper_AllocDmaSpecifyWaitType error.");
		goto error;
	}

#if 0
	nRet = nDmaWrapper_PowerVoting(PW_SVS2);
	if(nRet)
	{
		LOGE("nDmaWrapper_PowerVoting error.");
		goto error;
	}
#endif

	stWrapPrepParm.u32NumOfWorkDesc = 2;
    stWrapPrepParm.staWorkDesc = staWorkDesc;
	for (int i = 0; i < 1; i++)
    {
        for (int j = 0; j < 2; j++)
        {
            int descIndex = i * 2 + j;
            frameProp.aAddr = (addr_t)qurt_lookup_physaddr((qurt_addr_t)src_buff);
            frameProp.u16W = nFrameWidth;
            frameProp.u16H = nFrameHeight;
            frameProp.u16Stride = nFrameWidth;
            walkRoi.aAddr = 0;
            walkRoi.u16W = stRoi.u16W;
            walkRoi.u16H = stRoi.u16H;
            walkRoi.u16Stride = (j) ? chromaStride : lumaStride;
            nRet = nDmaWrapper_WorkDescrip_populate(&staWorkDesc[descIndex],
                                                    efmtLumaChroma[j],
                                                    eDmaWrapper_DdrToL2,
                                                    &walkRoi, &frameProp,
                                                    isUbwc2nv12 ? TRUE : FALSE,
                                                    PAD16BIT,
                                                    NULL);
        }
    }
    stWrapPrepParm.u32DescBufSize = nDmaWrapper_GetDescbuffsize(efmtLumaChroma, 2);
	stWrapPrepParm.pPingDescBuf = (void *)tcm_desc_vaddr;
	stWrapPrepParm.pPongDescBuf = (void *)(tcm_desc_vaddr + stWrapPrepParm.u32DescBufSize);
	LOGI("stWrapPrepParm.u32DescBufSize = %d", stWrapPrepParm.u32DescBufSize);
	nRet |= nDmaWrapper_Prepare(handle_rd, &stWrapPrepParm);
	if(nRet)
	{
		nRet = UBWC_ERR_DMAPREPARE;
		LOGE("nDmaWrapper_Prepare error.");
		goto error;
	}

    stWrapPrepParm.u32NumOfWorkDesc = 2;
    stWrapPrepParm.staWorkDesc = staWorkDesc;
	for (int i = 0; i < 1; i++)
    {
        for (int j = 0; j < 2; j++)
        {
            int descIndex = i * 2 + j;
            frameProp.aAddr = (addr_t)qurt_lookup_physaddr((qurt_addr_t)dst_buff);
            frameProp.u16W = nFrameWidth;
            frameProp.u16H = nFrameHeight;
            frameProp.u16Stride = nFrameWidth;
            walkRoi.aAddr = 0;
            walkRoi.u16W = stRoi.u16W;
            walkRoi.u16H = stRoi.u16H;
            walkRoi.u16Stride = (j) ? chromaStride : lumaStride;
            nRet = nDmaWrapper_WorkDescrip_populate(&staWorkDesc[descIndex],
                                                    efmtLumaChroma[j],
                                                    eDmaWrapper_L2ToDdr,
                                                    &walkRoi, &frameProp,
                                                    isUbwc2nv12 ? FALSE : TRUE,
                                                    PAD16BIT,
                                                    NULL);
        }
    }
    stWrapPrepParm.u32DescBufSize = nDmaWrapper_GetDescbuffsize(efmtLumaChroma, 2);
	stWrapPrepParm.pPingDescBuf = (void *)(tcm_desc_vaddr + stWrapPrepParm.u32DescBufSize * 2);
	stWrapPrepParm.pPongDescBuf = (void *)(tcm_desc_vaddr + stWrapPrepParm.u32DescBufSize * 3);
	LOGI("stWrapPrepParm.u32DescBufSize = %d", stWrapPrepParm.u32DescBufSize);
	nRet |= nDmaWrapper_Prepare(handle_wr, &stWrapPrepParm);
	if(nRet)
	{
		nRet = UBWC_ERR_DMAPREPARE;
		LOGE("nDmaWrapper_Prepare error.");
		goto error;
	}

	iRoiInfo_t iRoi;
	iRoi.nFrameHeight = nFrameHeight;
	iRoi.nFrameWidth = nFrameWidth;
	iRoi.nRoiWalkHeight = stRoi.u16H;
	iRoi.nRoiWalkWidth = stRoi.u16W;
	iRoi.tcm_buf_vaddr = tcm_buf_vaddr;
	iRoi.tcm_buf_size = tcm_buf_size;
	iRoi.ChromaOffset = ChromaOffset;
	
	updateRoiInfo(nRowIdx, nRow, nColIdx, nCol, nIdx, &iRoi, &stWrapUpdateParm[0]);
	nRet = nDmaWrapper_Update(handle_rd, &stWrapUpdateParm[0]);
	if(nRet)
	{
		nRet = UBWC_ERR_DMAUPDATE;
		LOGE("nDmaWrapper_Update error.");
		goto error;
	}

	nRet = nDmaWrapper_Update(handle_wr, &stWrapUpdateParm[0]);
	if(nRet)
	{
		nRet = UBWC_ERR_DMAUPDATE;
		LOGE("nDmaWrapper_Update error.");
		goto error;
	}

	nRet = nDmaWrapper_Move(handle_rd);
	if(nRet)
	{
		nRet = UBWC_ERR_DMAMOVE;
		LOGE("nDmaWrapper_Move error.");
		goto error;
	}
    nIdx = (nIdx + 1) % 2;

	bool preventFirstWriteWait = TRUE;

    for (nRowIdx = 0; nRowIdx < nRow; nRowIdx++)
    {
        for (nColIdx = (nRowIdx)? 0:1; nColIdx < nCol; nColIdx++)
        {
			updateRoiInfo(nRowIdx, nRow, nColIdx, nCol, nIdx, &iRoi, &stWrapUpdateParm[0]);
			if (!preventFirstWriteWait)
            {
               nRet = nDmaWrapper_Wait(handle_wr);
            }
            else
            {
                preventFirstWriteWait = FALSE;
            }
			
            nRet |= nDmaWrapper_Update(handle_rd, &stWrapUpdateParm[0]);
            nRet |= nDmaWrapper_Wait(handle_rd);
            nRet |= nDmaWrapper_Move(handle_rd);
			nRet |= nDmaWrapper_Move(handle_wr);
            nRet |= nDmaWrapper_Update(handle_wr, &stWrapUpdateParm[0]);
			if(nRet)
			{
	
				nRet = UBWC_ERR_DMAUPDATE | UBWC_ERR_DMAMOVE | UBWC_ERR_DMAWAIT;
				LOGE("nDmaWrapper_Move or nDmaWrapper_Update or nDmaWrapper_Wait error.");
				goto error;
			}		
            nIdx = (nIdx + 1) % 2;
        }
    }

    if (!preventFirstWriteWait)
    {
        nRet =nDmaWrapper_Wait(handle_wr);
    }
	
    nRet |=nDmaWrapper_Wait(handle_rd);
    nRet |=nDmaWrapper_Move(handle_wr);
    nRet |=nDmaWrapper_Wait(handle_wr);
	
	if(nRet)
	{
		nRet = UBWC_ERR_DMAMOVE | UBWC_ERR_DMAWAIT;
		LOGE("nDmaWrapper_Move or nDmaWrapper_Wait error.");
		goto error;
	}	

	nRet = nDmaWrapper_FinishFrame(handle_rd);
	nRet = nDmaWrapper_FinishFrame(handle_wr);
	if(nRet)
	{
		nRet = UBWC_ERR_DMAFINISH;
		LOGE("nDmaWrapper_FinishFrame error.");
		goto error;
	}

	nRet = nDmaWrapper_FreeDma(handle_rd);
	nRet = nDmaWrapper_FreeDma(handle_wr);
	if(nRet)
	{
		nRet = UBWC_ERR_DMAFREE;
		LOGE("nDmaWrapper_FreeDma error.");
		goto error;
	}
error:	
	HAP_cache_unlock((void*)tcm_buf_vaddr);
	HAP_cache_unlock((void*)tcm_desc_vaddr);
#ifdef PROFILING_ON
	uint64 endCycles = HAP_perf_get_pcycles();
	uint64 endTime = HAP_perf_get_time_us();
	FARF(ALWAYS, "[UBWCTRANSFER]profiling: %d PCycles, %d us. Observed clock rate %d MHz",
		(int)(endCycles - startCycles), (int)(endTime - startTime), (int)((endCycles - startCycles) / (endTime - startTime)));
#endif
	return nRet;
}


#if 0
static int ubwc_Dequant(const unsigned char *dsp_buff, int dsp_size, 
						int32 nFrameHeight, int32 nFrameWidth,
						float scale, int32 zero_point)
{
#ifdef PROFILING_ON
	uint64 startTime = HAP_perf_get_time_us();
	uint64 startCycles = HAP_perf_get_pcycles();
#endif
	int nRet = UBWC_OK;
	unsigned char *dequant_buff = (unsigned char *)dsp_buff;
	if((int)dequant_buff & 127)
	{
		nRet = UBWC_ERR_ADDRALIGNED;
		LOGE("Dequant buffer is not 128 bytes aligned, addr = %p", dequant_buff);
        return nRet;
	}
	
#ifndef DEQUANT_MULTI_THREADS
	const int size = nFrameHeight * nFrameWidth;
	const int vsize = size >> 7;
	
	struct qf16 qf_scale = f2q(scale * 255);
	HVX_Vector *in_vec = (HVX_Vector *)dsp_buff;
	HVX_Vector *out_vec = (HVX_Vector *)dequant_buff;

	if(zero_point == 0)
	{
		for(int i = 0; i < vsize; i++)
		{
			HVX_Vector a_mant0, a_mant1, a_exp0, a_exp1;
			HVX_Vector b_mant0, b_mant1, b_exp0, b_exp1;
			
			vs2q(in_vec, &a_mant0, &a_exp0, &a_mant1, &a_exp1);

			vsmpyqf(a_mant0, a_exp0, qf_scale.m, qf_scale.e, &b_mant0, &b_exp0);
			vsmpyqf(a_mant1, a_exp1, qf_scale.m, qf_scale.e, &b_mant1, &b_exp1);

			vq2s(out_vec, b_mant0, b_exp0, b_mant1, b_exp1);		
			
			in_vec++;
			out_vec++;
		}
	}
	else
	{
		struct qf16 qf_zero_point;
		i2q(zero_point, &qf_zero_point);
		HVX_Vector zero_point_mant = Q6_Vh_vsplat_R(qf_zero_point.m);
		HVX_Vector zero_point_exp = Q6_Vh_vsplat_R(qf_zero_point.e);
		
		for(int i = 0; i < vsize; i++)
		{
			HVX_Vector a_mant0, a_mant1, a_exp0, a_exp1;
			HVX_Vector b_mant0, b_mant1, b_exp0, b_exp1;

			vs2q(in_vec, &a_mant0, &a_exp0, &a_mant1, &a_exp1);

			vsubqf(a_mant0, a_exp0, zero_point_mant, zero_point_exp, &b_mant0, &b_exp0);
			vsubqf(a_mant1, a_exp1, zero_point_mant, zero_point_exp, &b_mant1, &b_exp1);

			vsmpyqf(b_mant0, b_exp0, qf_scale.m, qf_scale.e, &a_mant0, &a_exp0);
			vsmpyqf(b_mant1, b_exp1, qf_scale.m, qf_scale.e, &a_mant1, &a_exp1);

			vq2s(out_vec, a_mant0, a_exp0, a_mant1, a_exp1);

			in_vec++;
			out_vec++;
		}
	}
	for(int i = (vsize << 7); i < size; i++)
	{
		int tmp =  ((int)dsp_buff[i] - zero_point) * scale * 255;
		dequant_buff[i] = (unsigned char)CLIP_INTENAL(tmp, 0, 255);		
	}
#else
	dspCV_ConcurrencyAttribute attrib[1] = 
    {
        {COMPUTE_RECOMMENDATION, 0},
    };
    dspCV_concurrency_query(attrib, 1);
    if(COMPUTE_RECOMMENDATION_NOT_OK == attrib[0].value)
    {
    	nRet = UBWC_ERR_DSPCVQUERY;
		LOGE("It is safe to run a compute function now");
        return nRet;
    }

	dspCV_hvx_config_t hvxInfo = {0};
	hvxInfo.mode = DSPCV_HVX_MODE_128B;
	dspCV_hvx_prepare_mt_job(&hvxInfo);
	if(hvxInfo.numUnits <= 0)
    {
        dspCV_hvx_cleanup_mt_job(&hvxInfo);
    	nRet = UBWC_ERR_DSPCVNUMUNITS;
		LOGE("hvxInfo.numUnits is %d", hvxInfo.numUnits);
        return nRet;
    }

	int numWorkers = MIN(hvxInfo.numThreads, MAXTHREADS);
	int buff_size = nFrameHeight * nFrameWidth;
	while((buff_size % numWorkers) /*|| ((buff_size / numWorkers) & 127)*/)
	{
		numWorkers--;
		if(numWorkers == 0)
		{
			numWorkers = 1;
			break;
		}
	}
	LOGI("MultiThreds:hvxInfo.numThreads = %d, numWorkers = %d", hvxInfo.numThreads, numWorkers);

	dspCV_worker_job_t job;
    dspCV_synctoken_t token;
	
    dspCV_worker_pool_synctoken_init(&token, numWorkers);

	job.fptr = dequant_callback;
	dequant_callback_t dptr;

	dptr.token = &token;
	dptr.jobCount = 0;
	dptr.dsp_buff = dsp_buff;
	dptr.dequant_buff = dequant_buff;
	dptr.buff_size = buff_size;
	dptr.sizePerJob = buff_size / numWorkers;
	dptr.scale = scale;
	dptr.zero_point = zero_point;

	job.dptr = (void *)&dptr;
	
    for(int i = 0; i < numWorkers; i++)
    {
       (void)dspCV_worker_pool_submit(job);
    }
    dspCV_worker_pool_synctoken_wait(&token);
    
    dspCV_hvx_cleanup_mt_job(&hvxInfo);
#endif

#ifdef PROFILING_ON
	uint64 endCycles = HAP_perf_get_pcycles();
	uint64 endTime = HAP_perf_get_time_us();
	FARF(ALWAYS, "[DEQUANTIZE]profiling: %d PCycles, %d us. Observed clock rate %d MHz",
		(int)(endCycles - startCycles), (int)(endTime - startTime), (int)((endCycles - startCycles) / (endTime - startTime)));
#endif

	return nRet;
}
#endif

#ifdef RESIZE_MULTI_THREADS
static void resize_callback2(void* data)
{
    #define CHANNELS (2)
	resize_callback_t *dptr = (resize_callback_t*)data;
	int uv_in_width = dptr->uv_in_width;
	int uv_in_height = dptr->uv_in_height;
	int uv_out_width = dptr->uv_out_width;
	unsigned char *uv_input = dptr->uv_input;
	unsigned char *uv_output = dptr->uv_output;
	unsigned char *pad_buff = dptr->uv_buff;
	unsigned int h_in_block = dptr->h_in_block;
	unsigned int h_out_block = dptr->h_out_block;
	
	unsigned int jobCount = worker_pool_atomic_inc_return(&(dptr->jobCount)) - 1;

	if(jobCount * h_in_block >= uv_in_height)
    {
        return;
    }

	uv_input += jobCount * h_in_block * uv_in_width * CHANNELS;
	uv_output += jobCount * h_out_block * uv_out_width * CHANNELS;

	if(uv_in_width & (uv_in_width - 1))
	{
		int in_width_pad = ALIGN2POWN(uv_in_width);
		int out_width_pad = in_width_pad << 1;
		unsigned char *out_buff_pad = pad_buff;
		unsigned char *uv_buff_pad = pad_buff + out_width_pad * (uv_in_height << 1) * CHANNELS;
		uv_buff_pad += jobCount * h_in_block * in_width_pad * CHANNELS;
		out_buff_pad += jobCount * h_out_block * out_width_pad * CHANNELS;

		for(int i = 0; i < h_in_block; i++)
		{
			vmemcpy_asm(uv_buff_pad + i * in_width_pad * CHANNELS, uv_input + i * uv_in_width * CHANNELS, uv_in_width * CHANNELS);
			uv_buff_pad[i * in_width_pad * CHANNELS + uv_in_width * CHANNELS + 0] = uv_input[i * uv_in_width * CHANNELS + uv_in_width * CHANNELS - 2];
			uv_buff_pad[i * in_width_pad * CHANNELS + uv_in_width * CHANNELS + 1] = uv_input[i * uv_in_width * CHANNELS + uv_in_width * CHANNELS - 1];
		}

		resize_bilinear(uv_buff_pad, out_buff_pad, h_in_block, in_width_pad, CHANNELS, h_out_block, out_width_pad);

		for(int i = 0; i < h_out_block; i++)
		{
			vmemcpy_asm(uv_output + i * uv_out_width * CHANNELS, out_buff_pad + i * out_width_pad * CHANNELS, uv_out_width * CHANNELS);
		}
	}
	else
	{
		resize_bilinear(uv_input, uv_output, h_in_block, uv_in_width, CHANNELS, h_out_block, uv_out_width);
	}

    worker_pool_synctoken_jobdone(dptr->token);
}
#endif

#if 0
int aiboostubwc_dequantResize(unsigned char *dsp_buff, int dsp_size, 
							    unsigned char *pad_buff, int pad_size,
								int32 nFrameHeight, int32 nFrameWidth,
								float scale, int32 zero_point)
{
#ifdef PROFILING_ON
	uint64 startTime = HAP_perf_get_time_us();
	uint64 startCycles = HAP_perf_get_pcycles();
#endif
	int nRet = UBWC_OK;

	#define CHANNELS (2)
	int uv_in_height = nFrameHeight >> 2;
	int uv_in_width = nFrameWidth >> 2;
	int uv_out_height = nFrameHeight >> 1;
	int uv_out_width = nFrameWidth >> 1;
	unsigned char *uv_output = dsp_buff + (nFrameHeight * nFrameWidth);
	unsigned char *uv_input = uv_output + ((nFrameHeight * nFrameWidth) >> 1);

#ifndef RESIZE_MULTI_THREADS
	if(uv_in_width & (uv_in_width - 1))
	{
		int in_width_pad = ALIGN2POWN(uv_in_width);
		int out_width_pad = in_width_pad << 1;
		unsigned char *out_buff_pad = pad_buff;
		unsigned char *uv_buff_pad = pad_buff + out_width_pad * uv_out_height * CHANNELS;

		for(int i = 0; i < uv_in_height; i++)
		{
			vmemcpy_asm(uv_buff_pad + i * in_width_pad * CHANNELS, uv_input + i * uv_in_width * CHANNELS, uv_in_width * CHANNELS);
			uv_buff_pad[i * in_width_pad * CHANNELS + uv_in_width * CHANNELS + 0] = uv_input[i * uv_in_width * CHANNELS + uv_in_width * CHANNELS - 2];
			uv_buff_pad[i * in_width_pad * CHANNELS + uv_in_width * CHANNELS + 1] = uv_input[i * uv_in_width * CHANNELS + uv_in_width * CHANNELS - 1];
		}

		resize_bilinear(uv_buff_pad, out_buff_pad, uv_in_height, in_width_pad, CHANNELS, uv_out_height, out_width_pad);

		for(int i = 0; i < uv_out_height; i++)
		{
			vmemcpy_asm(uv_output + i * uv_out_width * CHANNELS, out_buff_pad + i * out_width_pad * CHANNELS, uv_out_width * CHANNELS);
		}
	}
	else
	{
		resize_bilinear(uv_input, uv_output, uv_in_height, uv_in_width, CHANNELS, uv_out_height, uv_out_width);
	}
#else
	dspCV_ConcurrencyAttribute attrib[1] = 
	{
		{COMPUTE_RECOMMENDATION, 0},
	};
	dspCV_concurrency_query(attrib, 1);
	if(COMPUTE_RECOMMENDATION_NOT_OK == attrib[0].value)
	{
		nRet = UBWC_ERR_DSPCVQUERY;
		LOGE("It is safe to run a compute function now");
		return nRet;
	}

	dspCV_hvx_config_t hvxInfo = {0};
	hvxInfo.mode = DSPCV_HVX_MODE_128B;
	dspCV_hvx_prepare_mt_job(&hvxInfo);
	if(hvxInfo.numUnits <= 0)
	{
		dspCV_hvx_cleanup_mt_job(&hvxInfo);
		nRet = UBWC_ERR_DSPCVNUMUNITS;
		LOGE("hvxInfo.numUnits is %d", hvxInfo.numUnits);
		return nRet;
	}

	int numWorkers = MIN(hvxInfo.numThreads, MAXTHREADS);
	while((uv_in_height % numWorkers))
	{
		numWorkers--;
		if(numWorkers == 0)
		{
			numWorkers = 1;
			break;
		}
	}
	LOGI("MultiThreds:hvxInfo.numThreads = %d, numWorkers = %d", hvxInfo.numThreads, numWorkers);

	dspCV_worker_job_t job;
	dspCV_synctoken_t token;

	dspCV_worker_pool_synctoken_init(&token, numWorkers);

	job.fptr = resize_callback2;
	resize_callback_t dptr;

	dptr.token = &token;
	dptr.jobCount = 0;
	dptr.h_in_block = uv_in_height / numWorkers; 
	dptr.h_out_block = uv_out_height / numWorkers;
	dptr.uv_buff = pad_buff;
	dptr.uv_input = uv_input;
	dptr.uv_output = uv_output;
	dptr.uv_in_height = uv_in_height;
	dptr.uv_in_width = uv_in_width;
	dptr.uv_out_width = uv_out_width;

	job.dptr = (void *)&dptr;

	for(int i = 0; i < numWorkers; i++)
	{
	   (void)dspCV_worker_pool_submit(job);
	}
	dspCV_worker_pool_synctoken_wait(&token);

	dspCV_hvx_cleanup_mt_job(&hvxInfo);
#endif
#ifdef PROFILING_ON
	uint64 endCycles = HAP_perf_get_pcycles();
	uint64 endTime = HAP_perf_get_time_us();
	FARF(ALWAYS, "[RESIZEBILINEAR]profiling: %d PCycles, %d us. Observed clock rate %d MHz",
		(int)(endCycles - startCycles), (int)(endTime - startTime), (int)((endCycles - startCycles) / (endTime - startTime)));
#endif

	nRet = ubwc_Dequant(dsp_buff, dsp_size, nFrameHeight, nFrameWidth, scale, zero_point);

	return nRet;
}

int depth_to_space_d16b4(const unsigned char *input_data, unsigned char *output_data, int height, int width);

int aiboostubwc_depthToSpaceD16B4(const unsigned char *input_data, int input_size, 
							    unsigned char *output_data, int output_size,
								int32 nFrameHeight, int32 nFrameWidth)
{
#ifdef PROFILING_ON
	uint64 startTime = HAP_perf_get_time_us();
	uint64 startCycles = HAP_perf_get_pcycles();
#endif
	int nRet = UBWC_OK;
#if 0
	const int depth = 16;
	const int block_size = 4;

	for(int h = 0; h < nFrameHeight; h++)
	{
		const unsigned char *input_ptr = input_data + h * nFrameWidth * depth;
		for(int b = 0; b < block_size; b++)
		{
			const unsigned char *src = input_ptr;
			for(int w = 0; w < nFrameWidth; w++)
			{
				memcpy(output_data, src, block_size);
				output_data += block_size;
				src += depth;
			}
			input_ptr += block_size;
		}
	}
#else
	nRet = depth_to_space_d16b4(input_data, output_data, nFrameHeight, nFrameWidth);
#endif

#ifdef PROFILING_ON
	uint64 endCycles = HAP_perf_get_pcycles();
	uint64 endTime = HAP_perf_get_time_us();
	FARF(ALWAYS, "[DEPTHTOSPACE]profiling: %d PCycles, %d us. Observed clock rate %d MHz",
		(int)(endCycles - startCycles), (int)(endTime - startTime), (int)((endCycles - startCycles) / (endTime - startTime)));
#endif

	return nRet;

}	
#else
int aiboostubwc_dequantResize(unsigned char *dsp_buff, int dsp_size, 
							    unsigned char *pad_buff, int pad_size,
								int32 nFrameHeight, int32 nFrameWidth,
								float scale, int32 zero_point)
{
#ifdef PROFILING_ON
	uint64 startTime = HAP_perf_get_time_us();
	uint64 startCycles = HAP_perf_get_pcycles();
#endif
	int nRet = UBWC_OK;

	int nPadHeight = nFrameHeight >> 16;
	int nPadWidth = nFrameWidth >> 16;
	nFrameHeight &= 0xffff;
	nFrameWidth &= 0xffff;

	#define CHANNELS (2)
	int uv_in_height = nFrameHeight >> 2;
	int uv_in_width = nFrameWidth >> 2;
	int uv_out_height = nFrameHeight >> 1;
	int uv_out_width = nFrameWidth >> 1;
	unsigned char *uv_output = dsp_buff + (nFrameHeight * nFrameWidth);
	unsigned char *uv_input = dsp_buff + (MAX(nPadHeight, nFrameHeight) * MAX(nPadWidth, nFrameWidth)) + ((nFrameHeight * nFrameWidth) >> 1);
	
	nRet = depth_to_space_d16b4_dequant(dsp_buff, pad_buff, nFrameHeight / 4, MAX(nPadWidth, nFrameWidth) / 4, scale, zero_point);

	if(nRet)
	{
		return nRet;
	}

	for(int i = 0; i < nFrameHeight; i++)
	{
		vmemcpy_asm(dsp_buff + i * nFrameWidth, pad_buff + i * MAX(nPadWidth, nFrameWidth), nFrameWidth);
	}

#ifndef RESIZE_MULTI_THREADS
	if(uv_in_width & (uv_in_width - 1))
	{
		int in_width_pad = ALIGN2POWN(uv_in_width);
		int out_width_pad = in_width_pad << 1;
		unsigned char *out_buff_pad = pad_buff;
		unsigned char *uv_buff_pad = pad_buff + out_width_pad * uv_out_height * CHANNELS;

		for(int i = 0; i < uv_in_height; i++)
		{
			vmemcpy_asm(uv_buff_pad + i * in_width_pad * CHANNELS, uv_input + i * uv_in_width * CHANNELS, uv_in_width * CHANNELS);
			uv_buff_pad[i * in_width_pad * CHANNELS + uv_in_width * CHANNELS + 0] = uv_input[i * uv_in_width * CHANNELS + uv_in_width * CHANNELS - 2];
			uv_buff_pad[i * in_width_pad * CHANNELS + uv_in_width * CHANNELS + 1] = uv_input[i * uv_in_width * CHANNELS + uv_in_width * CHANNELS - 1];
		}

		resize_bilinear(uv_buff_pad, out_buff_pad, uv_in_height, in_width_pad, CHANNELS, uv_out_height, out_width_pad);

		for(int i = 0; i < uv_out_height; i++)
		{
			vmemcpy_asm(uv_output + i * uv_out_width * CHANNELS, out_buff_pad + i * out_width_pad * CHANNELS, uv_out_width * CHANNELS);
		}
	}
	else
	{
		resize_bilinear(uv_input, uv_output, uv_in_height, uv_in_width, CHANNELS, uv_out_height, uv_out_width);
	}
#else
	int numWorkers = MIN((qurt_hvx_get_units() >> 8) & 0xFF, MAXTHREADS);

    worker_pool_job_t   job;
    worker_synctoken_t    token;
    worker_pool_context_t context = NULL;
	
	while((uv_in_height % numWorkers))
	{
		numWorkers--;
		if(numWorkers == 0)
		{
			numWorkers = 1;
			break;
		}
	}
	LOGI("MultiThreds:hvxInfo.numThreads = %d, numWorkers = %d", hvxInfo.numThreads, numWorkers);

	job.fptr = resize_callback2;
	resize_callback_t dptr;

	dptr.token = &token;
	dptr.jobCount = 0;
	dptr.h_in_block = uv_in_height / numWorkers; 
	dptr.h_out_block = uv_out_height / numWorkers;
	dptr.uv_buff = pad_buff;
	dptr.uv_input = uv_input;
	dptr.uv_output = uv_output;
	dptr.uv_in_height = uv_in_height;
	dptr.uv_in_width = uv_in_width;
	dptr.uv_out_width = uv_out_width;

	job.dptr = (void *)&dptr;

	//(void)worker_pool_init(&context);
    worker_pool_synctoken_init(&token, numWorkers);

	for(int i = 0; i < numWorkers; i++)
	{
	   (void)worker_pool_submit(context, job);
	}
	worker_pool_synctoken_wait(&token);

	//worker_pool_deinit(&context);
#endif
#ifdef PROFILING_ON
	uint64 endCycles = HAP_perf_get_pcycles();
	uint64 endTime = HAP_perf_get_time_us();
	FARF(ALWAYS, "[RESIZEBILINEAR]profiling: %d PCycles, %d us. Observed clock rate %d MHz",
		(int)(endCycles - startCycles), (int)(endTime - startTime), (int)((endCycles - startCycles) / (endTime - startTime)));
#endif

	return nRet;
}

int aiboostubwc_depthToSpaceD16B4(const unsigned char *input_data, int input_size, 
							    unsigned char *output_data, int output_size,
								int32 nFrameHeight, int32 nFrameWidth)
{
#ifdef PROFILING_ON
	uint64 startTime = HAP_perf_get_time_us();
	uint64 startCycles = HAP_perf_get_pcycles();
#endif
	int nRet = UBWC_OK;
#if 0
	const int depth = 16;
	const int block_size = 4;

	for(int h = 0; h < nFrameHeight; h++)
	{
		const unsigned char *input_ptr = input_data + h * nFrameWidth * depth;
		for(int b = 0; b < block_size; b++)
		{
			const unsigned char *src = input_ptr;
			for(int w = 0; w < nFrameWidth; w++)
			{
				memcpy(output_data, src, block_size);
				output_data += block_size;
				src += depth;
			}
			input_ptr += block_size;
		}
	}
#else
	nRet = depth_to_space_d16b4(input_data, output_data, nFrameHeight, nFrameWidth);
#endif

#ifdef PROFILING_ON
	uint64 endCycles = HAP_perf_get_pcycles();
	uint64 endTime = HAP_perf_get_time_us();
	FARF(ALWAYS, "[DEPTHTOSPACE]profiling: %d PCycles, %d us. Observed clock rate %d MHz",
		(int)(endCycles - startCycles), (int)(endTime - startTime), (int)((endCycles - startCycles) / (endTime - startTime)));
#endif

	return nRet;

}


#endif

int resizebilinear2x_uv_aligned(unsigned char* input, unsigned char* output, 
							   int in_height, int in_width, int channels, 
							   int out_height, int out_width);
int depth_to_space_d16b4_dequant(const unsigned char *input_data, unsigned char *output_data, int height, int width, float scale, int zero_point);
int aiboostubwc_writeBufferFinal(const unsigned char *dsp_buff, int dsp_size, 
												const unsigned char *uv_buff, int uv_size, 
												unsigned char *ubwc_buff, int ubwc_size, 
												int32 nFrameHeight, int32 nFrameWidth,
												float scale, int32 zero_point)
{
#ifdef PROFILING_ON
	uint64 startTime = HAP_perf_get_time_us();
	uint64 startCycles = HAP_perf_get_pcycles();
#endif

	int nRet = UBWC_OK;

	int nFrameHeightBak = nFrameHeight;
	int nFrameWidthBak = nFrameWidth;
	int nPadHeight = nFrameHeight >> 16;
	int nPadWidth = nFrameWidth >> 16;
	nFrameHeight &= 0xffff;
	nFrameWidth &= 0xffff;

	#define CHANNELS (2)
	int uv_in_height = nFrameHeight >> 2;
	int uv_in_width = nFrameWidth >> 2;
	int uv_out_height = nFrameHeight >> 1;
	int uv_out_width = nFrameWidth >> 1;
    unsigned char *uv_input = (unsigned char*)uv_buff + ((MAX(nFrameHeight, nPadHeight) * MAX(nFrameWidth, nPadWidth)) >> 2);
	unsigned char *uv_output = (unsigned char*)dsp_buff + (MAX(nFrameHeight, nPadHeight) * MAX(nFrameWidth, nPadWidth));

	uv_buff = (unsigned char*)ALIGNUP((unsigned int)uv_buff, 128);
	
#ifndef RESIZE_MULTI_THREADS
	if(uv_in_width & (uv_in_width - 1))
	{
		int in_width_pad = ALIGN2POWN(uv_in_width);
		int out_width_pad = in_width_pad << 1;
		unsigned char *uv_buff_pad = (unsigned char*)uv_buff;
		unsigned char *out_buff_pad = (unsigned char*)uv_buff + in_width_pad * uv_in_height * CHANNELS;

		for(int i = 0; i < uv_in_height; i++)
		{
			vmemcpy_asm(uv_buff_pad + i * in_width_pad * CHANNELS, uv_input + i * uv_in_width * CHANNELS, uv_in_width * CHANNELS);
			uv_buff_pad[i * in_width_pad * CHANNELS + uv_in_width * CHANNELS + 0] = uv_input[i * uv_in_width * CHANNELS + uv_in_width * CHANNELS - 2];
			uv_buff_pad[i * in_width_pad * CHANNELS + uv_in_width * CHANNELS + 1] = uv_input[i * uv_in_width * CHANNELS + uv_in_width * CHANNELS - 1];
		}

		resize_bilinear(uv_buff_pad, out_buff_pad, uv_in_height, in_width_pad, CHANNELS, uv_out_height, out_width_pad);

		for(int i = 0; i < uv_out_height; i++)
		{
			vmemcpy_asm(uv_output + i * uv_out_width * CHANNELS, out_buff_pad + i * out_width_pad * CHANNELS, uv_out_width * CHANNELS);
		}
	}
	else if(((int)uv_input & 127) || ((int)uv_output & 127))
	{
		unsigned char *uv_buff_pad = (unsigned char*)uv_buff;
		unsigned char *out_buff_pad = (unsigned char*)uv_buff + uv_in_width * uv_in_height * CHANNELS;
	
		vmemcpy_asm(uv_buff_pad, uv_input, uv_in_height * uv_in_width * CHANNELS);

		resize_bilinear(uv_buff_pad, out_buff_pad, uv_in_height, uv_in_width, CHANNELS, uv_out_height, uv_out_width);

		vmemcpy_asm(uv_output, out_buff_pad, uv_out_height * uv_out_width * CHANNELS);
	}
	else
	{
		resize_bilinear(uv_input, uv_output, uv_in_height, uv_in_width, CHANNELS, uv_out_height, uv_out_width);
	}
#else
#if 0
	dspCV_ConcurrencyAttribute attrib[1] = 
	{
		{COMPUTE_RECOMMENDATION, 0},
	};
	dspCV_concurrency_query(attrib, 1);
	if(COMPUTE_RECOMMENDATION_NOT_OK == attrib[0].value)
	{
		nRet = UBWC_ERR_DSPCVQUERY;
		LOGE("It is safe to run a compute function now");
		return nRet;
	}

	dspCV_hvx_config_t hvxInfo = {0};
	hvxInfo.mode = DSPCV_HVX_MODE_128B;
	dspCV_hvx_prepare_mt_job(&hvxInfo);
	if(hvxInfo.numUnits <= 0)
	{
		dspCV_hvx_cleanup_mt_job(&hvxInfo);
		nRet = UBWC_ERR_DSPCVNUMUNITS;
		LOGE("hvxInfo.numUnits is %d", hvxInfo.numUnits);
		return nRet;
	}

	int numWorkers = MIN(hvxInfo.numThreads, MAXTHREADS);
	while((uv_in_height % numWorkers))
	{
		numWorkers--;
		if(numWorkers == 0)
		{
			numWorkers = 1;
			break;
		}
	}
	LOGI("MultiThreds:hvxInfo.numThreads = %d, numWorkers = %d", hvxInfo.numThreads, numWorkers);

	dspCV_worker_job_t job;
	dspCV_synctoken_t token;

	dspCV_worker_pool_synctoken_init(&token, numWorkers);

	job.fptr = resize_callback;
	resize_callback_t dptr;

	dptr.token = &token;
	dptr.jobCount = 0;
	dptr.h_in_block = uv_in_height / numWorkers; 
	dptr.h_out_block = uv_out_height / numWorkers;
	dptr.uv_buff = (unsigned char *)uv_buff;
	dptr.uv_input = uv_input;
	dptr.uv_output = uv_output;
	dptr.uv_in_height = uv_in_height;
	dptr.uv_in_width = uv_in_width;
	dptr.uv_out_width = uv_out_width;

	job.dptr = (void *)&dptr;
	
	if(uv_in_width & (uv_in_width - 1))
	{
		int in_width_pad = ALIGN2POWN(uv_in_width);
		unsigned char *uv_buff_pad = (unsigned char*)uv_buff;
		for(int i = 0; i < uv_in_height; i++)
		{
			vmemcpy_asm(uv_buff_pad + i * in_width_pad * CHANNELS, uv_input + i * uv_in_width * CHANNELS, uv_in_width * CHANNELS);
			uv_buff_pad[i * in_width_pad * CHANNELS + uv_in_width * CHANNELS + 0] = uv_input[i * uv_in_width * CHANNELS + uv_in_width * CHANNELS - 2];
			uv_buff_pad[i * in_width_pad * CHANNELS + uv_in_width * CHANNELS + 1] = uv_input[i * uv_in_width * CHANNELS + uv_in_width * CHANNELS - 1];
		}
	}
	else if(((int)uv_input & 127) || ((int)uv_output & 127))
	{
		unsigned char *uv_buff_pad = (unsigned char*)uv_buff;
		vmemcpy_asm(uv_buff_pad, uv_input, uv_in_height * uv_in_width * CHANNELS);
	}

	for(int i = 0; i < numWorkers; i++)
	{
	   (void)dspCV_worker_pool_submit(job);
	}
	dspCV_worker_pool_synctoken_wait(&token);

	dspCV_hvx_cleanup_mt_job(&hvxInfo);
#else
	nRet = resizebilinear2x_uv_aligned(uv_input, uv_output, uv_in_height, uv_in_width, CHANNELS, uv_out_height, uv_out_width);
	if(nRet)
	{
		LOGE("resizebilinear2x_uv_aligned failed.");
		return nRet;		
	}
#endif
#endif

#if 0
	const unsigned char* input_data = dsp_buff;
	unsigned char* output_data = (unsigned char *)dsp_buff;
	nRet = depth_to_space_d16b4_dequant(input_data, output_data, nFrameHeight / 4, MAX(nPadWidth, nFrameWidth) / 4, scale, zero_point);

	if(nRet)
	{
		LOGE("depth_to_space_d16b4_dequant failed.");
		return nRet;		
	}
#endif

#ifdef PROFILING_ON
	uint64 endCycles = HAP_perf_get_pcycles();
	uint64 endTime = HAP_perf_get_time_us();
	FARF(ALWAYS, "[DTOS+DEQUANT+RESIZE]profiling: %d PCycles, %d us. Observed clock rate %d MHz",
		(int)(endCycles - startCycles), (int)(endTime - startTime), (int)((endCycles - startCycles) / (endTime - startTime)));
#endif

#if 1
	HAP_power_response_t response;
	int context = 1;
	response.type = HAP_power_get_clk_Freq;
	(void) HAP_power_get((void *)&context, &response);
	*((int *)(uv_buff)) = response.clkFreqHz;
#endif

	//nRet = aiboostubwc_writeBuffer(dsp_buff, dsp_size, ubwc_buff, ubwc_size, nFrameHeightBak, nFrameWidthBak);
	nRet = aiboostubwc_writeBuffer1(dsp_buff, dsp_size, ubwc_buff, ubwc_size, nFrameHeightBak, nFrameWidthBak, scale, zero_point);

	return nRet;
}

#ifdef RESIZE_MULTI_THREADS
static void resize_callback4(void* data)
{
    #define CHANNELS (2)
	resize_callback_t *dptr = (resize_callback_t*)data;
	int uv_in_width = dptr->uv_in_width;
	int uv_in_height = dptr->uv_in_height;
	int uv_out_width = dptr->uv_out_width;
	unsigned char *uv_input = dptr->uv_input;
	unsigned char *uv_buff = dptr->uv_buff;
	unsigned int h_in_block = dptr->h_in_block;
	unsigned int h_out_block = dptr->h_out_block;
	
	unsigned int jobCount = worker_pool_atomic_inc_return(&(dptr->jobCount)) - 1;

	if(jobCount * h_in_block >= uv_in_height)
    {
        return;
    }

	uv_input += jobCount * h_in_block * uv_in_width * CHANNELS;
	//uv_output += jobCount * h_out_block * uv_out_width * CHANNELS;
	int in_width_pad = ALIGN2POWN(uv_in_width);
	int out_width_pad = in_width_pad << 1;
	unsigned char *out_buff_pad = (unsigned char*)uv_buff + jobCount * h_out_block * out_width_pad * CHANNELS;

	if(uv_in_width & (uv_in_width - 1))
	{
		unsigned char *uv_buff_pad = (unsigned char*)uv_buff + out_width_pad * (uv_in_height << 1) * CHANNELS + jobCount * h_in_block * in_width_pad * CHANNELS;
		for(int i = 0; i < h_in_block; i++)
		{
			vmemcpy_asm(uv_buff_pad + i * in_width_pad * CHANNELS, uv_input + i * uv_in_width * CHANNELS, uv_in_width * CHANNELS);
			uv_buff_pad[i * in_width_pad * CHANNELS + uv_in_width * CHANNELS + 0] = uv_input[i * uv_in_width * CHANNELS + uv_in_width * CHANNELS - 2];
			uv_buff_pad[i * in_width_pad * CHANNELS + uv_in_width * CHANNELS + 1] = uv_input[i * uv_in_width * CHANNELS + uv_in_width * CHANNELS - 1];
		}

		resize_bilinear(uv_buff_pad, out_buff_pad, h_in_block, in_width_pad, CHANNELS, h_out_block, out_width_pad);
	}
	else
	{
		resize_bilinear(uv_input, out_buff_pad, h_in_block, uv_in_width, CHANNELS, h_out_block, uv_out_width);
	}

    worker_pool_synctoken_jobdone(dptr->token);
}
#endif


int depth_to_space_d32d16b4_dequant(const unsigned char *input_data, unsigned char *output_data, int height, int width, float scale, int zero_point);

int aiboostubwc_writeBufferFinal2(const unsigned char *y_buff, const unsigned char *uv_buff, 
												unsigned char *uvpad_buff, 
												unsigned char *ubwc_buff, int ubwc_size, 
												int32 nFrameHeight, int32 nFrameWidth,
												float scale, int32 zero_point)
{
#ifdef PROFILING_ON
	uint64 startTime = HAP_perf_get_time_us();
	uint64 startCycles = HAP_perf_get_pcycles();
#endif

	int nFrameHeightBak = nFrameHeight;
	int nFrameWidthBak = nFrameWidth;
	int nPadHeight = nFrameHeight >> 16;
	int nPadWidth = nFrameWidth >> 16;
	nFrameHeight &= 0xffff;
	nFrameWidth &= 0xffff;

	#define CHANNELS (2)
	int uv_in_height = nFrameHeight >> 2;
	int uv_in_width = nFrameWidth >> 2;
	int uv_out_height = nFrameHeight >> 1;
	int uv_out_width = nFrameWidth >> 1;
    unsigned char *uv_input = (unsigned char*)uv_buff + ((MAX(nFrameHeight, nPadHeight) * MAX(nFrameWidth, nPadWidth)) >> 2);
	const unsigned char* input_data = y_buff;
	uv_buff = (unsigned char*)ALIGNUP((unsigned int)uvpad_buff, 128);
	
#ifndef RESIZE_MULTI_THREADS
	int in_width_pad = ALIGN2POWN(uv_in_width);
	int out_width_pad = in_width_pad << 1;
	unsigned char *uv_buff_pad = (unsigned char*)uv_buff;
	unsigned char *out_buff_pad = (unsigned char*)uv_buff + in_width_pad * uv_in_height * CHANNELS;

	if(uv_in_width & (uv_in_width - 1))
	{
		for(int i = 0; i < uv_in_height; i++)
		{
			vmemcpy_asm(uv_buff_pad + i * in_width_pad * CHANNELS, uv_input + i * uv_in_width * CHANNELS, uv_in_width * CHANNELS);
			uv_buff_pad[i * in_width_pad * CHANNELS + uv_in_width * CHANNELS + 0] = uv_input[i * uv_in_width * CHANNELS + uv_in_width * CHANNELS - 2];
			uv_buff_pad[i * in_width_pad * CHANNELS + uv_in_width * CHANNELS + 1] = uv_input[i * uv_in_width * CHANNELS + uv_in_width * CHANNELS - 1];
		}

		resize_bilinear(uv_buff_pad, out_buff_pad, uv_in_height, in_width_pad, CHANNELS, uv_out_height, out_width_pad);
	}
	else
	{
		resize_bilinear(uv_input, out_buff_pad, uv_in_height, uv_in_width, CHANNELS, uv_out_height, uv_out_width);
	}
#else
	int numWorkers = MIN((qurt_hvx_get_units() >> 8) & 0xFF, MAXTHREADS);
	while((uv_in_height % numWorkers))
	{
		numWorkers--;
		if(numWorkers == 0)
		{
			numWorkers = 1;
			break;
		}
	}
	LOGI("MultiThreds:hvxInfo.numThreads = %d, numWorkers = %d", hvxInfo.numThreads, numWorkers);

    worker_pool_job_t   job;
    worker_synctoken_t    token;
    worker_pool_context_t context = NULL;
	
	job.fptr = resize_callback4;
	resize_callback_t dptr;

	dptr.token = &token;
	dptr.jobCount = 0;
	dptr.h_in_block = uv_in_height / numWorkers; 
	dptr.h_out_block = uv_out_height / numWorkers;
	dptr.uv_buff = (unsigned char *)uv_buff;
	dptr.uv_input = uv_input;
	dptr.uv_in_height = uv_in_height;
	dptr.uv_in_width = uv_in_width;
	dptr.uv_out_width = uv_out_width;

	job.dptr = (void *)&dptr;

	//(void)worker_pool_init(&context);
    worker_pool_synctoken_init(&token, numWorkers);

	for(int i = 0; i < numWorkers; i++)
	{
	   (void)worker_pool_submit(context, job);
	}
	worker_pool_synctoken_wait(&token);

	//worker_pool_deinit(&context);
#endif

#ifdef PROFILING_ON
	uint64 endCycles = HAP_perf_get_pcycles();
	uint64 endTime = HAP_perf_get_time_us();
	FARF(ALWAYS, "[DTOS+DEQUANT+RESIZE]profiling: %d PCycles, %d us. Observed clock rate %d MHz",
		(int)(endCycles - startCycles), (int)(endTime - startTime), (int)((endCycles - startCycles) / (endTime - startTime)));
#endif

	return aiboostubwc_writeBuffer2(input_data, uv_buff, ubwc_buff, ubwc_size, nFrameHeightBak, nFrameWidthBak, scale, zero_point);
}

int aiboostubwc_convolution(const unsigned char* input, int input_size, const int8 *weight, int weight_size,  unsigned char *output, int output_size, const int32* recip, int recip_size, int32 nFrameHeight, int32 nFrameWidth)
{
	return 0;//conv3x3x32x1S2(input, (char *)weight, output, (const int *)recip, nFrameHeight, nFrameWidth);
}

#if 0
int aiboostubwc_memcpy(const unsigned char* input, int input_size, unsigned char *output, int output_size)
{
	vmemcpy_asm(output, input, input_size);

	return UBWC_OK;
}
#endif

#if 1
static void resize_callback3(void *data)
{
	resize_callback2_t *dptr = (resize_callback2_t *)data;
	
	unsigned int jobCount = worker_pool_atomic_inc_return(&(dptr->jobCount)) - 1;
	if(jobCount >= dptr->numWorkers)
	{
		goto out;
	}

	if(jobCount * dptr->sizePerJob >= dptr->height)
	{
		goto out;
	}
	
	int width = dptr->width;
	int height = (jobCount == dptr->numWorkers - 1) ? dptr->height - dptr->sizePerJob * jobCount : dptr->sizePerJob;

	int outWidth = width << 1;
	int outHeight = height << 1;

	unsigned char *nv12_input = dptr->nv12_input + jobCount * dptr->sizePerJob * width;
	unsigned char *nv12_output = dptr->nv12_output + jobCount * (dptr->sizePerJob << 1) * outWidth;

	unsigned char *uv_input = dptr->nv12_input + dptr->height * width + jobCount * (dptr->sizePerJob >> 1) * width;
	unsigned char *uv_output = dptr->nv12_output + (dptr->height << 1) * outWidth + jobCount * dptr->sizePerJob * outWidth;

	resize_bilinear(nv12_input, nv12_output, height, width, 1, outHeight, outWidth);
	resize_bilinear(uv_input, uv_output, height >> 1, width >> 1, 2, outHeight >> 1, outWidth >> 1);
out:
    worker_pool_synctoken_jobdone(dptr->token);
}

int aiboostubwc_resizeBilinear(const unsigned char* ubwc_in, int ubwcin_size, 
								unsigned char *ubwc_out, int ubwcout_size,
								unsigned char* nv12_in, int nv12in_size,
								unsigned char* nv12_out, int nv12out_size,
								int32 nFrameHeight, int32 nFrameWidth)

{
	int nFrameHeightBak = nFrameHeight;
	int nFrameWidthBak = nFrameWidth;
	int nPadHeight = nFrameHeight >> 16;
	int nPadWidth = nFrameWidth >> 16;
	nFrameHeight &= 0xffff;
	nFrameWidth &= 0xffff;
	int outWidth = nFrameWidth << 1;
	int outHeight = nFrameHeight << 1;
	int outWidthPad = nPadWidth << 1;
	int outHeightPad = nPadHeight << 1;
	int nRet = UBWC_OK;

	nRet = aiboostubwc_readBuffer(ubwc_in, ubwcin_size, nv12_in, nv12in_size, nFrameHeightBak | (1 << 31), nFrameWidthBak);
	if(nRet)
	{
		return nRet;
	}

	int numWorkers = MIN((qurt_hvx_get_units() >> 8) & 0xFF, MAXTHREADS); 
	
    worker_pool_job_t   job;
    worker_synctoken_t    token;
    worker_pool_context_t context = NULL;

	job.fptr = resize_callback3;
	resize_callback2_t dptr;

	dptr.token = &token;
	dptr.jobCount = 0;
	dptr.nv12_input = nv12_in;
	dptr.nv12_output = nv12_out;
	dptr.width = nPadWidth;
	dptr.height = nFrameHeight;
	dptr.sizePerJob = ALIGNUP((nFrameHeight + numWorkers - 1) / numWorkers, 2);
	dptr.numWorkers = numWorkers;

	job.dptr = (void *)&dptr;

	//(void)worker_pool_init(&context);
    worker_pool_synctoken_init(&token, numWorkers);

	for(int i = 0; i < numWorkers; i++)
	{
	   (void)worker_pool_submit(context, job);
	}
	worker_pool_synctoken_wait(&token);

	//worker_pool_deinit(&context);
	
	nRet = aiboostubwc_writeBuffer(nv12_out, nv12out_size, ubwc_out, ubwcout_size, (outHeightPad << 16) | outHeight | (1 << 31), (outWidthPad << 16) | outWidth);
	if(nRet)
	{
		return nRet;
	}
	
	return UBWC_OK;
}

#else
typedef struct
{
    dspCV_synctoken_t *token;
	unsigned char *src_buff;
	unsigned char *dst_buff;
	int nFrameHeight;
	int nFrameWidth;
	int totalHeight;
	unsigned int jobCount;
	int numThreads;
	t_DmaWrapper_DmaEngineHandle handle_rd[MAXTHREADS];
	t_DmaWrapper_DmaEngineHandle handle_wr[MAXTHREADS];
	int nRet[MAXTHREADS];
}ubwcresize_callback_t;

typedef struct
{
	int nRoiWalkHeightI;
	int nRoiWalkWidthI;
	int nRoiWalkHeight;
	int nRoiWalkWidth;
	int nFrameHeight;
	int nFrameWidth;
	int nFrameHeightO;
	int nFrameWidthO;
	qurt_addr_t tcm_buf_vaddr;
	qurt_size_t ChromaOffset;
	qurt_size_t tcm_buf_size;
}iRoiInfo2_t;

static void updateRoiInfoThreadsRd(int nRowIdx, int nRow,
        				int nColIdx, int nCol, int nIdx,
        				iRoiInfo2_t *iRoi,
        				t_StDmaWrapper_UpdateParm* stWrapUpdateParm,
        				int yOff)
{
	for (int j = 0; j < 2; j++)
    {
        stWrapUpdateParm[j].u.stPixData.stRoi.u16Y = iRoi->nRoiWalkHeightI * nRowIdx + yOff;
        if (nRowIdx == (nRow - 1))
        {
            stWrapUpdateParm[j].u.stPixData.stRoi.u16H = iRoi->nFrameHeight - (iRoi->nRoiWalkHeightI * nRowIdx);
        }
        else
        {
            stWrapUpdateParm[j].u.stPixData.stRoi.u16H = iRoi->nRoiWalkHeight;
        }

        stWrapUpdateParm[j].u.stPixData.stRoi.u16X = iRoi->nRoiWalkWidthI * nColIdx;
        if (nColIdx == (nCol - 1))
        {
            stWrapUpdateParm[j].u.stPixData.stRoi.u16W = iRoi->nFrameWidth - (iRoi->nRoiWalkWidthI * nColIdx);
        }
        else
        {
            stWrapUpdateParm[j].u.stPixData.stRoi.u16W = iRoi->nRoiWalkWidth;
        }

        stWrapUpdateParm[j].aCacheAddr = qurt_lookup_physaddr(iRoi->tcm_buf_vaddr) + 
															(nIdx % 2 ? iRoi->tcm_buf_size : 0) + 
															j * iRoi->ChromaOffset;
	}
}


static void updateRoiInfoThreadsWr(int nRowIdx, int nRow,
        				int nColIdx, int nCol, int nIdx,
        				iRoiInfo2_t *iRoi,
        				t_StDmaWrapper_UpdateParm* stWrapUpdateParm,
        				int yOff)
{
	for (int j = 0; j < 2; j++)
    {
        stWrapUpdateParm[j].u.stPixData.stRoi.u16Y = iRoi->nRoiWalkHeight * nRowIdx + yOff;
        if (nRowIdx == (nRow - 1))
        {
            stWrapUpdateParm[j].u.stPixData.stRoi.u16H = iRoi->nFrameHeightO - (iRoi->nRoiWalkHeight * nRowIdx);
        }
        else
        {
            stWrapUpdateParm[j].u.stPixData.stRoi.u16H = iRoi->nRoiWalkHeight;
        }

        stWrapUpdateParm[j].u.stPixData.stRoi.u16X = iRoi->nRoiWalkWidth * nColIdx;
        if (nColIdx == (nCol - 1))
        {
            stWrapUpdateParm[j].u.stPixData.stRoi.u16W = iRoi->nFrameWidthO - (iRoi->nRoiWalkWidth * nColIdx);
        }
        else
        {
            stWrapUpdateParm[j].u.stPixData.stRoi.u16W = iRoi->nRoiWalkWidth;
        }

        stWrapUpdateParm[j].aCacheAddr = qurt_lookup_physaddr(iRoi->tcm_buf_vaddr) + iRoi->tcm_buf_size * 2 +
															(nIdx % 2 ? iRoi->tcm_buf_size : 0) + 
															j * iRoi->ChromaOffset;
	}
}

static void ubwc_resize_callback(void *data)
{
	ubwcresize_callback_t *dptr = (ubwcresize_callback_t *)data;
	unsigned int jobCount = 0;
    int nRet = UBWC_OK;

	int lockResult = dspCV_hvx_lock(DSPCV_HVX_MODE_128B, 0);
	if (0 > lockResult)
    {
		nRet = UBWC_ERR_DSPCVLOCK;
		LOGE("HVX is reserved but could not be locked.");
		goto out;
    }
	
	jobCount = dspCV_atomic_inc_return(&(dptr->jobCount)) - 1;

	int nFrameHeight = dptr->nFrameHeight;
	int nFrameWidth = dptr->nFrameWidth;
	int nFrameHeightO = nFrameHeight << 1;
	int nFrameWidthO = nFrameWidth << 1;

	t_DmaWrapper_DmaEngineHandle handle_rd = dptr->handle_rd[jobCount];
	t_DmaWrapper_DmaEngineHandle handle_wr = dptr->handle_wr[jobCount];
	t_eDmaFmt eFmtLuma = eDmaFmt_NV12_Y;
    t_eDmaFmt eFmtChroma = eDmaFmt_NV12_UV;

	t_eDmaFmt efmtLumaChroma[2] = {eFmtLuma, eFmtChroma};

	t_StDmaWrapper_Roi stRoi;
	t_StDmaWrapper_RoiAlignInfo stAlignInfo;
	int lumaStride, chromaStride;
	qurt_addr_t tcm_buf_vaddr, tcm_desc_vaddr;
	unsigned long long tcm_buf_paddr, tcm_desc_paddr;
	qurt_size_t tcm_buf_size, ChromaOffset, tcm_desc_size, region_tcm_size;
	t_StDmaWrapper_PrepareParm stWrapPrepParm;
	t_StDmaWrapper_WorkDescrip staWorkDesc[2];
    t_StDmaWrapper_FrameProp walkRoi;
    t_StDmaWrapper_FrameProp frameProp;
	int nIdx = 0;
    int nRow, nCol;
    int nRowIdx = 0, nColIdx = 0;
	t_StDmaWrapper_UpdateParm stWrapUpdateParm[2];

	stRoi.u16W = 256;
    stRoi.u16H = 32;
	nRet = nDmaWrapper_GetRecommendedRoi(eDmaFmt_NV12, TRUE, &stRoi);
	if(nRet)
	{
		LOGE("nDmaWrapper_GetRecommendedRoi error.");
	    goto out;
	}

	//int ubwcHeight = ALIGNUP(nFrameHeight, 32);
	int ubwcWidth = ALIGNUP(nFrameWidth, 128);

	int ubwcHeightO = ALIGNUP(nFrameHeightO, 32);
	int ubwcWidthO = ALIGNUP(nFrameWidthO, 128);
	
	nRow = ROUNDDIV(ubwcHeightO, stRoi.u16H);
	nCol = ROUNDDIV(ubwcWidthO,  stRoi.u16W);

	stAlignInfo.u16W = stRoi.u16W;
    stAlignInfo.u16H = stRoi.u16H;

	lumaStride = nDmaWrapper_GetRecommendedIntermBufStride(eFmtLuma, &stAlignInfo, TRUE);
	chromaStride = nDmaWrapper_GetRecommendedIntermBufStride(eFmtChroma, &stAlignInfo, TRUE);

	tcm_buf_size = nDmaWrapper_GetRecommendedIntermBufSize(eFmtLuma, PAD16BIT, &stAlignInfo, TRUE, lumaStride);
	ChromaOffset = tcm_buf_size;
	tcm_buf_size += nDmaWrapper_GetRecommendedIntermBufSize(eFmtChroma, PAD16BIT, &stAlignInfo, TRUE, chromaStride);
	region_tcm_size = ALIGNUP(tcm_buf_size * 4, 0x1000);
	tcm_desc_size = ALIGNUP(nDmaWrapper_GetDescbuffsize(efmtLumaChroma, 2) * 2 * 2, 0x1000);

	tcm_desc_vaddr = (addr_t)HAP_cache_lock((unsigned int)tcm_desc_size, &tcm_desc_paddr);
	tcm_buf_vaddr = (addr_t)HAP_cache_lock((unsigned int)region_tcm_size, &tcm_buf_paddr);
	if(!tcm_desc_vaddr || !tcm_buf_vaddr)
	{
		LOGE("HAP_cache_lock error.");
		nRet = UBWC_ERR_ALLOCTCM;
		goto out;
	}

	stWrapPrepParm.u32NumOfWorkDesc = 2;
    stWrapPrepParm.staWorkDesc = staWorkDesc;
	for (int i = 0; i < 1; i++)
    {
        for (int j = 0; j < 2; j++)
        {
            int descIndex = i * 2 + j;
            frameProp.aAddr = (addr_t)qurt_lookup_physaddr((qurt_addr_t)(dptr->src_buff));
            frameProp.u16W = ubwcWidth;
            frameProp.u16H = ALIGNUP(dptr->totalHeight, 32);
            frameProp.u16Stride = ubwcWidth;
            walkRoi.aAddr = 0;
            walkRoi.u16W = stRoi.u16W;
            walkRoi.u16H = stRoi.u16H;
            walkRoi.u16Stride = (j) ? chromaStride : lumaStride;
            nRet = nDmaWrapper_WorkDescrip_populate(&staWorkDesc[descIndex],
                                                    efmtLumaChroma[j],
                                                    eDmaWrapper_DdrToL2,
                                                    &walkRoi, &frameProp,
                                                    TRUE,
                                                    PAD16BIT,
                                                    NULL);
        }
    }
    stWrapPrepParm.u32DescBufSize = nDmaWrapper_GetDescbuffsize(efmtLumaChroma, 2);
	stWrapPrepParm.pPingDescBuf = (void *)tcm_desc_vaddr;
	stWrapPrepParm.pPongDescBuf = (void *)(tcm_desc_vaddr + stWrapPrepParm.u32DescBufSize);
	nRet |= nDmaWrapper_Prepare(handle_rd, &stWrapPrepParm);
	if(nRet)
	{
		nRet = UBWC_ERR_DMAPREPARE;
		LOGE("nDmaWrapper_Prepare error.");
		goto error;
	}

    stWrapPrepParm.u32NumOfWorkDesc = 2;
    stWrapPrepParm.staWorkDesc = staWorkDesc;
	for (int i = 0; i < 1; i++)
    {
        for (int j = 0; j < 2; j++)
        {
            int descIndex = i * 2 + j;
            frameProp.aAddr = (addr_t)qurt_lookup_physaddr((qurt_addr_t)(dptr->dst_buff));
            frameProp.u16W = ubwcWidthO;
            frameProp.u16H =  ALIGNUP((dptr->totalHeight << 1), 32);
            frameProp.u16Stride = ubwcWidthO;
            walkRoi.aAddr = 0;
            walkRoi.u16W = stRoi.u16W;
            walkRoi.u16H = stRoi.u16H;
            walkRoi.u16Stride = (j) ? chromaStride : lumaStride;
            nRet = nDmaWrapper_WorkDescrip_populate(&staWorkDesc[descIndex],
                                                    efmtLumaChroma[j],
                                                    eDmaWrapper_L2ToDdr,
                                                    &walkRoi, &frameProp,
                                                    TRUE,
                                                    PAD16BIT,
                                                    NULL);
        }
    }
    stWrapPrepParm.u32DescBufSize = nDmaWrapper_GetDescbuffsize(efmtLumaChroma, 2);
	stWrapPrepParm.pPingDescBuf = (void *)(tcm_desc_vaddr + stWrapPrepParm.u32DescBufSize * 2);
	stWrapPrepParm.pPongDescBuf = (void *)(tcm_desc_vaddr + stWrapPrepParm.u32DescBufSize * 3);
	nRet |= nDmaWrapper_Prepare(handle_wr, &stWrapPrepParm);
	if(nRet)
	{
		nRet = UBWC_ERR_DMAPREPARE;
		LOGE("nDmaWrapper_Prepare error.");
		goto error;
	}

	iRoiInfo2_t iRoi;
	iRoi.nFrameHeight = nFrameHeight;
	iRoi.nFrameWidth = nFrameWidth;

	iRoi.nFrameHeightO = nFrameHeightO;
	iRoi.nFrameWidthO = nFrameWidthO;
	iRoi.nRoiWalkHeight = stRoi.u16H;
	iRoi.nRoiWalkWidthI = 128;
	iRoi.nRoiWalkHeightI = 16;
	iRoi.nRoiWalkWidth = stRoi.u16W;
	iRoi.tcm_buf_vaddr = tcm_buf_vaddr;
	iRoi.tcm_buf_size = tcm_buf_size;
	iRoi.ChromaOffset = ChromaOffset;

	updateRoiInfoThreadsRd(nRowIdx, nRow, nColIdx, nCol, nIdx, &iRoi, &stWrapUpdateParm[0], jobCount * dptr->nFrameHeight);
	nRet = nDmaWrapper_Update(handle_rd, &stWrapUpdateParm[0]);
	if(nRet)
	{
		nRet = UBWC_ERR_DMAUPDATE;
		LOGE("nDmaWrapper_Update error.");
		goto error;
	}

	updateRoiInfoThreadsWr(nRowIdx, nRow, nColIdx, nCol, nIdx, &iRoi, &stWrapUpdateParm[0], jobCount * dptr->nFrameHeight);
	nRet = nDmaWrapper_Update(handle_wr, &stWrapUpdateParm[0]);
	if(nRet)
	{
		nRet = UBWC_ERR_DMAUPDATE;
		LOGE("nDmaWrapper_Update error.");
		goto error;
	}

	nRet = nDmaWrapper_Move(handle_rd);
	if(nRet)
	{
		nRet = UBWC_ERR_DMAMOVE;
		LOGE("nDmaWrapper_Move error.");
		goto error;
	}
	nIdx = (nIdx + 1) % 2;

	bool preventFirstWriteWait = TRUE;

    for (nRowIdx = 0; nRowIdx < nRow; nRowIdx++)
    {
        for (nColIdx = (nRowIdx)? 0:1; nColIdx < nCol; nColIdx++)
        {
			updateRoiInfoThreadsRd(nRowIdx, nRow, nColIdx, nCol, nIdx, &iRoi, &stWrapUpdateParm[0], jobCount * dptr->nFrameHeight);
			if (!preventFirstWriteWait)
            {
               nRet = nDmaWrapper_Wait(handle_wr);
            }
            else
            {
                preventFirstWriteWait = FALSE;
            }
			
            nRet |= nDmaWrapper_Update(handle_rd, &stWrapUpdateParm[0]);
            nRet |= nDmaWrapper_Wait(handle_rd);

			//unsigned char *tcm_buff = (unsigned char *)(tcm_buf_vaddr + (nIdx % 2 ? 0 : tcm_buf_size));
			//resize_bilinear(tcm_buff, tcm_buff + tcm_buf_size * 2, 16, 128, 1, 32, 256);
			//resize_bilinear(tcm_buff + ChromaOffset, tcm_buff + tcm_buf_size * 2 + ChromaOffset, 8, 64, 2, 16, 128);
			
            nRet |= nDmaWrapper_Move(handle_rd);
			nRet |= nDmaWrapper_Move(handle_wr);
			updateRoiInfoThreadsWr(nRowIdx, nRow, nColIdx, nCol, nIdx, &iRoi, &stWrapUpdateParm[0], jobCount * dptr->nFrameHeight);
            nRet |= nDmaWrapper_Update(handle_wr, &stWrapUpdateParm[0]);
			if(nRet)
			{
	
				nRet = UBWC_ERR_DMAUPDATE | UBWC_ERR_DMAMOVE | UBWC_ERR_DMAWAIT;
				LOGE("nDmaWrapper_Move or nDmaWrapper_Update or nDmaWrapper_Wait error.");
				goto error;
			}		
            nIdx = (nIdx + 1) % 2;
        }
    }

    if (!preventFirstWriteWait)
    {
        nRet =nDmaWrapper_Wait(handle_wr);
    }
	
    nRet |=nDmaWrapper_Wait(handle_rd);
    nRet |=nDmaWrapper_Move(handle_wr);
    nRet |=nDmaWrapper_Wait(handle_wr);
	
	if(nRet)
	{
		nRet = UBWC_ERR_DMAMOVE | UBWC_ERR_DMAWAIT;
		LOGE("nDmaWrapper_Move or nDmaWrapper_Wait error.");
		goto error;
	}	

	nRet = nDmaWrapper_FinishFrame(handle_rd);
	nRet = nDmaWrapper_FinishFrame(handle_wr);
	if(nRet)
	{
		nRet = UBWC_ERR_DMAFINISH;
		LOGE("nDmaWrapper_FinishFrame error.");
		goto error;
	}

error:	
	HAP_cache_unlock((void*)tcm_buf_vaddr);
	HAP_cache_unlock((void*)tcm_desc_vaddr);
	
out:
	dptr->nRet[jobCount] = nRet;
	dspCV_hvx_unlock();
	dspCV_worker_pool_synctoken_jobdone(dptr->token);

	return;
}

static int aiboostubwc_getpower()
{
	HAP_power_response_t response;
	response.type = HAP_power_get_clk_Freq;
	(void) HAP_power_get(NULL, &response);
	return response.clkFreqHz;
}
int aiboostubwc_resizeBilinear(const unsigned char* ubwc_in, int ubwcin_size, 
								unsigned char *ubwc_out, int ubwcout_size,
								unsigned char* nv12_in, int nv12in_size,
								unsigned char* nv12_out, int nv12out_size,
								int32 nFrameHeight, int32 nFrameWidth)

{
	int nRet = UBWC_OK;
	FARF(ALWAYS, "aiboostubwc_resizeBilinear========================>");

	//unsigned char *buff = NULL;
	//*buff = 1;
	FARF(ALWAYS, "DSP CLK FREQ = %d", aiboostubwc_getpower());

	return UBWC_OK;
		
	dspCV_ConcurrencyAttribute attrib[1] = 
	{
		{COMPUTE_RECOMMENDATION, 0},
	};
	dspCV_concurrency_query(attrib, 1);
	if(COMPUTE_RECOMMENDATION_NOT_OK == attrib[0].value)
	{
		nRet = UBWC_ERR_DSPCVQUERY;
		LOGE("It is safe to run a compute function now");
		return nRet;
	}

	dspCV_hvx_config_t hvxInfo = {0};
	hvxInfo.mode = DSPCV_HVX_MODE_128B;
	dspCV_hvx_prepare_mt_job(&hvxInfo);
	if(hvxInfo.numUnits <= 0)
	{
		dspCV_hvx_cleanup_mt_job(&hvxInfo);
		nRet = UBWC_ERR_DSPCVNUMUNITS;
		LOGE("hvxInfo.numUnits is %d", hvxInfo.numUnits);
		return nRet;
	}

	int numWorkers = MIN(hvxInfo.numThreads, MAXTHREADS);
	//numWorkers = 1;

	int h = nFrameHeight;
	int n = ROUNDDIV(h, 32);
	int nn = ROUNDDIV(n, numWorkers);
	
	dspCV_worker_job_t job;
	dspCV_synctoken_t token;
	dspCV_worker_pool_synctoken_init(&token, numWorkers);

	job.fptr = ubwc_resize_callback;
	ubwcresize_callback_t dptr;

	for(int i = 0; i < numWorkers; i++)
	{
		dptr.nRet[i] = UBWC_OK;
		dptr.handle_rd[i] = hDmaWrapper_AllocDmaSpecifyWaitType(eDmaWaitType_Polling);
		dptr.handle_wr[i] = hDmaWrapper_AllocDmaSpecifyWaitType(eDmaWaitType_Polling);
		if(!dptr.handle_rd[i] || !dptr.handle_wr[i])
		{
			return UBWC_ERR_ALLOCDMA;
		}
	}

	dptr.token = &token;
	dptr.jobCount = 0;
	dptr.src_buff = (unsigned char *)ubwc_in;
	dptr.dst_buff = ubwc_out;
	dptr.nFrameHeight = nn * 32;
	dptr.nFrameWidth = nFrameWidth;
	dptr.totalHeight = nFrameHeight;
	dptr.numThreads = numWorkers;

	job.dptr = (void *)&dptr;

	for(int i = 0; i < numWorkers; i++)
	{
	   (void)dspCV_worker_pool_submit(job);
	}
	dspCV_worker_pool_synctoken_wait(&token);

	dspCV_hvx_cleanup_mt_job(&hvxInfo);

	for(int i = 0; i < numWorkers; i++)
	{
		if(dptr.handle_rd[i]) nRet |= (nDmaWrapper_FreeDma(dptr.handle_rd[i]) < 0 ? UBWC_ERR_DMAFREE : UBWC_OK);
		if(dptr.handle_wr[i]) nRet |= (nDmaWrapper_FreeDma(dptr.handle_wr[i]) < 0 ? UBWC_ERR_DMAFREE : UBWC_OK);
		nRet |= dptr.nRet[i];
	}

	FARF(ALWAYS, "========================>aiboostubwc_resizeBilinear");
	
	return nRet;
}
#endif

#if 0
int gaussian5x5(const unsigned char* input, unsigned char *output, const unsigned char* table, int height, int width);
int osie_uv(const unsigned char* input, unsigned char *output, int height, int width);

int aiboostubwc_gaussian5x5(const unsigned char* nv12_in, int nv12in_size, 
								unsigned char* nv12_out, int nv12out_size,
								const unsigned char* table, int table_size,
								int32 nFrameHeight, int32 nFrameWidth)
{
#ifdef PROFILING_ON
	uint64 startTime = HAP_perf_get_time_us();
	uint64 startCycles = HAP_perf_get_pcycles();
#endif
	int nRet = UBWC_OK;

	nRet = gaussian5x5(nv12_in, nv12_out, table, nFrameHeight, nFrameWidth);
	nRet |= osie_uv(nv12_in + nFrameHeight * nFrameWidth, nv12_out + nFrameHeight * nFrameWidth, nFrameHeight >> 1, nFrameWidth);

#ifdef PROFILING_ON
	uint64 endCycles = HAP_perf_get_pcycles();
	uint64 endTime = HAP_perf_get_time_us();
	FARF(ALWAYS, "[OSIE]profiling: %d PCycles, %d us. Observed clock rate %d MHz",
		(int)(endCycles - startCycles), (int)(endTime - startTime), (int)((endCycles - startCycles) / (endTime - startTime)));
#endif

	return nRet;
}
#endif
