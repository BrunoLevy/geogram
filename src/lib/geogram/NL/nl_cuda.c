/*
 *  Copyright (c) 2000-2022 Inria
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice,
 *  this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *  this list of conditions and the following disclaimer in the documentation
 *  and/or other materials provided with the distribution.
 *  * Neither the name of the ALICE Project-Team nor the names of its
 *  contributors may be used to endorse or promote products derived from this
 *  software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 *  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *  Contact: Bruno Levy
 *
 *     https://www.inria.fr/fr/bruno-levy
 *
 *     Inria,
 *     Domaine de Voluceau,
 *     78150 Le Chesnay - Rocquencourt
 *     FRANCE
 *
 */

#include "nl_cuda.h"
#include "nl_context.h"
#include <stdint.h>

/**
 * \file nl_cuda.c
 * \brief Weak-coupling adapter to call CUDA from OpenNL.
 */

#ifdef __clang__
#pragma GCC diagnostic ignored "-Wcast-qual"
#endif

/**********************************************************/
/*      CUDA structures and functions                     */
/* Repeated here so that one can compile OpenNL without   */
/* requiring CUDA to be installed in the system.          */
/**********************************************************/

enum cudaComputeMode {
    cudaComputeModeDefault          = 0,
    cudaComputeModeExclusive        = 1,
    cudaComputeModeProhibited       = 2,
    cudaComputeModeExclusiveProcess = 3
};

enum cudaMemcpyKind {
    cudaMemcpyHostToHost          =   0,
    cudaMemcpyHostToDevice        =   1,
    cudaMemcpyDeviceToHost        =   2,
    cudaMemcpyDeviceToDevice      =   3,
    cudaMemcpyDefault             =   4
};

enum cudaDeviceAttribute {
    CU_DEVICE_ATTRIBUTE_MAX_THREADS_PER_BLOCK = 1,
    CU_DEVICE_ATTRIBUTE_MAX_BLOCK_DIM_X = 2,
    CU_DEVICE_ATTRIBUTE_MAX_BLOCK_DIM_Y = 3,
    CU_DEVICE_ATTRIBUTE_MAX_BLOCK_DIM_Z = 4,
    CU_DEVICE_ATTRIBUTE_MAX_GRID_DIM_X = 5,
    CU_DEVICE_ATTRIBUTE_MAX_GRID_DIM_Y = 6,
    CU_DEVICE_ATTRIBUTE_MAX_GRID_DIM_Z = 7,
    CU_DEVICE_ATTRIBUTE_MAX_SHARED_MEMORY_PER_BLOCK = 8,
    CU_DEVICE_ATTRIBUTE_SHARED_MEMORY_PER_BLOCK = 8,
    CU_DEVICE_ATTRIBUTE_TOTAL_CONSTANT_MEMORY = 9,
    CU_DEVICE_ATTRIBUTE_WARP_SIZE = 10,
    CU_DEVICE_ATTRIBUTE_MAX_PITCH = 11,
    CU_DEVICE_ATTRIBUTE_MAX_REGISTERS_PER_BLOCK = 12,
    CU_DEVICE_ATTRIBUTE_REGISTERS_PER_BLOCK = 12,
    CU_DEVICE_ATTRIBUTE_CLOCK_RATE = 13,
    CU_DEVICE_ATTRIBUTE_TEXTURE_ALIGNMENT = 14,
    CU_DEVICE_ATTRIBUTE_GPU_OVERLAP = 15,
    CU_DEVICE_ATTRIBUTE_MULTIPROCESSOR_COUNT = 16,
    CU_DEVICE_ATTRIBUTE_KERNEL_EXEC_TIMEOUT = 17,
    CU_DEVICE_ATTRIBUTE_INTEGRATED = 18,
    CU_DEVICE_ATTRIBUTE_CAN_MAP_HOST_MEMORY = 19,
    CU_DEVICE_ATTRIBUTE_COMPUTE_MODE = 20,
    CU_DEVICE_ATTRIBUTE_MAXIMUM_TEXTURE1D_WIDTH = 21,
    CU_DEVICE_ATTRIBUTE_MAXIMUM_TEXTURE2D_WIDTH = 22,
    CU_DEVICE_ATTRIBUTE_MAXIMUM_TEXTURE2D_HEIGHT = 23,
    CU_DEVICE_ATTRIBUTE_MAXIMUM_TEXTURE3D_WIDTH = 24,
    CU_DEVICE_ATTRIBUTE_MAXIMUM_TEXTURE3D_HEIGHT = 25,
    CU_DEVICE_ATTRIBUTE_MAXIMUM_TEXTURE3D_DEPTH = 26,
    CU_DEVICE_ATTRIBUTE_MAXIMUM_TEXTURE2D_LAYERED_WIDTH = 27,
    CU_DEVICE_ATTRIBUTE_MAXIMUM_TEXTURE2D_LAYERED_HEIGHT = 28,
    CU_DEVICE_ATTRIBUTE_MAXIMUM_TEXTURE2D_LAYERED_LAYERS = 29,
    CU_DEVICE_ATTRIBUTE_MAXIMUM_TEXTURE2D_ARRAY_WIDTH = 27,
    CU_DEVICE_ATTRIBUTE_MAXIMUM_TEXTURE2D_ARRAY_HEIGHT = 28,
    CU_DEVICE_ATTRIBUTE_MAXIMUM_TEXTURE2D_ARRAY_NUMSLICES = 29,
    CU_DEVICE_ATTRIBUTE_SURFACE_ALIGNMENT = 30,
    CU_DEVICE_ATTRIBUTE_CONCURRENT_KERNELS = 31,
    CU_DEVICE_ATTRIBUTE_ECC_ENABLED = 32,
    CU_DEVICE_ATTRIBUTE_PCI_BUS_ID = 33,
    CU_DEVICE_ATTRIBUTE_PCI_DEVICE_ID = 34,
    CU_DEVICE_ATTRIBUTE_TCC_DRIVER = 35,
    CU_DEVICE_ATTRIBUTE_MEMORY_CLOCK_RATE = 36,
    CU_DEVICE_ATTRIBUTE_GLOBAL_MEMORY_BUS_WIDTH = 37,
    CU_DEVICE_ATTRIBUTE_L2_CACHE_SIZE = 38,
    CU_DEVICE_ATTRIBUTE_MAX_THREADS_PER_MULTIPROCESSOR = 39,
    CU_DEVICE_ATTRIBUTE_ASYNC_ENGINE_COUNT = 40,
    CU_DEVICE_ATTRIBUTE_UNIFIED_ADDRESSING = 41,
    CU_DEVICE_ATTRIBUTE_MAXIMUM_TEXTURE1D_LAYERED_WIDTH = 42,
    CU_DEVICE_ATTRIBUTE_MAXIMUM_TEXTURE1D_LAYERED_LAYERS = 43,
    CU_DEVICE_ATTRIBUTE_CAN_TEX2D_GATHER = 44,
    CU_DEVICE_ATTRIBUTE_MAXIMUM_TEXTURE2D_GATHER_WIDTH = 45,
    CU_DEVICE_ATTRIBUTE_MAXIMUM_TEXTURE2D_GATHER_HEIGHT = 46,
    CU_DEVICE_ATTRIBUTE_MAXIMUM_TEXTURE3D_WIDTH_ALTERNATE = 47,
    CU_DEVICE_ATTRIBUTE_MAXIMUM_TEXTURE3D_HEIGHT_ALTERNATE = 48,
    CU_DEVICE_ATTRIBUTE_MAXIMUM_TEXTURE3D_DEPTH_ALTERNATE = 49,
    CU_DEVICE_ATTRIBUTE_PCI_DOMAIN_ID = 50,
    CU_DEVICE_ATTRIBUTE_TEXTURE_PITCH_ALIGNMENT = 51,
    CU_DEVICE_ATTRIBUTE_MAXIMUM_TEXTURECUBEMAP_WIDTH = 52,
    CU_DEVICE_ATTRIBUTE_MAXIMUM_TEXTURECUBEMAP_LAYERED_WIDTH = 53,
    CU_DEVICE_ATTRIBUTE_MAXIMUM_TEXTURECUBEMAP_LAYERED_LAYERS = 54,
    CU_DEVICE_ATTRIBUTE_MAXIMUM_SURFACE1D_WIDTH = 55,
    CU_DEVICE_ATTRIBUTE_MAXIMUM_SURFACE2D_WIDTH = 56,
    CU_DEVICE_ATTRIBUTE_MAXIMUM_SURFACE2D_HEIGHT = 57,
    CU_DEVICE_ATTRIBUTE_MAXIMUM_SURFACE3D_WIDTH = 58,
    CU_DEVICE_ATTRIBUTE_MAXIMUM_SURFACE3D_HEIGHT = 59,
    CU_DEVICE_ATTRIBUTE_MAXIMUM_SURFACE3D_DEPTH = 60,
    CU_DEVICE_ATTRIBUTE_MAXIMUM_SURFACE1D_LAYERED_WIDTH = 61,
    CU_DEVICE_ATTRIBUTE_MAXIMUM_SURFACE1D_LAYERED_LAYERS = 62,
    CU_DEVICE_ATTRIBUTE_MAXIMUM_SURFACE2D_LAYERED_WIDTH = 63,
    CU_DEVICE_ATTRIBUTE_MAXIMUM_SURFACE2D_LAYERED_HEIGHT = 64,
    CU_DEVICE_ATTRIBUTE_MAXIMUM_SURFACE2D_LAYERED_LAYERS = 65,
    CU_DEVICE_ATTRIBUTE_MAXIMUM_SURFACECUBEMAP_WIDTH = 66,
    CU_DEVICE_ATTRIBUTE_MAXIMUM_SURFACECUBEMAP_LAYERED_WIDTH = 67,
    CU_DEVICE_ATTRIBUTE_MAXIMUM_SURFACECUBEMAP_LAYERED_LAYERS = 68,
    CU_DEVICE_ATTRIBUTE_MAXIMUM_TEXTURE1D_LINEAR_WIDTH = 69,
    CU_DEVICE_ATTRIBUTE_MAXIMUM_TEXTURE2D_LINEAR_WIDTH = 70,
    CU_DEVICE_ATTRIBUTE_MAXIMUM_TEXTURE2D_LINEAR_HEIGHT = 71,
    CU_DEVICE_ATTRIBUTE_MAXIMUM_TEXTURE2D_LINEAR_PITCH = 72,
    CU_DEVICE_ATTRIBUTE_MAXIMUM_TEXTURE2D_MIPMAPPED_WIDTH = 73,
    CU_DEVICE_ATTRIBUTE_MAXIMUM_TEXTURE2D_MIPMAPPED_HEIGHT = 74,
    CU_DEVICE_ATTRIBUTE_COMPUTE_CAPABILITY_MAJOR = 75,
    CU_DEVICE_ATTRIBUTE_COMPUTE_CAPABILITY_MINOR = 76,
    CU_DEVICE_ATTRIBUTE_MAXIMUM_TEXTURE1D_MIPMAPPED_WIDTH = 77,
    CU_DEVICE_ATTRIBUTE_STREAM_PRIORITIES_SUPPORTED = 78,
    CU_DEVICE_ATTRIBUTE_GLOBAL_L1_CACHE_SUPPORTED = 79,
    CU_DEVICE_ATTRIBUTE_LOCAL_L1_CACHE_SUPPORTED = 80,
    CU_DEVICE_ATTRIBUTE_MAX_SHARED_MEMORY_PER_MULTIPROCESSOR = 81,
    CU_DEVICE_ATTRIBUTE_MAX_REGISTERS_PER_MULTIPROCESSOR = 82,
    CU_DEVICE_ATTRIBUTE_MANAGED_MEMORY = 83,
    CU_DEVICE_ATTRIBUTE_MULTI_GPU_BOARD = 84,
    CU_DEVICE_ATTRIBUTE_MULTI_GPU_BOARD_GROUP_ID = 85,
    CU_DEVICE_ATTRIBUTE_HOST_NATIVE_ATOMIC_SUPPORTED = 86,
    CU_DEVICE_ATTRIBUTE_SINGLE_TO_DOUBLE_PRECISION_PERF_RATIO = 87,
    CU_DEVICE_ATTRIBUTE_PAGEABLE_MEMORY_ACCESS = 88,
    CU_DEVICE_ATTRIBUTE_CONCURRENT_MANAGED_ACCESS = 89,
    CU_DEVICE_ATTRIBUTE_COMPUTE_PREEMPTION_SUPPORTED = 90,
    CU_DEVICE_ATTRIBUTE_CAN_USE_HOST_POINTER_FOR_REGISTERED_MEM = 91,
    CU_DEVICE_ATTRIBUTE_CAN_USE_STREAM_MEM_OPS = 92,
    CU_DEVICE_ATTRIBUTE_CAN_USE_64_BIT_STREAM_MEM_OPS = 93,
    CU_DEVICE_ATTRIBUTE_CAN_USE_STREAM_WAIT_VALUE_NOR = 94,
    CU_DEVICE_ATTRIBUTE_COOPERATIVE_LAUNCH = 95,
    CU_DEVICE_ATTRIBUTE_COOPERATIVE_MULTI_DEVICE_LAUNCH = 96,
    CU_DEVICE_ATTRIBUTE_MAX_SHARED_MEMORY_PER_BLOCK_OPTIN = 97,
    CU_DEVICE_ATTRIBUTE_CAN_FLUSH_REMOTE_WRITES = 98,
    CU_DEVICE_ATTRIBUTE_HOST_REGISTER_SUPPORTED = 99,
    CU_DEVICE_ATTRIBUTE_PAGEABLE_MEMORY_ACCESS_USES_HOST_PAGE_TABLES = 100,
    CU_DEVICE_ATTRIBUTE_DIRECT_MANAGED_MEM_ACCESS_FROM_HOST = 101,
    CU_DEVICE_ATTRIBUTE_MAX
};

/*
 * We use cudaGetDeviceProperties() to get the name of the device.
 * There should be cudaDeviceGetName() but it is not always there.
 * We do not use the other device properties because the order of
 * the fields change in the different CUDA versions.
 */
struct cudaDeviceProp {
    char name[256];
    char buff[4096];
};

typedef int cudaError_t;

typedef cudaError_t (*FUNPTR_cudaDriverGetVersion)(int* version);
typedef cudaError_t (*FUNPTR_cudaRuntimeGetVersion)(int* version);
typedef cudaError_t (*FUNPTR_cudaGetDeviceCount)(int* device_count);
typedef cudaError_t (*FUNPTR_cudaGetDeviceProperties)(
    struct cudaDeviceProp *props, int device
);
typedef cudaError_t (*FUNPTR_cudaDeviceGetAttribute)(
    int* attrib_value, enum cudaDeviceAttribute attrib, int device
);
typedef cudaError_t (*FUNPTR_cudaSetDevice)(int device);
typedef cudaError_t (*FUNPTR_cudaGetDevice)(int* device);
typedef cudaError_t (*FUNPTR_cudaDeviceReset)(void);
typedef cudaError_t (*FUNPTR_cudaDeviceCanAccessPeer)(
    int* canAccessPeer, int device, int peerDevice
);
typedef cudaError_t (*FUNPTR_cudaDeviceDisablePeerAccess)(int peerDevice);
typedef cudaError_t (*FUNPTR_cudaDeviceEnablePeerAccess)(
    int  peerDevice, unsigned int  flags
);

typedef cudaError_t (*FUNPTR_cudaMalloc)(void **devPtr, size_t size);
typedef cudaError_t (*FUNPTR_cudaFree)(void* devPtr);
typedef cudaError_t (*FUNPTR_cudaMallocHost)(void **devPtr, size_t size);
typedef cudaError_t (*FUNPTR_cudaFreeHost)(void* devPtr);
typedef cudaError_t (*FUNPTR_cudaMemcpy)(
    void *dst, const void *src, size_t count, enum cudaMemcpyKind kind
);
typedef cudaError_t (*FUNPTR_cudaMemset)(
    void* devPtr, int value, size_t count
);
typedef cudaError_t (*FUNPTR_cudaMemGetInfo)(size_t* free, size_t* total);

typedef cudaError_t (*FUNPTR_cudaGetLastError)(void);
typedef cudaError_t (*FUNPTR_cudaPeekAtLastError)(void);
typedef const char* (*FUNPTR_cudaGetErrorString)(cudaError_t error);
typedef const char* (*FUNPTR_cudaGetErrorName)(cudaError_t error);

/**
 * \brief Finds and initializes a function pointer to
 *  one of the functions in CUDA.
 * \details Function pointers are stored into the
 *  CUDAContext returned by the function CUDA().
 *  If a symbol is not found, returns NL_FALSE from the
 *  calling function.
 */
#define find_cuda_func(name)                                            \
    if(                                                                 \
        (                                                               \
            CUDA()->name =                                              \
            (FUNPTR_##name)nlFindFunction(                              \
                CUDA()->DLL_cudart,#name                                \
            )                                                           \
        ) == NULL                                                       \
    ) {                                                                 \
        nlError("nlInitExtension_CUDA: function not found", #name);     \
        return NL_FALSE;                                                \
    }

/**********************************************************/
/*      CUBLAS structures and functions                   */
/**********************************************************/

struct cublasContext;
typedef struct cublasContext *cublasHandle_t;
typedef int cublasStatus_t;

typedef enum {
    CUBLAS_SIDE_LEFT =0,
    CUBLAS_SIDE_RIGHT=1
} cublasSideMode_t;

typedef enum {
    CUBLAS_FILL_MODE_LOWER=0,
    CUBLAS_FILL_MODE_UPPER=1
} cublasFillMode_t;

typedef enum {
    CUBLAS_OP_N=0,
    CUBLAS_OP_T=1,
    CUBLAS_OP_C=2
} cublasOperation_t;

typedef enum {
    CUBLAS_DIAG_NON_UNIT=0,
    CUBLAS_DIAG_UNIT=1
} cublasDiagType_t;

typedef cublasStatus_t (*FUNPTR_cublasCreate)(cublasHandle_t* handle);
typedef cublasStatus_t (*FUNPTR_cublasDestroy)(cublasHandle_t handle);

typedef cublasStatus_t (*FUNPTR_cublasGetVersion)(
    cublasHandle_t handle, int* version
);

typedef cublasStatus_t (*FUNPTR_cublasDdot)(
    cublasHandle_t handle, int n,
    const double *x, int incx,
    const double *y, int incy,
    double *result
);

typedef cublasStatus_t (*FUNPTR_cublasDcopy)(
    cublasHandle_t handle, int n,
    const double *x, int incx,
    const double *y, int incy
);

typedef cublasStatus_t (*FUNPTR_cublasDaxpy)(
    cublasHandle_t handle, int n,
    const double* alpha,
    const double *x, int incx,
    const double *y, int incy
);

typedef cublasStatus_t (*FUNPTR_cublasDscal)(
    cublasHandle_t handle, int n,
    const double* alpha,
    const double *x, int incx
);

typedef cublasStatus_t (*FUNPTR_cublasDnrm2)(
    cublasHandle_t handle, int n,
    const double *x, int incx,
    double* result
);

typedef cublasStatus_t (*FUNPTR_cublasDdgmm)(
    cublasHandle_t handle, cublasSideMode_t mode,
    int m, int n,
    const double* A, int lda,
    const double* x, int incx,
    double* C, int ldc
);

typedef cublasStatus_t (*FUNPTR_cublasDgemv)(
    cublasHandle_t handle,
    cublasOperation_t trans,
    int m,
    int n,
    const double *alpha,
    const double *A,
    int lda,
    const double *x,
    int incx,
    const double *beta,
    double *y,
    int incy
);

typedef cublasStatus_t (*FUNPTR_cublasDtpsv)(
    cublasHandle_t handle, cublasFillMode_t uplo,
    cublasOperation_t trans, cublasDiagType_t diag,
    int n, const double *AP,
    double* x, int incx
);


/**
 * \brief Finds and initializes a function pointer to
 *  one of the functions in CUBLAS.
 * \details Function pointers are stored into the
 *  CUDAContext returned by the function CUDA().
 *  If a symbol is not found, returns NL_FALSE from the
 *  calling function. Here we use the functions suffixed
 *  by "_v2".
 */
#define find_cublas_func(name)                                          \
    if(                                                                 \
        (                                                               \
            CUDA()->name =                                              \
            (FUNPTR_##name)nlFindFunction(                              \
                CUDA()->DLL_cublas,#name "_v2"				\
            )                                                           \
        ) == NULL                                                       \
    ) {                                                                 \
        nlError("nlInitExtension_CUDA: function not found", #name);     \
        return NL_FALSE;                                                \
    }

/**
 * \brief Finds and initializes a function pointer to
 *  one of the functions in CUBLAS.
 * \details Function pointers are stored into the
 *  CUDAContext returned by the function CUDA().
 *  If a symbol is not found, returns NL_FALSE from the
 *  calling function.
 */
#define find_cublas_func_v1(name)                                       \
    if(                                                                 \
        (                                                               \
            CUDA()->name =                                              \
            (FUNPTR_##name)nlFindFunction(                              \
                CUDA()->DLL_cublas,#name                                \
            )                                                           \
        ) == NULL                                                       \
    ) {                                                                 \
        nlError("nlInitExtension_CUDA: function not found", #name);     \
        return NL_FALSE;                                                \
    }



/**********************************************************/
/*      CUSPARSE structures and functions                 */
/**********************************************************/

struct cusparseContext;
typedef struct cusparseContext *cusparseHandle_t;
typedef int cusparseStatus_t;

typedef enum {
    CUSPARSE_MATRIX_TYPE_GENERAL = 0,
    CUSPARSE_MATRIX_TYPE_SYMMETRIC = 1,
    CUSPARSE_MATRIX_TYPE_HERMITIAN = 2,
    CUSPARSE_MATRIX_TYPE_TRIANGULAR = 3
} cusparseMatrixType_t;

typedef enum {
    CUSPARSE_INDEX_BASE_ZERO = 0,
    CUSPARSE_INDEX_BASE_ONE = 1
} cusparseIndexBase_t;

typedef enum {
    CUSPARSE_OPERATION_NON_TRANSPOSE = 0,
    CUSPARSE_OPERATION_TRANSPOSE = 1,
    CUSPARSE_OPERATION_CONJUGATE_TRANSPOSE = 2
} cusparseOperation_t;

typedef cusparseStatus_t (*FUNPTR_cusparseCreate)(cusparseHandle_t* handle);
typedef cusparseStatus_t (*FUNPTR_cusparseDestroy)(cusparseHandle_t handle);
typedef cusparseStatus_t (*FUNPTR_cusparseGetVersion)(
    cusparseHandle_t handle, int* version
);

/**********************************************************/
/* CUSPARSE v10 structures and functions (generic matrix) */
/**********************************************************/

typedef enum cudaDataType_t {
    CUDA_R_16F= 2,  CUDA_C_16F= 6,
    CUDA_R_32F= 0,  CUDA_C_32F= 4,
    CUDA_R_64F= 1,  CUDA_C_64F= 5,
    CUDA_R_8I = 3,  CUDA_C_8I = 7,
    CUDA_R_8U = 8,  CUDA_C_8U = 9,
    CUDA_R_32I= 10, CUDA_C_32I= 11,
    CUDA_R_32U= 12, CUDA_C_32U= 13
} cudaDataType;

struct cusparseDnVecDescr;
struct cusparseSpMatDescr;
typedef struct cusparseDnVecDescr* cusparseDnVecDescr_t;
typedef struct cusparseSpMatDescr* cusparseSpMatDescr_t;

typedef enum {
    CUSPARSE_INDEX_16U = 1,
    CUSPARSE_INDEX_32I = 2,
    CUSPARSE_INDEX_64I = 3
} cusparseIndexType_t;

typedef cusparseStatus_t (*FUNPTR_cusparseCreateDnVec)(
    cusparseDnVecDescr_t* dnVecDescr,
    int64_t size, void* values, cudaDataType valueType
);

typedef cusparseStatus_t (*FUNPTR_cusparseDestroyDnVec)(
    cusparseDnVecDescr_t dnVecDescr
);

typedef cusparseStatus_t (*FUNPTR_cusparseDnVecSetValues)(
    cusparseDnVecDescr_t dnVecDescr, void* values
);

typedef cusparseStatus_t (*FUNPTR_cusparseCreateCsr)(
    cusparseSpMatDescr_t* spMatDescr,
    int64_t rows, int64_t cols, int64_t nnz,
    void* csrRowOffsets, void* csrColInd, void* csrValues,
    cusparseIndexType_t csrRowOffsetsType,
    cusparseIndexType_t csrColIndType,
    cusparseIndexBase_t idxBase,
    cudaDataType valueType
);

typedef FUNPTR_cusparseCreateCsr FUNPTR_cusparseCreateConstCsr;

typedef cusparseStatus_t (*FUNPTR_cusparseDestroySpMat)(
    cusparseSpMatDescr_t spMatDescr
);

typedef enum {
    CUSPARSE_SPMV_ALG_DEFAULT = 0,
    CUSPARSE_COOMV_ALG        = 1,
    CUSPARSE_CSRMV_ALG1       = 2,
    CUSPARSE_CSRMV_ALG2       = 3
} cusparseSpMVAlg_t;

typedef cusparseStatus_t  (*FUNPTR_cusparseSpMV)(
    cusparseHandle_t handle, cusparseOperation_t opA,
    const void* alpha, const cusparseSpMatDescr_t matA,
    const cusparseDnVecDescr_t vecX, const void* beta,
    const cusparseDnVecDescr_t vecY, cudaDataType computeType,
    cusparseSpMVAlg_t alg, void* externalBuffer
);

typedef cusparseStatus_t (*FUNPTR_cusparseSpMV_bufferSize)(
    cusparseHandle_t handle, cusparseOperation_t opA,
    const void* alpha, const cusparseSpMatDescr_t matA,
    const cusparseDnVecDescr_t vecX, const void* beta,
    const cusparseDnVecDescr_t vecY, cudaDataType computeType,
    cusparseSpMVAlg_t alg, size_t* bufferSize
);

typedef cusparseStatus_t (*FUNPTR_cusparseSpMV_preprocess)(
    cusparseHandle_t handle, cusparseOperation_t opA,
    const void* alpha, const cusparseSpMatDescr_t matA,
    const cusparseDnVecDescr_t vecX, const void* beta,
    const cusparseDnVecDescr_t vecY, cudaDataType computeType,
    cusparseSpMVAlg_t alg, void* externalBuffer
);

/**
 * \brief Finds and initializes a function pointer to
 *  one of the functions in CUSPARSE.
 * \details Function pointers are stored into the
 *  CUDAContext returned by the function CUDA().
 *  If a symbol is not found, returns NL_FALSE from the
 *  calling function.
 */
#define find_cusparse_func(name)                                        \
    if(                                                                 \
        (                                                               \
            CUDA()->name =                                              \
            (FUNPTR_##name)nlFindFunction(                              \
                CUDA()->DLL_cusparse,#name                              \
            )                                                           \
        ) == NULL                                                       \
    ) {                                                                 \
        nlError("nlInitExtension_CUDA : function not found", #name);    \
        return NL_FALSE;                                                \
    }

#define find_cusparse_func_quiet(name)                                  \
            CUDA()->name =                                              \
            (FUNPTR_##name)nlFindFunction(                              \
                CUDA()->DLL_cusparse,#name                              \
            )


/**********************************************************/

/**
 * \brief Per-device shared objects (CuBLAS and CuSparse handles).
 */
typedef struct {
    int devID;
    cublasHandle_t HNDL_cublas;
    cusparseHandle_t HNDL_cusparse;
} CUDADeviceContext;

static void nlInitDevice_CUDA(CUDADeviceContext* device, int dev_id);
static void nlTerminateDevice_CUDA(CUDADeviceContext* device);

/**
 * \brief The structure that stores the handle to
 *  the CUDA shared object, the function pointers
 *  and the detected version.
 */
typedef struct {
    NLdll DLL_cudart;

    FUNPTR_cudaDriverGetVersion cudaDriverGetVersion;
    FUNPTR_cudaRuntimeGetVersion cudaRuntimeGetVersion;
    FUNPTR_cudaGetDeviceCount cudaGetDeviceCount;
    FUNPTR_cudaGetDeviceProperties cudaGetDeviceProperties;
    FUNPTR_cudaDeviceGetAttribute cudaDeviceGetAttribute;
    FUNPTR_cudaSetDevice cudaSetDevice;
    FUNPTR_cudaGetDevice cudaGetDevice;
    FUNPTR_cudaDeviceReset cudaDeviceReset;
    FUNPTR_cudaDeviceCanAccessPeer cudaDeviceCanAccessPeer;
    FUNPTR_cudaDeviceEnablePeerAccess cudaDeviceEnablePeerAccess;
    FUNPTR_cudaDeviceDisablePeerAccess cudaDeviceDisablePeerAccess;
    FUNPTR_cudaMalloc cudaMalloc;
    FUNPTR_cudaFree cudaFree;
    FUNPTR_cudaMalloc cudaMallocHost;
    FUNPTR_cudaFree cudaFreeHost;
    FUNPTR_cudaMemcpy cudaMemcpy;
    FUNPTR_cudaMemset cudaMemset;
    FUNPTR_cudaMemGetInfo cudaMemGetInfo;
    FUNPTR_cudaGetLastError cudaGetLastError;
    FUNPTR_cudaPeekAtLastError cudaPeekAtLastError;
    FUNPTR_cudaGetErrorString cudaGetErrorString;
    FUNPTR_cudaGetErrorName cudaGetErrorName;

    NLdll DLL_cublas;
    FUNPTR_cublasCreate cublasCreate;
    FUNPTR_cublasDestroy cublasDestroy;
    FUNPTR_cublasGetVersion cublasGetVersion;
    FUNPTR_cublasDdot cublasDdot;
    FUNPTR_cublasDcopy cublasDcopy;
    FUNPTR_cublasDaxpy cublasDaxpy;
    FUNPTR_cublasDscal cublasDscal;
    FUNPTR_cublasDnrm2 cublasDnrm2;
    FUNPTR_cublasDdgmm cublasDdgmm;
    FUNPTR_cublasDgemv cublasDgemv;
    FUNPTR_cublasDtpsv cublasDtpsv;

    NLdll DLL_cusparse;
    FUNPTR_cusparseCreate cusparseCreate;
    FUNPTR_cusparseDestroy cusparseDestroy;
    FUNPTR_cusparseGetVersion cusparseGetVersion;
    FUNPTR_cusparseCreateDnVec cusparseCreateDnVec;
    FUNPTR_cusparseDestroyDnVec cusparseDestroyDnVec;
    FUNPTR_cusparseDnVecSetValues cusparseDnVecSetValues;
    FUNPTR_cusparseCreateCsr cusparseCreateCsr;
    FUNPTR_cusparseCreateConstCsr cusparseCreateConstCsr;
    FUNPTR_cusparseDestroySpMat cusparseDestroySpMat;
    FUNPTR_cusparseSpMV cusparseSpMV;
    FUNPTR_cusparseSpMV_bufferSize cusparseSpMV_bufferSize;
    FUNPTR_cusparseSpMV_preprocess cusparseSpMV_preprocess;

    int nb_devices;
    CUDADeviceContext* device;
    CUDADeviceContext* main_device;
} CUDAContext;

/**
 * \brief Gets the CUDA context.
 * \return a pointer to the CUDA context
 */
static CUDAContext* CUDA(void) {
    static CUDAContext context;
    static NLboolean init = NL_FALSE;
    if(!init) {
        init = NL_TRUE;
        memset(&context, 0, sizeof(context));
    }
    return &context;
}

NLboolean nlExtensionIsInitialized_CUDA(void) {
    if(
        CUDA()->DLL_cudart == NULL ||
        CUDA()->cudaDriverGetVersion == NULL ||
        CUDA()->cudaRuntimeGetVersion == NULL ||
        CUDA()->cudaGetDeviceCount == NULL ||
        CUDA()->cudaGetDeviceProperties == NULL ||
        CUDA()->cudaDeviceGetAttribute == NULL ||
        CUDA()->cudaSetDevice == NULL ||
        CUDA()->cudaGetDevice == NULL ||
        CUDA()->cudaDeviceReset == NULL ||
	CUDA()->cudaDeviceCanAccessPeer == NULL ||
	CUDA()->cudaDeviceEnablePeerAccess == NULL ||
	CUDA()->cudaDeviceDisablePeerAccess == NULL ||
        CUDA()->cudaMalloc == NULL ||
        CUDA()->cudaFree == NULL ||
        CUDA()->cudaMallocHost == NULL ||
        CUDA()->cudaFreeHost == NULL ||
        CUDA()->cudaMemcpy == NULL ||
	CUDA()->cudaMemset == NULL ||
	CUDA()->cudaMemGetInfo == NULL ||
	CUDA()->cudaGetLastError == NULL ||
	CUDA()->cudaPeekAtLastError == NULL ||
	CUDA()->cudaGetErrorString == NULL ||
	CUDA()->cudaGetErrorName == NULL ||

        CUDA()->DLL_cublas == NULL ||
        CUDA()->cublasCreate == NULL ||
        CUDA()->cublasDestroy == NULL ||
        CUDA()->cublasGetVersion == NULL ||
        CUDA()->cublasDdot == NULL ||
        CUDA()->cublasDcopy == NULL ||
        CUDA()->cublasDaxpy == NULL ||
        CUDA()->cublasDscal == NULL ||
        CUDA()->cublasDnrm2 == NULL ||
        CUDA()->cublasDdgmm == NULL ||

        CUDA()->DLL_cusparse == NULL ||
        CUDA()->cusparseCreate == NULL ||
        CUDA()->cusparseDestroy == NULL ||
        CUDA()->cusparseGetVersion == NULL
    ) {
        return NL_FALSE;
    }
    return NL_TRUE;
}

static void nlTerminateExtension_CUDA(void) {
    if(!nlExtensionIsInitialized_CUDA()) {
        return;
    }

    for(int dev_id=0; dev_id<CUDA()->nb_devices; ++dev_id) {
	nlTerminateDevice_CUDA(&(CUDA()->device[dev_id]));
    }

    nlCloseDLL(CUDA()->DLL_cusparse);
    nlCloseDLL(CUDA()->DLL_cublas);
    nlCloseDLL(CUDA()->DLL_cudart);

    memset(CUDA(), 0, sizeof(CUDAContext));
}

/**************************************************************************/

/**
 * \brief Finds the number of cores from the major and minor versions of the
 *  shader model.
 * \details Highly inspired by the helpers library in CUDA examples,
 *  see https://github.com/NVIDIA/cuda-samples/blob/master/Common/helper_cuda.h
 */
static int ConvertSMVer2Cores(int major, int minor) {
    /* Defines for GPU Architecture types (using the SM version
       to determine the # of cores per SM */
    typedef struct {
        int SM; /* 0xMm (hexadecimal notation),
                   M = SM Major version,
                   and m = SM minor version */
        int Cores;
    } sSMtoCores;

    sSMtoCores nGpuArchCoresPerSM[] = {
        { 0x10,  8 }, /* Tesla Generation   (SM 1.0) G80 class    */
        { 0x11,  8 }, /* Tesla Generation   (SM 1.1) G8x class    */
        { 0x12,  8 }, /* Tesla Generation   (SM 1.2) G9x class    */
        { 0x13,  8 }, /* Tesla Generation   (SM 1.3) GT200 class  */
        { 0x20, 32 }, /* Fermi Generation   (SM 2.0) GF100 class  */
        { 0x21, 48 }, /* Fermi Generation   (SM 2.1) GF10x class  */
        { 0x30, 192}, /* Kepler Generation  (SM 3.0) GK10x class  */
        { 0x32, 192}, /* Kepler Generation  (SM 3.0) */
        { 0x35, 192}, /* Kepler Generation  (SM 3.5) GK11x class  */
	{ 0x37, 192},
        { 0x50, 128}, /* Maxwell Generation (SM 5.0) GM10x class
                         (yes, #cores smaller than with 3.x)  */
        { 0x52, 128}, /* Maxwell Generation (SM 5.2) GM20x class  */
        { 0x53, 128},
        { 0x60, 64 }, /* Pascal Generation  (SM 6.0) GP100,GP102
                         (yes, 64, but GP100 has superfast double precision) */
        { 0x61, 128}, /* Pascal Generation  (SM 6.1) GP104 class
                         (but FP64 runs as 1/32 FP32 speed) */
	{ 0x62, 128},
        { 0x70, 64 }, /* Volta Generation (SM 7.0)
		         yes, nb cores decreased in SM 7.x  */
        { 0x72, 64 }, /* Volta Generation (SM 7.2) */
        { 0x75, 64 }, /* Volta Generation (SM 7.5) */
        { 0x80, 64 }, /* Ampere Generation (SM 8.0) A30,A100 */
        { 0x86, 128}, /* Ampere Generation (SM 8.6) A40 */
        { 0x87, 128}, /* Ampere Generation (SM 8.7) */
        { 0x89, 128}, /* Ada Generation*/
        { 0x90, 128}, /* Hopper Generation */
        {   -1, -1 }
    };
    int index = 0;
    if(major == 0 && minor == 0) {
        return 0;
    }
    while (nGpuArchCoresPerSM[index].SM != -1) {
        if (nGpuArchCoresPerSM[index].SM == ((major << 4) + minor)) {
            return nGpuArchCoresPerSM[index].Cores;
        }
        index++;
    }
    /* If we don't find the values, we use a default value (64) */
    nl_printf(
        "MapSMtoCores for SM %d.%d is undefined.  Default to use %d Cores/SM\n",
        major, minor, 64
    );
    return 64;
}

/**
 * \brief Gets the double-precision performance for a device.
 * \param[in] device the device
 * \return the peak GFlops double-precision performance of the device,
 *  or 0.0 if device's compute mode is prohibited.
 */
static double getDeviceDoublePrecisionGFlops(int device) {
    int compute_mode;
    int compute_capability_major;
    int compute_capability_minor;
    int cores_per_multiprocessor;
    int multiprocessor_count;
    int double_precision_perf_ratio;
    int clock_rate;
    double result = 0.0;

    CUDA()->cudaDeviceGetAttribute(
        &compute_mode,
        CU_DEVICE_ATTRIBUTE_COMPUTE_MODE,
        device
    );

    if(compute_mode == cudaComputeModeProhibited) {
        return 0.0;
    }

    CUDA()->cudaDeviceGetAttribute(
        &compute_capability_major,
        CU_DEVICE_ATTRIBUTE_COMPUTE_CAPABILITY_MAJOR,
        device
    );

    CUDA()->cudaDeviceGetAttribute(
        &compute_capability_minor,
        CU_DEVICE_ATTRIBUTE_COMPUTE_CAPABILITY_MINOR,
        device
    );

    CUDA()->cudaDeviceGetAttribute(
        &multiprocessor_count,
        CU_DEVICE_ATTRIBUTE_MULTIPROCESSOR_COUNT,
        device
    );

    CUDA()->cudaDeviceGetAttribute(
        &double_precision_perf_ratio,
        CU_DEVICE_ATTRIBUTE_SINGLE_TO_DOUBLE_PRECISION_PERF_RATIO,
        device
    );

    CUDA()->cudaDeviceGetAttribute(
        &clock_rate,
        CU_DEVICE_ATTRIBUTE_CLOCK_RATE,
        device
    );

    cores_per_multiprocessor = ConvertSMVer2Cores(
        compute_capability_major, compute_capability_minor
    );

    /*
     * I need this 2.0 factor to match the specs,
     * does it mean a CUDA core does two FPs per cycle ?
     * They probably count FMAs as 2 ops for the "peak perf"
     * stat...
     */
    result = 2.0 *
        ((double)(clock_rate) / (1024.0 * 1024.0)) *
        (double)(multiprocessor_count) *
        (double)(cores_per_multiprocessor) /
        (double)(double_precision_perf_ratio) ;

    return result;
}

/**
 * \brief Gets the device ID with the maximum double precision
 *  performance.
 * \return the ID of the fastest device or -1 is no device is
 *  available.
 */
static int getBestDeviceID(void) {
    int result = -1;
    double fastest_GFlops = 0.0;
    int device_count;
    int retval = CUDA()->cudaGetDeviceCount(&device_count);
    int device;
    double device_GFlops;
    int driver_ver;
    int runtime_ver;
    if(retval == 35) {
        nl_printf("Error: Driver/CUDA versions mismatch\n");
        retval = CUDA()->cudaDriverGetVersion(&driver_ver);
        nl_printf("cudaDriverGetVersion()   retval=%d\n",retval);
        retval = CUDA()->cudaRuntimeGetVersion(&runtime_ver);
        nl_printf("cudaRuntimeGetVersion()  retval=%d\n",retval);

        nl_printf("  Driver  version=%d\n",driver_ver);
        nl_printf("  Runtime version=%d\n",driver_ver);
        return result;
    }
    for(device=0; device<device_count; ++device) {
        device_GFlops = getDeviceDoublePrecisionGFlops(device);
        if(device_GFlops > fastest_GFlops) {
            fastest_GFlops = device_GFlops;
            result = device;
        }
    }
    return result;
}

/**************************************************************************/

/**
 * \brief A function to implement the nlCUDACheck macro
 */
static void nlCUDACheckImpl(int status, int line) {
    cudaError_t last_error = CUDA()->cudaGetLastError();
    if(status != 0) {
        nl_fprintf(stderr,"nl_cuda.c:%d fatal error %d\n",line, status);
	nl_fprintf(
	    stderr,"%s (%s)\n",
	    CUDA()->cudaGetErrorName(last_error),
	    CUDA()->cudaGetErrorString(last_error)
	);
        CUDA()->cudaDeviceReset();
        exit(-1);
    }
}

/**
 * \brief A macro to check all calls to CUDA api functions
 * \details All code to CUDA api functions return a status code.
 *  This macro is meant to wrap each call to CUDA api, checks the
 *  result. If there was an error, it displays an error message
 *  with the line number of the call that raised the error,
 *  then exits the program.
 */
#define nlCUDACheck(status) nlCUDACheckImpl(status, __LINE__)

/**************************************************************************/

static void nlDisplayDeviceInformation(int dev_id, NLboolean detailed) {
    static struct cudaDeviceProp deviceProp;
    int compute_capability_major;
    int compute_capability_minor;
    int multiprocessor_count;
    int max_shared_mem_per_block;
    int max_shared_mem_per_multiprocessor;
    int max_regs_per_block;
    int max_regs_per_multiprocessor;
    int warp_size;
    int double_precision_perf_ratio;
    size_t total_RAM, free_RAM;
    int unified_addressing;
    int can_map_host_memory;
    int dev_id_bkp;
    double float64Gflops = getDeviceDoublePrecisionGFlops(dev_id);

    nlCUDACheck(CUDA()->cudaGetDevice(&dev_id_bkp));
    nlCUDACheck(CUDA()->cudaSetDevice(dev_id));
    nlCUDACheck(CUDA()->cudaGetDeviceProperties(&deviceProp, dev_id));


    nlCUDACheck(
	CUDA()->cudaDeviceGetAttribute(
	    &compute_capability_major,
	    CU_DEVICE_ATTRIBUTE_COMPUTE_CAPABILITY_MAJOR,
	    dev_id
	)
    );

    nlCUDACheck(
	CUDA()->cudaDeviceGetAttribute(
	    &compute_capability_minor,
	    CU_DEVICE_ATTRIBUTE_COMPUTE_CAPABILITY_MINOR,
	    dev_id
	)
    );

    nlCUDACheck(
	CUDA()->cudaDeviceGetAttribute(
	    &multiprocessor_count,
	    CU_DEVICE_ATTRIBUTE_MULTIPROCESSOR_COUNT,
	    dev_id
	)
    );

    nlCUDACheck(
	CUDA()->cudaDeviceGetAttribute(
	    &max_shared_mem_per_block,
	    CU_DEVICE_ATTRIBUTE_MAX_SHARED_MEMORY_PER_BLOCK,
	    dev_id
	)
    );

    nlCUDACheck(
	CUDA()->cudaDeviceGetAttribute(
	    &max_shared_mem_per_multiprocessor,
	    CU_DEVICE_ATTRIBUTE_MAX_SHARED_MEMORY_PER_MULTIPROCESSOR,
	    dev_id
	)
    );

    nlCUDACheck(
	CUDA()->cudaDeviceGetAttribute(
	    &max_regs_per_block,
	    CU_DEVICE_ATTRIBUTE_MAX_REGISTERS_PER_BLOCK,
	    dev_id
	)
    );

    nlCUDACheck(
	CUDA()->cudaDeviceGetAttribute(
	    &max_regs_per_multiprocessor,
	    CU_DEVICE_ATTRIBUTE_MAX_REGISTERS_PER_MULTIPROCESSOR,
	    dev_id
	)
    );

    nlCUDACheck(
	CUDA()->cudaDeviceGetAttribute(
	    &warp_size,
	    CU_DEVICE_ATTRIBUTE_WARP_SIZE,
	    dev_id
	)
    );

    nlCUDACheck(
	CUDA()->cudaDeviceGetAttribute(
	    &double_precision_perf_ratio,
	    CU_DEVICE_ATTRIBUTE_SINGLE_TO_DOUBLE_PRECISION_PERF_RATIO,
	    dev_id
	)
    );

    nlCUDACheck(
	CUDA()->cudaDeviceGetAttribute(
	    &can_map_host_memory,
	    CU_DEVICE_ATTRIBUTE_CAN_MAP_HOST_MEMORY,
	    dev_id
	)
    );

    nlCUDACheck(
	CUDA()->cudaDeviceGetAttribute(
	    &unified_addressing,
	    CU_DEVICE_ATTRIBUTE_UNIFIED_ADDRESSING,
	    dev_id
	)
    );

    nlCUDACheck(
	CUDA()->cudaMemGetInfo(&free_RAM, &total_RAM)
    );

    nl_printf("OpenNL CUDA[%d]: %s\n", dev_id, deviceProp.name);

    nl_printf(
	"OpenNL CUDA[%d]: total RAM: %f GB   free RAM: %f GB\n",
	dev_id, (double)total_RAM/1e9, (double)free_RAM/1e9
    );

    if(float64Gflops > 1000.0) {
	nl_printf(
	    "OpenNL CUDA[%d]: theoretical peak float64 perf: %f TFlops\n",
	    dev_id, float64Gflops / 1000.0
	);
    } else {
	nl_printf(
	    "OpenNL CUDA[%d]: theoretical peak float64 perf: %f GFlops\n",
	    dev_id, float64Gflops
	);
    }

    if(detailed) {
	nl_printf(
	    "OpenNL CUDA[%d]: SM %d.%d compute capabilities\n",
	    dev_id, compute_capability_major, compute_capability_minor
	);

	nl_printf(
	    "OpenNL CUDA[%d]: %d Multi-Processors, "
	    "%d cores per Multi-Processor, SM %d.%d compute capabilities\n",
	    dev_id, multiprocessor_count,
	    ConvertSMVer2Cores(
		compute_capability_major, compute_capability_minor
	    ),
	    compute_capability_major, compute_capability_minor
	);


	nl_printf(
	    "OpenNL CUDA[%d]: %d kB shared mem. per block, %d per MP\n",
	    dev_id,
	    (int)(max_shared_mem_per_block / 1024),
	    (int)(max_shared_mem_per_multiprocessor / 1024)
	);

	nl_printf(
	    "OpenNL CUDA[%d]: %d regs. per block, %d per MP\n",
	    dev_id, max_regs_per_block, max_regs_per_multiprocessor
	);

	nl_printf("OpenNL CUDA[%d]: warpsize = %d\n", dev_id, warp_size);

	nl_printf(
	    "OpenNL CUDA[%d]: double precision perf ratio = %d\n",
	    dev_id, double_precision_perf_ratio
	);
	nl_printf(
	    "OpenNL CUDA[%d]: can map host memory: %d\n",
	    dev_id, can_map_host_memory
	);
	nl_printf(
	    "OpenNL CUDA[%d]: unified_addressing: %d\n",
	    dev_id, unified_addressing
	);
    }

    nlCUDACheck(CUDA()->cudaSetDevice(dev_id_bkp));
}

/**************************************************************************/

void nlInitDevice_CUDA(CUDADeviceContext* device, int dev_id) {
    device->devID = dev_id;
    nlDisplayDeviceInformation(dev_id,NL_FALSE);
    nlCUDACheck(CUDA()->cudaSetDevice(dev_id));
    nlCUDACheck(CUDA()->cublasCreate(&device->HNDL_cublas));
    nlCUDACheck(CUDA()->cusparseCreate(&device->HNDL_cusparse));
}

void nlTerminateDevice_CUDA(CUDADeviceContext* device) {
    nlCUDACheck(CUDA()->cudaSetDevice(device->devID));
    nlCUDACheck(CUDA()->cusparseDestroy(device->HNDL_cusparse));
    nlCUDACheck(CUDA()->cublasDestroy(device->HNDL_cublas));
    nlCUDACheck(CUDA()->cudaDeviceReset());
    memset(device, 0, sizeof(CUDADeviceContext));
}

/**************************************************************************/


#ifdef NL_OS_UNIX
#  define LIBPREFIX "lib"
#  ifdef NL_OS_APPLE
#      define LIBEXTENSION ".dylib"
#  else
#      define LIBEXTENSION ".so"
#  endif
#else
#  define LIBPREFIX
#  define LIBEXTENSION ".dll"
#endif

/**************************************************************************/

NLboolean nlInitExtension_CUDA(void) {
    int cublas_version;
    int cusparse_version;
    int compute_capability_major;
    int compute_capability_minor;
    int main_dev_id;
    int can_access_peer;

    NLenum flags = NL_LINK_LAZY | NL_LINK_GLOBAL;
    if(nlCurrentContext == NULL || !nlCurrentContext->verbose) {
        flags |= NL_LINK_QUIET;
    }

    if(nlExtensionIsInitialized_CUDA()) {
        return NL_TRUE;
    }

    CUDA()->DLL_cudart = nlOpenDLL(
        LIBPREFIX "cudart" LIBEXTENSION, flags
    );

    find_cuda_func(cudaDriverGetVersion);
    find_cuda_func(cudaRuntimeGetVersion);
    find_cuda_func(cudaGetDeviceCount);
    find_cuda_func(cudaGetDeviceProperties);
    find_cuda_func(cudaDeviceGetAttribute);
    find_cuda_func(cudaSetDevice);
    find_cuda_func(cudaGetDevice);
    find_cuda_func(cudaDeviceReset);
    find_cuda_func(cudaDeviceCanAccessPeer);
    find_cuda_func(cudaDeviceEnablePeerAccess);
    find_cuda_func(cudaDeviceDisablePeerAccess);
    find_cuda_func(cudaMalloc);
    find_cuda_func(cudaFree);
    find_cuda_func(cudaMallocHost);
    find_cuda_func(cudaFreeHost);
    find_cuda_func(cudaMemcpy);
    find_cuda_func(cudaMemset);
    find_cuda_func(cudaMemGetInfo);
    find_cuda_func(cudaGetLastError);
    find_cuda_func(cudaPeekAtLastError);
    find_cuda_func(cudaGetErrorString);
    find_cuda_func(cudaGetErrorName);

    main_dev_id = getBestDeviceID();

    if(main_dev_id == -1) {
        nl_fprintf(stderr,"OpenNL CUDA: could not find a CUDA device\n");
        return NL_FALSE;
    }

    nlCUDACheck(
	CUDA()->cudaDeviceGetAttribute(
	    &compute_capability_major,
	    CU_DEVICE_ATTRIBUTE_COMPUTE_CAPABILITY_MAJOR,
	    main_dev_id
	)
    );

    nlCUDACheck(
	CUDA()->cudaDeviceGetAttribute(
	    &compute_capability_minor,
	    CU_DEVICE_ATTRIBUTE_COMPUTE_CAPABILITY_MINOR,
	    main_dev_id
	)
    );

    if ((compute_capability_major * 0x10 + compute_capability_minor) < 0x11 ) {
	nl_fprintf(
	    stderr,
	    "OpenNL CUDA requires a minimum CUDA compute 1.1 capability\n"
	);
	CUDA()->cudaDeviceReset();
	return NL_FALSE;
    }

    CUDA()->DLL_cublas = nlOpenDLL(
        LIBPREFIX "cublas" LIBEXTENSION, flags
    );

    find_cublas_func(cublasCreate);
    find_cublas_func(cublasDestroy);
    find_cublas_func(cublasGetVersion);
    find_cublas_func(cublasDdot);
    find_cublas_func(cublasDaxpy);
    find_cublas_func(cublasDcopy);
    find_cublas_func(cublasDscal);
    find_cublas_func(cublasDnrm2);
    find_cublas_func(cublasDgemv);
    find_cublas_func(cublasDtpsv);
    find_cublas_func_v1(cublasDdgmm);

    CUDA()->DLL_cusparse = nlOpenDLL(
        LIBPREFIX "cusparse" LIBEXTENSION, flags
    );
    find_cusparse_func(cusparseCreate);
    find_cusparse_func(cusparseDestroy);
    find_cusparse_func(cusparseGetVersion);
    find_cusparse_func(cusparseCreateDnVec);
    find_cusparse_func(cusparseDestroyDnVec);
    find_cusparse_func(cusparseDnVecSetValues);
    find_cusparse_func(cusparseCreateCsr);
    find_cusparse_func(cusparseDestroySpMat);
    find_cusparse_func(cusparseSpMV);
    find_cusparse_func(cusparseSpMV_bufferSize);
    find_cusparse_func_quiet(cusparseSpMV_preprocess);
    find_cusparse_func_quiet(cusparseCreateConstCsr);

    if(CUDA()->cusparseCreateConstCsr != NULL) {
	nl_printf("OpenNL CUDA: has cusparseCreateConstCsr()\n");
    } else {
	nl_printf(
	    "OpenNL CUDA: does not have cusparseCreateConstCsr()"
	    "  (can do without it)\n"
	);
	CUDA()->cusparseCreateConstCsr = CUDA()->cusparseCreateCsr;
    }

    if(CUDA()->cusparseSpMV_preprocess != NULL) {
	nl_printf("OpenNL CUDA: has cusparseSpMV_preprocess()\n");
    } else {
	nl_printf(
	    "OpenNL CUDA: does not have cusparseSpMV_preprocess()"
	    " (can do without it)\n"
	);
    }

    nlCUDACheck(CUDA()->cudaGetDeviceCount(&CUDA()->nb_devices));
    CUDA()->device = malloc(
	sizeof(CUDADeviceContext)*(size_t)(CUDA()->nb_devices)
    );

    for(int dev_id=0; dev_id<CUDA()->nb_devices; ++dev_id) {
	nlInitDevice_CUDA(&(CUDA()->device[dev_id]),dev_id);
    }
    CUDA()->main_device = &(CUDA()->device[main_dev_id]);

    for(int dev_id=0; dev_id<CUDA()->nb_devices; ++dev_id) {
	if(dev_id != main_dev_id) {
	    nlCUDACheck(
		CUDA()->cudaDeviceCanAccessPeer(
		    &can_access_peer, dev_id, main_dev_id
		)
	    );
	    if(can_access_peer) {
		nl_printf("OpenNL CUDA[%d]: can access peer", dev_id);
	    } else {
		nl_printf("OpenNL CUDA[%d]: cannot access peer", dev_id);
	    }
	}
    }

    if(!nlExtensionIsInitialized_CUDA()) {
        return NL_FALSE;
    }

    nlCUDACheck(
	CUDA()->cublasGetVersion(
	    CUDA()->main_device->HNDL_cublas, &cublas_version
	)
    );
    nl_printf("OpenNL CUDA: cublas version = %d\n", cublas_version);


    nlCUDACheck(
	CUDA()->cusparseGetVersion(
	    CUDA()->main_device->HNDL_cusparse, &cusparse_version
	)
    );
    nl_printf("OpenNL CUDA: cusparse version = %d\n", cusparse_version);


    atexit(nlTerminateExtension_CUDA);
    return NL_TRUE;

}

/**************************************************************************/

/**
 * Abstract matrix interface for a CRS matrix stored on the GPU.
 */
typedef struct NLCUDASparseMatrixStruct {

    /* common NLMatrix fields */
    NLuint m;
    NLuint n;
    NLenum type;
    NLDestroyMatrixFunc destroy_func;
    NLMultMatrixVectorFunc mult_func;

    /* CuSparse data structures and work space */
    void* dummy; /* former CUDAV9 handle (no longer used) */
    cusparseSpMatDescr_t descr;
    cusparseDnVecDescr_t X;
    cusparseDnVecDescr_t Y;
    NLboolean work_init;
    void* work;
    size_t work_size;

    /* CRS matrix in device memory */
    NLuint_big nnz;
    int* colind;
    int* rowptr;
    double* val;

    /* Management of multi-slice matrices,
     * used when NNZ is larger than NL_MAX_SLICE_SIZE
     * then matrix is stored as a linked-list of "slices"
     * each slice corresponds to a slice with the rows
     * [row_offset .. row_offset+m] of the matrix.
     * In a multi-slice matrix, the "master" stores:
     * - colind and val
     * - descriptor for X
     * Each slice stores:
     * - rowptr for the slice
     * - descriptor for Y with slice offset
     */
    struct NLCUDASparseMatrixStruct* master;     /* for slices */
    struct NLCUDASparseMatrixStruct* next_slice; /* for master & slices */
    NLuint row_offset; /* for slices */
    NLuint nb_slices;  /* for master */
} NLCUDASparseMatrix;


/**
 * \brief Deallocates just the CRS part of a CUDA matrix.
 */
static void nlCRSMatrixCUDADestroyCRS(NLCUDASparseMatrix* Mcuda) {
    if(!nlExtensionIsInitialized_CUDA()) {
        return;
    }
    /* delete CRS in slices (recursively) if any */
    if(Mcuda->next_slice != NULL) {
	nlCRSMatrixCUDADestroyCRS(Mcuda->next_slice);
    }
    if(Mcuda->colind != NULL) {
	if(Mcuda->master == NULL) { /* only master owns colind */
	    nlCUDACheck(CUDA()->cudaFree(Mcuda->colind));
	}
        Mcuda->colind = NULL;
    }
    if(Mcuda->rowptr != NULL) { /* each slice has its own rowptr */
        nlCUDACheck(CUDA()->cudaFree(Mcuda->rowptr));
        Mcuda->rowptr = NULL;
    }
    if(Mcuda->val != NULL) {
	if(Mcuda->master == NULL) { /* only master slice owns val */
	    nlCUDACheck(CUDA()->cudaFree(Mcuda->val));
	}
        Mcuda->val = NULL;
    }
}

static void nlCRSMatrixCUDADestroy(NLCUDASparseMatrix* Mcuda) {
    if(!nlExtensionIsInitialized_CUDA()) {
        return;
    }
    /* delete slices (recursively) if any */
    if(Mcuda->next_slice != NULL) {
	nlCRSMatrixCUDADestroy(Mcuda->next_slice);
	Mcuda->next_slice = NULL;
    }

    nlCRSMatrixCUDADestroyCRS(Mcuda);
    nlCUDACheck(CUDA()->cusparseDestroySpMat(Mcuda->descr));
    if(Mcuda->X != NULL) {
	/* each slice has its own X descriptor */
	nlCUDACheck(CUDA()->cusparseDestroyDnVec(Mcuda->X));
    }
    if(Mcuda->Y != NULL) {
	/* each slice has its own Y descriptor */
	nlCUDACheck(CUDA()->cusparseDestroyDnVec(Mcuda->Y));
    }
    if(Mcuda->work != NULL) {
	/* each slice has its own workspace */
        nlCUDACheck(CUDA()->cudaFree(Mcuda->work));
    }
    memset(Mcuda, 0, sizeof(*Mcuda));
}

/**
 * \brief computes a sparse matrix vector product
 *   for a single slice of a matrix.
 * \details Computes y <= alpha * Mcuda * X + beta * y
 */
static void nlCRSMatrixCUDASliceSpMV(
    NLCUDASparseMatrix* Mcuda, const double* x, double* y,
    double alpha, double beta
) {
    const cusparseSpMVAlg_t algo = CUSPARSE_SPMV_ALG_DEFAULT;
                                  /* or CUSPARSE_CSRMV_ALG2 */

    /*
     * Apply offset when multiplying a slice of a multi-slice matrix
     */
    y += Mcuda->row_offset;

    if(Mcuda->X == NULL) {
        nlCUDACheck(
            CUDA()->cusparseCreateDnVec(
                &Mcuda->X, Mcuda->n, (void*)x, CUDA_R_64F
            )
        );
    } else {
        nlCUDACheck(CUDA()->cusparseDnVecSetValues(Mcuda->X, (void*)x));
    }
    if(Mcuda->Y == NULL) {
        nlCUDACheck(
	    CUDA()->cusparseCreateDnVec(&Mcuda->Y, Mcuda->m, y, CUDA_R_64F)
        );
    } else {
        nlCUDACheck(CUDA()->cusparseDnVecSetValues(Mcuda->Y, y));
    }
    if(!Mcuda->work_init) {
        nlCUDACheck(
            CUDA()->cusparseSpMV_bufferSize(
                CUDA()->main_device->HNDL_cusparse,
                CUSPARSE_OPERATION_NON_TRANSPOSE,
                &alpha,
                Mcuda->descr,
                Mcuda->X,
                &beta,
                Mcuda->Y,
                CUDA_R_64F,
                algo,
                &Mcuda->work_size
            )
        );
        if(Mcuda->work_size != 0) {
            nlCUDACheck(
                CUDA()->cudaMalloc(&Mcuda->work,Mcuda->work_size)
            );
        }
        Mcuda->work_init = NL_TRUE;
	if(CUDA()->cusparseSpMV_preprocess != NULL) {
	    nlCUDACheck(
		CUDA()->cusparseSpMV_preprocess(
		    CUDA()->main_device->HNDL_cusparse,
		    CUSPARSE_OPERATION_NON_TRANSPOSE,
		    &alpha,
		    Mcuda->descr,
		    Mcuda->X,
		    &beta,
		    Mcuda->Y,
		    CUDA_R_64F,
		    algo,
		    Mcuda->work
		)
	    );
	}
    }
    nlCUDACheck(
        CUDA()->cusparseSpMV(
            CUDA()->main_device->HNDL_cusparse,
            CUSPARSE_OPERATION_NON_TRANSPOSE,
            &alpha,
            Mcuda->descr,
            Mcuda->X,
            &beta,
            Mcuda->Y,
            CUDA_R_64F,
            algo,
            Mcuda->work
        )
    );
    nlCUDABlas()->flops += (NLulong)(2*Mcuda->nnz);
}

void nlCUDAMatrixSpMV(
    NLMatrix M, const double* x, double* y, double alpha, double beta
) {
    NLCUDASparseMatrix* Mcuda = (NLCUDASparseMatrix*)M;
    /* single-slice matrix */
    if(Mcuda->next_slice == NULL) {
	nlCRSMatrixCUDASliceSpMV(Mcuda, x, y, alpha, beta);
	return;
    }

    /* multi-slice matrix */
    for(
	Mcuda = Mcuda->next_slice;
	Mcuda != NULL;
	Mcuda = Mcuda->next_slice
    ) {
	/*
	 * Note: each slice computes a different part of y
	 */
	nlCRSMatrixCUDASliceSpMV(Mcuda, x, y, alpha, beta);
    }
}

static void nlCRSMatrixCUDAMult(
    NLCUDASparseMatrix* Mcuda, const double* x, double* y
) {
    nlCUDAMatrixSpMV((NLMatrix)Mcuda, x, y, 1.0, 0.0);
}

#ifdef GARGANTUA
/**
 * \brief Converts in-place an array of 64-bit ints to 32-bit ints
 */
static void int64_to_int32(void* data, size_t N) {
    NLuint_big* from = (NLuint_big*)data;
    NLuint* to = (NLuint*)data;
    for(size_t i=0; i<N; ++i) {
	*to++ = (NLuint)*from++;
    }
}

/**
 * \brief Converts in-place an array of 32-bit ints to 64-bit ints
 */
static void int32_to_int64(void* data, size_t N) {
    NLuint_big* to = (NLuint_big*)data + N - 1;
    NLuint* from = (NLuint*)data + N - 1;
    for(size_t i=0; i<N; ++i) {
	*to-- = (NLuint_big)*from--;
    }
}
#endif

/*
 * Maximum slice size. It is limited by the maximum size that
 * one can CudaMalloc (2 GB), this makes 256 M entries
 * (size is determined by the VAL array)
 */
#define NL_MAX_SLICE_SIZE (256u*1024u*1024u)

/**
 * \brief Decomposes a CRS matrix into multiple slices.
 * \details The row pointers of each slice are smaller than NL_MAX_SLICE_SIZE
 * \param[in] master a pointer to the master NLCUDASParseMatrix
 * \param[in] CRS a pointer to the CRS matrix to be sliced
 * \param[in] row_offset the first row of the slice
 */
static NLCUDASparseMatrix* CreateCUDASlicesFromCRSMatrixSlices(
    NLCUDASparseMatrix* master,
    NLCRSMatrix* CRS,
    NLuint row_offset
) {
    NLCUDASparseMatrix* Mcuda = NL_NEW(NLCUDASparseMatrix);
    NLuint* rowptr = NULL;
    size_t rowptr_sz = 0;
    double t0;

    Mcuda->type=NL_MATRIX_OTHER;
    Mcuda->destroy_func=(NLDestroyMatrixFunc)nlCRSMatrixCUDADestroy;
    Mcuda->mult_func=(NLMultMatrixVectorFunc)nlCRSMatrixCUDAMult;
    Mcuda->master = master;
    Mcuda->n = master->n;
    Mcuda->row_offset = row_offset;
    ++master->nb_slices;

    for(NLuint i=row_offset; i<CRS->m; ++i) {
	NLuint_big row_len = CRS->rowptr[i+1] - CRS->rowptr[i];
	if(Mcuda->nnz+row_len > NL_MAX_SLICE_SIZE) {
	    break;
	}
	Mcuda->m++;
	Mcuda->nnz += row_len;
    }

    /* apply offsets to row pointers and send them to CUDA */
    rowptr = NL_NEW_ARRAY(NLuint, Mcuda->m+1);
    for(NLuint i=0; i<=Mcuda->m; ++i) {
	rowptr[i] = (NLuint)(
	    CRS->rowptr[i+row_offset] - CRS->rowptr[row_offset]
	);
    }
    rowptr_sz = (size_t)(Mcuda->m+1)*sizeof(NLuint);
    nlCUDACheck(CUDA()->cudaMalloc((void**)&Mcuda->rowptr,rowptr_sz));
    nlCUDACheck(CUDA()->cudaMemcpy(
                    Mcuda->rowptr, rowptr, rowptr_sz, cudaMemcpyHostToDevice)
               );
    NL_DELETE_ARRAY(rowptr);

    Mcuda->colind = master->colind + CRS->rowptr[row_offset];
    Mcuda->val = master->val + CRS->rowptr[row_offset];

    t0 = nlCurrentTime();

    nlCUDACheck(
        CUDA()->cusparseCreateConstCsr(
            &Mcuda->descr,
            Mcuda->m,
            Mcuda->n,
            (int64_t)Mcuda->nnz,
            Mcuda->rowptr,
            Mcuda->colind,
            Mcuda->val,
            CUSPARSE_INDEX_32I,
            CUSPARSE_INDEX_32I,
            CUSPARSE_INDEX_BASE_ZERO,
            CUDA_R_64F
        )
    );

    nlCUDABlas()->aux_time += (nlCurrentTime() - t0);

    /*
     * If there are still rows in the CRS matrix, create new slices (recursively)
     */
    if(row_offset + Mcuda->m < CRS->m) {
	Mcuda->next_slice = CreateCUDASlicesFromCRSMatrixSlices(
	    master, CRS, row_offset + Mcuda->m
	);
    }
    return Mcuda;
}

NLMatrix nlCUDAMatrixNewFromCRSMatrix(NLMatrix M_in) {
    NLCUDASparseMatrix* Mcuda = NL_NEW(NLCUDASparseMatrix);
    NLCRSMatrix* M = (NLCRSMatrix*)(M_in);
    size_t colind_sz, rowptr_sz, val_sz;
    double t0;
    nl_assert(M_in->type == NL_MATRIX_CRS);
    Mcuda->m = M->m;
    Mcuda->n = M->n;
    Mcuda->nnz = nlCRSMatrixNNZ(M);

    Mcuda->type=NL_MATRIX_OTHER;
    Mcuda->destroy_func=(NLDestroyMatrixFunc)nlCRSMatrixCUDADestroy;
    Mcuda->mult_func=(NLMultMatrixVectorFunc)nlCRSMatrixCUDAMult;

    colind_sz = (size_t)Mcuda->nnz*sizeof(NLuint);
    val_sz    = (size_t)Mcuda->nnz*sizeof(double);

    nlCUDACheck(CUDA()->cudaMalloc((void**)&Mcuda->colind,colind_sz));
    nlCUDACheck(CUDA()->cudaMalloc((void**)&Mcuda->val,val_sz));
    nlCUDACheck(CUDA()->cudaMemcpy(
                    Mcuda->colind, M->colind, colind_sz, cudaMemcpyHostToDevice)
               );
    nlCUDACheck(CUDA()->cudaMemcpy(
                    Mcuda->val, M->val, val_sz, cudaMemcpyHostToDevice)
               );

    /* Need to decompose matrix into slices if arrays are too large */
    if(Mcuda->nnz > NL_MAX_SLICE_SIZE) {
	Mcuda->next_slice = CreateCUDASlicesFromCRSMatrixSlices(
	    Mcuda, M, 0
	);
	return (NLMatrix)Mcuda;
    }

    /*
     * At this point, everything can fit in a single slice, stored
     * in the CUDASparseMatrix structure.
     */

#ifdef GARGANTUA
    /* convert the rowptr array into 32 bits in-place */
    int64_to_int32(M->rowptr, M->m+1);
#endif

    rowptr_sz = (size_t)(Mcuda->m+1)*sizeof(NLuint);

    nlCUDACheck(CUDA()->cudaMalloc((void**)&Mcuda->rowptr,rowptr_sz));
    nlCUDACheck(CUDA()->cudaMemcpy(
                    Mcuda->rowptr, M->rowptr, rowptr_sz, cudaMemcpyHostToDevice)
               );

    t0 = nlCurrentTime();

    nlCUDACheck(
        CUDA()->cusparseCreateConstCsr(
            &Mcuda->descr,
            Mcuda->m,
            Mcuda->n,
            (int64_t)Mcuda->nnz,
            Mcuda->rowptr,
            Mcuda->colind,
            Mcuda->val,
            CUSPARSE_INDEX_32I,
            CUSPARSE_INDEX_32I,
            CUSPARSE_INDEX_BASE_ZERO,
            CUDA_R_64F
        )
    );

    nlCUDABlas()->aux_time += (nlCurrentTime() - t0);

#ifdef GARGANTUA
    /* convert the rowptr array back to 64-bits, in-place */
    int32_to_int64(M->rowptr, M->m+1);
#endif

    return (NLMatrix)Mcuda;
}

/**************************************************************************/

/**
 * Abstract matrix interface for a diagonal matrix stored on the GPU.
 */
typedef struct {
    NLuint m;
    NLuint n;
    NLenum type;
    NLDestroyMatrixFunc destroy_func;
    NLMultMatrixVectorFunc mult_func;
    double* val;
} NLDiagonalMatrixCUDA;

static void nlDiagonalMatrixCUDADestroy(NLDiagonalMatrixCUDA* Mcuda) {
    if(!nlExtensionIsInitialized_CUDA()) {
        return;
    }
    nlCUDACheck(CUDA()->cudaFree(Mcuda->val));
    memset(Mcuda, 0, sizeof(*Mcuda));
}

static void nlDiagonalMatrixCUDAMult(
    NLDiagonalMatrixCUDA* Mcuda, const double* x, double* y
) {
    int N = (int)Mcuda->n;
    /*
     * vector x vector component-wise product implemented
     * using diagonal matrix x matrix function.
     */
    nlCUDACheck(CUDA()->cublasDdgmm(
                    CUDA()->main_device->HNDL_cublas, CUBLAS_SIDE_LEFT,
                    N, 1,
                    x, N,
                    Mcuda->val, 1,
                    y, N
                ));
    nlCUDABlas()->flops += (NLulong)N;
}

static NLMatrix nlDiagonalMatrixCUDANew(const double* diag, NLuint n) {
    NLDiagonalMatrixCUDA* Mcuda = NL_NEW(NLDiagonalMatrixCUDA);
    Mcuda->m = n;
    Mcuda->n = n;
    Mcuda->type = NL_MATRIX_OTHER;
    nlCUDACheck(CUDA()->cudaMalloc(
                    (void**)&Mcuda->val, n*sizeof(double))
               );
    nlCUDACheck(CUDA()->cudaMemcpy(
                    Mcuda->val, diag, n*sizeof(double), cudaMemcpyHostToDevice)
               );
    Mcuda->destroy_func=(NLDestroyMatrixFunc)nlDiagonalMatrixCUDADestroy;
    Mcuda->mult_func=(NLMultMatrixVectorFunc)nlDiagonalMatrixCUDAMult;
    return (NLMatrix)Mcuda;
}

NLMatrix nlCUDAJacobiPreconditionerNewFromCRSMatrix(NLMatrix M_in) {
    NLuint N = M_in->n;
    NLuint i;
    NLuint_big jj;
    double* diag = NULL;
    NLMatrix result = NULL;
    NLCRSMatrix* M = (NLCRSMatrix*)(M_in);
    nl_assert(M_in->type == NL_MATRIX_CRS);
    diag = NL_NEW_ARRAY(double,N);
    for(i=0; i<N; ++i) {
        for(jj=M->rowptr[i]; jj<M->rowptr[i+1]; ++jj) {
            if(M->colind[jj] == i) {
                diag[i] = M->val[jj];
            }
        }
    }
    for(i=0; i<N; ++i) {
        diag[i] = ((diag[i] == 0.0) ? 1.0 : 1.0 / diag[i]);
    }
    result = nlDiagonalMatrixCUDANew(diag, N);
    NL_DELETE_ARRAY(diag);
    return result;
}

/**************************************************************************/

static void* cuda_blas_malloc(
    NLBlas_t blas, NLmemoryType type, size_t size
) {
    void* result = NULL;
    blas->used_ram[type] += (NLulong)size;
    blas->max_used_ram[type] = MAX(
        blas->max_used_ram[type],blas->used_ram[type]
    );
    if(type == NL_HOST_MEMORY) {
        // result = malloc(size);
	// pinned memory, makes Host <-> device xfers faster
	nlCUDACheck(CUDA()->cudaMallocHost(&result,size));
    } else {
        nlCUDACheck(CUDA()->cudaMalloc(&result,size));
    }
    return result;
}

static void cuda_blas_free(
    NLBlas_t blas, NLmemoryType type, size_t size, void* ptr
) {
    blas->used_ram[type] -= (NLulong)size;
    if(type == NL_HOST_MEMORY) {
        // free(ptr);
	// pinned memory, makes Host <-> device xfers faster
	nlCUDACheck(CUDA()->cudaFreeHost(ptr));
    } else {
        nlCUDACheck(CUDA()->cudaFree(ptr));
    }
}

static void cuda_blas_memcpy(
    NLBlas_t blas,
    void* to, NLmemoryType to_type,
    void* from, NLmemoryType from_type,
    size_t size
) {
    enum cudaMemcpyKind kind = cudaMemcpyDefault;
    nl_arg_used(blas);
    if(from_type == NL_HOST_MEMORY) {
        if(to_type == NL_HOST_MEMORY) {
            kind = cudaMemcpyHostToHost;
        } else {
            kind = cudaMemcpyHostToDevice;
        }
    } else {
        if(to_type == NL_HOST_MEMORY) {
            kind = cudaMemcpyDeviceToHost;
        } else {
            kind = cudaMemcpyDeviceToDevice;
        }
    }
    nlCUDACheck(CUDA()->cudaMemcpy(to, from, size, kind));
}

static void cuda_blas_memset(
    NLBlas_t blas,
    void* to, NLmemoryType to_type,
    int c, size_t size
) {
    nl_arg_used(blas);
    if(to_type == NL_HOST_MEMORY) {
	memset(to,c,size);
    } else {
	nlCUDACheck(CUDA()->cudaMemset(to, c, size));
    }
}

static void cuda_blas_dcopy(
    NLBlas_t blas, int n, const double *x, int incx, double *y, int incy
) {
    nl_arg_used(blas);
    CUDA()->cublasDcopy(CUDA()->main_device->HNDL_cublas,n,x,incx,y,incy);
}

static double cuda_blas_ddot(
    NLBlas_t blas, int n, const double *x, int incx, const double *y, int incy
) {
    double result = 0.0;
    blas->flops += (NLulong)(2*n);
    CUDA()->cublasDdot(CUDA()->main_device->HNDL_cublas,n,x,incx,y,incy,&result);
    return result;
}

static double cuda_blas_dnrm2(
    NLBlas_t blas, int n, const double *x, int incx
) {
    double result = 0.0;
    blas->flops += (NLulong)(2*n);
    CUDA()->cublasDnrm2(CUDA()->main_device->HNDL_cublas,n,x,incx,&result);
    return result;
}

static void cuda_blas_daxpy(
    NLBlas_t blas, int n,
    double a, const double *x, int incx, double *y, int incy
) {
    blas->flops += (NLulong)(2*n);
    CUDA()->cublasDaxpy(CUDA()->main_device->HNDL_cublas,n,&a,x,incx,y,incy);
}

static void cuda_blas_dmul(
    NLBlas_t blas, int n,
    const double *x, const double *y, double* z
) {
    blas->flops += (NLulong)(n);
    /*
     * vector x vector component-wise product implemented
     * using diagonal matrix x matrix function.
     */
    nlCUDACheck(CUDA()->cublasDdgmm(
                    CUDA()->main_device->HNDL_cublas, CUBLAS_SIDE_LEFT,
                    n, 1,
                    x, n,
                    y, 1,
                    z, n
                ));
}

static void cuda_blas_dscal(
    NLBlas_t blas, int n, double a, double *x, int incx
) {
    blas->flops += (NLulong)n;
    CUDA()->cublasDscal(CUDA()->main_device->HNDL_cublas,n,&a,x,incx);
}


static void cuda_blas_dgemv(
    NLBlas_t blas, MatrixTranspose trans, int m, int n, double alpha,
    const double *A, int ldA, const double *x, int incx,
    double beta, double *y, int incy
) {
    nl_arg_used(blas);
    /* TODO: update FLOPS */
    CUDA()->cublasDgemv(
        CUDA()->main_device->HNDL_cublas, (cublasOperation_t)trans,
        m, n, &alpha, A, ldA, x, incx, &beta, y, incy
    );
}

static void cuda_blas_dtpsv(
    NLBlas_t blas, MatrixTriangle uplo, MatrixTranspose trans,
    MatrixUnitTriangular diag, int n, const double *AP,
    double *x, int incx
) {
    nl_arg_used(blas);
    /* TODO: update FLOPS */
    CUDA()->cublasDtpsv(
        CUDA()->main_device->HNDL_cublas,
        (cublasFillMode_t)uplo,
        (cublasOperation_t)trans,
        (cublasDiagType_t)diag, n,
        AP, x, incx
    );
}

static void cuda_blas_reset_stats(NLBlas_t blas) {
    blas->start_time = nlCurrentTime();
    blas->flops = 0;
    blas->used_ram[0] = 0;
    blas->used_ram[1] = 0;
    blas->max_used_ram[0] = 0;
    blas->max_used_ram[1] = 0;
    blas->sq_rnorm = 0.0;
    blas->sq_bnorm = 0.0;
    blas->aux_time = 0.0;
}

static void cuda_blas_show_stats(NLBlas_t blas) {
    size_t free_RAM;
    size_t total_RAM;
    CUDA()->cudaMemGetInfo(&free_RAM, &total_RAM);
    nl_printf(
	"NLBlas: used GPU RAM: %f / total: %f GB (free: %f GB)\n",
	(double)(total_RAM - free_RAM)/1e9,
	(double)total_RAM/1e9,
	(double)free_RAM/1e9
    );
    if(blas->aux_time != 0.0) {
	nl_printf("  Aux time: %f\n",blas->aux_time);
    }
}


NLBlas_t nlCUDABlas(void) {
    static NLboolean initialized = NL_FALSE;
    static struct NLBlas blas;
    if(!initialized) {
        memset(&blas, 0, sizeof(blas));
        blas.has_unified_memory = NL_FALSE;
        blas.Malloc = cuda_blas_malloc;
        blas.Free = cuda_blas_free;
        blas.Memcpy = cuda_blas_memcpy;
        blas.Memset = cuda_blas_memset;
        blas.Dcopy = cuda_blas_dcopy;
        blas.Ddot = cuda_blas_ddot;
        blas.Dnrm2 = cuda_blas_dnrm2;
        blas.Daxpy = cuda_blas_daxpy;
	blas.Dmul = cuda_blas_dmul;
        blas.Dscal = cuda_blas_dscal;
        blas.Dgemv = cuda_blas_dgemv;
        blas.Dtpsv = cuda_blas_dtpsv;
	blas.reset_stats = cuda_blas_reset_stats;
	blas.show_stats = cuda_blas_show_stats;
        nlBlasResetStats(&blas);
        initialized = NL_TRUE;
    }
    return &blas;
}


/**************************************************************************/
