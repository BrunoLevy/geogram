/*
MIT License

Copyright (c) 2018-2019 Jonathan Young

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/
/*
thekla_atlas
https://github.com/Thekla/thekla_atlas
MIT License
Copyright (c) 2013 Thekla, Inc
Copyright NVIDIA Corporation 2006 -- Ignacio Castano <icastano@nvidia.com>

Fast-BVH
https://github.com/brandonpelfrey/Fast-BVH
MIT License
Copyright (c) 2012 Brandon Pelfrey
*/
#include <atomic>
#include <condition_variable>
#include <mutex>
#include <thread>
#include <assert.h>
#include <float.h> // FLT_MAX
#include <limits.h>
#include <math.h>
#define __STDC_LIMIT_MACROS
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "xatlas.h"

#ifndef XA_DEBUG
#ifdef NDEBUG
#define XA_DEBUG 0
#else
#define XA_DEBUG 1
#endif
#endif

#ifndef XA_PROFILE
#define XA_PROFILE 0
#endif
#if XA_PROFILE
#include <time.h>
#endif

#ifndef XA_MULTITHREADED
#define XA_MULTITHREADED 1
#endif

#define XA_STR(x) #x
#define XA_XSTR(x) XA_STR(x)

#ifndef XA_ASSERT
#define XA_ASSERT(exp) if (!(exp)) { XA_PRINT_WARNING("\rASSERT: %s %s %d\n", XA_XSTR(exp), __FILE__, __LINE__); }
#endif

#ifndef XA_DEBUG_ASSERT
#define XA_DEBUG_ASSERT(exp) assert(exp)
#endif

#ifndef XA_PRINT
#define XA_PRINT(...) \
        if (xatlas::internal::s_print && xatlas::internal::s_printVerbose) \
                xatlas::internal::s_print(__VA_ARGS__);
#endif

#ifndef XA_PRINT_WARNING
#define XA_PRINT_WARNING(...) \
        if (xatlas::internal::s_print) \
                xatlas::internal::s_print(__VA_ARGS__);
#endif

#define XA_ALLOC(tag, type) (type *)internal::Realloc(nullptr, sizeof(type), tag, __FILE__, __LINE__)
#define XA_ALLOC_ARRAY(tag, type, num) (type *)internal::Realloc(nullptr, sizeof(type) * num, tag, __FILE__, __LINE__)
#define XA_REALLOC(tag, ptr, type, num) (type *)internal::Realloc(ptr, sizeof(type) * num, tag, __FILE__, __LINE__)
#define XA_REALLOC_SIZE(tag, ptr, size) (uint8_t *)internal::Realloc(ptr, size, tag, __FILE__, __LINE__)
#define XA_FREE(ptr) internal::Realloc(ptr, 0, internal::MemTag::Default, __FILE__, __LINE__)
#define XA_NEW(tag, type) new (XA_ALLOC(tag, type)) type()
#define XA_NEW_ARGS(tag, type, ...) new (XA_ALLOC(tag, type)) type(__VA_ARGS__)

#ifdef _MSC_VER
#define XA_INLINE __forceinline
#else
#define XA_INLINE inline
#endif

#if defined(__clang__) || defined(__GNUC__)
#define XA_NODISCARD [[nodiscard]]
#elif defined(_MSC_VER)
#define XA_NODISCARD _Check_return_
#else
#define XA_NODISCARD
#endif

#define XA_UNUSED(a) ((void)(a))

#define XA_MERGE_CHARTS 1
#define XA_MERGE_CHARTS_MIN_NORMAL_DEVIATION 0.5f
#define XA_RECOMPUTE_CHARTS 1
#define XA_CLOSE_HOLES_CHECK_EDGE_INTERSECTION 0
#define XA_FIX_INTERNAL_BOUNDARY_LOOPS 1
#define XA_PRINT_CHART_WARNINGS 0

#define XA_DEBUG_HEAP 0
#define XA_DEBUG_SINGLE_CHART 0
#define XA_DEBUG_EXPORT_ATLAS_IMAGES 0
#define XA_DEBUG_EXPORT_ATLAS_IMAGES_PER_CHART 0 // Export an atlas image after each chart is added.
#define XA_DEBUG_EXPORT_BOUNDARY_GRID 0
#define XA_DEBUG_EXPORT_TGA (XA_DEBUG_EXPORT_ATLAS_IMAGES || XA_DEBUG_EXPORT_BOUNDARY_GRID)
#define XA_DEBUG_EXPORT_OBJ_SOURCE_MESHES 0
#define XA_DEBUG_EXPORT_OBJ_CHART_GROUPS 0
#define XA_DEBUG_EXPORT_OBJ_PLANAR_REGIONS 0
#define XA_DEBUG_EXPORT_OBJ_CHARTS 0
#define XA_DEBUG_EXPORT_OBJ_BEFORE_FIX_TJUNCTION 0
#define XA_DEBUG_EXPORT_OBJ_CLOSE_HOLES_ERROR 0
#define XA_DEBUG_EXPORT_OBJ_CHARTS_AFTER_PARAMETERIZATION 0
#define XA_DEBUG_EXPORT_OBJ_INVALID_PARAMETERIZATION 0
#define XA_DEBUG_EXPORT_OBJ_RECOMPUTED_CHARTS 0

#define XA_DEBUG_EXPORT_OBJ (0 \
        || XA_DEBUG_EXPORT_OBJ_SOURCE_MESHES \
        || XA_DEBUG_EXPORT_OBJ_CHART_GROUPS \
        || XA_DEBUG_EXPORT_OBJ_PLANAR_REGIONS \
        || XA_DEBUG_EXPORT_OBJ_CHARTS \
        || XA_DEBUG_EXPORT_OBJ_BEFORE_FIX_TJUNCTION \
        || XA_DEBUG_EXPORT_OBJ_CLOSE_HOLES_ERROR \
        || XA_DEBUG_EXPORT_OBJ_CHARTS_AFTER_PARAMETERIZATION \
        || XA_DEBUG_EXPORT_OBJ_INVALID_PARAMETERIZATION \
        || XA_DEBUG_EXPORT_OBJ_RECOMPUTED_CHARTS)

#ifdef _MSC_VER
#define XA_FOPEN(_file, _filename, _mode) { if (fopen_s(&_file, _filename, _mode) != 0) _file = NULL; }
#define XA_SPRINTF(_buffer, _size, _format, ...) sprintf_s(_buffer, _size, _format, __VA_ARGS__)
#else
#define XA_FOPEN(_file, _filename, _mode) _file = fopen(_filename, _mode)
#define XA_SPRINTF(_buffer, _size, _format, ...) sprintf(_buffer, _format, __VA_ARGS__)
#endif

// [Bruno Levy]
#if defined(__APPLE__) || defined(__EMSCRIPTEN__)
#undef XA_MULTITHREADED
#define XA_MULTITHREADED 0
#endif
#undef XA_NODISCARD
#define XA_NODISCARD
// End of [Bruno Levy]

namespace xatlas {
namespace internal {

static ReallocFunc s_realloc = realloc;
static FreeFunc s_free = free;
static PrintFunc s_print = printf;
static bool s_printVerbose = false;

struct MemTag
{
        enum
        {
                Default,
                BitImage,
                BVH,
                FullVector,
                Matrix,
                Mesh,
                MeshBoundaries,
                MeshColocals,
                MeshEdgeMap,
                MeshIndices,
                MeshNormals,
                MeshPositions,
                MeshTexcoords,
                SegmentAtlasChartCandidates,
                SegmentAtlasChartFaces,
                SegmentAtlasMeshData,
                SegmentAtlasPlanarRegions,
                Count
        };
};

#if XA_DEBUG_HEAP
struct AllocHeader
{
        size_t size;
        const char *file;
        int line;
        int tag;
        uint32_t id;
        AllocHeader *prev, *next;
        bool free;
};

static std::mutex s_allocMutex;
static AllocHeader *s_allocRoot = nullptr;
static size_t s_allocTotalCount = 0, s_allocTotalSize = 0, s_allocPeakSize = 0, s_allocCount[MemTag::Count] = { 0 }, s_allocTotalTagSize[MemTag::Count] = { 0 }, s_allocPeakTagSize[MemTag::Count] = { 0 };
static uint32_t s_allocId =0 ;
static constexpr uint32_t kAllocRedzone = 0x12345678;

static void *Realloc(void *ptr, size_t size, int tag, const char *file, int line)
{
        std::unique_lock<std::mutex> lock(s_allocMutex);
        if (!size && !ptr)
                return nullptr;
        uint8_t *realPtr = nullptr;
        AllocHeader *header = nullptr;
        if (ptr) {
                realPtr = ((uint8_t *)ptr) - sizeof(AllocHeader);
                header = (AllocHeader *)realPtr;
        }
        if (realPtr && size) {
                s_allocTotalSize -= header->size;
                s_allocTotalTagSize[header->tag] -= header->size;
                // realloc, remove.
                if (header->prev)
                        header->prev->next = header->next;
                else
                        s_allocRoot = header->next;
                if (header->next)
                        header->next->prev = header->prev;
        }
        if (!size) {
                s_allocTotalSize -= header->size;
                s_allocTotalTagSize[header->tag] -= header->size;
                XA_ASSERT(!header->free); // double free
                header->free = true;
                return nullptr;
        }
        size += sizeof(AllocHeader) + sizeof(kAllocRedzone);
        uint8_t *newPtr = (uint8_t *)s_realloc(realPtr, size);
        if (!newPtr)
                return nullptr;
        header = (AllocHeader *)newPtr;
        header->size = size;
        header->file = file;
        header->line = line;
        header->tag = tag;
        header->id = s_allocId++;
        header->free = false;
        if (!s_allocRoot) {
                s_allocRoot = header;
                header->prev = header->next = 0;
        } else {
                header->prev = nullptr;
                header->next = s_allocRoot;
                s_allocRoot = header;
                header->next->prev = header;
        }
        s_allocTotalCount++;
        s_allocTotalSize += size;
        if (s_allocTotalSize > s_allocPeakSize)
                s_allocPeakSize = s_allocTotalSize;
        s_allocCount[tag]++;
        s_allocTotalTagSize[tag] += size;
        if (s_allocTotalTagSize[tag] > s_allocPeakTagSize[tag])
                s_allocPeakTagSize[tag] = s_allocTotalTagSize[tag];
        auto redzone = (uint32_t *)(newPtr + size - sizeof(kAllocRedzone));
        *redzone = kAllocRedzone;
        return newPtr + sizeof(AllocHeader);
}

static void ReportLeaks()
{
        printf("Checking for memory leaks...\n");
        bool anyLeaks = false;
        AllocHeader *header = s_allocRoot;
        while (header) {
                if (!header->free) {
                        printf("   Leak: ID %u, %zu bytes, %s %d\n", header->id, header->size, header->file, header->line);
                        anyLeaks = true;
                }
                auto redzone = (const uint32_t *)((const uint8_t *)header + header->size - sizeof(kAllocRedzone));
                if (*redzone != kAllocRedzone)
                        printf("   Redzone corrupted: %zu bytes %s %d\n", header->size, header->file, header->line);
                header = header->next;
        }
        if (!anyLeaks)
                printf("   No memory leaks\n");
        header = s_allocRoot;
        while (header) {
                AllocHeader *destroy = header;
                header = header->next;
                s_realloc(destroy, 0);
        }
        s_allocRoot = nullptr;
        s_allocTotalSize = s_allocPeakSize = 0;
        for (int i = 0; i < MemTag::Count; i++)
                s_allocTotalTagSize[i] = s_allocPeakTagSize[i] = 0;
}

static void PrintMemoryUsage()
{
        XA_PRINT("Total allocations: %zu\n", s_allocTotalCount);
        XA_PRINT("Memory usage: %0.2fMB current, %0.2fMB peak\n", internal::s_allocTotalSize / 1024.0f / 1024.0f, internal::s_allocPeakSize / 1024.0f / 1024.0f);
        static const char *labels[] = { // Sync with MemTag
                "Default",
                "BitImage",
                "BVH",
                "FullVector",
                "Matrix",
                "Mesh",
                "MeshBoundaries",
                "MeshColocals",
                "MeshEdgeMap",
                "MeshIndices",
                "MeshNormals",
                "MeshPositions",
                "MeshTexcoords",
                "SegmentAtlasChartCandidates",
                "SegmentAtlasChartFaces",
                "SegmentAtlasMeshData",
                "SegmentAtlasPlanarRegions"
        };
        for (int i = 0; i < MemTag::Count; i++) {
                XA_PRINT("   %s: %zu allocations, %0.2fMB current, %0.2fMB peak\n", labels[i], internal::s_allocCount[i], internal::s_allocTotalTagSize[i] / 1024.0f / 1024.0f, internal::s_allocPeakTagSize[i] / 1024.0f / 1024.0f);
        }
}

#define XA_PRINT_MEM_USAGE internal::PrintMemoryUsage();
#else
static void *Realloc(void *ptr, size_t size, int /*tag*/, const char * /*file*/, int /*line*/)
{
        if (size == 0 && !ptr)
                return nullptr;
        if (size == 0 && s_free) {
                s_free(ptr);
                return nullptr;
        }
        void *mem = s_realloc(ptr, size);
        if (size > 0) {
                XA_DEBUG_ASSERT(mem);
        }
        return mem;
}
#define XA_PRINT_MEM_USAGE
#endif

#if XA_PROFILE
#define XA_PROFILE_START(var) const clock_t var##Start = clock();
#define XA_PROFILE_END(var) internal::s_profile.var += clock() - var##Start;
#define XA_PROFILE_PRINT_AND_RESET(label, var) XA_PRINT("%s%.2f seconds (%g ms)\n", label, internal::clockToSeconds(internal::s_profile.var), internal::clockToMs(internal::s_profile.var)); internal::s_profile.var = 0;

struct ProfileData
{
        clock_t addMeshReal;
        clock_t addMeshCopyData;
        std::atomic<clock_t> addMeshThread;
        std::atomic<clock_t> addMeshCreateColocals;
        std::atomic<clock_t> addMeshCreateFaceGroups;
        std::atomic<clock_t> addMeshCreateChartGroupsReal;
        std::atomic<clock_t> addMeshCreateChartGroupsThread;
        clock_t computeChartsReal;
        std::atomic<clock_t> computeChartsThread;
        std::atomic<clock_t> buildAtlas;
        std::atomic<clock_t> buildAtlasInit;
        std::atomic<clock_t> buildAtlasPlaceSeeds;
        std::atomic<clock_t> buildAtlasRelocateSeeds;
        std::atomic<clock_t> buildAtlasResetCharts;
        std::atomic<clock_t> buildAtlasGrowCharts;
        std::atomic<clock_t> buildAtlasMergeCharts;
        std::atomic<clock_t> buildAtlasFillHoles;
        std::atomic<clock_t> createChartMeshesReal;
        std::atomic<clock_t> createChartMeshesThread;
        std::atomic<clock_t> fixChartMeshTJunctions;
        std::atomic<clock_t> closeChartMeshHoles;
        clock_t parameterizeChartsReal;
        std::atomic<clock_t> parameterizeChartsThread;
        std::atomic<clock_t> parameterizeChartsOrthogonal;
        std::atomic<clock_t> parameterizeChartsLSCM;
        std::atomic<clock_t> parameterizeChartsEvaluateQuality;
        clock_t packCharts;
        clock_t packChartsAddCharts;
        std::atomic<clock_t> packChartsAddChartsThread;
        std::atomic<clock_t> packChartsAddChartsRestoreTexcoords;
        clock_t packChartsRasterize;
        clock_t packChartsDilate;
        clock_t packChartsFindLocation;
        clock_t packChartsBlit;
        clock_t buildOutputMeshes;
};

static ProfileData s_profile;

static double clockToMs(clock_t c)
{
        return c * 1000.0 / CLOCKS_PER_SEC;
}

static double clockToSeconds(clock_t c)
{
        return c / (double)CLOCKS_PER_SEC;
}
#else
#define XA_PROFILE_START(var)
#define XA_PROFILE_END(var)
#define XA_PROFILE_PRINT_AND_RESET(label, var)
#endif

static constexpr float kPi = 3.14159265358979323846f;
static constexpr float kPi2 = 6.28318530717958647692f;
static constexpr float kPi4 = 12.56637061435917295384f;
static constexpr float kEpsilon = 0.0001f;
static constexpr float kAreaEpsilon = FLT_EPSILON;
static constexpr float kNormalEpsilon = 0.001f;

static int align(int x, int a)
{
        return (x + a - 1) & ~(a - 1);
}

template <typename T>
static T max(const T &a, const T &b)
{
        return a > b ? a : b;
}

template <typename T>
static T min(const T &a, const T &b)
{
        return a < b ? a : b;
}

template <typename T>
static T max3(const T &a, const T &b, const T &c)
{
        return max(a, max(b, c));
}

/// Return the maximum of the three arguments.
template <typename T>
static T min3(const T &a, const T &b, const T &c)
{
        return min(a, min(b, c));
}

/// Clamp between two values.
template <typename T>
static T clamp(const T &x, const T &a, const T &b)
{
        return min(max(x, a), b);
}

template <typename T>
static void swap(T &a, T &b)
{
        T temp = a;
        a = b;
        b = temp;
}

union FloatUint32
{
        float f;
        uint32_t u;
};

static bool isFinite(float f)
{
        FloatUint32 fu;
        fu.f = f;
        return fu.u != 0x7F800000u && fu.u != 0x7F800001u;
}

static bool isNan(float f)
{
        return f != f;
}

// Robust floating point comparisons:
// http://realtimecollisiondetection.net/blog/?p=89
static bool equal(const float f0, const float f1, const float epsilon)
{
        //return fabs(f0-f1) <= epsilon;
        return fabs(f0 - f1) <= epsilon * max3(1.0f, fabsf(f0), fabsf(f1));
}

static int ftoi_ceil(float val)
{
        return (int)ceilf(val);
}

static bool isZero(const float f, const float epsilon)
{
        return fabs(f) <= epsilon;
}

static float square(float f)
{
        return f * f;
}

/** Return the next power of two.
* @see http://graphics.stanford.edu/~seander/bithacks.html
* @warning Behaviour for 0 is undefined.
* @note isPowerOfTwo(x) == true -> nextPowerOfTwo(x) == x
* @note nextPowerOfTwo(x) = 2 << log2(x-1)
*/
static uint32_t nextPowerOfTwo(uint32_t x)
{
        XA_DEBUG_ASSERT( x != 0 );
        // On modern CPUs this is supposed to be as fast as using the bsr instruction.
        x--;
        x |= x >> 1;
        x |= x >> 2;
        x |= x >> 4;
        x |= x >> 8;
        x |= x >> 16;
        return x + 1;
}

static uint32_t sdbmHash(const void *data_in, uint32_t size, uint32_t h = 5381)
{
        const uint8_t *data = (const uint8_t *) data_in;
        uint32_t i = 0;
        while (i < size) {
                h = (h << 16) + (h << 6) - h + (uint32_t ) data[i++];
        }
        return h;
}

template <typename T>
static uint32_t hash(const T &t, uint32_t h = 5381)
{
        return sdbmHash(&t, sizeof(T), h);
}

// Functors for hash table:
template <typename Key> struct Hash
{
        uint32_t operator()(const Key &k) const { return hash(k); }
};

template <typename Key> struct Equal
{
        bool operator()(const Key &k0, const Key &k1) const { return k0 == k1; }
};

class Vector2
{
public:
        Vector2() {}
        explicit Vector2(float f) : x(f), y(f) {}
        Vector2(float x, float y): x(x), y(y) {}

        Vector2 operator-() const
        {
                return Vector2(-x, -y);
        }

        void operator+=(const Vector2 &v)
        {
                x += v.x;
                y += v.y;
        }

        void operator-=(const Vector2 &v)
        {
                x -= v.x;
                y -= v.y;
        }

        void operator*=(float s)
        {
                x *= s;
                y *= s;
        }

        void operator*=(const Vector2 &v)
        {
                x *= v.x;
                y *= v.y;
        }

        float x, y;
};

static bool operator==(const Vector2 &a, const Vector2 &b)
{
        return a.x == b.x && a.y == b.y;
}

static bool operator!=(const Vector2 &a, const Vector2 &b)
{
        return a.x != b.x || a.y != b.y;
}

/*static Vector2 operator+(const Vector2 &a, const Vector2 &b)
{
        return Vector2(a.x + b.x, a.y + b.y);
}*/

static Vector2 operator-(const Vector2 &a, const Vector2 &b)
{
        return Vector2(a.x - b.x, a.y - b.y);
}

static Vector2 operator*(const Vector2 &v, float s)
{
        return Vector2(v.x * s, v.y * s);
}

static float dot(const Vector2 &a, const Vector2 &b)
{
        return a.x * b.x + a.y * b.y;
}

static float lengthSquared(const Vector2 &v)
{
        return v.x * v.x + v.y * v.y;
}

static float length(const Vector2 &v)
{
        return sqrtf(lengthSquared(v));
}

#if XA_DEBUG
static bool isNormalized(const Vector2 &v, float epsilon = kNormalEpsilon)
{
        return equal(length(v), 1, epsilon);
}
#endif

static Vector2 normalize(const Vector2 &v, float epsilon)
{
        float l = length(v);
        XA_DEBUG_ASSERT(!isZero(l, epsilon));
        XA_UNUSED(epsilon);
        Vector2 n = v * (1.0f / l);
        XA_DEBUG_ASSERT(isNormalized(n));
        return n;
}

static Vector2 normalizeSafe(const Vector2 &v, const Vector2 &fallback, float epsilon)
{
        float l = length(v);
        if (isZero(l, epsilon))
                return fallback;
        return v * (1.0f / l);
}

static bool equal(const Vector2 &v1, const Vector2 &v2, float epsilon)
{
        return equal(v1.x, v2.x, epsilon) && equal(v1.y, v2.y, epsilon);
}

static Vector2 min(const Vector2 &a, const Vector2 &b)
{
        return Vector2(min(a.x, b.x), min(a.y, b.y));
}

static Vector2 max(const Vector2 &a, const Vector2 &b)
{
        return Vector2(max(a.x, b.x), max(a.y, b.y));
}

static bool isFinite(const Vector2 &v)
{
        return isFinite(v.x) && isFinite(v.y);
}

static float triangleArea(const Vector2 &a, const Vector2 &b, const Vector2 &c)
{
        // IC: While it may be appealing to use the following expression:
        //return (c.x * a.y + a.x * b.y + b.x * c.y - b.x * a.y - c.x * b.y - a.x * c.y) * 0.5f;
        // That's actually a terrible idea. Small triangles far from the origin can end up producing fairly large floating point
        // numbers and the results becomes very unstable and dependent on the order of the factors.
        // Instead, it's preferable to subtract the vertices first, and multiply the resulting small values together. The result
        // in this case is always much more accurate (as long as the triangle is small) and less dependent of the location of
        // the triangle.
        //return ((a.x - c.x) * (b.y - c.y) - (a.y - c.y) * (b.x - c.x)) * 0.5f;
        const Vector2 v0 = a - c;
        const Vector2 v1 = b - c;
        return (v0.x * v1.y - v0.y * v1.x) * 0.5f;
}

static bool linesIntersect(const Vector2 &a1, const Vector2 &a2, const Vector2 &b1, const Vector2 &b2, float epsilon)
{
        const Vector2 v0 = a2 - a1;
        const Vector2 v1 = b2 - b1;
        const float denom = -v1.x * v0.y + v0.x * v1.y;
        if (equal(denom, 0.0f, epsilon))
                return false;
        const float s = (-v0.y * (a1.x - b1.x) + v0.x * (a1.y - b1.y)) / denom;
        if (s > epsilon && s < 1.0f - epsilon) {
                const float t = ( v1.x * (a1.y - b1.y) - v1.y * (a1.x - b1.x)) / denom;
                return t > epsilon && t < 1.0f - epsilon;
        }
        return false;
}

struct Vector2i
{
        Vector2i() {}
        Vector2i(int32_t x, int32_t y) : x(x), y(y) {}

        int32_t x, y;
};

class Vector3
{
public:
        Vector3() {}
        explicit Vector3(float f) : x(f), y(f), z(f) {}
        Vector3(float x, float y, float z) : x(x), y(y), z(z) {}
        Vector3(const Vector2 &v, float z) : x(v.x), y(v.y), z(z) {}

        Vector2 xy() const
        {
                return Vector2(x, y);
        }

        Vector3 operator-() const
        {
                return Vector3(-x, -y, -z);
        }

        void operator+=(const Vector3 &v)
        {
                x += v.x;
                y += v.y;
                z += v.z;
        }

        void operator-=(const Vector3 &v)
        {
                x -= v.x;
                y -= v.y;
                z -= v.z;
        }

        void operator*=(float s)
        {
                x *= s;
                y *= s;
                z *= s;
        }

        void operator/=(float s)
        {
                float is = 1.0f / s;
                x *= is;
                y *= is;
                z *= is;
        }

        void operator*=(const Vector3 &v)
        {
                x *= v.x;
                y *= v.y;
                z *= v.z;
        }

        void operator/=(const Vector3 &v)
        {
                x /= v.x;
                y /= v.y;
                z /= v.z;
        }

        float x, y, z;
};

static Vector3 operator+(const Vector3 &a, const Vector3 &b)
{
        return Vector3(a.x + b.x, a.y + b.y, a.z + b.z);
}

static Vector3 operator-(const Vector3 &a, const Vector3 &b)
{
        return Vector3(a.x - b.x, a.y - b.y, a.z - b.z);
}

static Vector3 cross(const Vector3 &a, const Vector3 &b)
{
        return Vector3(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x);
}

static Vector3 operator*(const Vector3 &v, float s)
{
        return Vector3(v.x * s, v.y * s, v.z * s);
}

static Vector3 operator/(const Vector3 &v, float s)
{
        return v * (1.0f / s);
}

static float dot(const Vector3 &a, const Vector3 &b)
{
        return a.x * b.x + a.y * b.y + a.z * b.z;
}

static float lengthSquared(const Vector3 &v)
{
        return v.x * v.x + v.y * v.y + v.z * v.z;
}

static float length(const Vector3 &v)
{
        return sqrtf(lengthSquared(v));
}

static bool isNormalized(const Vector3 &v, float epsilon = kNormalEpsilon)
{
        return equal(length(v), 1, epsilon);
}

static Vector3 normalize(const Vector3 &v, float epsilon)
{
        float l = length(v);
        XA_DEBUG_ASSERT(!isZero(l, epsilon));
        XA_UNUSED(epsilon);
        Vector3 n = v * (1.0f / l);
        XA_DEBUG_ASSERT(isNormalized(n));
        return n;
}

static Vector3 normalizeSafe(const Vector3 &v, const Vector3 &fallback, float epsilon)
{
        float l = length(v);
        if (isZero(l, epsilon)) {
                return fallback;
        }
        return v * (1.0f / l);
}

static bool equal(const Vector3 &v0, const Vector3 &v1, float epsilon)
{
        return fabs(v0.x - v1.x) <= epsilon && fabs(v0.y - v1.y) <= epsilon && fabs(v0.z - v1.z) <= epsilon;
}

static Vector3 min(const Vector3 &a, const Vector3 &b)
{
        return Vector3(min(a.x, b.x), min(a.y, b.y), min(a.z, b.z));
}

static Vector3 max(const Vector3 &a, const Vector3 &b)
{
        return Vector3(max(a.x, b.x), max(a.y, b.y), max(a.z, b.z));
}

#if XA_DEBUG
bool isFinite(const Vector3 &v)
{
        return isFinite(v.x) && isFinite(v.y) && isFinite(v.z);
}
#endif

struct Extents2
{
        Vector2 min, max;

        void reset()
        {
                min.x = min.y = FLT_MAX;
                max.x = max.y = -FLT_MAX;
        }

        void add(Vector2 p)
        {
                min = xatlas::internal::min(min, p);
                max = xatlas::internal::max(max, p);
        }

        Vector2 midpoint() const
        {
                return Vector2(min.x + (max.x - min.x) * 0.5f, min.y + (max.y - min.y) * 0.5f);
        }

        static bool intersect(Extents2 e1, Extents2 e2)
        {
                return e1.min.x <= e2.max.x && e1.max.x >= e2.min.x && e1.min.y <= e2.max.y && e1.max.y >= e2.min.y;
        }
};

struct Plane
{
        Plane() = default;

        Plane(const Vector3 &p1, const Vector3 &p2, const Vector3 &p3)
        {
                normal = cross(p2 - p1, p3 - p1);
                dist = dot(normal, p1);
        }

        float distance(const Vector3 &p) const
        {
                return dot(normal, p) - dist;
        }

        void normalize()
        {
                const float len = length(normal);
                if (len > 0.0f) {
                        const float il = 1.0f / len;
                        normal *= il;
                        dist *= il;
                }
        }

        Vector3 normal;
        float dist;
};

static bool lineIntersectsPoint(const Vector3 &point, const Vector3 &lineStart, const Vector3 &lineEnd, float *t, float epsilon)
{
        float tt;
        if (!t)
                t = &tt;
        *t = 0.0f;
        if (equal(lineStart, point, epsilon) || equal(lineEnd, point, epsilon))
                return false; // Vertex lies on either line vertices.
        const Vector3 v01 = point - lineStart;
        const Vector3 v21 = lineEnd - lineStart;
        const float l = length(v21);
        const float d = length(cross(v01, v21)) / l;
        if (!isZero(d, epsilon))
                return false;
        *t = dot(v01, v21) / (l * l);
        return *t > kEpsilon && *t < 1.0f - kEpsilon;
}

static bool sameSide(const Vector3 &p1, const Vector3 &p2, const Vector3 &a, const Vector3 &b)
{
        const Vector3 &ab = b - a;
        return dot(cross(ab, p1 - a), cross(ab, p2 - a)) >= 0.0f;
}

// http://blackpawn.com/texts/pointinpoly/default.html
static bool pointInTriangle(const Vector3 &p, const Vector3 &a, const Vector3 &b, const Vector3 &c)
{
        return sameSide(p, a, b, c) && sameSide(p, b, a, c) && sameSide(p, c, a, b);
}

#if XA_CLOSE_HOLES_CHECK_EDGE_INTERSECTION
// https://en.wikipedia.org/wiki/M%C3%B6ller%E2%80%93Trumbore_intersection_algorithm
static bool rayIntersectsTriangle(const Vector3 &rayOrigin, const Vector3 &rayDir, const Vector3 *tri, float *t)
{
        *t = 0.0f;
        const Vector3 &edge1 = tri[1] - tri[0];
        const Vector3 &edge2 = tri[2] - tri[0];
        const Vector3 h = cross(rayDir, edge2);
        const float a = dot(edge1, h);
        if (a > -kEpsilon && a < kEpsilon)
                return false; // This ray is parallel to this triangle.
        const float f = 1.0f / a;
        const Vector3 s = rayOrigin - tri[0];
        const float u = f * dot(s, h);
        if (u < 0.0f || u > 1.0f)
                return false;
        const Vector3 q = cross(s, edge1);
        const float v = f * dot(rayDir, q);
        if (v < 0.0f || u + v > 1.0f)
                return false;
        // At this stage we can compute t to find out where the intersection point is on the line.
        *t = f * dot(edge2, q);
        if (*t > kEpsilon && *t < 1.0f - kEpsilon)
                return true;
        // This means that there is a line intersection but not a ray intersection.
        return false;
}
#endif

// From Fast-BVH
struct AABB
{
        AABB() : min(FLT_MAX, FLT_MAX, FLT_MAX), max(-FLT_MAX, -FLT_MAX, -FLT_MAX) {}
        AABB(const Vector3 &min, const Vector3 &max) : min(min), max(max) { }
        AABB(const Vector3 &p, float radius = 0.0f) : min(p), max(p) { if (radius > 0.0f) expand(radius); }

        bool intersect(const AABB &other) const
        {
                return min.x <= other.max.x && max.x >= other.min.x && min.y <= other.max.y && max.y >= other.min.y && min.z <= other.max.z && max.z >= other.min.z;
        }

        void expandToInclude(const Vector3 &p)
        {
                min = internal::min(min, p);
                max = internal::max(max, p);
        }

        void expandToInclude(const AABB &aabb)
        {
                min = internal::min(min, aabb.min);
                max = internal::max(max, aabb.max);
        }

        void expand(float amount)
        {
                min -= Vector3(amount);
                max += Vector3(amount);
        }

        Vector3 centroid() const
        {
                return min + (max - min) * 0.5f;
        }

        uint32_t maxDimension() const
        {
                const Vector3 extent = max - min;
                uint32_t result = 0;
                if (extent.y > extent.x) {
                        result = 1;
                        if (extent.z > extent.y)
                                result = 2;
                }
                else if(extent.z > extent.x)
                        result = 2;
                return result;
        }

        Vector3 min, max;
};

struct ArrayBase
{
        ArrayBase(uint32_t elementSize, int memTag = MemTag::Default) : buffer(nullptr), elementSize(elementSize), size(0), capacity(0)
        {
#if XA_DEBUG_HEAP
                this->memTag = memTag;
#else
                XA_UNUSED(memTag);
#endif
        }

        ~ArrayBase()
        {
                XA_FREE(buffer);
        }

        XA_INLINE void clear()
        {
                size = 0;
        }

        void copyFrom(const uint8_t *data, uint32_t length)
        {
                resize(length, true);
                memcpy(buffer, data, length * elementSize);
        }

        void copyTo(ArrayBase &other) const
        {
                XA_DEBUG_ASSERT(elementSize == other.elementSize);
                other.resize(size, true);
                memcpy(other.buffer, buffer, size * elementSize);
        }

        void destroy()
        {
                size = 0;
                XA_FREE(buffer);
                buffer = nullptr;
                capacity = 0;
                size = 0;
        }

        // Insert the given element at the given index shifting all the elements up.
        void insertAt(uint32_t index, const uint8_t *value)
        {
                XA_DEBUG_ASSERT(index <= size); // [Bruno Levy] removed >= 0 test (it is an uint !)
                resize(size + 1, false);
                if (index < size - 1)
                        memmove(buffer + elementSize * (index + 1), buffer + elementSize * index, elementSize * (size - 1 - index));
                memcpy(&buffer[index * elementSize], value, elementSize);
        }

        void moveTo(ArrayBase &other)
        {
                XA_DEBUG_ASSERT(elementSize == other.elementSize);
                other.destroy();
                other.buffer = buffer;
                other.elementSize = elementSize;
                other.size = size;
                other.capacity = capacity;
#if XA_DEBUG_HEAP
                other.memTag = memTag;
#endif
                buffer = nullptr;
                elementSize = size = capacity = 0;
        }

        void pop_back()
        {
                XA_DEBUG_ASSERT(size > 0);
                resize(size - 1, false);
        }

        void push_back(const uint8_t *value)
        {
                XA_DEBUG_ASSERT(value < buffer || value >= buffer + size);
                resize(size + 1, false);
                memcpy(&buffer[(size - 1) * elementSize], value, elementSize);
        }

        void push_back(const ArrayBase &other)
        {
                XA_DEBUG_ASSERT(elementSize == other.elementSize);
                if (other.size == 0)
                        return;
                const uint32_t oldSize = size;
                resize(size + other.size, false);
                memcpy(buffer + oldSize * elementSize, other.buffer, other.size * other.elementSize);
        }

        // Remove the element at the given index. This is an expensive operation!
        void removeAt(uint32_t index)
        {
                XA_DEBUG_ASSERT(index < size); // [Bruno Levy] Removed >= 0 test (this is an uint)
                if (size != 1)
                        memmove(buffer + elementSize * index, buffer + elementSize * (index + 1), elementSize * (size - 1 - index));
                size--;
        }

        void reserve(uint32_t desiredSize)
        {
                if (desiredSize > capacity)
                        setArrayCapacity(desiredSize);
        }

        void resize(uint32_t newSize, bool exact)
        {
                size = newSize;
                if (size > capacity) {
                        // First allocation is always exact. Otherwise, following allocations grow array to 150% of desired size.
                        uint32_t newBufferSize;
                        if (capacity == 0 || exact)
                                newBufferSize = size;
                        else
                                newBufferSize = size + (size >> 2);
                        setArrayCapacity(newBufferSize);
                }
        }

        void setArrayCapacity(uint32_t newCapacity)
        {
                XA_DEBUG_ASSERT(newCapacity >= size);
                if (newCapacity == 0) {
                        // free the buffer.
                        if (buffer != nullptr) {
                                XA_FREE(buffer);
                                buffer = nullptr;
                        }
                } else {
                        // realloc the buffer
#if XA_DEBUG_HEAP
                        buffer = XA_REALLOC_SIZE(memTag, buffer, newCapacity * elementSize);
#else
                        buffer = XA_REALLOC_SIZE(MemTag::Default, buffer, newCapacity * elementSize);
#endif
                }
                capacity = newCapacity;
        }

#if XA_DEBUG_HEAP
        void setMemTag(int memTag)
        {
                this->memTag = memTag;
        }
#endif

        uint8_t *buffer;
        uint32_t elementSize;
        uint32_t size;
        uint32_t capacity;
#if XA_DEBUG_HEAP
        int memTag;
#endif
};

template<typename T>
class Array
{
public:
        Array(int memTag = MemTag::Default) : m_base(sizeof(T), memTag) {}
        Array(const Array&) = delete;
        Array &operator=(const Array &) = delete;

        XA_INLINE const T &operator[](uint32_t index) const
        {
                XA_DEBUG_ASSERT(index < m_base.size);
                return ((const T *)m_base.buffer)[index];
        }

        XA_INLINE T &operator[](uint32_t index)
        {
                XA_DEBUG_ASSERT(index < m_base.size);
                return ((T *)m_base.buffer)[index];
        }

        XA_INLINE const T &back() const
        {
                XA_DEBUG_ASSERT(!isEmpty());
                return ((const T *)m_base.buffer)[m_base.size - 1];
        }

        XA_INLINE T *begin() { return (T *)m_base.buffer; }
        XA_INLINE void clear() { m_base.clear(); }

        bool contains(const T &value) const
        {
                for (uint32_t i = 0; i < m_base.size; i++) {
                        if (((const T *)m_base.buffer)[i] == value)
                                return true;
                }
                return false;
        }

        void copyFrom(const T *data, uint32_t length) { m_base.copyFrom((const uint8_t *)data, length); }
        void copyTo(Array &other) const { m_base.copyTo(other.m_base); }
        XA_INLINE const T *data() const { return (const T *)m_base.buffer; }
        XA_INLINE T *data() { return (T *)m_base.buffer; }
        XA_INLINE T *end() { return (T *)m_base.buffer + m_base.size; }
        XA_INLINE bool isEmpty() const { return m_base.size == 0; }
        void insertAt(uint32_t index, const T &value) { m_base.insertAt(index, (const uint8_t *)&value); }
        void moveTo(Array &other) { m_base.moveTo(other.m_base); }
        void push_back(const T &value) { m_base.push_back((const uint8_t *)&value); }
        void push_back(const Array &other) { m_base.push_back(other.m_base); }
        void pop_back() { m_base.pop_back(); }
        void removeAt(uint32_t index) { m_base.removeAt(index); }
        void reserve(uint32_t desiredSize) { m_base.reserve(desiredSize); }
        void resize(uint32_t newSize) { m_base.resize(newSize, true); }

        void runCtors()
        {
                for (uint32_t i = 0; i < m_base.size; i++)
                        new (&((T *)m_base.buffer)[i]) T;
        }

        void runDtors()
        {
                for (uint32_t i = 0; i < m_base.size; i++)
                        ((T *)m_base.buffer)[i].~T();
        }

        void setAll(const T &value)
        {
                auto buffer = (T *)m_base.buffer;
                for (uint32_t i = 0; i < m_base.size; i++)
                        buffer[i] = value;
        }

#if XA_DEBUG_HEAP
        void setMemTag(int memTag) { m_base.setMemTag(memTag); }
#endif

        XA_INLINE uint32_t size() const { return m_base.size; }
        XA_INLINE void zeroOutMemory() { memset(m_base.buffer, 0, m_base.elementSize * m_base.size); }

private:
        ArrayBase m_base;
};

template<typename T>
struct ArrayView
{
        ArrayView(Array<T> &a) : data(a.data()), length(a.size()) {}
        ArrayView(T *data, uint32_t length) : data(data), length(length) {}
        ArrayView &operator=(Array<T> &a) { data = a.data(); length = a.size(); return *this; }
        XA_INLINE const T &operator[](uint32_t index) const { XA_DEBUG_ASSERT(index < length); return data[index]; }
        T *data;
        uint32_t length;
};

template<typename T>
struct ConstArrayView
{
        ConstArrayView(const Array<T> &a) : data(a.data()), length(a.size()) {}
        ConstArrayView(const T *data, uint32_t length) : data(data), length(length) {}
        ConstArrayView &operator=(const Array<T> &a) { data = a.data(); length = a.size(); return *this; }
        XA_INLINE const T &operator[](uint32_t index) const { XA_DEBUG_ASSERT(index < length); return data[index]; }
        const T *data;
        uint32_t length;
};

/// Basis class to compute tangent space basis, ortogonalizations and to transform vectors from one space to another.
struct Basis
{
        XA_NODISCARD static Vector3 computeTangent(const Vector3 &normal)
        {
                XA_ASSERT(isNormalized(normal));
                // Choose minimum axis.
                Vector3 tangent;
                if (fabsf(normal.x) < fabsf(normal.y) && fabsf(normal.x) < fabsf(normal.z))
                        tangent = Vector3(1, 0, 0);
                else if (fabsf(normal.y) < fabsf(normal.z))
                        tangent = Vector3(0, 1, 0);
                else
                        tangent = Vector3(0, 0, 1);
                // Ortogonalize
                tangent -= normal * dot(normal, tangent);
                tangent = normalize(tangent, kEpsilon);
                return tangent;
        }

        XA_NODISCARD static Vector3 computeBitangent(const Vector3 &normal, const Vector3 &tangent)
        {
                return cross(normal, tangent);
        }

        Vector3 tangent = Vector3(0.0f);
        Vector3 bitangent = Vector3(0.0f);
        Vector3 normal = Vector3(0.0f);
};

// Simple bit array.
class BitArray
{
public:
        BitArray() : m_size(0) {}

        BitArray(uint32_t sz)
        {
                resize(sz);
        }

        void resize(uint32_t new_size)
        {
                m_size = new_size;
                m_wordArray.resize((m_size + 31) >> 5);
        }

        bool get(uint32_t index) const
        {
                XA_DEBUG_ASSERT(index < m_size);
                return (m_wordArray[index >> 5] & (1 << (index & 31))) != 0;
        }

        void set(uint32_t index)
        {
                XA_DEBUG_ASSERT(index < m_size);
                m_wordArray[index >> 5] |=  (1 << (index & 31));
        }

        void zeroOutMemory()
        {
                m_wordArray.zeroOutMemory();
        }

private:
        uint32_t m_size; // Number of bits stored.
        Array<uint32_t> m_wordArray;
};

class BitImage
{
public:
        BitImage() : m_width(0), m_height(0), m_rowStride(0), m_data(MemTag::BitImage) {}

        BitImage(uint32_t w, uint32_t h) : m_width(w), m_height(h), m_data(MemTag::BitImage)
        {
                m_rowStride = (m_width + 63) >> 6;
                m_data.resize(m_rowStride * m_height);
                m_data.zeroOutMemory();
        }

        BitImage(const BitImage &other) = delete;
        BitImage &operator=(const BitImage &other) = delete;
        uint32_t width() const { return m_width; }
        uint32_t height() const { return m_height; }

        void copyTo(BitImage &other)
        {
                other.m_width = m_width;
                other.m_height = m_height;
                other.m_rowStride = m_rowStride;
                m_data.copyTo(other.m_data);
        }

        void resize(uint32_t w, uint32_t h, bool discard)
        {
                const uint32_t rowStride = (w + 63) >> 6;
                if (discard) {
                        m_data.resize(rowStride * h);
                        m_data.zeroOutMemory();
                } else {
                        Array<uint64_t> tmp;
                        tmp.resize(rowStride * h);
                        memset(tmp.data(), 0, tmp.size() * sizeof(uint64_t));
                        // If only height has changed, can copy all rows at once.
                        if (rowStride == m_rowStride) {
                                memcpy(tmp.data(), m_data.data(), m_rowStride * min(m_height, h) * sizeof(uint64_t));
                        } else if (m_width > 0 && m_height > 0) {
                                const uint32_t height = min(m_height, h);
                                for (uint32_t i = 0; i < height; i++)
                                        memcpy(&tmp[i * rowStride], &m_data[i * m_rowStride], min(rowStride, m_rowStride) * sizeof(uint64_t));
                        }
                        tmp.moveTo(m_data);
                }
                m_width = w;
                m_height = h;
                m_rowStride = rowStride;
        }

        bool get(uint32_t x, uint32_t y) const
        {
                XA_DEBUG_ASSERT(x < m_width && y < m_height);
                const uint32_t index = (x >> 6) + y * m_rowStride;
                return (m_data[index] & (UINT64_C(1) << (uint64_t(x) & UINT64_C(63)))) != 0;
        }

        void set(uint32_t x, uint32_t y)
        {
                XA_DEBUG_ASSERT(x < m_width && y < m_height);
                const uint32_t index = (x >> 6) + y * m_rowStride;
                m_data[index] |= UINT64_C(1) << (uint64_t(x) & UINT64_C(63));
                XA_DEBUG_ASSERT(get(x, y));
        }

        void zeroOutMemory()
        {
                m_data.zeroOutMemory();
        }

        bool canBlit(const BitImage &image, uint32_t offsetX, uint32_t offsetY) const
        {
                for (uint32_t y = 0; y < image.m_height; y++) {
                        const uint32_t thisY = y + offsetY;
                        if (thisY >= m_height)
                                continue;
                        uint32_t x = 0;
                        for (;;) {
                                const uint32_t thisX = x + offsetX;
                                if (thisX >= m_width)
                                        break;
                                const uint32_t thisBlockShift = thisX % 64;
                                const uint64_t thisBlock = m_data[(thisX >> 6) + thisY * m_rowStride] >> thisBlockShift;
                                const uint32_t blockShift = x % 64;
                                const uint64_t block = image.m_data[(x >> 6) + y * image.m_rowStride] >> blockShift;
                                if ((thisBlock & block) != 0)
                                        return false;
                                x += 64 - max(thisBlockShift, blockShift);
                                if (x >= image.m_width)
                                        break;
                        }
                }
                return true;
        }

        void dilate(uint32_t padding)
        {
                BitImage tmp(m_width, m_height);
                for (uint32_t p = 0; p < padding; p++) {
                        tmp.zeroOutMemory();
                        for (uint32_t y = 0; y < m_height; y++) {
                                for (uint32_t x = 0; x < m_width; x++) {
                                        bool b = get(x, y);
                                        if (!b) {
                                                if (x > 0) {
                                                        b |= get(x - 1, y);
                                                        if (y > 0) b |= get(x - 1, y - 1);
                                                        if (y < m_height - 1) b |= get(x - 1, y + 1);
                                                }
                                                if (y > 0) b |= get(x, y - 1);
                                                if (y < m_height - 1) b |= get(x, y + 1);
                                                if (x < m_width - 1) {
                                                        b |= get(x + 1, y);
                                                        if (y > 0) b |= get(x + 1, y - 1);
                                                        if (y < m_height - 1) b |= get(x + 1, y + 1);
                                                }
                                        }
                                        if (b)
                                                tmp.set(x, y);
                                }
                        }
                        tmp.m_data.copyTo(m_data);
                }
        }

private:
        uint32_t m_width;
        uint32_t m_height;
        uint32_t m_rowStride; // In uint64_t's
        Array<uint64_t> m_data;
};

// From Fast-BVH
class BVH
{
public:
        BVH(const Array<AABB> &objectAabbs, uint32_t leafSize = 4) : m_objectIds(MemTag::BVH), m_nodes(MemTag::BVH)
        {
                m_objectAabbs = &objectAabbs;
                if (m_objectAabbs->isEmpty())
                        return;
                m_objectIds.resize(objectAabbs.size());
                for (uint32_t i = 0; i < m_objectIds.size(); i++)
                        m_objectIds[i] = i;
                BuildEntry todo[128];
                uint32_t stackptr = 0;
                const uint32_t kRoot = 0xfffffffc;
                const uint32_t kUntouched = 0xffffffff;
                const uint32_t kTouchedTwice = 0xfffffffd;
                // Push the root
                todo[stackptr].start = 0;
                todo[stackptr].end = objectAabbs.size();
                todo[stackptr].parent = kRoot;
                stackptr++;
                Node node;
                m_nodes.reserve(objectAabbs.size() * 2);
                uint32_t nNodes = 0;
                while(stackptr > 0) {
                        // Pop the next item off of the stack
                        const BuildEntry &bnode = todo[--stackptr];
                        const uint32_t start = bnode.start;
                        const uint32_t end = bnode.end;
                        const uint32_t nPrims = end - start;
                        nNodes++;
                        node.start = start;
                        node.nPrims = nPrims;
                        node.rightOffset = kUntouched;
                        // Calculate the bounding box for this node
                        AABB bb(objectAabbs[m_objectIds[start]]);
                        AABB bc(objectAabbs[m_objectIds[start]].centroid());
                        for(uint32_t p = start + 1; p < end; ++p) {
                                bb.expandToInclude(objectAabbs[m_objectIds[p]]);
                                bc.expandToInclude(objectAabbs[m_objectIds[p]].centroid());
                        }
                        node.aabb = bb;
                        // If the number of primitives at this point is less than the leaf
                        // size, then this will become a leaf. (Signified by rightOffset == 0)
                        if (nPrims <= leafSize)
                                node.rightOffset = 0;
                        m_nodes.push_back(node);
                        // Child touches parent...
                        // Special case: Don't do this for the root.
                        if (bnode.parent != kRoot) {
                                m_nodes[bnode.parent].rightOffset--;
                                // When this is the second touch, this is the right child.
                                // The right child sets up the offset for the flat tree.
                                if (m_nodes[bnode.parent].rightOffset == kTouchedTwice )
                                        m_nodes[bnode.parent].rightOffset = nNodes - 1 - bnode.parent;
                        }
                        // If this is a leaf, no need to subdivide.
                        if (node.rightOffset == 0)
                                continue;
                        // Set the split dimensions
                        const uint32_t split_dim = bc.maxDimension();
                        // Split on the center of the longest axis
                        const float split_coord = 0.5f * ((&bc.min.x)[split_dim] + (&bc.max.x)[split_dim]);
                        // Partition the list of objects on this split
                        uint32_t mid = start;
                        for (uint32_t i = start; i < end; ++i) {
                                const Vector3 centroid(objectAabbs[m_objectIds[i]].centroid());
                                if ((&centroid.x)[split_dim] < split_coord) {
                                        swap(m_objectIds[i], m_objectIds[mid]);
                                        ++mid;
                                }
                        }
                        // If we get a bad split, just choose the center...
                        if (mid == start || mid == end)
                                mid = start + (end - start) / 2;
                        // Push right child
                        todo[stackptr].start = mid;
                        todo[stackptr].end = end;
                        todo[stackptr].parent = nNodes - 1;
                        stackptr++;
                        // Push left child
                        todo[stackptr].start = start;
                        todo[stackptr].end = mid;
                        todo[stackptr].parent = nNodes - 1;
                        stackptr++;
                }
        }

        void query(const AABB &queryAabb, Array<uint32_t> &result) const
        {
                result.clear();
                // Working set
                uint32_t todo[64];
                int32_t stackptr = 0;
                // "Push" on the root node to the working set
                todo[stackptr] = 0;
                while(stackptr >= 0) {
                        // Pop off the next node to work on.
                        const int ni = todo[stackptr--];
                        const Node &node = m_nodes[ni];
                        // Is leaf -> Intersect
                        if (node.rightOffset == 0) {
                                for(uint32_t o = 0; o < node.nPrims; ++o) {
                                        const uint32_t obj = node.start + o;
                                        if (queryAabb.intersect((*m_objectAabbs)[m_objectIds[obj]]))
                                                result.push_back(m_objectIds[obj]);
                                }
                        } else { // Not a leaf
                                const uint32_t left = ni + 1;
                                const uint32_t right = ni + node.rightOffset;
                                if (queryAabb.intersect(m_nodes[left].aabb))
                                        todo[++stackptr] = left;
                                if (queryAabb.intersect(m_nodes[right].aabb))
                                        todo[++stackptr] = right;
                        }
                }
        }

private:
        struct BuildEntry
        {
                uint32_t parent; // If non-zero then this is the index of the parent. (used in offsets)
                uint32_t start, end; // The range of objects in the object list covered by this node.
        };

        struct Node
        {
                AABB aabb;
                uint32_t start, nPrims, rightOffset;
        };

        const Array<AABB> *m_objectAabbs;
        Array<uint32_t> m_objectIds;
        Array<Node> m_nodes;
};

struct Fit
{
        static bool computeBasis(const Vector3 *points, uint32_t pointsCount, Basis *basis)
        {
                if (computeLeastSquaresNormal(points, pointsCount, &basis->normal)) {
                        basis->tangent = Basis::computeTangent(basis->normal);
                        basis->bitangent = Basis::computeBitangent(basis->normal, basis->tangent);
                        return true;
                }
                return computeEigen(points, pointsCount, basis);
        }

private:
        // Fit a plane to a collection of points.
        // Fast, and accurate to within a few degrees.
        // Returns None if the points do not span a plane.
        // https://www.ilikebigbits.com/2015_03_04_plane_from_points.html
        static bool computeLeastSquaresNormal(const Vector3 *points, uint32_t pointsCount, Vector3 *normal)
        {
                XA_DEBUG_ASSERT(pointsCount >= 3);
                if (pointsCount == 3) {
                        *normal = normalize(cross(points[2] - points[0], points[1] - points[0]), kEpsilon);
                        return true;
                }
                const float invN = 1.0f / float(pointsCount);
                Vector3 centroid(0.0f);
                for (uint32_t i = 0; i < pointsCount; i++)
                        centroid += points[i];
                centroid *= invN;
                // Calculate full 3x3 covariance matrix, excluding symmetries:
                float xx = 0.0f, xy = 0.0f, xz = 0.0f, yy = 0.0f, yz = 0.0f, zz = 0.0f;
                for (uint32_t i = 0; i < pointsCount; i++) {
                        Vector3 r = points[i] - centroid;
                        xx += r.x * r.x;
                        xy += r.x * r.y;
                        xz += r.x * r.z;
                        yy += r.y * r.y;
                        yz += r.y * r.z;
                        zz += r.z * r.z;
                }
#if 0
                xx *= invN;
                xy *= invN;
                xz *= invN;
                yy *= invN;
                yz *= invN;
                zz *= invN;
                Vector3 weighted_dir(0.0f);
                {
                        float det_x = yy * zz - yz * yz;
                        const Vector3 axis_dir(det_x, xz * yz - xy * zz, xy * yz - xz * yy);
                        float weight = det_x * det_x;
                        if (dot(weighted_dir, axis_dir) < 0.0f)
                                weight = -weight;
                        weighted_dir += axis_dir * weight;
                }
                {
                        float det_y = xx * zz - xz * xz;
                        const Vector3 axis_dir(xz * yz - xy * zz, det_y, xy * xz - yz * xx);
                        float weight = det_y * det_y;
                        if (dot(weighted_dir, axis_dir) < 0.0f)
                                weight = -weight;
                        weighted_dir += axis_dir * weight;
                }
                {
                        float det_z = xx * yy - xy * xy;
                        const Vector3 axis_dir(xy * yz - xz * yy, xy * xz - yz * xx, det_z);
                        float weight = det_z * det_z;
                        if (dot(weighted_dir, axis_dir) < 0.0f)
                                weight = -weight;
                        weighted_dir += axis_dir * weight;
                }
                *normal = normalize(weighted_dir, kEpsilon);
#else
                const float det_x = yy * zz - yz * yz;
                const float det_y = xx * zz - xz * xz;
                const float det_z = xx * yy - xy * xy;
                const float det_max = max(det_x, max(det_y, det_z));
                if (det_max <= 0.0f)
                        return false; // The points don't span a plane
                // Pick path with best conditioning:
                Vector3 dir(0.0f);
                if (det_max == det_x)
                        dir = Vector3(det_x,xz * yz - xy * zz,xy * yz - xz * yy);
                else if (det_max == det_y)
                        dir = Vector3(xz * yz - xy * zz, det_y, xy * xz - yz * xx);
                else if (det_max == det_z)
                        dir = Vector3(xy * yz - xz * yy, xy * xz - yz * xx, det_z);
                const float len = length(dir);
                if (isZero(len, kEpsilon))
                        return false;
                *normal = dir * (1.0f / len);
#endif
                return isNormalized(*normal);
        }

        static bool computeEigen(const Vector3 *points, uint32_t pointsCount, Basis *basis)
        {
                float matrix[6];
                computeCovariance(pointsCount, points, matrix);
                if (matrix[0] == 0 && matrix[3] == 0 && matrix[5] == 0)
                        return false;
                float eigenValues[3];
                Vector3 eigenVectors[3];
                if (!eigenSolveSymmetric3(matrix, eigenValues, eigenVectors))
                        return false;
                basis->normal = normalize(eigenVectors[2], kEpsilon);
                basis->tangent = normalize(eigenVectors[0], kEpsilon);
                basis->bitangent = normalize(eigenVectors[1], kEpsilon);
                return true;
        }

        static Vector3 computeCentroid(int n, const Vector3 * points)
        {
                Vector3 centroid(0.0f);
                for (int i = 0; i < n; i++) {
                        centroid += points[i];
                }
                centroid /= float(n);
                return centroid;
        }

        static Vector3 computeCovariance(int n, const Vector3 * points, float * covariance)
        {
                // compute the centroid
                Vector3 centroid = computeCentroid(n, points);
                // compute covariance matrix
                for (int i = 0; i < 6; i++) {
                        covariance[i] = 0.0f;
                }
                for (int i = 0; i < n; i++) {
                        Vector3 v = points[i] - centroid;
                        covariance[0] += v.x * v.x;
                        covariance[1] += v.x * v.y;
                        covariance[2] += v.x * v.z;
                        covariance[3] += v.y * v.y;
                        covariance[4] += v.y * v.z;
                        covariance[5] += v.z * v.z;
                }
                return centroid;
        }

        // Tridiagonal solver from Charles Bloom.
        // Householder transforms followed by QL decomposition.
        // Seems to be based on the code from Numerical Recipes in C.
        static bool eigenSolveSymmetric3(const float matrix[6], float eigenValues[3], Vector3 eigenVectors[3])
        {
                XA_DEBUG_ASSERT(matrix != nullptr && eigenValues != nullptr && eigenVectors != nullptr);
                float subd[3];
                float diag[3];
                float work[3][3];
                work[0][0] = matrix[0];
                work[0][1] = work[1][0] = matrix[1];
                work[0][2] = work[2][0] = matrix[2];
                work[1][1] = matrix[3];
                work[1][2] = work[2][1] = matrix[4];
                work[2][2] = matrix[5];
                EigenSolver3_Tridiagonal(work, diag, subd);
                if (!EigenSolver3_QLAlgorithm(work, diag, subd)) {
                        for (int i = 0; i < 3; i++) {
                                eigenValues[i] = 0;
                                eigenVectors[i] = Vector3(0);
                        }
                        return false;
                }
                for (int i = 0; i < 3; i++) {
                        eigenValues[i] = (float)diag[i];
                }
                // eigenvectors are the columns; make them the rows :
                for (int i = 0; i < 3; i++) {
                        for (int j = 0; j < 3; j++) {
                                (&eigenVectors[j].x)[i] = (float) work[i][j];
                        }
                }
                // shuffle to sort by singular value :
                if (eigenValues[2] > eigenValues[0] && eigenValues[2] > eigenValues[1]) {
                        swap(eigenValues[0], eigenValues[2]);
                        swap(eigenVectors[0], eigenVectors[2]);
                }
                if (eigenValues[1] > eigenValues[0]) {
                        swap(eigenValues[0], eigenValues[1]);
                        swap(eigenVectors[0], eigenVectors[1]);
                }
                if (eigenValues[2] > eigenValues[1]) {
                        swap(eigenValues[1], eigenValues[2]);
                        swap(eigenVectors[1], eigenVectors[2]);
                }
                XA_DEBUG_ASSERT(eigenValues[0] >= eigenValues[1] && eigenValues[0] >= eigenValues[2]);
                XA_DEBUG_ASSERT(eigenValues[1] >= eigenValues[2]);
                return true;
        }

private:
        static void EigenSolver3_Tridiagonal(float mat[3][3], float *diag, float *subd)
        {
                // Householder reduction T = Q^t M Q
                //   Input:
                //     mat, symmetric 3x3 matrix M
                //   Output:
                //     mat, orthogonal matrix Q
                //     diag, diagonal entries of T
                //     subd, subdiagonal entries of T (T is symmetric)
                const float epsilon = 1e-08f;
                float a = mat[0][0];
                float b = mat[0][1];
                float c = mat[0][2];
                float d = mat[1][1];
                float e = mat[1][2];
                float f = mat[2][2];
                diag[0] = a;
                subd[2] = 0.f;
                if (fabsf(c) >= epsilon) {
                        const float ell = sqrtf(b * b + c * c);
                        b /= ell;
                        c /= ell;
                        const float q = 2 * b * e + c * (f - d);
                        diag[1] = d + c * q;
                        diag[2] = f - c * q;
                        subd[0] = ell;
                        subd[1] = e - b * q;
                        mat[0][0] = 1;
                        mat[0][1] = 0;
                        mat[0][2] = 0;
                        mat[1][0] = 0;
                        mat[1][1] = b;
                        mat[1][2] = c;
                        mat[2][0] = 0;
                        mat[2][1] = c;
                        mat[2][2] = -b;
                } else {
                        diag[1] = d;
                        diag[2] = f;
                        subd[0] = b;
                        subd[1] = e;
                        mat[0][0] = 1;
                        mat[0][1] = 0;
                        mat[0][2] = 0;
                        mat[1][0] = 0;
                        mat[1][1] = 1;
                        mat[1][2] = 0;
                        mat[2][0] = 0;
                        mat[2][1] = 0;
                        mat[2][2] = 1;
                }
        }

        static bool EigenSolver3_QLAlgorithm(float mat[3][3], float *diag, float *subd)
        {
                // QL iteration with implicit shifting to reduce matrix from tridiagonal
                // to diagonal
                const int maxiter = 32;
                for (int ell = 0; ell < 3; ell++) {
                        int iter;
                        for (iter = 0; iter < maxiter; iter++) {
                                int m;
                                for (m = ell; m <= 1; m++) {
                                        float dd = fabsf(diag[m]) + fabsf(diag[m + 1]);
                                        if ( fabsf(subd[m]) + dd == dd )
                                                break;
                                }
                                if ( m == ell )
                                        break;
                                float g = (diag[ell + 1] - diag[ell]) / (2 * subd[ell]);
                                float r = sqrtf(g * g + 1);
                                if ( g < 0 )
                                        g = diag[m] - diag[ell] + subd[ell] / (g - r);
                                else
                                        g = diag[m] - diag[ell] + subd[ell] / (g + r);
                                float s = 1, c = 1, p = 0;
                                for (int i = m - 1; i >= ell; i--) {
                                        float f = s * subd[i], b = c * subd[i];
                                        if ( fabsf(f) >= fabsf(g) ) {
                                                c = g / f;
                                                r = sqrtf(c * c + 1);
                                                subd[i + 1] = f * r;
                                                c *= (s = 1 / r);
                                        } else {
                                                s = f / g;
                                                r = sqrtf(s * s + 1);
                                                subd[i + 1] = g * r;
                                                s *= (c = 1 / r);
                                        }
                                        g = diag[i + 1] - p;
                                        r = (diag[i] - g) * s + 2 * b * c;
                                        p = s * r;
                                        diag[i + 1] = g + p;
                                        g = c * r - b;
                                        for (int k = 0; k < 3; k++) {
                                                f = mat[k][i + 1];
                                                mat[k][i + 1] = s * mat[k][i] + c * f;
                                                mat[k][i] = c * mat[k][i] - s * f;
                                        }
                                }
                                diag[ell] -= p;
                                subd[ell] = g;
                                subd[m] = 0;
                        }
                        if ( iter == maxiter )
                                // should not get here under normal circumstances
                                return false;
                }
                return true;
        }
};

/// Fixed size vector class.
class FullVector
{
public:
        FullVector(uint32_t dim) : m_array(MemTag::FullVector) { m_array.resize(dim); }
        FullVector(const FullVector &v) : m_array(MemTag::FullVector) { v.m_array.copyTo(m_array); }
        FullVector &operator=(const FullVector &v) = delete;
        XA_INLINE uint32_t dimension() const { return m_array.size(); }
        XA_INLINE const float &operator[](uint32_t index) const { return m_array[index]; }
        XA_INLINE float &operator[](uint32_t index) { return m_array[index]; }

        void fill(float f)
        {
                const uint32_t dim = dimension();
                for (uint32_t i = 0; i < dim; i++)
                        m_array[i] = f;
        }

private:
        Array<float> m_array;
};

template<typename Key, typename H = Hash<Key>, typename E = Equal<Key> >
class HashMap
{
public:
        HashMap(int memTag, uint32_t size) : m_memTag(memTag), m_size(size), m_numSlots(0), m_slots(nullptr), m_keys(memTag), m_next(memTag)
        {
        }

        ~HashMap()
        {
                if (m_slots)
                        XA_FREE(m_slots);
        }

        void add(const Key &key)
        {
                if (!m_slots)
                        alloc();
                const uint32_t hash = computeHash(key);
                m_keys.push_back(key);
                m_next.push_back(m_slots[hash]);
                m_slots[hash] = m_next.size() - 1;
        }

        uint32_t get(const Key &key) const
        {
                if (!m_slots)
                        return UINT32_MAX;
                const uint32_t hash = computeHash(key);
                uint32_t i = m_slots[hash];
                E equal;
                while (i != UINT32_MAX) {
                        if (equal(m_keys[i], key))
                                return i;
                        i = m_next[i];
                }
                return UINT32_MAX;
        }

        uint32_t getNext(uint32_t current) const
        {
                uint32_t i = m_next[current];
                E equal;
                while (i != UINT32_MAX) {
                        if (equal(m_keys[i], m_keys[current]))
                                return i;
                        i = m_next[i];
                }
                return UINT32_MAX;
        }

private:
        void alloc()
        {
                XA_DEBUG_ASSERT(m_size > 0);
                m_numSlots = (uint32_t)(m_size * 1.3);
                m_slots = XA_ALLOC_ARRAY(m_memTag, uint32_t, m_numSlots);
                for (uint32_t i = 0; i < m_numSlots; i++)
                        m_slots[i] = UINT32_MAX;
                m_keys.reserve(m_size);
                m_next.reserve(m_size);
        }

        uint32_t computeHash(const Key &key) const
        {
                H hash;
                return hash(key) % m_numSlots;
        }

        int m_memTag;
        uint32_t m_size;
        uint32_t m_numSlots;
        uint32_t *m_slots;
        Array<Key> m_keys;
        Array<uint32_t> m_next;
};

template<typename T>
static void insertionSort(T *data, uint32_t length)
{
        for (int32_t i = 1; i < (int32_t)length; i++) {
                T x = data[i];
                int32_t j = i - 1;
                while (j >= 0 && x < data[j]) {
                        data[j + 1] = data[j];
                        j--;
                }
                data[j + 1] = x;
        }
}

class KISSRng
{
public:
        KISSRng() { reset(); }

        void reset()
        {
                x = 123456789;
                y = 362436000;
                z = 521288629;
                c = 7654321;
        }

        uint32_t getRange(uint32_t range)
        {
                if (range == 0)
                        return 0;
                x = 69069 * x + 12345;
                y ^= (y << 13);
                y ^= (y >> 17);
                y ^= (y << 5);
                uint64_t t = 698769069ULL * z + c;
                c = (t >> 32);
                return (x + y + (z = (uint32_t)t)) % range;
        }

private:
        uint32_t x, y, z, c;
};

// Based on Pierre Terdiman's and Michael Herf's source code.
// http://www.codercorner.com/RadixSortRevisited.htm
// http://www.stereopsis.com/radix.html
class RadixSort
{
public:
        RadixSort() : m_size(0), m_ranks(nullptr), m_ranks2(nullptr), m_validRanks(false) {}

        ~RadixSort()
        {
                // Release everything
                XA_FREE(m_ranks2);
                XA_FREE(m_ranks);
        }

        RadixSort &sort(const float *input, uint32_t count)
        {
                if (input == nullptr || count == 0) return *this;
                // Resize lists if needed
                if (count != m_size) {
                        if (count > m_size) {
                                m_ranks2 = XA_REALLOC(MemTag::Default, m_ranks2, uint32_t, count);
                                m_ranks = XA_REALLOC(MemTag::Default, m_ranks, uint32_t, count);
                        }
                        m_size = count;
                        m_validRanks = false;
                }
                if (count < 32) {
                        insertionSort(input, count);
                } else {
                        // @@ Avoid touching the input multiple times.
                        for (uint32_t i = 0; i < count; i++) {
                                FloatFlip((uint32_t &)input[i]);
                        }
                        radixSort<uint32_t>((const uint32_t *)input, count);
                        for (uint32_t i = 0; i < count; i++) {
                                IFloatFlip((uint32_t &)input[i]);
                        }
                }
                return *this;
        }

        RadixSort &sort(const Array<float> &input)
        {
                return sort(input.data(), input.size());
        }

        // Access to results. m_ranks is a list of indices in sorted order, i.e. in the order you may further process your data
        const uint32_t *ranks() const
        {
                XA_DEBUG_ASSERT(m_validRanks);
                return m_ranks;
        }

        uint32_t *ranks()
        {
                XA_DEBUG_ASSERT(m_validRanks);
                return m_ranks;
        }

private:
        uint32_t m_size;
        uint32_t *m_ranks;
        uint32_t *m_ranks2;
        bool m_validRanks;

        void FloatFlip(uint32_t &f)
        {
                int32_t mask = (int32_t(f) >> 31) | 0x80000000; // Warren Hunt, Manchor Ko.
                f ^= mask;
        }

        void IFloatFlip(uint32_t &f)
        {
                uint32_t mask = ((f >> 31) - 1) | 0x80000000; // Michael Herf.
                f ^= mask;
        }

        template<typename T>
        void createHistograms(const T *buffer, uint32_t count, uint32_t *histogram)
        {
                const uint32_t bucketCount = sizeof(T); // (8 * sizeof(T)) / log2(radix)
                // Init bucket pointers.
                uint32_t *h[bucketCount];
                for (uint32_t i = 0; i < bucketCount; i++) {
                        h[i] = histogram + 256 * i;
                }
                // Clear histograms.
                memset(histogram, 0, 256 * bucketCount * sizeof(uint32_t ));
                // @@ Add support for signed integers.
                // Build histograms.
                const uint8_t *p = (const uint8_t *)buffer;  // @@ Does this break aliasing rules?
                const uint8_t *pe = p + count * sizeof(T);
                while (p != pe) {
                        h[0][*p++]++, h[1][*p++]++, h[2][*p++]++, h[3][*p++]++;
#ifdef _MSC_VER
#pragma warning(push)
#pragma warning(disable : 4127)
#endif
                        if (bucketCount == 8) h[4][*p++]++, h[5][*p++]++, h[6][*p++]++, h[7][*p++]++;
#ifdef _MSC_VER
#pragma warning(pop)
#endif
                }
        }

        template <typename T> void insertionSort(const T *input, uint32_t count)
        {
                if (!m_validRanks) {
                        m_ranks[0] = 0;
                        for (uint32_t i = 1; i != count; ++i) {
                                int rank = m_ranks[i] = i;
                                uint32_t j = i;
                                while (j != 0 && input[rank] < input[m_ranks[j - 1]]) {
                                        m_ranks[j] = m_ranks[j - 1];
                                        --j;
                                }
                                if (i != j) {
                                        m_ranks[j] = rank;
                                }
                        }
                        m_validRanks = true;
                } else {
                        for (uint32_t i = 1; i != count; ++i) {
                                int rank = m_ranks[i];
                                uint32_t j = i;
                                while (j != 0 && input[rank] < input[m_ranks[j - 1]]) {
                                        m_ranks[j] = m_ranks[j - 1];
                                        --j;
                                }
                                if (i != j) {
                                        m_ranks[j] = rank;
                                }
                        }
                }
        }

        template <typename T> void radixSort(const T *input, uint32_t count)
        {
                const uint32_t P = sizeof(T); // pass count
                // Allocate histograms & offsets on the stack
                uint32_t histogram[256 * P];
                uint32_t *link[256];
                createHistograms(input, count, histogram);
                // Radix sort, j is the pass number (0=LSB, P=MSB)
                for (uint32_t j = 0; j < P; j++) {
                        // Pointer to this bucket.
                        const uint32_t *h = &histogram[j * 256];
                        const uint8_t *inputBytes = (const uint8_t *)input; // @@ Is this aliasing legal?
                        inputBytes += j;
                        if (h[inputBytes[0]] == count) {
                                // Skip this pass, all values are the same.
                                continue;
                        }
                        // Create offsets
                        link[0] = m_ranks2;
                        for (uint32_t i = 1; i < 256; i++) link[i] = link[i - 1] + h[i - 1];
                        // Perform Radix Sort
                        if (!m_validRanks) {
                                for (uint32_t i = 0; i < count; i++) {
                                        *link[inputBytes[i * P]]++ = i;
                                }
                                m_validRanks = true;
                        } else {
                                for (uint32_t i = 0; i < count; i++) {
                                        const uint32_t idx = m_ranks[i];
                                        *link[inputBytes[idx * P]]++ = idx;
                                }
                        }
                        // Swap pointers for next pass. Valid indices - the most recent ones - are in m_ranks after the swap.
                        swap(m_ranks, m_ranks2);
                }
                // All values were equal, generate linear ranks.
                if (!m_validRanks) {
                        for (uint32_t i = 0; i < count; i++) {
                                m_ranks[i] = i;
                        }
                        m_validRanks = true;
                }
        }
};

// Wrapping this in a class allows temporary arrays to be re-used.
class BoundingBox2D
{
public:
        Vector2 majorAxis, minorAxis, minCorner, maxCorner;

        void clear()
        {
                m_boundaryVertices.clear();
        }

        void appendBoundaryVertex(Vector2 v)
        {
                m_boundaryVertices.push_back(v);
        }

        // This should compute convex hull and use rotating calipers to find the best box. Currently it uses a brute force method.
        // If vertices is null or vertexCount is 0, the boundary vertices are used.
        void compute(const Vector2 *vertices = nullptr, uint32_t vertexCount = 0)
        {
                if (!vertices || vertexCount == 0) {
                        vertices = m_boundaryVertices.data();
                        vertexCount = m_boundaryVertices.size();
                }
                convexHull(m_boundaryVertices.data(), m_boundaryVertices.size(), m_hull, 0.00001f);
                // @@ Ideally I should use rotating calipers to find the best box. Using brute force for now.
                float best_area = FLT_MAX;
                Vector2 best_min(0);
                Vector2 best_max(0);
                Vector2 best_axis(0);
                const uint32_t hullCount = m_hull.size();
                for (uint32_t i = 0, j = hullCount - 1; i < hullCount; j = i, i++) {
                        if (equal(m_hull[i], m_hull[j], kEpsilon))
                                continue;
                        Vector2 axis = normalize(m_hull[i] - m_hull[j], 0.0f);
                        XA_DEBUG_ASSERT(isFinite(axis));
                        // Compute bounding box.
                        Vector2 box_min(FLT_MAX, FLT_MAX);
                        Vector2 box_max(-FLT_MAX, -FLT_MAX);
                        // Consider all points, not only boundary points, in case the input chart is malformed.
                        for (uint32_t v = 0; v < vertexCount; v++) {
                                const Vector2 &point = vertices[v];
                                const float x = dot(axis, point);
                                const float y = dot(Vector2(-axis.y, axis.x), point);
                                box_min.x = min(box_min.x, x);
                                box_max.x = max(box_max.x, x);
                                box_min.y = min(box_min.y, y);
                                box_max.y = max(box_max.y, y);
                        }
                        // Compute box area.
                        const float area = (box_max.x - box_min.x) * (box_max.y - box_min.y);
                        if (area < best_area) {
                                best_area = area;
                                best_min = box_min;
                                best_max = box_max;
                                best_axis = axis;
                        }
                }
                majorAxis = best_axis;
                minorAxis = Vector2(-best_axis.y, best_axis.x);
                minCorner = best_min;
                maxCorner = best_max;
                XA_ASSERT(isFinite(majorAxis) && isFinite(minorAxis) && isFinite(minCorner));
        }

private:
        // Compute the convex hull using Graham Scan.
        void convexHull(const Vector2 *input, uint32_t inputCount, Array<Vector2> &output, float epsilon)
        {
                m_coords.resize(inputCount);
                for (uint32_t i = 0; i < inputCount; i++)
                        m_coords[i] = input[i].x;
                RadixSort radix;
                radix.sort(m_coords);
                const uint32_t *ranks = radix.ranks();
                m_top.clear();
                m_bottom.clear();
                m_top.reserve(inputCount);
                m_bottom.reserve(inputCount);
                Vector2 P = input[ranks[0]];
                Vector2 Q = input[ranks[inputCount - 1]];
                float topy = max(P.y, Q.y);
                float boty = min(P.y, Q.y);
                for (uint32_t i = 0; i < inputCount; i++) {
                        Vector2 p = input[ranks[i]];
                        if (p.y >= boty)
                                m_top.push_back(p);
                }
                for (uint32_t i = 0; i < inputCount; i++) {
                        Vector2 p = input[ranks[inputCount - 1 - i]];
                        if (p.y <= topy)
                                m_bottom.push_back(p);
                }
                // Filter top list.
                output.clear();
                XA_DEBUG_ASSERT(m_top.size() >= 2);
                output.push_back(m_top[0]);
                output.push_back(m_top[1]);
                for (uint32_t i = 2; i < m_top.size(); ) {
                        Vector2 a = output[output.size() - 2];
                        Vector2 b = output[output.size() - 1];
                        Vector2 c = m_top[i];
                        float area = triangleArea(a, b, c);
                        if (area >= -epsilon)
                                output.pop_back();
                        if (area < -epsilon || output.size() == 1) {
                                output.push_back(c);
                                i++;
                        }
                }
                uint32_t top_count = output.size();
                XA_DEBUG_ASSERT(m_bottom.size() >= 2);
                output.push_back(m_bottom[1]);
                // Filter bottom list.
                for (uint32_t i = 2; i < m_bottom.size(); ) {
                        Vector2 a = output[output.size() - 2];
                        Vector2 b = output[output.size() - 1];
                        Vector2 c = m_bottom[i];
                        float area = triangleArea(a, b, c);
                        if (area >= -epsilon)
                                output.pop_back();
                        if (area < -epsilon || output.size() == top_count) {
                                output.push_back(c);
                                i++;
                        }
                }
                // Remove duplicate element.
                XA_DEBUG_ASSERT(output.size() > 0);
                output.pop_back();
        }

        Array<Vector2> m_boundaryVertices;
        Array<float> m_coords;
        Array<Vector2> m_top, m_bottom, m_hull;
};

static uint32_t meshEdgeFace(uint32_t edge) { return edge / 3; }
static uint32_t meshEdgeIndex0(uint32_t edge) { return edge; }

static uint32_t meshEdgeIndex1(uint32_t edge)
{
        const uint32_t faceFirstEdge = edge / 3 * 3;
        return faceFirstEdge + (edge - faceFirstEdge + 1) % 3;
}

struct MeshFlags
{
        enum
        {
                HasFaceGroups = 1<<0,
                HasIgnoredFaces = 1<<1,
                HasNormals = 1<<2
        };
};

class Mesh;
static void meshGetBoundaryLoops(const Mesh &mesh, Array<uint32_t> &boundaryLoops);

class Mesh
{
public:
        Mesh(float epsilon, uint32_t approxVertexCount, uint32_t approxFaceCount, uint32_t flags = 0, uint32_t id = UINT32_MAX) : m_epsilon(epsilon), m_flags(flags), m_id(id), m_faceIgnore(MemTag::Mesh), m_ignoredFaceCount(0), m_indices(MemTag::MeshIndices), m_positions(MemTag::MeshPositions), m_normals(MemTag::MeshNormals), m_texcoords(MemTag::MeshTexcoords), m_faceGroups(MemTag::Mesh), m_faceGroupFirstFace(MemTag::Mesh), m_faceGroupNextFace(MemTag::Mesh), m_faceGroupFaceCounts(MemTag::Mesh), m_colocalVertexCount(0), m_nextColocalVertex(MemTag::MeshColocals), m_boundaryEdges(MemTag::MeshBoundaries), m_oppositeEdges(MemTag::MeshBoundaries), m_nextBoundaryEdges(MemTag::MeshBoundaries), m_edgeMap(MemTag::MeshEdgeMap, approxFaceCount * 3)
        {
                m_indices.reserve(approxFaceCount * 3);
                m_positions.reserve(approxVertexCount);
                m_texcoords.reserve(approxVertexCount);
                if (m_flags & MeshFlags::HasFaceGroups)
                        m_faceGroups.reserve(approxFaceCount);
                if (m_flags & MeshFlags::HasIgnoredFaces)
                        m_faceIgnore.reserve(approxFaceCount);
                if (m_flags & MeshFlags::HasNormals)
                        m_normals.reserve(approxVertexCount);
        }

        static constexpr uint16_t kInvalidFaceGroup = UINT16_MAX;
        uint32_t flags() const { return m_flags; }
        uint32_t id() const { return m_id; }

        void addVertex(const Vector3 &pos, const Vector3 &normal = Vector3(0.0f), const Vector2 &texcoord = Vector2(0.0f))
        {
                XA_DEBUG_ASSERT(isFinite(pos));
                m_positions.push_back(pos);
                if (m_flags & MeshFlags::HasNormals)
                        m_normals.push_back(normal);
                m_texcoords.push_back(texcoord);
        }

        struct AddFaceResult
        {
                enum Enum
                {
                        OK,
                        DuplicateEdge = 1
                };
        };

        AddFaceResult::Enum addFace(uint32_t v0, uint32_t v1, uint32_t v2, bool ignore = false, bool hashEdge = true)
        {
                uint32_t indexArray[3];
                indexArray[0] = v0;
                indexArray[1] = v1;
                indexArray[2] = v2;
                return addFace(indexArray, ignore, hashEdge);
        }

        AddFaceResult::Enum addFace(const uint32_t *indices, bool ignore = false, bool hashEdge = true)
        {
                AddFaceResult::Enum result = AddFaceResult::OK;
                if (m_flags & MeshFlags::HasFaceGroups)
                        m_faceGroups.push_back(kInvalidFaceGroup);
                if (m_flags & MeshFlags::HasIgnoredFaces) {
                        m_faceIgnore.push_back(ignore);
                        if (ignore)
                                m_ignoredFaceCount++;
                }
                const uint32_t firstIndex = m_indices.size();
                for (uint32_t i = 0; i < 3; i++)
                        m_indices.push_back(indices[i]);
                if (hashEdge) {
                        for (uint32_t i = 0; i < 3; i++) {
                                const uint32_t vertex0 = m_indices[firstIndex + i];
                                const uint32_t vertex1 = m_indices[firstIndex + (i + 1) % 3];
                                const EdgeKey key(vertex0, vertex1);
                                if (m_edgeMap.get(key) != UINT32_MAX)
                                        result = AddFaceResult::DuplicateEdge;
                                m_edgeMap.add(key);
                        }
                }
                return result;
        }

        void createColocals()
        {
                const uint32_t vertexCount = m_positions.size();
                Array<AABB> aabbs(MemTag::BVH);
                aabbs.resize(vertexCount);
                for (uint32_t i = 0; i < m_positions.size(); i++)
                        aabbs[i] = AABB(m_positions[i], m_epsilon);
                BVH bvh(aabbs);
                Array<uint32_t> colocals(MemTag::MeshColocals);
                Array<uint32_t> potential(MemTag::MeshColocals);
                m_colocalVertexCount = 0;
                m_nextColocalVertex.resize(vertexCount);
                for (uint32_t i = 0; i < vertexCount; i++)
                        m_nextColocalVertex[i] = UINT32_MAX;
                for (uint32_t i = 0; i < vertexCount; i++) {
                        if (m_nextColocalVertex[i] != UINT32_MAX)
                                continue; // Already linked.
                        // Find other vertices colocal to this one.
                        colocals.clear();
                        colocals.push_back(i); // Always add this vertex.
                        bvh.query(AABB(m_positions[i], m_epsilon), potential);
                        for (uint32_t j = 0; j < potential.size(); j++) {
                                const uint32_t otherVertex = potential[j];
                                if (otherVertex != i && equal(m_positions[i], m_positions[otherVertex], m_epsilon) && m_nextColocalVertex[otherVertex] == UINT32_MAX)
                                        colocals.push_back(otherVertex);
                        }
                        if (colocals.size() == 1) {
                                // No colocals for this vertex.
                                m_nextColocalVertex[i] = i;
                                continue;
                        }
                        m_colocalVertexCount += colocals.size();
                        // Link in ascending order.
                        insertionSort(colocals.data(), colocals.size());
                        for (uint32_t j = 0; j < colocals.size(); j++)
                                m_nextColocalVertex[colocals[j]] = colocals[(j + 1) % colocals.size()];
                        XA_DEBUG_ASSERT(m_nextColocalVertex[i] != UINT32_MAX);
                }
        }

        // Check if the face duplicates any edges of any face already in the group.
        bool faceDuplicatesGroupEdge(uint16_t group, uint32_t face) const
        {
                for (FaceEdgeIterator edgeIt(this, face); !edgeIt.isDone(); edgeIt.advance()) {
                        for (ColocalEdgeIterator colocalEdgeIt(this, edgeIt.vertex0(), edgeIt.vertex1()); !colocalEdgeIt.isDone(); colocalEdgeIt.advance()) {
                                if (m_faceGroups[meshEdgeFace(colocalEdgeIt.edge())] == group)
                                        return true;
                        }
                }
                return false;
        }

        void createFaceGroups()
        {
                uint32_t firstUnassignedFace = 0;
                uint16_t group = 0;
                Array<uint32_t> growFaces;
                const uint32_t n = faceCount();
                m_faceGroupNextFace.resize(n);
                for (;;) {
                        // Find an unassigned face.
                        uint32_t face = UINT32_MAX;
                        for (uint32_t f = firstUnassignedFace; f < n; f++) {
                                if (m_faceGroups[f] == kInvalidFaceGroup && !isFaceIgnored(f)) {
                                        face = f;
                                        firstUnassignedFace = f + 1;
                                        break;
                                }
                        }
                        if (face == UINT32_MAX)
                                break; // All faces assigned to a group (except ignored faces).
                        m_faceGroups[face] = group;
                        m_faceGroupNextFace[face] = UINT32_MAX;
                        m_faceGroupFirstFace.push_back(face);
                        growFaces.clear();
                        growFaces.push_back(face);
                        uint32_t prevFace = face, groupFaceCount = 1;
                        // Find faces connected to the face and assign them to the same group as the face, unless they are already assigned to another group.
                        for (;;) {
                                if (growFaces.isEmpty())
                                        break;
                                const uint32_t f = growFaces.back();
                                growFaces.pop_back();
                                for (FaceEdgeIterator edgeIt(this, f); !edgeIt.isDone(); edgeIt.advance()) {
                                        // Iterate opposite edges. There may be more than one - non-manifold geometry can have duplicate edges.
                                        // Prioritize the one with exact vertex match, not just colocal.
                                        // If *any* of the opposite edges are already assigned to this group, don't do anything.
                                        bool alreadyAssignedToThisGroup = false;
                                        uint32_t bestConnectedFace = UINT32_MAX;
                                        for (ColocalEdgeIterator oppositeEdgeIt(this, edgeIt.vertex1(), edgeIt.vertex0()); !oppositeEdgeIt.isDone(); oppositeEdgeIt.advance()) {
                                                const uint32_t oppositeEdge = oppositeEdgeIt.edge();
                                                const uint32_t oppositeFace = meshEdgeFace(oppositeEdge);
                                                if (isFaceIgnored(oppositeFace))
                                                        continue; // Don't add ignored faces to group.
                                                if (m_faceGroups[oppositeFace] == group) {
                                                        alreadyAssignedToThisGroup = true;
                                                        break;
                                                }
                                                if (m_faceGroups[oppositeFace] != kInvalidFaceGroup)
                                                        continue; // Connected face is already assigned to another group.
                                                if (faceDuplicatesGroupEdge(group, oppositeFace))
                                                        continue; // Don't want duplicate edges in a group.
                                                const uint32_t oppositeVertex0 = m_indices[meshEdgeIndex0(oppositeEdge)];
                                                const uint32_t oppositeVertex1 = m_indices[meshEdgeIndex1(oppositeEdge)];
                                                if (bestConnectedFace == UINT32_MAX || (oppositeVertex0 == edgeIt.vertex1() && oppositeVertex1 == edgeIt.vertex0()))
                                                        bestConnectedFace = oppositeFace;
#if 0
                                                else {
                                                        // Choose the opposite face with the smallest dihedral angle.
                                                        const float d1 = 1.0f - dot(computeFaceNormal(f), computeFaceNormal(bestConnectedFace));
                                                        const float d2 = 1.0f - dot(computeFaceNormal(f), computeFaceNormal(oppositeFace));
                                                        if (d2 < d1)
                                                                bestConnectedFace = oppositeFace;
                                                }
#endif
                                        }
                                        if (!alreadyAssignedToThisGroup && bestConnectedFace != UINT32_MAX) {
                                                m_faceGroups[bestConnectedFace] = group;
                                                m_faceGroupNextFace[bestConnectedFace] = UINT32_MAX;
                                                if (prevFace != UINT32_MAX)
                                                        m_faceGroupNextFace[prevFace] = bestConnectedFace;
                                                prevFace = bestConnectedFace;
                                                groupFaceCount++;
                                                growFaces.push_back(bestConnectedFace);
                                        }
                                }
                        }
                        m_faceGroupFaceCounts.push_back(groupFaceCount);
                        group++;
                        XA_ASSERT(group < kInvalidFaceGroup);
                }
        }

        void createBoundaries()
        {
                const uint32_t edgeCount = m_indices.size();
                const uint32_t vertexCount = m_positions.size();
                m_oppositeEdges.resize(edgeCount);
                m_boundaryEdges.reserve(uint32_t(edgeCount * 0.1f));
                m_isBoundaryVertex.resize(vertexCount);
                m_isBoundaryVertex.zeroOutMemory();
                for (uint32_t i = 0; i < edgeCount; i++)
                        m_oppositeEdges[i] = UINT32_MAX;
                const uint32_t faceCount = m_indices.size() / 3;
                for (uint32_t i = 0; i < faceCount; i++) {
                        if (isFaceIgnored(i))
                                continue;
                        for (uint32_t j = 0; j < 3; j++) {
                                const uint32_t edge = i * 3 + j;
                                const uint32_t vertex0 = m_indices[edge];
                                const uint32_t vertex1 = m_indices[i * 3 + (j + 1) % 3];
                                // If there is an edge with opposite winding to this one, the edge isn't on a boundary.
                                const uint32_t oppositeEdge = findEdge(vertex1, vertex0);
                                if (oppositeEdge != UINT32_MAX) {
                                        m_oppositeEdges[edge] = oppositeEdge;
                                } else {
                                        m_boundaryEdges.push_back(edge);
                                        m_isBoundaryVertex.set(vertex0);
                                        m_isBoundaryVertex.set(vertex1);
                                }
                        }
                }
        }

        void linkBoundaries()
        {
                const uint32_t edgeCount = m_indices.size();
                HashMap<uint32_t> vertexToEdgeMap(MemTag::Mesh, edgeCount); // Edge is index / 2
                for (uint32_t i = 0; i < edgeCount; i++) {
                        vertexToEdgeMap.add(m_indices[meshEdgeIndex0(i)]);
                        vertexToEdgeMap.add(m_indices[meshEdgeIndex1(i)]);
                }
                m_nextBoundaryEdges.resize(edgeCount);
                for (uint32_t i = 0; i < edgeCount; i++)
                        m_nextBoundaryEdges[i] = UINT32_MAX;
                uint32_t numBoundaryLoops = 0, numUnclosedBoundaries = 0;
                BitArray linkedEdges(edgeCount);
                linkedEdges.zeroOutMemory();
                for (;;) {
                        // Find the first boundary edge that hasn't been linked yet.
                        uint32_t firstEdge = UINT32_MAX;
                        for (uint32_t i = 0; i < edgeCount; i++) {
                                if (m_oppositeEdges[i] == UINT32_MAX && !linkedEdges.get(i)) {
                                        firstEdge = i;
                                        break;
                                }
                        }
                        if (firstEdge == UINT32_MAX)
                                break;
                        uint32_t currentEdge = firstEdge;
                        for (;;) {
                                // Find the next boundary edge. The first vertex will be the same as (or colocal to) the current edge second vertex.
                                const uint32_t startVertex = m_indices[meshEdgeIndex1(currentEdge)];
                                uint32_t bestNextEdge = UINT32_MAX;
                                for (ColocalVertexIterator it(this, startVertex); !it.isDone(); it.advance()) {
                                        uint32_t mapIndex = vertexToEdgeMap.get(it.vertex());
                                        while (mapIndex != UINT32_MAX) {
                                                const uint32_t otherEdge = mapIndex / 2; // Two vertices added per edge.
                                                if (m_oppositeEdges[otherEdge] != UINT32_MAX)
                                                        goto next; // Not a boundary edge.
                                                if (linkedEdges.get(otherEdge))
                                                        goto next; // Already linked.
                                                if (m_indices[meshEdgeIndex0(otherEdge)] != it.vertex())
                                                        goto next; // Edge contains the vertex, but it's the wrong one.
                                                // First edge (closing the boundary loop) has the highest priority.
                                                // Non-colocal vertex has the next highest.
                                                if (bestNextEdge != firstEdge && (bestNextEdge == UINT32_MAX || it.vertex() == startVertex))
                                                        bestNextEdge = otherEdge;
                                        next:
                                                mapIndex = vertexToEdgeMap.getNext(mapIndex);
                                        }
                                }
                                if (bestNextEdge == UINT32_MAX) {
                                        numUnclosedBoundaries++;
                                        if (currentEdge == firstEdge)
                                                linkedEdges.set(firstEdge); // Only 1 edge in this boundary "loop".
                                        break; // Can't find a next edge.
                                }
                                m_nextBoundaryEdges[currentEdge] = bestNextEdge;
                                linkedEdges.set(bestNextEdge);
                                currentEdge = bestNextEdge;
                                if (currentEdge == firstEdge) {
                                        numBoundaryLoops++;
                                        break; // Closed the boundary loop.
                                }
                        }
                }
#if XA_FIX_INTERNAL_BOUNDARY_LOOPS
                // Find internal boundary loops and separate them.
                // Detect by finding two edges in a boundary loop that have a colocal end vertex.
                // Fix by swapping their next boundary edge.
                // Need to start over after every fix since known boundary loops have changed.
                Array<uint32_t> boundaryLoops;
        fixInternalBoundary:
                meshGetBoundaryLoops(*this, boundaryLoops);
                for (uint32_t loop = 0; loop < boundaryLoops.size(); loop++) {
                        linkedEdges.zeroOutMemory();
                        for (Mesh::BoundaryLoopEdgeIterator it1(this, boundaryLoops[loop]); !it1.isDone(); it1.advance()) {
                                const uint32_t e1 = it1.edge();
                                if (linkedEdges.get(e1))
                                        continue;
                                for (Mesh::BoundaryLoopEdgeIterator it2(this, boundaryLoops[loop]); !it2.isDone(); it2.advance()) {
                                        const uint32_t e2 = it2.edge();
                                        if (e1 == e2 || !isBoundaryEdge(e2) || linkedEdges.get(e2))
                                                continue;
                                        if (!areColocal(m_indices[meshEdgeIndex1(e1)], m_indices[meshEdgeIndex1(e2)]))
                                                continue;
                                        swap(m_nextBoundaryEdges[e1], m_nextBoundaryEdges[e2]);
                                        linkedEdges.set(e1);
                                        linkedEdges.set(e2);
                                        goto fixInternalBoundary; // start over
                                }
                        }
                }
#endif
        }

        /// Find edge, test all colocals.
        uint32_t findEdge(uint32_t vertex0, uint32_t vertex1) const
        {
                uint32_t result = UINT32_MAX;
                if (m_nextColocalVertex.isEmpty()) {
                        EdgeKey key(vertex0, vertex1);
                        uint32_t edge = m_edgeMap.get(key);
                        while (edge != UINT32_MAX) {
                                // Don't find edges of ignored faces.
                                if (!isFaceIgnored(meshEdgeFace(edge))) {
                                        //XA_DEBUG_ASSERT(m_id != UINT32_MAX || (m_id == UINT32_MAX && result == UINT32_MAX)); // duplicate edge - ignore on initial meshes
                                        result = edge;
#if !XA_DEBUG
                                        return result;
#endif
                                }
                                edge = m_edgeMap.getNext(edge);
                        }
                } else {
                        for (ColocalVertexIterator it0(this, vertex0); !it0.isDone(); it0.advance()) {
                                for (ColocalVertexIterator it1(this, vertex1); !it1.isDone(); it1.advance()) {
                                        EdgeKey key(it0.vertex(), it1.vertex());
                                        uint32_t edge = m_edgeMap.get(key);
                                        while (edge != UINT32_MAX) {
                                                // Don't find edges of ignored faces.
                                                if (!isFaceIgnored(meshEdgeFace(edge))) {
                                                        XA_DEBUG_ASSERT(m_id != UINT32_MAX || (m_id == UINT32_MAX && result == UINT32_MAX)); // duplicate edge - ignore on initial meshes
                                                        result = edge;
#if !XA_DEBUG
                                                        return result;
#endif
                                                }
                                                edge = m_edgeMap.getNext(edge);
                                        }
                                }
                        }
                }
                return result;
        }

#if XA_DEBUG_EXPORT_OBJ
        void writeObjVertices(FILE *file) const
        {
                for (uint32_t i = 0; i < m_positions.size(); i++)
                        fprintf(file, "v %g %g %g\n", m_positions[i].x, m_positions[i].y, m_positions[i].z);
                if (m_flags & MeshFlags::HasNormals) {
                        for (uint32_t i = 0; i < m_normals.size(); i++)
                                fprintf(file, "vn %g %g %g\n", m_normals[i].x, m_normals[i].y, m_normals[i].z);
                }
                for (uint32_t i = 0; i < m_texcoords.size(); i++)
                        fprintf(file, "vt %g %g\n", m_texcoords[i].x, m_texcoords[i].y);
        }

        void writeObjFace(FILE *file, uint32_t face) const
        {
                fprintf(file, "f ");
                for (uint32_t j = 0; j < 3; j++) {
                        const uint32_t index = m_indices[face * 3 + j] + 1; // 1-indexed
                        fprintf(file, "%d/%d/%d%c", index, index, index, j == 2 ? '\n' : ' ');
                }
        }

        void writeObjBoundaryEges(FILE *file) const
        {
                if (m_oppositeEdges.isEmpty())
                        return; // Boundaries haven't been created.
                fprintf(file, "o boundary_edges\n");
                for (uint32_t i = 0; i < edgeCount(); i++) {
                        if (m_oppositeEdges[i] != UINT32_MAX)
                                continue;
                        fprintf(file, "l %d %d\n", m_indices[meshEdgeIndex0(i)] + 1, m_indices[meshEdgeIndex1(i)] + 1); // 1-indexed
                }
        }

        void writeObjLinkedBoundaries(FILE *file) const
        {
                if (m_oppositeEdges.isEmpty() || m_nextBoundaryEdges.isEmpty())
                        return; // Boundaries haven't been created and/or linked.
                Array<uint32_t> boundaryLoops;
                meshGetBoundaryLoops(*this, boundaryLoops);
                for (uint32_t i = 0; i < boundaryLoops.size(); i++) {
                        uint32_t edge = boundaryLoops[i];
                        fprintf(file, "o boundary_%04d\n", i);
                        fprintf(file, "l");
                        for (;;) {
                                const uint32_t vertex0 = m_indices[meshEdgeIndex0(edge)];
                                const uint32_t vertex1 = m_indices[meshEdgeIndex1(edge)];
                                fprintf(file, " %d", vertex0 + 1); // 1-indexed
                                edge = m_nextBoundaryEdges[edge];
                                if (edge == boundaryLoops[i] || edge == UINT32_MAX) {
                                        fprintf(file, " %d\n", vertex1 + 1); // 1-indexed
                                        break;
                                }
                        }
                }
        }

        void writeObjFile(const char *filename) const
        {
                FILE *file;
                XA_FOPEN(file, filename, "w");
                if (!file)
                        return;
                writeObjVertices(file);
                fprintf(file, "s off\n");
                fprintf(file, "o object\n");
                for (uint32_t i = 0; i < faceCount(); i++)
                        writeObjFace(file, i);
                writeObjBoundaryEges(file);
                writeObjLinkedBoundaries(file);
                fclose(file);
        }
#endif

        float computeSurfaceArea() const
        {
                float area = 0;
                for (uint32_t f = 0; f < faceCount(); f++)
                        area += computeFaceArea(f);
                XA_DEBUG_ASSERT(area >= 0);
                return area;
        }

        float computeParametricArea() const
        {
                float area = 0;
                for (uint32_t f = 0; f < faceCount(); f++)
                        area += computeFaceParametricArea(f);
                return fabsf(area); // May be negative, depends on texcoord winding.
        }

        float computeFaceArea(uint32_t face) const
        {
                const Vector3 &p0 = m_positions[m_indices[face * 3 + 0]];
                const Vector3 &p1 = m_positions[m_indices[face * 3 + 1]];
                const Vector3 &p2 = m_positions[m_indices[face * 3 + 2]];
                return length(cross(p1 - p0, p2 - p0)) * 0.5f;
        }

        Vector3 computeFaceCentroid(uint32_t face) const
        {
                Vector3 sum(0.0f);
                for (uint32_t i = 0; i < 3; i++)
                        sum += m_positions[m_indices[face * 3 + i]];
                return sum / 3.0f;
        }

        // Average of the edge midpoints weighted by the edge length.
        // I want a point inside the triangle, but closer to the cirumcenter.
        Vector3 computeFaceCenter(uint32_t face) const
        {
                const Vector3 &p0 = m_positions[m_indices[face * 3 + 0]];
                const Vector3 &p1 = m_positions[m_indices[face * 3 + 1]];
                const Vector3 &p2 = m_positions[m_indices[face * 3 + 2]];
                const float l0 = length(p1 - p0);
                const float l1 = length(p2 - p1);
                const float l2 = length(p0 - p2);
                const Vector3 m0 = (p0 + p1) * l0 / (l0 + l1 + l2);
                const Vector3 m1 = (p1 + p2) * l1 / (l0 + l1 + l2);
                const Vector3 m2 = (p2 + p0) * l2 / (l0 + l1 + l2);
                return m0 + m1 + m2;
        }

        Vector3 computeFaceNormal(uint32_t face) const
        {
                const Vector3 &p0 = m_positions[m_indices[face * 3 + 0]];
                const Vector3 &p1 = m_positions[m_indices[face * 3 + 1]];
                const Vector3 &p2 = m_positions[m_indices[face * 3 + 2]];
                const Vector3 e0 = p2 - p0;
                const Vector3 e1 = p1 - p0;
                const Vector3 normalAreaScaled = cross(e0, e1);
                return normalizeSafe(normalAreaScaled, Vector3(0, 0, 1), 0.0f);
        }

        float computeFaceParametricArea(uint32_t face) const
        {
                const Vector2 &t0 = m_texcoords[m_indices[face * 3 + 0]];
                const Vector2 &t1 = m_texcoords[m_indices[face * 3 + 1]];
                const Vector2 &t2 = m_texcoords[m_indices[face * 3 + 2]];
                return triangleArea(t0, t1, t2);
        }

        // @@ This is not exactly accurate, we should compare the texture coordinates...
        bool isSeam(uint32_t edge) const
        {
                const uint32_t oppositeEdge = m_oppositeEdges[edge];
                if (oppositeEdge == UINT32_MAX)
                        return false; // boundary edge
                const uint32_t e0 = meshEdgeIndex0(edge);
                const uint32_t e1 = meshEdgeIndex1(edge);
                const uint32_t oe0 = meshEdgeIndex0(oppositeEdge);
                const uint32_t oe1 = meshEdgeIndex1(oppositeEdge);
                return m_indices[e0] != m_indices[oe1] || m_indices[e1] != m_indices[oe0];
        }

        bool isTextureSeam(uint32_t edge) const
        {
                const uint32_t oppositeEdge = m_oppositeEdges[edge];
                if (oppositeEdge == UINT32_MAX)
                        return false; // boundary edge
                const uint32_t e0 = meshEdgeIndex0(edge);
                const uint32_t e1 = meshEdgeIndex1(edge);
                const uint32_t oe0 = meshEdgeIndex0(oppositeEdge);
                const uint32_t oe1 = meshEdgeIndex1(oppositeEdge);
                return m_texcoords[m_indices[e0]] != m_texcoords[m_indices[oe1]] || m_texcoords[m_indices[e1]] != m_texcoords[m_indices[oe0]];
        }

        uint32_t firstColocal(uint32_t vertex) const
        {
                for (ColocalVertexIterator it(this, vertex); !it.isDone(); it.advance()) {
                        if (it.vertex() < vertex)
                                vertex = it.vertex();
                }
                return vertex;
        }

        bool areColocal(uint32_t vertex0, uint32_t vertex1) const
        {
                if (vertex0 == vertex1)
                        return true;
                if (m_nextColocalVertex.isEmpty())
                        return false;
                for (ColocalVertexIterator it(this, vertex0); !it.isDone(); it.advance()) {
                        if (it.vertex() == vertex1)
                                return true;
                }
                return false;
        }

        XA_INLINE float epsilon() const { return m_epsilon; }
        XA_INLINE uint32_t edgeCount() const { return m_indices.size(); }
        XA_INLINE uint32_t oppositeEdge(uint32_t edge) const { return m_oppositeEdges[edge]; }
        XA_INLINE bool isBoundaryEdge(uint32_t edge) const { return m_oppositeEdges[edge] == UINT32_MAX; }
        XA_INLINE const Array<uint32_t> &boundaryEdges() const { return m_boundaryEdges; }
        XA_INLINE bool isBoundaryVertex(uint32_t vertex) const { return m_isBoundaryVertex.get(vertex); }
        XA_INLINE uint32_t colocalVertexCount() const { return m_colocalVertexCount; }
        XA_INLINE uint32_t vertexCount() const { return m_positions.size(); }
        XA_INLINE uint32_t vertexAt(uint32_t i) const { return m_indices[i]; }
        XA_INLINE const Vector3 &position(uint32_t vertex) const { return m_positions[vertex]; }
        XA_INLINE const Vector3 &normal(uint32_t vertex) const { XA_DEBUG_ASSERT(m_flags & MeshFlags::HasNormals); return m_normals[vertex]; }
        XA_INLINE const Vector2 &texcoord(uint32_t vertex) const { return m_texcoords[vertex]; }
        XA_INLINE Vector2 &texcoord(uint32_t vertex) { return m_texcoords[vertex]; }
        XA_INLINE const Vector2 *texcoords() const { return m_texcoords.data(); }
        XA_INLINE Vector2 *texcoords() { return m_texcoords.data(); }
        XA_INLINE uint32_t ignoredFaceCount() const { return m_ignoredFaceCount; }
        XA_INLINE uint32_t faceCount() const { return m_indices.size() / 3; }
        XA_INLINE uint16_t faceGroupAt(uint32_t face) const { XA_DEBUG_ASSERT(m_flags & MeshFlags::HasFaceGroups); return m_faceGroups[face]; }
        XA_INLINE uint32_t faceGroupCount() const { XA_DEBUG_ASSERT(m_flags & MeshFlags::HasFaceGroups); return m_faceGroupFaceCounts.size(); }
        XA_INLINE uint32_t faceGroupNextFace(uint32_t face) const { XA_DEBUG_ASSERT(m_flags & MeshFlags::HasFaceGroups); return m_faceGroupNextFace[face]; }
        XA_INLINE uint32_t faceGroupFaceCount(uint32_t group) const { XA_DEBUG_ASSERT(m_flags & MeshFlags::HasFaceGroups); return m_faceGroupFaceCounts[group]; }
        XA_INLINE const uint32_t *indices() const { return m_indices.data(); }
        XA_INLINE uint32_t indexCount() const { return m_indices.size(); }

private:
        bool isFaceIgnored(uint32_t face) const { return (m_flags & MeshFlags::HasIgnoredFaces) && m_faceIgnore[face]; }

        float m_epsilon;
        uint32_t m_flags;
        uint32_t m_id;
        Array<bool> m_faceIgnore;
        uint32_t m_ignoredFaceCount;
        Array<uint32_t> m_indices;
        Array<Vector3> m_positions;
        Array<Vector3> m_normals;
        Array<Vector2> m_texcoords;

        // Populated by createFaceGroups
        Array<uint16_t> m_faceGroups;
        Array<uint32_t> m_faceGroupFirstFace;
        Array<uint32_t> m_faceGroupNextFace; // In: face. Out: the next face in the same group.
        Array<uint32_t> m_faceGroupFaceCounts; // In: face group. Out: number of faces in the group.

        // Populated by createColocals
        uint32_t m_colocalVertexCount;
        Array<uint32_t> m_nextColocalVertex; // In: vertex index. Out: the vertex index of the next colocal position.

        // Populated by createBoundaries
        BitArray m_isBoundaryVertex;
        Array<uint32_t> m_boundaryEdges;
        Array<uint32_t> m_oppositeEdges; // In: edge index. Out: the index of the opposite edge (i.e. wound the opposite direction). UINT32_MAX if the input edge is a boundary edge.

        // Populated by linkBoundaries
        Array<uint32_t> m_nextBoundaryEdges; // The index of the next boundary edge. UINT32_MAX if the edge is not a boundary edge.

        struct EdgeKey
        {
                EdgeKey() {}
                EdgeKey(const EdgeKey &k) : v0(k.v0), v1(k.v1) {}
                EdgeKey(uint32_t v0, uint32_t v1) : v0(v0), v1(v1) {}
                bool operator==(const EdgeKey &k) const { return v0 == k.v0 && v1 == k.v1; }

                uint32_t v0;
                uint32_t v1;
        };

        struct EdgeHash
        {
                uint32_t operator()(const EdgeKey &k) const { return k.v0 * 32768u + k.v1; }
        };

        HashMap<EdgeKey, EdgeHash> m_edgeMap;

public:
        class BoundaryLoopEdgeIterator
        {
        public:
                BoundaryLoopEdgeIterator(const Mesh *mesh, uint32_t edge) : m_mesh(mesh), m_first(UINT32_MAX), m_current(edge) {}

                void advance()
                {
                        if (m_first == UINT32_MAX)
                                m_first = m_current;
                        m_current = m_mesh->m_nextBoundaryEdges[m_current];
                }

                bool isDone() const
                {
                        return m_first == m_current || m_current == UINT32_MAX;
                }

                uint32_t edge() const
                {
                        return m_current;
                }

                uint32_t nextEdge() const
                {
                        return m_mesh->m_nextBoundaryEdges[m_current];
                }

        private:
                const Mesh *m_mesh;
                uint32_t m_first;
                uint32_t m_current;
        };

        class ColocalVertexIterator
        {
        public:
                ColocalVertexIterator(const Mesh *mesh, uint32_t v) : m_mesh(mesh), m_first(UINT32_MAX), m_current(v) {}

                void advance()
                {
                        if (m_first == UINT32_MAX)
                                m_first = m_current;
                        if (!m_mesh->m_nextColocalVertex.isEmpty())
                                m_current = m_mesh->m_nextColocalVertex[m_current];
                }

                bool isDone() const
                {
                        return m_first == m_current;
                }

                uint32_t vertex() const
                {
                        return m_current;
                }

                const Vector3 *pos() const
                {
                        return &m_mesh->m_positions[m_current];
                }

        private:
                const Mesh *m_mesh;
                uint32_t m_first;
                uint32_t m_current;
        };

        class ColocalEdgeIterator
        {
        public:
                ColocalEdgeIterator(const Mesh *mesh, uint32_t vertex0, uint32_t vertex1) : m_mesh(mesh), m_vertex0It(mesh, vertex0), m_vertex1It(mesh, vertex1), m_vertex1(vertex1)
                {
                        do {
                                if (!resetElement()) {
                                        advanceVertex1();
                                }
                                else {
                                        break;
                                }
                        } while (!isDone());
                }

                void advance()
                {
                        advanceElement();
                }

                bool isDone() const
                {
                        return m_vertex0It.isDone() && m_vertex1It.isDone() && m_edge == UINT32_MAX;
                }

                uint32_t edge() const
                {
                        return m_edge;
                }

        private:
                bool resetElement()
                {
                        m_edge = m_mesh->m_edgeMap.get(Mesh::EdgeKey(m_vertex0It.vertex(), m_vertex1It.vertex()));
                        while (m_edge != UINT32_MAX) {
                                if (!isIgnoredFace())
                                        break;
                                m_edge = m_mesh->m_edgeMap.getNext(m_edge);
                        }
                        if (m_edge == UINT32_MAX) {
                                return false;
                        }
                        return true;
                }

                void advanceElement()
                {
                        for (;;) {
                                m_edge = m_mesh->m_edgeMap.getNext(m_edge);
                                if (m_edge == UINT32_MAX)
                                        break;
                                if (!isIgnoredFace())
                                        break;
                        }
                        if (m_edge == UINT32_MAX)
                                advanceVertex1();
                }

                void advanceVertex1()
                {
                        auto successful = false;
                        while (!successful)        {
                                m_vertex1It.advance();
                                if (m_vertex1It.isDone()) {
                                        if (!m_vertex0It.isDone()) {
                                                m_vertex0It.advance();
                                                m_vertex1It = ColocalVertexIterator(m_mesh, m_vertex1);
                                        }
                                        else {
                                                return;
                                        }
                                }
                                successful = resetElement();
                        }
                }

                bool isIgnoredFace() const
                {
                        return m_mesh->m_faceIgnore[meshEdgeFace(m_edge)];
                }

                const Mesh *m_mesh;
                ColocalVertexIterator m_vertex0It, m_vertex1It;
                const uint32_t m_vertex1;
                uint32_t m_edge;
        };

        class FaceEdgeIterator
        {
        public:
                FaceEdgeIterator (const Mesh *mesh, uint32_t face) : m_mesh(mesh), m_face(face), m_relativeEdge(0)
                {
                        m_edge = m_face * 3;
                }

                void advance()
                {
                        if (m_relativeEdge < 3) {
                                m_edge++;
                                m_relativeEdge++;
                        }
                }

                bool isDone() const
                {
                        return m_relativeEdge == 3;
                }

                bool isBoundary() const { return m_mesh->m_oppositeEdges[m_edge] == UINT32_MAX; }
                bool isSeam() const { return m_mesh->isSeam(m_edge); }
                bool isTextureSeam() const { return m_mesh->isTextureSeam(m_edge); }
                uint32_t edge() const { return m_edge; }
                uint32_t relativeEdge() const { return m_relativeEdge; }
                uint32_t face() const { return m_face; }
                uint32_t oppositeEdge() const { return m_mesh->m_oppositeEdges[m_edge]; }

                uint32_t oppositeFace() const
                {
                        const uint32_t oedge = m_mesh->m_oppositeEdges[m_edge];
                        if (oedge == UINT32_MAX)
                                return UINT32_MAX;
                        return meshEdgeFace(oedge);
                }

                uint32_t vertex0() const { return m_mesh->m_indices[m_face * 3 + m_relativeEdge]; }
                uint32_t vertex1() const { return m_mesh->m_indices[m_face * 3 + (m_relativeEdge + 1) % 3]; }
                const Vector3 &position0() const { return m_mesh->m_positions[vertex0()]; }
                const Vector3 &position1() const { return m_mesh->m_positions[vertex1()]; }
                const Vector3 &normal0() const { return m_mesh->m_normals[vertex0()]; }
                const Vector3 &normal1() const { return m_mesh->m_normals[vertex1()]; }
                const Vector2 &texcoord0() const { return m_mesh->m_texcoords[vertex0()]; }
                const Vector2 &texcoord1() const { return m_mesh->m_texcoords[vertex1()]; }

        private:
                const Mesh *m_mesh;
                uint32_t m_face;
                uint32_t m_edge;
                uint32_t m_relativeEdge;
        };

        class GroupFaceIterator
        {
        public:
                GroupFaceIterator(const Mesh *mesh, uint32_t group) : m_mesh(mesh)
                {
                        XA_DEBUG_ASSERT(group != UINT32_MAX);
                        m_current = mesh->m_faceGroupFirstFace[group];
                }

                void advance()
                {
                        m_current = m_mesh->m_faceGroupNextFace[m_current];
                }

                bool isDone() const
                {
                        return m_current == UINT32_MAX;
                }

                uint32_t face() const
                {
                        return m_current;
                }

        private:
                const Mesh *m_mesh;
                uint32_t m_current;
        };
};

constexpr uint16_t Mesh::kInvalidFaceGroup;

static bool meshCloseHole(Mesh *mesh, const Array<uint32_t> &holeVertices, const Vector3 &normal)
{
#if XA_CLOSE_HOLES_CHECK_EDGE_INTERSECTION
        const uint32_t faceCount = mesh->faceCount();
#endif
        const bool compareNormal = equal(normal, Vector3(0.0f), FLT_EPSILON);
        uint32_t frontCount = holeVertices.size();
        Array<uint32_t> frontVertices;
        Array<Vector3> frontPoints;
        Array<float> frontAngles;
        frontVertices.resize(frontCount);
        frontPoints.resize(frontCount);
        for (uint32_t i = 0; i < frontCount; i++) {
                frontVertices[i] = holeVertices[i];
                frontPoints[i] = mesh->position(frontVertices[i]);
        }
        while (frontCount >= 3) {
                frontAngles.resize(frontCount);
                float smallestAngle = kPi2, smallestAngleIgnoringNormal = kPi2;
                uint32_t smallestAngleIndex = UINT32_MAX, smallestAngleIndexIgnoringNormal = UINT32_MAX;
                for (uint32_t i = 0; i < frontCount; i++) {
                        const uint32_t i1 = i == 0 ? frontCount - 1 : i - 1;
                        const uint32_t i2 = i;
                        const uint32_t i3 = (i + 1) % frontCount;
                        const Vector3 edge1 = frontPoints[i1] - frontPoints[i2];
                        const Vector3 edge2 = frontPoints[i3] - frontPoints[i2];
                        frontAngles[i] = atan2f(length(cross(edge1, edge2)), dot(edge1, edge2));
                        if (frontAngles[i] >= smallestAngle || isNan(frontAngles[i]))
                                continue;
                        // Don't duplicate edges.
                        if (mesh->findEdge(frontVertices[i1], frontVertices[i2]) != UINT32_MAX)
                                continue;
                        if (mesh->findEdge(frontVertices[i2], frontVertices[i3]) != UINT32_MAX)
                                continue;
                        if (mesh->findEdge(frontVertices[i3], frontVertices[i1]) != UINT32_MAX)
                                continue;
                        /*
                        Make sure he new edge that would be formed by (i3, i1) doesn't intersect any vertices. This often happens when fixing t-junctions.

                               i2
                               *
                              / \
                             /   \
                         i1 *--*--* i3
                             \ | /
                                  \|/
                                   *
                        */
                        bool intersection = false;
                        for (uint32_t j = 0; j < frontCount; j++) {
                                if (j == i1 || j == i2 || j == i3)
                                        continue;
                                if (lineIntersectsPoint(frontPoints[j], frontPoints[i3], frontPoints[i1], nullptr, mesh->epsilon())) {
                                        intersection = true;
                                        break;
                                }
                        }
                        if (intersection)
                                continue;
                        // Don't add the triangle if a boundary point lies on the same plane as the triangle, and is inside it.
                        intersection = false;
                        const Plane plane(frontPoints[i1], frontPoints[i2], frontPoints[i3]);
                        for (uint32_t j = 0; j < frontCount; j++) {
                                if (j == i1 || j == i2 || j == i3)
                                        continue;
                                if (!isZero(plane.distance(frontPoints[j]), mesh->epsilon()))
                                        continue;
                                if (pointInTriangle(frontPoints[j], frontPoints[i1], frontPoints[i2], frontPoints[i3])) {
                                        intersection = true;
                                        break;
                                }
                        }
                        if (intersection)
                                continue;
#if XA_CLOSE_HOLES_CHECK_EDGE_INTERSECTION
                        // Don't add the triangle if the new edge (i3, i1), intersects any other triangle that isn't part of the filled hole.
                        intersection = false;
                        const Vector3 newEdgeVector = frontPoints[i1] - frontPoints[i3];
                        for (uint32_t f = 0; f < faceCount; f++) {
                                Vector3 tri[3];
                                for (uint32_t j = 0; j < 3; j++)
                                        tri[j] = mesh->position(mesh->vertexAt(f * 3 + j));
                                float t;
                                if (rayIntersectsTriangle(frontPoints[i3], newEdgeVector, tri, &t)) {
                                        intersection = true;
                                        break;
                                }
                        }
                        if (intersection)
                                continue;
#endif
                        // Skip backwards facing triangles.
                        if (compareNormal) {
                                if (frontAngles[i] < smallestAngleIgnoringNormal) {
                                        smallestAngleIgnoringNormal = frontAngles[i];
                                        smallestAngleIndexIgnoringNormal = i;
                                }
                                const Vector3 e0 = frontPoints[i3] - frontPoints[i1];
                                const Vector3 e1 = frontPoints[i2] - frontPoints[i1];
                                const Vector3 triNormal = normalizeSafe(cross(e0, e1), Vector3(0.0f), mesh->epsilon());
                                if (dot(normal, triNormal) <= 0.0f)
                                        continue;
                        }
                        smallestAngle = smallestAngleIgnoringNormal = frontAngles[i];
                        smallestAngleIndex = smallestAngleIndexIgnoringNormal = i;
                }
                // Closing holes failed if we don't have a smallest angle.
                // Fallback to ignoring the backwards facing normal test if possible.
                if (smallestAngleIndex == UINT32_MAX || smallestAngle <= 0.0f || smallestAngle >= kPi) {
                    if (/* [Bruno: does not make sense to me] smallestAngleIgnoringNormal == UINT32_MAX ||*/ smallestAngleIgnoringNormal <= 0.0f || smallestAngleIgnoringNormal >= kPi)
                                return false;
                        else
                                smallestAngleIndex = smallestAngleIndexIgnoringNormal;
                }
                const uint32_t i1 = smallestAngleIndex == 0 ? frontCount - 1 : smallestAngleIndex - 1;
                const uint32_t i2 = smallestAngleIndex;
                const uint32_t i3 = (smallestAngleIndex + 1) % frontCount;
                const Mesh::AddFaceResult::Enum result = mesh->addFace(frontVertices[i1], frontVertices[i2], frontVertices[i3]);
                XA_DEBUG_ASSERT(result == Mesh::AddFaceResult::OK); // Shouldn't happen due to the findEdge calls above.
                XA_UNUSED(result);
                frontVertices.removeAt(i2);
                frontPoints.removeAt(i2);
                frontCount = frontVertices.size();
        }
        return true;
}

static bool meshCloseHoles(Mesh *mesh, const Array<uint32_t> &boundaryLoops, const Vector3 &normal, uint32_t *holeCount, Array<uint32_t> *holeFaceCounts)
{
        if (holeFaceCounts)
                holeFaceCounts->clear();
        // Compute lengths.
        const uint32_t boundaryCount = boundaryLoops.size();
        Array<float> boundaryLengths;
        Array<uint32_t> boundaryEdgeCounts;
        boundaryEdgeCounts.resize(boundaryCount);
        for (uint32_t i = 0; i < boundaryCount; i++) {
                float boundaryLength = 0.0f;
                boundaryEdgeCounts[i] = 0;
                for (Mesh::BoundaryLoopEdgeIterator it(mesh, boundaryLoops[i]); !it.isDone(); it.advance()) {
                        const Vector3 &t0 = mesh->position(mesh->vertexAt(meshEdgeIndex0(it.edge())));
                        const Vector3 &t1 = mesh->position(mesh->vertexAt(meshEdgeIndex1(it.edge())));
                        boundaryLength += length(t1 - t0);
                        boundaryEdgeCounts[i]++;
                }
                boundaryLengths.push_back(boundaryLength);
        }
        // Find disk boundary.
        uint32_t diskBoundary = 0;
        float maxLength = boundaryLengths[0];
        for (uint32_t i = 1; i < boundaryCount; i++) {
                if (boundaryLengths[i] > maxLength) {
                        maxLength = boundaryLengths[i];
                        diskBoundary = i;
                }
        }
        // Close holes.
        Array<uint32_t> holeVertices;
        Array<Vector3> holePoints;
        bool result = true;
        for (uint32_t i = 0; i < boundaryCount; i++) {
                if (diskBoundary == i)
                        continue; // Skip disk boundary.
                holeVertices.resize(boundaryEdgeCounts[i]);
                holePoints.resize(boundaryEdgeCounts[i]);
                // Winding is backwards for internal boundaries.
                uint32_t e = 0;
                for (Mesh::BoundaryLoopEdgeIterator it(mesh, boundaryLoops[i]); !it.isDone(); it.advance()) {
                        const uint32_t vertex = mesh->vertexAt(meshEdgeIndex0(it.edge()));
                        holeVertices[boundaryEdgeCounts[i] - 1 - e] = vertex;
                        holePoints[boundaryEdgeCounts[i] - 1 - e] = mesh->position(vertex);
                        e++;
                }
                const uint32_t oldFaceCount = mesh->faceCount();
                if (!meshCloseHole(mesh, holeVertices, normal))
                        result = false; // Return false if any hole failed to close, but keep trying to close other holes.
                if (holeCount)
                        (*holeCount)++;
                if (holeFaceCounts)
                        holeFaceCounts->push_back(mesh->faceCount() - oldFaceCount);
        }
        return result;
}

static bool meshIsPlanar(const Mesh &mesh)
{
        const Vector3 p1 = mesh.position(mesh.vertexAt(0));
        const Vector3 p2 = mesh.position(mesh.vertexAt(1));
        const Vector3 p3 = mesh.position(mesh.vertexAt(2));
        const Plane plane(p1, p2, p3);
        const uint32_t vertexCount = mesh.vertexCount();
        for (uint32_t v = 0; v < vertexCount; v++) {
                const float d = plane.distance(mesh.position(v));
                if (!isZero(d, mesh.epsilon()))
                        return false;
        }
        return true;
}

/*
Fixing T-junctions.

- Find T-junctions. Find  vertices that are on an edge.
- This test is approximate.
- Insert edges on a spatial index to speedup queries.
- Consider only open edges, that is edges that have no pairs.
- Consider only vertices on boundaries.
- Close T-junction.
- Split edge.

*/
struct SplitEdge
{
        uint32_t edge;
        float t;
        uint32_t vertex;

        bool operator<(const SplitEdge &other) const
        {
                if (edge < other.edge)
                        return true;
                else if (edge == other.edge) {
                        if (t < other.t)
                                return true;
                }
                return false;
        }
};

// Returns nullptr if there were no t-junctions to fix.
static Mesh *meshFixTJunctions(const Mesh &inputMesh, bool *duplicatedEdge, bool *failed, uint32_t *fixedTJunctionsCount)
{
        if (duplicatedEdge)
                *duplicatedEdge = false;
        if (failed)
                *failed = false;
        Array<SplitEdge> splitEdges;
        const uint32_t vertexCount = inputMesh.vertexCount();
        const uint32_t edgeCount = inputMesh.edgeCount();
        for (uint32_t v = 0; v < vertexCount; v++) {
                if (!inputMesh.isBoundaryVertex(v))
                        continue;
                // Find edges that this vertex overlaps with.
                const Vector3 &pos = inputMesh.position(v);
                for (uint32_t e = 0; e < edgeCount; e++) {
                        if (!inputMesh.isBoundaryEdge(e))
                                continue;
                        const Vector3 &edgePos1 = inputMesh.position(inputMesh.vertexAt(meshEdgeIndex0(e)));
                        const Vector3 &edgePos2 = inputMesh.position(inputMesh.vertexAt(meshEdgeIndex1(e)));
                        float t;
                        if (!lineIntersectsPoint(pos, edgePos1, edgePos2, &t, inputMesh.epsilon()))
                                continue;
                        SplitEdge splitEdge;
                        splitEdge.edge = e;
                        splitEdge.t = t;
                        splitEdge.vertex = v;
                        splitEdges.push_back(splitEdge);
                }
        }
        if (splitEdges.isEmpty())
                return nullptr;
        const uint32_t faceCount = inputMesh.faceCount();
        Mesh *mesh = XA_NEW_ARGS(MemTag::Mesh, Mesh, inputMesh.epsilon(), vertexCount + splitEdges.size(), faceCount);
        for (uint32_t v = 0; v < vertexCount; v++)
                mesh->addVertex(inputMesh.position(v));
        Array<uint32_t> indexArray;
        indexArray.reserve(4);
        Array<SplitEdge> faceSplitEdges;
        faceSplitEdges.reserve(4);
        for (uint32_t f = 0; f < faceCount; f++) {
                // Find t-junctions in this face.
                faceSplitEdges.clear();
                for (uint32_t i = 0; i < splitEdges.size(); i++) {
                        if (meshEdgeFace(splitEdges[i].edge) == f)
                                faceSplitEdges.push_back(splitEdges[i]);
                }
                if (!faceSplitEdges.isEmpty()) {
                        // Need to split edges in winding order when a single edge has multiple t-junctions.
                        insertionSort(faceSplitEdges.data(), faceSplitEdges.size());
                        indexArray.clear();
                        for (Mesh::FaceEdgeIterator it(&inputMesh, f); !it.isDone(); it.advance()) {
                                indexArray.push_back(it.vertex0());
                                for (uint32_t se = 0; se < faceSplitEdges.size(); se++) {
                                        const SplitEdge &splitEdge = faceSplitEdges[se];
                                        if (splitEdge.edge == it.edge())
                                                indexArray.push_back(splitEdge.vertex);
                                }
                        }
                        if (!meshCloseHole(mesh, indexArray, Vector3(0.0f))) {
                                if (failed)
                                        *failed = true;
                        }
                } else {
                        // No t-junctions in this face. Copy from input mesh.
                        if (mesh->addFace(&inputMesh.indices()[f * 3]) == Mesh::AddFaceResult::DuplicateEdge) {
                                if (duplicatedEdge)
                                        *duplicatedEdge = true;
                        }
                }
        }
        if (fixedTJunctionsCount)
                *fixedTJunctionsCount = splitEdges.size();
        return mesh;
}

// boundaryLoops are the first edges for each boundary loop.
static void meshGetBoundaryLoops(const Mesh &mesh, Array<uint32_t> &boundaryLoops)
{
        const uint32_t edgeCount = mesh.edgeCount();
        BitArray bitFlags(edgeCount);
        bitFlags.zeroOutMemory();
        boundaryLoops.clear();
        // Search for boundary edges. Mark all the edges that belong to the same boundary.
        for (uint32_t e = 0; e < edgeCount; e++) {
                if (bitFlags.get(e) || !mesh.isBoundaryEdge(e))
                        continue;
                for (Mesh::BoundaryLoopEdgeIterator it(&mesh, e); !it.isDone(); it.advance())
                        bitFlags.set(it.edge());
                boundaryLoops.push_back(e);
        }
}

struct Progress
{
        Progress(ProgressCategory::Enum category, ProgressFunc func, void *userData, uint32_t maxValue) : value(0), cancel(false), m_category(category), m_func(func), m_userData(userData), m_maxValue(maxValue), m_progress(0)
        {
                if (m_func) {
                        if (!m_func(category, 0, userData))
                                cancel = true;
                }
        }

        ~Progress()
        {
                if (m_func) {
                        if (!m_func(m_category, 100, m_userData))
                                cancel = true;
                }
        }

        void update()
        {
                if (!m_func)
                        return;
                m_mutex.lock();
                const uint32_t newProgress = uint32_t(ceilf(value.load() / (float)m_maxValue * 100.0f));
                if (newProgress != m_progress && newProgress < 100) {
                        m_progress = newProgress;
                        if (!m_func(m_category, m_progress, m_userData))
                                cancel = true;
                }
                m_mutex.unlock();
        }

        void setMaxValue(uint32_t maxValue)
        {
                m_mutex.lock();
                m_maxValue = maxValue;
                m_mutex.unlock();
        }

        std::atomic<uint32_t> value;
        std::atomic<bool> cancel;

private:
        ProgressCategory::Enum m_category;
        ProgressFunc m_func;
        void *m_userData;
        uint32_t m_maxValue;
        uint32_t m_progress;
        std::mutex m_mutex;
};

struct Spinlock
{
        void lock() { while(m_lock.test_and_set(std::memory_order_acquire)) {} }
        void unlock() { m_lock.clear(std::memory_order_release); }

private:
        std::atomic_flag m_lock = ATOMIC_FLAG_INIT;
};

struct TaskGroupHandle
{
        uint32_t value = UINT32_MAX;
};

struct Task
{
        void (*func)(void *userData);
        void *userData;
};

#if XA_MULTITHREADED
class TaskScheduler
{
public:
        TaskScheduler() : m_shutdown(false)
        {
                m_threadIndex = 0;
                // Max with current task scheduler usage is 1 per thread + 1 deep nesting, but allow for some slop.
                m_maxGroups = std::thread::hardware_concurrency() * 4;
                m_groups = XA_ALLOC_ARRAY(MemTag::Default, TaskGroup, m_maxGroups);
                for (uint32_t i = 0; i < m_maxGroups; i++) {
                        new (&m_groups[i]) TaskGroup();
                        m_groups[i].free = true;
                        m_groups[i].ref = 0;
                }
                m_workers.resize(std::thread::hardware_concurrency() <= 1 ? 1 : std::thread::hardware_concurrency() - 1);
                for (uint32_t i = 0; i < m_workers.size(); i++) {
                        new (&m_workers[i]) Worker();
                        m_workers[i].wakeup = false;
                        m_workers[i].thread = XA_NEW_ARGS(MemTag::Default, std::thread, workerThread, this, &m_workers[i], i + 1);
                }
        }

        ~TaskScheduler()
        {
                m_shutdown = true;
                for (uint32_t i = 0; i < m_workers.size(); i++) {
                        Worker &worker = m_workers[i];
                        XA_DEBUG_ASSERT(worker.thread);
                        worker.wakeup = true;
                        worker.cv.notify_one();
                        if (worker.thread->joinable())
                                worker.thread->join();
                        worker.thread->~thread();
                        XA_FREE(worker.thread);
                        worker.~Worker();
                }
                for (uint32_t i = 0; i < m_maxGroups; i++)
                        m_groups[i].~TaskGroup();
                XA_FREE(m_groups);
        }

        uint32_t threadCount() const
        {
                return max(1u, std::thread::hardware_concurrency()); // Including the main thread.
        }

        TaskGroupHandle createTaskGroup(uint32_t reserveSize = 0)
        {
                // Claim the first free group.
                for (uint32_t i = 0; i < m_maxGroups; i++) {
                        TaskGroup &group = m_groups[i];
                        bool expected = true;
                        if (!group.free.compare_exchange_strong(expected, false))
                                continue;
                        group.queueLock.lock();
                        group.queueHead = 0;
                        group.queue.clear();
                        group.queue.reserve(reserveSize);
                        group.queueLock.unlock();
                        TaskGroupHandle handle;
                        handle.value = i;
                        return handle;
                }
                XA_DEBUG_ASSERT(false);
                TaskGroupHandle handle;
                handle.value = UINT32_MAX;
                return handle;
        }

        void run(TaskGroupHandle handle, Task task)
        {
                XA_DEBUG_ASSERT(handle.value != UINT32_MAX);
                TaskGroup &group = m_groups[handle.value];
                group.queueLock.lock();
                group.queue.push_back(task);
                group.queueLock.unlock();
                group.ref++;
                // Wake up a worker to run this task.
                for (uint32_t i = 0; i < m_workers.size(); i++) {
                        m_workers[i].wakeup = true;
                        m_workers[i].cv.notify_one();
                }
        }

        void wait(TaskGroupHandle *handle)
        {
                if (handle->value == UINT32_MAX) {
                        XA_DEBUG_ASSERT(false);
                        return;
                }
                // Run tasks from the group queue until empty.
                TaskGroup &group = m_groups[handle->value];
                for (;;) {
                        Task *task = nullptr;
                        group.queueLock.lock();
                        if (group.queueHead < group.queue.size())
                                task = &group.queue[group.queueHead++];
                        group.queueLock.unlock();
                        if (!task)
                                break;
                        task->func(task->userData);
                        group.ref--;
                }
                // Even though the task queue is empty, workers can still be running tasks.
                while (group.ref > 0)
                        std::this_thread::yield();
                group.free = true;
                handle->value = UINT32_MAX;
        }

        static uint32_t currentThreadIndex() { return m_threadIndex; }

private:
        struct TaskGroup
        {
                std::atomic<bool> free;
                Array<Task> queue; // Items are never removed. queueHead is incremented to pop items.
                uint32_t queueHead = 0;
                Spinlock queueLock;
                std::atomic<uint32_t> ref; // Increment when a task is enqueued, decrement when a task finishes.
        };

        struct Worker
        {
                std::thread *thread = nullptr;
                std::mutex mutex;
                std::condition_variable cv;
                std::atomic<bool> wakeup;
        };

        TaskGroup *m_groups;
        uint32_t m_maxGroups;
        Array<Worker> m_workers;
        std::atomic<bool> m_shutdown;
        static thread_local uint32_t m_threadIndex;

        static void workerThread(TaskScheduler *scheduler, Worker *worker, uint32_t threadIndex)
        {
                m_threadIndex = threadIndex;
                std::unique_lock<std::mutex> lock(worker->mutex);
                for (;;) {
                        worker->cv.wait(lock, [=]{ return worker->wakeup.load(); });
                        worker->wakeup = false;
                        for (;;) {
                                if (scheduler->m_shutdown)
                                        return;
                                // Look for a task in any of the groups and run it.
                                TaskGroup *group = nullptr;
                                Task *task = nullptr;
                                for (uint32_t i = 0; i < scheduler->m_maxGroups; i++) {
                                        group = &scheduler->m_groups[i];
                                        if (group->free || group->ref == 0)
                                                continue;
                                        group->queueLock.lock();
                                        if (group->queueHead < group->queue.size()) {
                                                task = &group->queue[group->queueHead++];
                                                group->queueLock.unlock();
                                                break;
                                        }
                                        group->queueLock.unlock();
                                }
                                if (!task)
                                        break;
                                task->func(task->userData);
                                group->ref--;
                        }
                }
        }
};

thread_local uint32_t TaskScheduler::m_threadIndex;
#else
class TaskScheduler
{
public:
        ~TaskScheduler()
        {
            // [Bruno Levy] older compiler does not understand
            // {i} as struct initialization.
            TaskGroupHandle h;
            for (uint32_t i = 0; i < m_groups.size(); i++) {
                h.value = i;
                destroyGroup(h);
            }
        }

        uint32_t threadCount() const
        {
                return 1;
        }

        TaskGroupHandle createTaskGroup(uint32_t reserveSize = 0)
        {
                TaskGroup *group = XA_NEW(MemTag::Default, TaskGroup);
                group->queue.reserve(reserveSize);
                m_groups.push_back(group);
                TaskGroupHandle handle;
                handle.value = m_groups.size() - 1;
                return handle;
        }

        void run(TaskGroupHandle handle, Task task)
        {
                m_groups[handle.value]->queue.push_back(task);
        }

        void wait(TaskGroupHandle *handle)
        {
                if (handle->value == UINT32_MAX) {
                        XA_DEBUG_ASSERT(false);
                        return;
                }
                TaskGroup *group = m_groups[handle->value];
                for (uint32_t i = 0; i < group->queue.size(); i++)
                        group->queue[i].func(group->queue[i].userData);
                group->queue.clear();
                destroyGroup(*handle);
                handle->value = UINT32_MAX;
        }

        static uint32_t currentThreadIndex() { return 0; }

private:
        void destroyGroup(TaskGroupHandle handle)
        {
                TaskGroup *group = m_groups[handle.value];
                if (group) {
                        group->~TaskGroup();
                        XA_FREE(group);
                        m_groups[handle.value] = nullptr;
                }
        }

        struct TaskGroup
        {
                Array<Task> queue;
        };

        Array<TaskGroup *> m_groups;
};
#endif

#if XA_DEBUG_EXPORT_TGA
const uint8_t TGA_TYPE_RGB = 2;
const uint8_t TGA_ORIGIN_UPPER = 0x20;

#pragma pack(push, 1)
struct TgaHeader
{
        uint8_t id_length;
        uint8_t colormap_type;
        uint8_t image_type;
        uint16_t colormap_index;
        uint16_t colormap_length;
        uint8_t colormap_size;
        uint16_t x_origin;
        uint16_t y_origin;
        uint16_t width;
        uint16_t height;
        uint8_t pixel_size;
        uint8_t flags;
        enum { Size = 18 };
};
#pragma pack(pop)

static void WriteTga(const char *filename, const uint8_t *data, uint32_t width, uint32_t height)
{
        XA_DEBUG_ASSERT(sizeof(TgaHeader) == TgaHeader::Size);
        FILE *f;
        XA_FOPEN(f, filename, "wb");
        if (!f)
                return;
        TgaHeader tga;
        tga.id_length = 0;
        tga.colormap_type = 0;
        tga.image_type = TGA_TYPE_RGB;
        tga.colormap_index = 0;
        tga.colormap_length = 0;
        tga.colormap_size = 0;
        tga.x_origin = 0;
        tga.y_origin = 0;
        tga.width = (uint16_t)width;
        tga.height = (uint16_t)height;
        tga.pixel_size = 24;
        tga.flags = TGA_ORIGIN_UPPER;
        fwrite(&tga, sizeof(TgaHeader), 1, f);
        fwrite(data, sizeof(uint8_t), width * height * 3, f);
        fclose(f);
}
#endif

template<typename T>
class ThreadLocal
{
public:
        ThreadLocal()
        {
#if XA_MULTITHREADED
                const uint32_t n = std::thread::hardware_concurrency();
#else
                const uint32_t n = 1;
#endif
                m_array = XA_ALLOC_ARRAY(MemTag::Default, T, n);
                for (uint32_t i = 0; i < n; i++)
                        new (&m_array[i]) T;
        }

        ~ThreadLocal()
        {
#if XA_MULTITHREADED
                const uint32_t n = std::thread::hardware_concurrency();
#else
                const uint32_t n = 1;
#endif
                for (uint32_t i = 0; i < n; i++)
                        m_array[i].~T();
                XA_FREE(m_array);
        }

        T &get() const
        {
                return m_array[TaskScheduler::currentThreadIndex()];
        }

private:
        T *m_array;
};

class UniformGrid2
{
public:
        void reset(const Vector2 *positions, const uint32_t *indices = nullptr, uint32_t reserveEdgeCount = 0)
        {
                m_edges.clear();
                if (reserveEdgeCount > 0)
                        m_edges.reserve(reserveEdgeCount);
                m_positions = positions;
                m_indices = indices;
                m_cellDataOffsets.clear();
        }

        void append(uint32_t edge)
        {
                XA_DEBUG_ASSERT(m_cellDataOffsets.isEmpty());
                m_edges.push_back(edge);
        }

        bool intersect(Vector2 v1, Vector2 v2, float epsilon)
        {
                const uint32_t edgeCount = m_edges.size();
                bool bruteForce = edgeCount <= 64;
                if (!bruteForce && m_cellDataOffsets.isEmpty())
                        bruteForce = !createGrid();
                if (bruteForce) {
                        for (uint32_t j = 0; j < edgeCount; j++) {
                                const uint32_t edge = m_edges[j];
                                if (linesIntersect(v1, v2, edgePosition0(edge), edgePosition1(edge), epsilon))
                                        return true;
                        }
                } else {
                        computePotentialEdges(v1, v2);
                        uint32_t prevEdge = UINT32_MAX;
                        for (uint32_t j = 0; j < m_potentialEdges.size(); j++) {
                                const uint32_t edge = m_potentialEdges[j];
                                if (edge == prevEdge)
                                        continue;
                                if (linesIntersect(v1, v2, edgePosition0(edge), edgePosition1(edge), epsilon))
                                        return true;
                                prevEdge = edge;
                        }
                }
                return false;
        }

        bool intersectSelf(float epsilon)
        {
                const uint32_t edgeCount = m_edges.size();
                bool bruteForce = edgeCount <= 64;
                if (!bruteForce && m_cellDataOffsets.isEmpty())
                        bruteForce = !createGrid();
                for (uint32_t i = 0; i < edgeCount; i++) {
                        const uint32_t edge1 = m_edges[i];
                        if (bruteForce) {
                                for (uint32_t j = 0; j < edgeCount; j++) {
                                        const uint32_t edge2 = m_edges[j];
                                        if (edgesIntersect(edge1, edge2, epsilon))
                                                return true;
                                }
                        } else {
                                computePotentialEdges(edgePosition0(edge1), edgePosition1(edge1));
                                uint32_t prevEdge = UINT32_MAX;
                                for (uint32_t j = 0; j < m_potentialEdges.size(); j++) {
                                        const uint32_t edge2 = m_potentialEdges[j];
                                        if (edge2 == prevEdge)
                                                continue;
                                        if (edgesIntersect(edge1, edge2, epsilon))
                                                return true;
                                        prevEdge = edge2;
                                }
                        }
                }
                return false;
        }

#if XA_DEBUG_EXPORT_BOUNDARY_GRID
        void debugExport(const char *filename)
        {
                Array<uint8_t> image;
                image.resize(m_gridWidth * m_gridHeight * 3);
                for (uint32_t y = 0; y < m_gridHeight; y++) {
                        for (uint32_t x = 0; x < m_gridWidth; x++) {
                                uint8_t *bgr = &image[(x + y * m_gridWidth) * 3];
                                bgr[0] = bgr[1] = bgr[2] = 32;
                                uint32_t offset = m_cellDataOffsets[x + y * m_gridWidth];
                                while (offset != UINT32_MAX) {
                                        const uint32_t edge2 = m_cellData[offset];
                                        srand(edge2);
                                        for (uint32_t i = 0; i < 3; i++)
                                                bgr[i] = uint8_t(bgr[i] * 0.5f + (rand() % 255) * 0.5f);
                                        offset = m_cellData[offset + 1];
                                }
                        }
                }
                WriteTga(filename, image.data(), m_gridWidth, m_gridHeight);
        }
#endif

private:
        bool createGrid()
        {
                // Compute edge extents. Min will be the grid origin.
                const uint32_t edgeCount = m_edges.size();
                Extents2 edgeExtents;
                edgeExtents.reset();
                for (uint32_t i = 0; i < edgeCount; i++) {
                        const uint32_t edge = m_edges[i];
                        edgeExtents.add(edgePosition0(edge));
                        edgeExtents.add(edgePosition1(edge));
                }
                m_gridOrigin = edgeExtents.min;
                // Size grid to approximately one edge per cell.
                const Vector2 extentsSize(edgeExtents.max - edgeExtents.min);
                m_cellSize = min(extentsSize.x, extentsSize.y) / sqrtf((float)edgeCount);
                if (m_cellSize <= 0.0f)
                        return false;
                m_gridWidth = uint32_t(ceilf(extentsSize.x / m_cellSize));
                m_gridHeight = uint32_t(ceilf(extentsSize.y / m_cellSize));
                if (m_gridWidth == 0 || m_gridHeight == 0)
                        return false;
                // Insert edges into cells.
                m_cellDataOffsets.resize(m_gridWidth * m_gridHeight);
                for (uint32_t i = 0; i < m_cellDataOffsets.size(); i++)
                        m_cellDataOffsets[i] = UINT32_MAX;
                m_cellData.clear();
                m_cellData.reserve(edgeCount * 2);
                for (uint32_t i = 0; i < edgeCount; i++) {
                        const uint32_t edge = m_edges[i];
                        traverse(edgePosition0(edge), edgePosition1(edge));
                        XA_DEBUG_ASSERT(!m_traversedCellOffsets.isEmpty());
                        for (uint32_t j = 0; j < m_traversedCellOffsets.size(); j++) {
                                const uint32_t cell = m_traversedCellOffsets[j];
                                uint32_t offset = m_cellDataOffsets[cell];
                                if (offset == UINT32_MAX)
                                        m_cellDataOffsets[cell] = m_cellData.size();
                                else {
                                        for (;;) {
                                                uint32_t &nextOffset = m_cellData[offset + 1];
                                                if (nextOffset == UINT32_MAX) {
                                                        nextOffset = m_cellData.size();
                                                        break;
                                                }
                                                offset = nextOffset;
                                        }
                                }
                                m_cellData.push_back(edge);
                                m_cellData.push_back(UINT32_MAX);
                        }
                }
                return true;
        }

        void computePotentialEdges(Vector2 p1, Vector2 p2)
        {
                m_potentialEdges.clear();
                traverse(p1, p2);
                for (uint32_t j = 0; j < m_traversedCellOffsets.size(); j++) {
                        const uint32_t cell = m_traversedCellOffsets[j];
                        uint32_t offset = m_cellDataOffsets[cell];
                        while (offset != UINT32_MAX) {
                                const uint32_t edge2 = m_cellData[offset];
                                m_potentialEdges.push_back(edge2);
                                offset = m_cellData[offset + 1];
                        }
                }
                if (m_potentialEdges.isEmpty())
                        return;
                insertionSort(m_potentialEdges.data(), m_potentialEdges.size());
        }

        // "A Fast Voxel Traversal Algorithm for Ray Tracing"
        void traverse(Vector2 p1, Vector2 p2)
        {
                const Vector2 dir = p2 - p1;
                const Vector2 normal = normalizeSafe(dir, Vector2(0.0f), kEpsilon);
                const int stepX = dir.x >= 0 ? 1 : -1;
                const int stepY = dir.y >= 0 ? 1 : -1;
                const uint32_t firstCell[2] = { cellX(p1.x), cellY(p1.y) };
                const uint32_t lastCell[2] = { cellX(p2.x), cellY(p2.y) };
                float distToNextCellX;
                if (stepX == 1)
                        distToNextCellX = (firstCell[0] + 1) * m_cellSize - (p1.x - m_gridOrigin.x);
                else
                        distToNextCellX = (p1.x - m_gridOrigin.x) - firstCell[0] * m_cellSize;
                float distToNextCellY;
                if (stepY == 1)
                        distToNextCellY = (firstCell[1] + 1) * m_cellSize - (p1.y - m_gridOrigin.y);
                else
                        distToNextCellY = (p1.y - m_gridOrigin.y) - firstCell[1] * m_cellSize;
                float tMaxX, tMaxY, tDeltaX, tDeltaY;
                if (normal.x > kEpsilon || normal.x < -kEpsilon) {
                        tMaxX = (distToNextCellX * stepX) / normal.x;
                        tDeltaX = (m_cellSize * stepX) / normal.x;
                }
                else
                        tMaxX = tDeltaX = FLT_MAX;
                if (normal.y > kEpsilon || normal.y < -kEpsilon) {
                        tMaxY = (distToNextCellY * stepY) / normal.y;
                        tDeltaY = (m_cellSize * stepY) / normal.y;
                }
                else
                        tMaxY = tDeltaY = FLT_MAX;
                m_traversedCellOffsets.clear();
                m_traversedCellOffsets.push_back(firstCell[0] + firstCell[1] * m_gridWidth);
                uint32_t currentCell[2] = { firstCell[0], firstCell[1] };
                while (!(currentCell[0] == lastCell[0] && currentCell[1] == lastCell[1])) {
                        if (tMaxX < tMaxY) {
                                tMaxX += tDeltaX;
                                currentCell[0] += stepX;
                        } else {
                                tMaxY += tDeltaY;
                                currentCell[1] += stepY;
                        }
                        if (currentCell[0] >= m_gridWidth || currentCell[1] >= m_gridHeight)
                                break;
                        if (stepX == 0 && currentCell[0] < lastCell[0])
                                break;
                        if (stepX == 1 && currentCell[0] > lastCell[0])
                                break;
                        if (stepY == 0 && currentCell[1] < lastCell[1])
                                break;
                        if (stepY == 1 && currentCell[1] > lastCell[1])
                                break;
                        m_traversedCellOffsets.push_back(currentCell[0] + currentCell[1] * m_gridWidth);
                }
        }

        bool edgesIntersect(uint32_t edge1, uint32_t edge2, float epsilon) const
        {
                if (edge1 == edge2)
                        return false;
                const uint32_t ai[2] = { vertexAt(meshEdgeIndex0(edge1)), vertexAt(meshEdgeIndex1(edge1)) };
                const uint32_t bi[2] = { vertexAt(meshEdgeIndex0(edge2)), vertexAt(meshEdgeIndex1(edge2)) };
                // Ignore connected edges, since they can't intersect (only overlap), and may be detected as false positives.
                if (ai[0] == bi[0] || ai[0] == bi[1] || ai[1] == bi[0] || ai[1] == bi[1])
                        return false;
                return linesIntersect(m_positions[ai[0]], m_positions[ai[1]], m_positions[bi[0]], m_positions[bi[1]], epsilon);
        }

        uint32_t cellX(float x) const
        {
                return min((uint32_t)max(0.0f, (x - m_gridOrigin.x) / m_cellSize), m_gridWidth - 1u);
        }

        uint32_t cellY(float y) const
        {
                return min((uint32_t)max(0.0f, (y - m_gridOrigin.y) / m_cellSize), m_gridHeight - 1u);
        }

        Vector2 edgePosition0(uint32_t edge) const
        {
                return m_positions[vertexAt(meshEdgeIndex0(edge))];
        }

        Vector2 edgePosition1(uint32_t edge) const
        {
                return m_positions[vertexAt(meshEdgeIndex1(edge))];
        }

        uint32_t vertexAt(uint32_t index) const
        {
                return m_indices ? m_indices[index] : index;
        }

        Array<uint32_t> m_edges;
        const Vector2 *m_positions;
        const uint32_t *m_indices; // Optional
        float m_cellSize;
        Vector2 m_gridOrigin;
        uint32_t m_gridWidth, m_gridHeight; // in cells
        Array<uint32_t> m_cellDataOffsets;
        Array<uint32_t> m_cellData;
        Array<uint32_t> m_potentialEdges;
        Array<uint32_t> m_traversedCellOffsets;
};

struct UvMeshChart
{
        Array<uint32_t> faces;
        Array<uint32_t> indices;
        uint32_t material;
};

struct UvMesh
{
        UvMeshDecl decl;
        Array<uint32_t> indices;
        Array<UvMeshChart *> charts;
        Array<uint32_t> vertexToChartMap;
};

struct UvMeshInstance
{
        UvMesh *mesh;
        Array<Vector2> texcoords;
        bool rotateCharts;
};

namespace raster {
class ClippedTriangle
{
public:
        ClippedTriangle(const Vector2 &a, const Vector2 &b, const Vector2 &c)
        {
                m_numVertices = 3;
                m_activeVertexBuffer = 0;
                m_verticesA[0] = a;
                m_verticesA[1] = b;
                m_verticesA[2] = c;
                m_vertexBuffers[0] = m_verticesA;
                m_vertexBuffers[1] = m_verticesB;
        }

        void clipHorizontalPlane(float offset, float clipdirection)
        {
                Vector2 *v  = m_vertexBuffers[m_activeVertexBuffer];
                m_activeVertexBuffer ^= 1;
                Vector2 *v2 = m_vertexBuffers[m_activeVertexBuffer];
                v[m_numVertices] = v[0];
                float dy2,   dy1 = offset - v[0].y;
                int   dy2in, dy1in = clipdirection * dy1 >= 0;
                uint32_t  p = 0;
                for (uint32_t k = 0; k < m_numVertices; k++) {
                        dy2   = offset - v[k + 1].y;
                        dy2in = clipdirection * dy2 >= 0;
                        if (dy1in) v2[p++] = v[k];
                        if ( dy1in + dy2in == 1 ) { // not both in/out
                                float dx = v[k + 1].x - v[k].x;
                                float dy = v[k + 1].y - v[k].y;
                                v2[p++] = Vector2(v[k].x + dy1 * (dx / dy), offset);
                        }
                        dy1 = dy2;
                        dy1in = dy2in;
                }
                m_numVertices = p;
        }

        void clipVerticalPlane(float offset, float clipdirection)
        {
                Vector2 *v  = m_vertexBuffers[m_activeVertexBuffer];
                m_activeVertexBuffer ^= 1;
                Vector2 *v2 = m_vertexBuffers[m_activeVertexBuffer];
                v[m_numVertices] = v[0];
                float dx2,   dx1   = offset - v[0].x;
                int   dx2in, dx1in = clipdirection * dx1 >= 0;
                uint32_t  p = 0;
                for (uint32_t k = 0; k < m_numVertices; k++) {
                        dx2 = offset - v[k + 1].x;
                        dx2in = clipdirection * dx2 >= 0;
                        if (dx1in) v2[p++] = v[k];
                        if ( dx1in + dx2in == 1 ) { // not both in/out
                                float dx = v[k + 1].x - v[k].x;
                                float dy = v[k + 1].y - v[k].y;
                                v2[p++] = Vector2(offset, v[k].y + dx1 * (dy / dx));
                        }
                        dx1 = dx2;
                        dx1in = dx2in;
                }
                m_numVertices = p;
        }

        void computeArea()
        {
                Vector2 *v  = m_vertexBuffers[m_activeVertexBuffer];
                v[m_numVertices] = v[0];
                m_area = 0;
                float centroidx = 0, centroidy = 0;
                for (uint32_t k = 0; k < m_numVertices; k++) {
                        // http://local.wasp.uwa.edu.au/~pbourke/geometry/polyarea/
                        float f = v[k].x * v[k + 1].y - v[k + 1].x * v[k].y;
                        m_area += f;
                        centroidx += f * (v[k].x + v[k + 1].x);
                        centroidy += f * (v[k].y + v[k + 1].y);
                }
                m_area = 0.5f * fabsf(m_area);
        }

        void clipAABox(float x0, float y0, float x1, float y1)
        {
                clipVerticalPlane(x0, -1);
                clipHorizontalPlane(y0, -1);
                clipVerticalPlane(x1, 1);
                clipHorizontalPlane(y1, 1);
                computeArea();
        }

        float area() const
        {
                return m_area;
        }

private:
        Vector2 m_verticesA[7 + 1];
        Vector2 m_verticesB[7 + 1];
        Vector2 *m_vertexBuffers[2];
        uint32_t m_numVertices;
        uint32_t m_activeVertexBuffer;
        float m_area;
};

/// A callback to sample the environment. Return false to terminate rasterization.
typedef bool (*SamplingCallback)(void *param, int x, int y);

/// A triangle for rasterization.
struct Triangle
{
        Triangle(const Vector2 &v0, const Vector2 &v1, const Vector2 &v2)
        {
                // Init vertices.
                this->v1 = v0;
                this->v2 = v2;
                this->v3 = v1;
                // make sure every triangle is front facing.
                flipBackface();
                // Compute deltas.
                if (isValid())
                        computeUnitInwardNormals();
        }

        bool isValid()
        {
                const Vector2 e0 = v3 - v1;
                const Vector2 e1 = v2 - v1;
                const float area = e0.y * e1.x - e1.y * e0.x;
                return area != 0.0f;
        }

        // extents has to be multiple of BK_SIZE!!
        bool drawAA(const Vector2 &extents, SamplingCallback cb, void *param)
        {
                const float PX_INSIDE = 1.0f/sqrtf(2.0f);
                const float PX_OUTSIDE = -1.0f/sqrtf(2.0f);
                const float BK_SIZE = 8;
                const float BK_INSIDE = sqrtf(BK_SIZE*BK_SIZE/2.0f);
                const float BK_OUTSIDE = -sqrtf(BK_SIZE*BK_SIZE/2.0f);
                // Bounding rectangle
                float minx = floorf(max(min3(v1.x, v2.x, v3.x), 0.0f));
                float miny = floorf(max(min3(v1.y, v2.y, v3.y), 0.0f));
                float maxx = ceilf( min(max3(v1.x, v2.x, v3.x), extents.x - 1.0f));
                float maxy = ceilf( min(max3(v1.y, v2.y, v3.y), extents.y - 1.0f));
                // There's no reason to align the blocks to the viewport, instead we align them to the origin of the triangle bounds.
                minx = floorf(minx);
                miny = floorf(miny);
                //minx = (float)(((int)minx) & (~((int)BK_SIZE - 1))); // align to blocksize (we don't need to worry about blocks partially out of viewport)
                //miny = (float)(((int)miny) & (~((int)BK_SIZE - 1)));
                minx += 0.5;
                miny += 0.5; // sampling at texel centers!
                maxx += 0.5;
                maxy += 0.5;
                // Half-edge constants
                float C1 = n1.x * (-v1.x) + n1.y * (-v1.y);
                float C2 = n2.x * (-v2.x) + n2.y * (-v2.y);
                float C3 = n3.x * (-v3.x) + n3.y * (-v3.y);
                // Loop through blocks
                for (float y0 = miny; y0 <= maxy; y0 += BK_SIZE) {
                        for (float x0 = minx; x0 <= maxx; x0 += BK_SIZE) {
                                // Corners of block
                                float xc = (x0 + (BK_SIZE - 1) / 2.0f);
                                float yc = (y0 + (BK_SIZE - 1) / 2.0f);
                                // Evaluate half-space functions
                                float aC = C1 + n1.x * xc + n1.y * yc;
                                float bC = C2 + n2.x * xc + n2.y * yc;
                                float cC = C3 + n3.x * xc + n3.y * yc;
                                // Skip block when outside an edge
                                if ( (aC <= BK_OUTSIDE) || (bC <= BK_OUTSIDE) || (cC <= BK_OUTSIDE) ) continue;
                                // Accept whole block when totally covered
                                if ( (aC >= BK_INSIDE) && (bC >= BK_INSIDE) && (cC >= BK_INSIDE) ) {
                                        for (float y = y0; y < y0 + BK_SIZE; y++) {
                                                for (float x = x0; x < x0 + BK_SIZE; x++) {
                                                        if (!cb(param, (int)x, (int)y))
                                                                return false;
                                                }
                                        }
                                } else { // Partially covered block
                                        float CY1 = C1 + n1.x * x0 + n1.y * y0;
                                        float CY2 = C2 + n2.x * x0 + n2.y * y0;
                                        float CY3 = C3 + n3.x * x0 + n3.y * y0;
                                        for (float y = y0; y < y0 + BK_SIZE; y++) { // @@ This is not clipping to scissor rectangle correctly.
                                                float CX1 = CY1;
                                                float CX2 = CY2;
                                                float CX3 = CY3;
                                                for (float x = x0; x < x0 + BK_SIZE; x++) { // @@ This is not clipping to scissor rectangle correctly.
                                                        if (CX1 >= PX_INSIDE && CX2 >= PX_INSIDE && CX3 >= PX_INSIDE) {
                                                                if (!cb(param, (int)x, (int)y))
                                                                        return false;
                                                        } else if ((CX1 >= PX_OUTSIDE) && (CX2 >= PX_OUTSIDE) && (CX3 >= PX_OUTSIDE)) {
                                                                // triangle partially covers pixel. do clipping.
                                                                ClippedTriangle ct(v1 - Vector2(x, y), v2 - Vector2(x, y), v3 - Vector2(x, y));
                                                                ct.clipAABox(-0.5, -0.5, 0.5, 0.5);
                                                                if (ct.area() > 0.0f) {
                                                                        if (!cb(param, (int)x, (int)y))
                                                                                return false;
                                                                }
                                                        }
                                                        CX1 += n1.x;
                                                        CX2 += n2.x;
                                                        CX3 += n3.x;
                                                }
                                                CY1 += n1.y;
                                                CY2 += n2.y;
                                                CY3 += n3.y;
                                        }
                                }
                        }
                }
                return true;
        }

private:
        void flipBackface()
        {
                // check if triangle is backfacing, if so, swap two vertices
                if ( ((v3.x - v1.x) * (v2.y - v1.y) - (v3.y - v1.y) * (v2.x - v1.x)) < 0 ) {
                        Vector2 hv = v1;
                        v1 = v2;
                        v2 = hv; // swap pos
                }
        }

        // compute unit inward normals for each edge.
        void computeUnitInwardNormals()
        {
                n1 = v1 - v2;
                n1 = Vector2(-n1.y, n1.x);
                n1 = n1 * (1.0f / sqrtf(dot(n1, n1)));
                n2 = v2 - v3;
                n2 = Vector2(-n2.y, n2.x);
                n2 = n2 * (1.0f / sqrtf(dot(n2, n2)));
                n3 = v3 - v1;
                n3 = Vector2(-n3.y, n3.x);
                n3 = n3 * (1.0f / sqrtf(dot(n3, n3)));
        }

        // Vertices.
        Vector2 v1, v2, v3;
        Vector2 n1, n2, n3; // unit inward normals
};

// Process the given triangle. Returns false if rasterization was interrupted by the callback.
static bool drawTriangle(const Vector2 &extents, const Vector2 v[3], SamplingCallback cb, void *param)
{
        Triangle tri(v[0], v[1], v[2]);
        // @@ It would be nice to have a conservative drawing mode that enlarges the triangle extents by one texel and is able to handle degenerate triangles.
        // @@ Maybe the simplest thing to do would be raster triangle edges.
        if (tri.isValid())
                return tri.drawAA(extents, cb, param);
        return true;
}

} // namespace raster

// Full and sparse vector and matrix classes. BLAS subset.
// Pseudo-BLAS interface.
namespace sparse {

/**
* Sparse matrix class. The matrix is assumed to be sparse and to have
* very few non-zero elements, for this reason it's stored in indexed
* format. To multiply column vectors efficiently, the matrix stores
* the elements in indexed-column order, there is a list of indexed
* elements for each row of the matrix. As with the FullVector the
* dimension of the matrix is constant.
**/
class Matrix
{
public:
        // An element of the sparse array.
        struct Coefficient
        {
                uint32_t x;  // column
                float v; // value
        };

        Matrix(uint32_t d) : m_width(d), m_array(MemTag::Matrix)
        {
                m_array.resize(d);
                m_array.runCtors();
#if XA_DEBUG_HEAP
                for (uint32_t i = 0; i < d; i++)
                        m_array[i].setMemTag(MemTag::Matrix);
#endif
        }

        Matrix(uint32_t w, uint32_t h) : m_width(w), m_array(MemTag::Matrix)
        {
                m_array.resize(h);
                m_array.runCtors();
#if XA_DEBUG_HEAP
                for (uint32_t i = 0; i < h; i++)
                        m_array[i].setMemTag(MemTag::Matrix);
#endif
        }

        ~Matrix()
        {
                m_array.runDtors();
        }

        Matrix(const Matrix &m) = delete;
        Matrix &operator=(const Matrix &m) = delete;
        uint32_t width() const { return m_width; }
        uint32_t height() const { return m_array.size(); }
        bool isSquare() const { return width() == height(); }

        // x is column, y is row
        float getCoefficient(uint32_t x, uint32_t y) const
        {
                XA_DEBUG_ASSERT( x < width() );
                XA_DEBUG_ASSERT( y < height() );
                const uint32_t count = m_array[y].size();
                for (uint32_t i = 0; i < count; i++) {
                        if (m_array[y][i].x == x) return m_array[y][i].v;
                }
                return 0.0f;
        }

        void setCoefficient(uint32_t x, uint32_t y, float f)
        {
                XA_DEBUG_ASSERT( x < width() );
                XA_DEBUG_ASSERT( y < height() );
                const uint32_t count = m_array[y].size();
                for (uint32_t i = 0; i < count; i++) {
                        if (m_array[y][i].x == x) {
                                m_array[y][i].v = f;
                                return;
                        }
                }
                if (f != 0.0f) {
                        Coefficient c = { x, f };
                        m_array[y].push_back( c );
                }
        }

        float dotRow(uint32_t y, const FullVector &v) const
        {
                XA_DEBUG_ASSERT( y < height() );
                const uint32_t count = m_array[y].size();
                float sum = 0;
                for (uint32_t i = 0; i < count; i++) {
                        sum += m_array[y][i].v * v[m_array[y][i].x];
                }
                return sum;
        }

        void madRow(uint32_t y, float alpha, FullVector &v) const
        {
                XA_DEBUG_ASSERT(y < height());
                const uint32_t count = m_array[y].size();
                for (uint32_t i = 0; i < count; i++) {
                        v[m_array[y][i].x] += alpha * m_array[y][i].v;
                }
        }

        void clearRow(uint32_t y)
        {
                XA_DEBUG_ASSERT( y < height() );
                m_array[y].clear();
        }

        const Array<Coefficient> &getRow(uint32_t y) const { return m_array[y]; }

private:
        /// Number of columns.
        const uint32_t m_width;

        /// Array of matrix elements.
        Array< Array<Coefficient> > m_array;
};

// y = a * x + y
static void saxpy(float a, const FullVector &x, FullVector &y)
{
        XA_DEBUG_ASSERT(x.dimension() == y.dimension());
        const uint32_t dim = x.dimension();
        for (uint32_t i = 0; i < dim; i++) {
                y[i] += a * x[i];
        }
}

static void copy(const FullVector &x, FullVector &y)
{
        XA_DEBUG_ASSERT(x.dimension() == y.dimension());
        const uint32_t dim = x.dimension();
        for (uint32_t i = 0; i < dim; i++) {
                y[i] = x[i];
        }
}

static void scal(float a, FullVector &x)
{
        const uint32_t dim = x.dimension();
        for (uint32_t i = 0; i < dim; i++) {
                x[i] *= a;
        }
}

static float dot(const FullVector &x, const FullVector &y)
{
        XA_DEBUG_ASSERT(x.dimension() == y.dimension());
        const uint32_t dim = x.dimension();
        float sum = 0;
        for (uint32_t i = 0; i < dim; i++) {
                sum += x[i] * y[i];
        }
        return sum;
}

// y = M * x
static void mult(const Matrix &M, const FullVector &x, FullVector &y)
{
        uint32_t w = M.width();
        uint32_t h = M.height();
        XA_DEBUG_ASSERT( w == x.dimension() );
        XA_UNUSED(w);
        XA_DEBUG_ASSERT( h == y.dimension() );
        for (uint32_t i = 0; i < h; i++)
                y[i] = M.dotRow(i, x);
}

// y = alpha*A*x + beta*y
static void sgemv(float alpha, const Matrix &A, const FullVector &x, float beta, FullVector &y)
{
        const uint32_t w = A.width();
        const uint32_t h = A.height();
        XA_DEBUG_ASSERT( w == x.dimension() );
        XA_DEBUG_ASSERT( h == y.dimension() );
        XA_UNUSED(w);
        XA_UNUSED(h);
        for (uint32_t i = 0; i < h; i++)
                y[i] = alpha * A.dotRow(i, x) + beta * y[i];
}

// dot y-row of A by x-column of B
static float dotRowColumn(int y, const Matrix &A, int x, const Matrix &B)
{
        const Array<Matrix::Coefficient> &row = A.getRow(y);
        const uint32_t count = row.size();
        float sum = 0.0f;
        for (uint32_t i = 0; i < count; i++) {
                const Matrix::Coefficient &c = row[i];
                sum += c.v * B.getCoefficient(x, c.x);
        }
        return sum;
}

static void transpose(const Matrix &A, Matrix &B)
{
        XA_DEBUG_ASSERT(A.width() == B.height());
        XA_DEBUG_ASSERT(B.width() == A.height());
        const uint32_t w = A.width();
        for (uint32_t x = 0; x < w; x++) {
                B.clearRow(x);
        }
        const uint32_t h = A.height();
        for (uint32_t y = 0; y < h; y++) {
                const Array<Matrix::Coefficient> &row = A.getRow(y);
                const uint32_t count = row.size();
                for (uint32_t i = 0; i < count; i++) {
                        const Matrix::Coefficient &c = row[i];
                        XA_DEBUG_ASSERT(c.x < w);
                        B.setCoefficient(y, c.x, c.v);
                }
        }
}

static void sgemm(float alpha, const Matrix &A, const Matrix &B, float beta, Matrix &C)
{
        const uint32_t w = C.width();
        const uint32_t h = C.height();
#if XA_DEBUG
        const uint32_t aw = A.width();
        const uint32_t ah = A.height();
        const uint32_t bw = B.width();
        const uint32_t bh = B.height();
        XA_DEBUG_ASSERT(aw == bh);
        XA_DEBUG_ASSERT(bw == ah);
        XA_DEBUG_ASSERT(w == bw);
        XA_DEBUG_ASSERT(h == ah);
#endif
        for (uint32_t y = 0; y < h; y++) {
                for (uint32_t x = 0; x < w; x++) {
                        float c = beta * C.getCoefficient(x, y);
                        // dot y-row of A by x-column of B.
                        c += alpha * dotRowColumn(y, A, x, B);
                        C.setCoefficient(x, y, c);
                }
        }
}

// C = A * B
static void mult(const Matrix &A, const Matrix &B, Matrix &C)
{
        sgemm(1.0f, A, B, 0.0f, C);
}

} // namespace sparse

namespace segment {

// - Insertion is o(n)
// - Smallest element goes at the end, so that popping it is o(1).
struct CostQueue
{
        CostQueue(uint32_t size = UINT32_MAX) : m_maxSize(size), m_pairs(MemTag::SegmentAtlasChartCandidates) {}

        float peekCost() const
        {
                return m_pairs.back().cost;
        }

        uint32_t peekFace() const
        {
                return m_pairs.back().face;
        }

        void push(float cost, uint32_t face)
        {
                const Pair p = { cost, face };
                if (m_pairs.isEmpty() || cost < peekCost())
                        m_pairs.push_back(p);
                else {
                        uint32_t i = 0;
                        const uint32_t count = m_pairs.size();
                        for (; i < count; i++) {
                                if (m_pairs[i].cost < cost)
                                        break;
                        }
                        m_pairs.insertAt(i, p);
                        if (m_pairs.size() > m_maxSize)
                                m_pairs.removeAt(0);
                }
        }

        uint32_t pop()
        {
                XA_DEBUG_ASSERT(!m_pairs.isEmpty());
                uint32_t f = m_pairs.back().face;
                m_pairs.pop_back();
                return f;
        }

        XA_INLINE void clear()
        {
                m_pairs.clear();
        }

        XA_INLINE uint32_t count() const
        {
                return m_pairs.size();
        }

private:
        const uint32_t m_maxSize;

        struct Pair
        {
                float cost;
                uint32_t face;
        };

        Array<Pair> m_pairs;
};

struct Chart
{
        Chart() : faces(MemTag::SegmentAtlasChartFaces) {}

        int id = -1;
        Basis basis; // Best fit normal.
        float area = 0.0f;
        float boundaryLength = 0.0f;
        Vector3 centroidSum = Vector3(0.0f); // Sum of chart face centroids.
        Vector3 centroid = Vector3(0.0f); // Average centroid of chart faces.
        Array<uint32_t> seeds;
        Array<uint32_t> faces;
        Array<uint32_t> failedPlanarRegions;
        CostQueue candidates;
};

struct Atlas
{
        Atlas() : m_edgeLengths(MemTag::SegmentAtlasMeshData), m_faceAreas(MemTag::SegmentAtlasMeshData), m_faceNormals(MemTag::SegmentAtlasMeshData), m_texcoords(MemTag::SegmentAtlasMeshData), m_bestTriangles(10), m_nextPlanarRegionFace(MemTag::SegmentAtlasPlanarRegions), m_facePlanarRegionId(MemTag::SegmentAtlasPlanarRegions) {}

        ~Atlas()
        {
                const uint32_t chartCount = m_charts.size();
                for (uint32_t i = 0; i < chartCount; i++) {
                        m_charts[i]->~Chart();
                        XA_FREE(m_charts[i]);
                }
        }

        uint32_t facesLeft() const { return m_facesLeft; }
        uint32_t chartCount() const { return m_charts.size(); }
        const Array<uint32_t> &chartFaces(uint32_t i) const { return m_charts[i]->faces; }
        const Basis &chartBasis(uint32_t chartIndex) const { return m_charts[chartIndex]->basis; }

        void reset(uint32_t meshId, uint32_t chartGroupId, const Mesh *mesh, const ChartOptions &options)
        {
                XA_UNUSED(meshId);
                XA_UNUSED(chartGroupId);
                XA_PROFILE_START(buildAtlasInit)
                m_mesh = mesh;
                const uint32_t faceCount = m_mesh->faceCount();
                m_facesLeft = faceCount;
                m_options = options;
                m_rand.reset();
                const uint32_t chartCount = m_charts.size();
                for (uint32_t i = 0; i < chartCount; i++) {
                        m_charts[i]->~Chart();
                        XA_FREE(m_charts[i]);
                }
                m_charts.clear();
                m_faceCharts.resize(faceCount);
                m_faceCharts.setAll(-1);
                m_texcoords.resize(faceCount * 3);
                // Precompute edge lengths and face areas.
                const uint32_t edgeCount = m_mesh->edgeCount();
                m_edgeLengths.resize(edgeCount);
                m_faceAreas.resize(faceCount);
                m_faceNormals.resize(faceCount);
                for (uint32_t f = 0; f < faceCount; f++) {
                        for (uint32_t i = 0; i < 3; i++) {
                                const uint32_t edge = f * 3 + i;
                                const Vector3 &p0 = mesh->position(m_mesh->vertexAt(meshEdgeIndex0(edge)));
                                const Vector3 &p1 = mesh->position(m_mesh->vertexAt(meshEdgeIndex1(edge)));
                                m_edgeLengths[edge] = length(p1 - p0);
                                XA_DEBUG_ASSERT(m_edgeLengths[edge] > 0.0f);
                        }
                        m_faceAreas[f] = m_mesh->computeFaceArea(f);
                        XA_DEBUG_ASSERT(m_faceAreas[f] > 0.0f);
                        m_faceNormals[f] = m_mesh->computeFaceNormal(f);
                }
                // Precompute regions of coplanar incident faces.
                m_nextPlanarRegionFace.resize(faceCount);
                m_facePlanarRegionId.resize(faceCount);
                for (uint32_t f = 0; f < faceCount; f++) {
                        m_nextPlanarRegionFace[f] = f;
                        m_facePlanarRegionId[f] = UINT32_MAX;
                }
                Array<uint32_t> faceStack;
                faceStack.reserve(min(faceCount, 16u));
                uint32_t planarRegionCount = 0;
                for (uint32_t f = 0; f < faceCount; f++) {
                        if (m_nextPlanarRegionFace[f] != f)
                                continue; // Already assigned.
                        faceStack.clear();
                        faceStack.push_back(f);
                        for (;;) {
                                if (faceStack.isEmpty())
                                        break;
                                const uint32_t face = faceStack.back();
                                m_facePlanarRegionId[face] = planarRegionCount;
                                faceStack.pop_back();
                                for (Mesh::FaceEdgeIterator it(m_mesh, face); !it.isDone(); it.advance()) {
                                        const uint32_t oface = it.oppositeFace();
                                        if (it.isBoundary())
                                                continue;
                                        if (m_nextPlanarRegionFace[oface] != oface)
                                                continue; // Already assigned.
                                        if (!equal(dot(m_faceNormals[face], m_faceNormals[oface]), 1.0f, kEpsilon))
                                                continue; // Not coplanar.
                                        const uint32_t next = m_nextPlanarRegionFace[face];
                                        m_nextPlanarRegionFace[face] = oface;
                                        m_nextPlanarRegionFace[oface] = next;
                                        m_facePlanarRegionId[oface] = planarRegionCount;
                                        faceStack.push_back(oface);
                                }
                        }
                        planarRegionCount++;
                }
#if XA_DEBUG_EXPORT_OBJ_PLANAR_REGIONS
                char filename[256];
                XA_SPRINTF(filename, sizeof(filename), "debug_mesh_%03u_chartgroup_%03u_planar_regions.obj", meshId, chartGroupId);
                FILE *file;
                XA_FOPEN(file, filename, "w");
                if (file) {
                        m_mesh->writeObjVertices(file);
                        fprintf(file, "s off\n");
                        for (uint32_t i = 0; i < planarRegionCount; i++) {
                                fprintf(file, "o region%u\n", i);
                                for (uint32_t j = 0; j < faceCount; j++) {
                                        if (m_facePlanarRegionId[j] == i)
                                                m_mesh->writeObjFace(file, j);
                                }
                        }
                        fclose(file);
                }
#endif
                XA_PROFILE_END(buildAtlasInit)
        }

        void placeSeeds(float threshold)
        {
                XA_PROFILE_START(buildAtlasPlaceSeeds)
                // Instead of using a predefiened number of seeds:
                // - Add seeds one by one, growing chart until a certain treshold.
                // - Undo charts and restart growing process.
                // @@ How can we give preference to faces far from sharp features as in the LSCM paper?
                //   - those points can be found using a simple flood filling algorithm.
                //   - how do we weight the probabilities?
                while (m_facesLeft > 0)
                        createRandomChart(threshold);
                XA_PROFILE_END(buildAtlasPlaceSeeds)
        }

        // Returns true if any of the charts can grow more.
        void growCharts(float threshold)
        {
                XA_PROFILE_START(buildAtlasGrowCharts)
                for (;;) {
                        if (m_facesLeft == 0)
                                break;
                        // Get the single best candidate out of the chart best candidates.
                        uint32_t bestFace = UINT32_MAX, bestChart = UINT32_MAX;
                        float lowestCost = FLT_MAX;
                        for (uint32_t i = 0; i < m_charts.size(); i++) {
                                Chart *chart = m_charts[i];
                                // Get the best candidate from the chart.
                                // Cleanup any best candidates that have been claimed by another chart.
                                uint32_t face = UINT32_MAX;
                                float cost = FLT_MAX;
                                for (;;) {
                                        if (chart->candidates.count() == 0)
                                                break;
                                        cost = chart->candidates.peekCost();
                                        face = chart->candidates.peekFace();
                                        if (m_faceCharts[face] == -1)
                                                break;
                                        else {
                                                // Face belongs to another chart. Pop from queue so the next best candidate can be retrieved.
                                                chart->candidates.pop();
                                                face = UINT32_MAX;
                                        }
                                }
                                if (face == UINT32_MAX)
                                        continue; // No candidates for this chart.
                                // See if best candidate overall.
                                if (cost < lowestCost) {
                                        lowestCost = cost;
                                        bestFace = face;
                                        bestChart = i;
                                }
                        }
                        if (bestFace == UINT32_MAX || lowestCost > threshold)
                                break;
                        Chart *chart = m_charts[bestChart];
                        chart->candidates.pop(); // Pop the selected candidate from the queue.
                        if (!addFaceToChart(chart, bestFace))
                                chart->failedPlanarRegions.push_back(m_facePlanarRegionId[bestFace]);
                }
                XA_PROFILE_END(buildAtlasGrowCharts)
        }

        void resetCharts()
        {
                XA_PROFILE_START(buildAtlasResetCharts)
                const uint32_t faceCount = m_mesh->faceCount();
                for (uint32_t i = 0; i < faceCount; i++)
                        m_faceCharts[i] = -1;
                m_facesLeft = faceCount;
                const uint32_t chartCount = m_charts.size();
                for (uint32_t i = 0; i < chartCount; i++) {
                        Chart *chart = m_charts[i];
                        const uint32_t seed = chart->seeds.back();
                        chart->area = 0.0f;
                        chart->boundaryLength = 0.0f;
                        chart->basis.normal = Vector3(0.0f);
                        chart->basis.tangent = Vector3(0.0f);
                        chart->basis.bitangent = Vector3(0.0f);
                        chart->centroidSum = Vector3(0.0f);
                        chart->centroid = Vector3(0.0f);
                        chart->faces.clear();
                        chart->candidates.clear();
                        chart->failedPlanarRegions.clear();
                        addFaceToChart(chart, seed);
                }
                XA_PROFILE_END(buildAtlasResetCharts)
        }

        bool relocateSeeds()
        {
                XA_PROFILE_START(buildAtlasRelocateSeeds)
                bool anySeedChanged = false;
                const uint32_t chartCount = m_charts.size();
                for (uint32_t i = 0; i < chartCount; i++) {
                        if (relocateSeed(m_charts[i])) {
                                anySeedChanged = true;
                        }
                }
                XA_PROFILE_END(buildAtlasRelocateSeeds)
                return anySeedChanged;
        }

        void fillHoles(float threshold)
        {
                XA_PROFILE_START(buildAtlasFillHoles)
                while (m_facesLeft > 0)
                        createRandomChart(threshold);
                XA_PROFILE_END(buildAtlasFillHoles)
        }

#if XA_MERGE_CHARTS
        void mergeCharts()
        {
                XA_PROFILE_START(buildAtlasMergeCharts)
                const uint32_t chartCount = m_charts.size();
                // Merge charts progressively until there's none left to merge.
                for (;;) {
                        bool merged = false;
                        for (int c = chartCount - 1; c >= 0; c--) {
                                Chart *chart = m_charts[c];
                                if (chart == nullptr)
                                        continue;
                                float externalBoundaryLength = 0.0f;
                                m_sharedBoundaryLengths.resize(chartCount);
                                m_sharedBoundaryLengths.zeroOutMemory();
                                m_sharedBoundaryLengthsNoSeams.resize(chartCount);
                                m_sharedBoundaryLengthsNoSeams.zeroOutMemory();
                                m_sharedBoundaryEdgeCountNoSeams.resize(chartCount);
                                m_sharedBoundaryEdgeCountNoSeams.zeroOutMemory();
                                const uint32_t faceCount = chart->faces.size();
                                for (uint32_t i = 0; i < faceCount; i++) {
                                        const uint32_t f = chart->faces[i];
                                        for (Mesh::FaceEdgeIterator it(m_mesh, f); !it.isDone(); it.advance()) {
                                                const float l = m_edgeLengths[it.edge()];
                                                if (it.isBoundary()) {
                                                        externalBoundaryLength += l;
                                                } else {
                                                        const int neighborChart = m_faceCharts[it.oppositeFace()];
                                                        if (m_charts[neighborChart] != chart) {
                                                                if ((it.isSeam() && (isNormalSeam(it.edge()) || it.isTextureSeam()))) {
                                                                        externalBoundaryLength += l;
                                                                } else {
                                                                        m_sharedBoundaryLengths[neighborChart] += l;
                                                                }
                                                                m_sharedBoundaryLengthsNoSeams[neighborChart] += l;
                                                                m_sharedBoundaryEdgeCountNoSeams[neighborChart]++;
                                                        }
                                                }
                                        }
                                }
                                for (int cc = chartCount - 1; cc >= 0; cc--) {
                                        if (cc == c)
                                                continue;
                                        Chart *chart2 = m_charts[cc];
                                        if (chart2 == nullptr)
                                                continue;
                                        // Must share a boundary.
                                        if (m_sharedBoundaryLengths[cc] <= 0.0f)
                                                continue;
                                        // Compare proxies.
                                        if (dot(chart2->basis.normal, chart->basis.normal) < XA_MERGE_CHARTS_MIN_NORMAL_DEVIATION)
                                                continue;
                                        // Obey max chart area and boundary length.
                                        if (m_options.maxChartArea > 0.0f && chart->area + chart2->area > m_options.maxChartArea)
                                                continue;
                                        if (m_options.maxBoundaryLength > 0.0f && chart->boundaryLength + chart2->boundaryLength - m_sharedBoundaryLengthsNoSeams[cc] > m_options.maxBoundaryLength)
                                                continue;
                                        // Merge if chart2 has a single face.
                                        // chart1 must have more than 1 face.
                                        // chart2 area must be <= 10% of chart1 area.
                                        if (m_sharedBoundaryLengthsNoSeams[cc] > 0.0f && chart->faces.size() > 1 && chart2->faces.size() == 1 && chart2->area <= chart->area * 0.1f)
                                                goto merge;
                                        // Merge if chart2 has two faces (probably a quad), and chart1 bounds at least 2 of its edges.
                                        if (chart2->faces.size() == 2 && m_sharedBoundaryEdgeCountNoSeams[cc] >= 2)
                                                goto merge;
                                        // Merge if chart2 is wholely inside chart1, ignoring seams.
                                        if (m_sharedBoundaryLengthsNoSeams[cc] > 0.0f && equal(m_sharedBoundaryLengthsNoSeams[cc], chart2->boundaryLength, kEpsilon))
                                                goto merge;
                                        if (m_sharedBoundaryLengths[cc] > 0.2f * max(0.0f, chart->boundaryLength - externalBoundaryLength) ||
                                                m_sharedBoundaryLengths[cc] > 0.75f * chart2->boundaryLength)
                                                goto merge;
                                        continue;
                                merge:
                                        if (!mergeChart(chart, chart2, m_sharedBoundaryLengthsNoSeams[cc]))
                                                continue;
                                        merged = true;
                                        break;
                                }
                                if (merged)
                                        break;
                        }
                        if (!merged)
                                break;
                }
                // Remove deleted charts.
                for (int c = 0; c < int32_t(m_charts.size()); /*do not increment if removed*/) {
                        if (m_charts[c] == nullptr) {
                                m_charts.removeAt(c);
                                // Update m_faceCharts.
                                const uint32_t faceCount = m_faceCharts.size();
                                for (uint32_t i = 0; i < faceCount; i++) {
                                        XA_DEBUG_ASSERT(m_faceCharts[i] != c);
                                        XA_DEBUG_ASSERT(m_faceCharts[i] <= int32_t(m_charts.size()));
                                        if (m_faceCharts[i] > c) {
                                                m_faceCharts[i]--;
                                        }
                                }
                        } else {
                                m_charts[c]->id = c;
                                c++;
                        }
                }
                XA_PROFILE_END(buildAtlasMergeCharts)
        }
#endif

private:
        void createRandomChart(float threshold)
        {
                Chart *chart = XA_NEW(MemTag::Default, Chart);
                chart->id = (int)m_charts.size();
                m_charts.push_back(chart);
                // Pick random face that is not used by any chart yet.
                uint32_t face = m_rand.getRange(m_mesh->faceCount() - 1);
                while (m_faceCharts[face] != -1) {
                        if (++face >= m_mesh->faceCount())
                                face = 0;
                }
                chart->seeds.push_back(face);
                addFaceToChart(chart, face);
                // Grow the chart as much as possible within the given threshold.
                for (;;) {
                        if (chart->candidates.count() == 0 || chart->candidates.peekCost() > threshold)
                                break;
                        const uint32_t f = chart->candidates.pop();
                        if (m_faceCharts[f] != -1)
                                continue;
                        if (!addFaceToChart(chart, f)) {
                                chart->failedPlanarRegions.push_back(m_facePlanarRegionId[f]);
                                continue;
                        }
                }
        }

        bool isChartBoundaryEdge(const Chart *chart, uint32_t edge) const
        {
                const uint32_t oppositeEdge = m_mesh->oppositeEdge(edge);
                const uint32_t oppositeFace = meshEdgeFace(oppositeEdge);
                return oppositeEdge == UINT32_MAX || m_faceCharts[oppositeFace] != chart->id;
        }

        bool computeChartBasis(Chart *chart, Basis *basis)
        {
                const uint32_t faceCount = chart->faces.size();
                m_tempPoints.resize(chart->faces.size() * 3);
                for (uint32_t i = 0; i < faceCount; i++) {
                        const uint32_t f = chart->faces[i];
                        for (uint32_t j = 0; j < 3; j++)
                                m_tempPoints[i * 3 + j] = m_mesh->position(m_mesh->vertexAt(f * 3 + j));
                }
                return Fit::computeBasis(m_tempPoints.data(), m_tempPoints.size(), basis);
        }

        bool isFaceFlipped(uint32_t face) const
        {
                const Vector2 &v1 = m_texcoords[face * 3 + 0];
                const Vector2 &v2 = m_texcoords[face * 3 + 1];
                const Vector2 &v3 = m_texcoords[face * 3 + 2];
                const float parametricArea = ((v2.x - v1.x) * (v3.y - v1.y) - (v3.x - v1.x) * (v2.y - v1.y)) * 0.5f;
                return parametricArea < 0.0f;
        }

        void parameterizeChart(const Chart *chart)
        {
                const uint32_t faceCount = chart->faces.size();
                for (uint32_t i = 0; i < faceCount; i++) {
                        const uint32_t face = chart->faces[i];
                        for (uint32_t j = 0; j < 3; j++) {
                                const uint32_t offset = face * 3 + j;
                                const Vector3 &pos = m_mesh->position(m_mesh->vertexAt(offset));
                                m_texcoords[offset] = Vector2(dot(chart->basis.tangent, pos), dot(chart->basis.bitangent, pos));
                        }
                }
        }

        // m_faceCharts for the chart faces must be set to the chart ID. Needed to compute boundary edges.
        bool isChartParameterizationValid(const Chart *chart)
        {
                const uint32_t faceCount = chart->faces.size();
                // Check for flipped faces in the parameterization. OK if all are flipped.
                uint32_t flippedFaceCount = 0;
                for (uint32_t i = 0; i < faceCount; i++) {
                        if (isFaceFlipped(chart->faces[i]))
                                flippedFaceCount++;
                }
                if (flippedFaceCount != 0 && flippedFaceCount != faceCount)
                        return false;
                // Check for boundary intersection in the parameterization.
                m_boundaryGrid.reset(m_texcoords.data());
                for (uint32_t i = 0; i < faceCount; i++) {
                        const uint32_t f = chart->faces[i];
                        for (uint32_t j = 0; j < 3; j++) {
                                const uint32_t edge = f * 3 + j;
                                if (isChartBoundaryEdge(chart, edge))
                                        m_boundaryGrid.append(edge);
                        }
                }
                if (m_boundaryGrid.intersectSelf(m_mesh->epsilon()))
                        return false;
                return true;
        }

        bool addFaceToChart(Chart *chart, uint32_t face)
        {
                XA_DEBUG_ASSERT(m_faceCharts[face] == -1);
                const uint32_t oldFaceCount = chart->faces.size();
                const bool firstFace = oldFaceCount == 0;
                // Append the face and any coplanar connected faces to the chart faces array.
                chart->faces.push_back(face);
                uint32_t coplanarFace = m_nextPlanarRegionFace[face];
                while (coplanarFace != face) {
                        XA_DEBUG_ASSERT(m_faceCharts[coplanarFace] == -1);
                        chart->faces.push_back(coplanarFace);
                        coplanarFace = m_nextPlanarRegionFace[coplanarFace];
                }
                const uint32_t faceCount = chart->faces.size();
                // Compute basis.
                Basis basis;
                if (firstFace) {
                        // Use the first face normal.
                        // Use any edge as the tangent vector.
                        basis.normal = m_faceNormals[face];
                        basis.tangent = normalize(m_mesh->position(m_mesh->vertexAt(face * 3 + 0)) - m_mesh->position(m_mesh->vertexAt(face * 3 + 1)), kEpsilon);
                        basis.bitangent = cross(basis.normal, basis.tangent);
                } else {
                        // Use best fit normal.
                        if (!computeChartBasis(chart, &basis)) {
                                chart->faces.resize(oldFaceCount);
                                return false;
                        }
                        if (dot(basis.normal, m_faceNormals[face]) < 0.0f) // Flip normal if oriented in the wrong direction.
                                basis.normal = -basis.normal;
                }
                if (!firstFace) {
                        // Compute orthogonal parameterization and check that it is valid.
                        parameterizeChart(chart);
                        for (uint32_t i = oldFaceCount; i < faceCount; i++)
                                m_faceCharts[chart->faces[i]] = chart->id;
                        if (!isChartParameterizationValid(chart)) {
                                for (uint32_t i = oldFaceCount; i < faceCount; i++)
                                        m_faceCharts[chart->faces[i]] = -1;
                                chart->faces.resize(oldFaceCount);
                                return false;
                        }
                }
                // Add face(s) to chart.
                chart->basis = basis;
                chart->area = computeArea(chart, face);
                chart->boundaryLength = computeBoundaryLength(chart, face);
                for (uint32_t i = oldFaceCount; i < faceCount; i++) {
                        const uint32_t f = chart->faces[i];
                        m_faceCharts[f] = chart->id;
                        m_facesLeft--;
                        chart->centroidSum += m_mesh->computeFaceCenter(f);
                }
                chart->centroid = chart->centroidSum / float(chart->faces.size());
                // Refresh candidates.
                chart->candidates.clear();
                for (uint32_t i = 0; i < faceCount; i++) {
                        // Traverse neighboring faces, add the ones that do not belong to any chart yet.
                        const uint32_t f = chart->faces[i];
                        for (uint32_t j = 0; j < 3; j++) {
                                const uint32_t edge = f * 3 + j;
                                const uint32_t oedge = m_mesh->oppositeEdge(edge);
                                if (oedge == UINT32_MAX)
                                        continue; // Boundary edge.
                                const uint32_t oface = meshEdgeFace(oedge);
                                if (m_faceCharts[oface] != -1)
                                        continue; // Face belongs to another chart.
                                if (chart->failedPlanarRegions.contains(m_facePlanarRegionId[oface]))
                                        continue; // Failed to add this faces planar region to the chart before.
                                const float cost = evaluateCost(chart, oface);
                                if (cost < FLT_MAX)
                                        chart->candidates.push(cost, oface);
                        }
                }
                return true;
        }

        // Returns true if the seed has changed.
        bool relocateSeed(Chart *chart)
        {
                // Find the first N triangles that fit the proxy best.
                const uint32_t faceCount = chart->faces.size();
                m_bestTriangles.clear();
                for (uint32_t i = 0; i < faceCount; i++) {
                        const float cost = evaluateProxyFitMetric(chart, chart->faces[i]);
                        m_bestTriangles.push(cost, chart->faces[i]);
                }
                // Of those, choose the least central triangle.
                uint32_t leastCentral = 0;
                float maxDistance = -1;
                for (;;) {
                        if (m_bestTriangles.count() == 0)
                                break;
                        const uint32_t face = m_bestTriangles.pop();
                        Vector3 faceCentroid = m_mesh->computeFaceCenter(face);
                        const float distance = length(chart->centroid - faceCentroid);
                        if (distance > maxDistance) {
                                maxDistance = distance;
                                leastCentral = face;
                        }
                }
                XA_DEBUG_ASSERT(maxDistance >= 0);
                // In order to prevent k-means cyles we record all the previously chosen seeds.
                for (uint32_t i = 0; i < chart->seeds.size(); i++) {
                        // Treat seeds belong to the same planar region as equal.
                        if (chart->seeds[i] == leastCentral || m_facePlanarRegionId[chart->seeds[i]] == m_facePlanarRegionId[leastCentral]) {
                                // Move new seed to the end of the seed array.
                                uint32_t last = chart->seeds.size() - 1;
                                swap(chart->seeds[i], chart->seeds[last]);
                                return false;
                        }
                }
                // Append new seed.
                chart->seeds.push_back(leastCentral);
                return true;
        }

        // Evaluate combined metric.
        float evaluateCost(Chart *chart, uint32_t face) const
        {
                if (dot(m_faceNormals[face], chart->basis.normal) <= 0.26f) // ~75 degrees
                        return FLT_MAX;
                // Estimate boundary length and area:
                float newChartArea = 0.0f, newBoundaryLength = 0.0f;
                if (m_options.maxChartArea > 0.0f || m_options.roundnessMetricWeight > 0.0f)
                        newChartArea = computeArea(chart, face);
                if (m_options.maxBoundaryLength > 0.0f || m_options.roundnessMetricWeight > 0.0f)
                        newBoundaryLength = computeBoundaryLength(chart, face);
                // Enforce limits strictly:
                if (m_options.maxChartArea > 0.0f && newChartArea > m_options.maxChartArea)
                        return FLT_MAX;
                if (m_options.maxBoundaryLength > 0.0f && newBoundaryLength > m_options.maxBoundaryLength)
                        return FLT_MAX;
                float cost = 0.0f;
                if (m_options.normalSeamMetricWeight > 0.0f) {
                        // Penalize faces that cross seams, reward faces that close seams or reach boundaries.
                        // Make sure normal seams are fully respected:
                        const float N = evaluateNormalSeamMetric(chart, face);
                        if (m_options.normalSeamMetricWeight >= 1000.0f && N > 0.0f)
                                return FLT_MAX;
                        cost += m_options.normalSeamMetricWeight * N;
                }
                if (m_options.proxyFitMetricWeight > 0.0f)
                        cost += m_options.proxyFitMetricWeight * evaluateProxyFitMetric(chart, face);
                if (m_options.roundnessMetricWeight > 0.0f)
                        cost += m_options.roundnessMetricWeight * evaluateRoundnessMetric(chart, newBoundaryLength, newChartArea);
                if (m_options.straightnessMetricWeight > 0.0f)
                        cost += m_options.straightnessMetricWeight * evaluateStraightnessMetric(chart, face);
                if (m_options.textureSeamMetricWeight > 0.0f)
                        cost += m_options.textureSeamMetricWeight * evaluateTextureSeamMetric(chart, face);
                //float R = evaluateCompletenessMetric(chart, face);
                //float D = evaluateDihedralAngleMetric(chart, face);
                // @@ Add a metric based on local dihedral angle.
                // @@ Tweaking the normal and texture seam metrics.
                // - Cause more impedance. Never cross 90 degree edges.
                XA_DEBUG_ASSERT(isFinite(cost));
                return cost;
        }

        // Returns a value in [0-1].
        float evaluateProxyFitMetric(Chart *chart, uint32_t face) const
        {
                // All faces in coplanar regions have the same normal, can use any face.
                const Vector3 faceNormal = m_faceNormals[face];
                // Use plane fitting metric for now:
                return 1 - dot(faceNormal, chart->basis.normal); // @@ normal deviations should be weighted by face area
        }

        float evaluateRoundnessMetric(Chart *chart, float newBoundaryLength, float newChartArea) const
        {
                const float roundness = square(chart->boundaryLength) / chart->area;
                const float newBoundaryLengthSq = square(newBoundaryLength);
                const float newRoundness = newBoundaryLengthSq / newChartArea;
                if (newRoundness > roundness)
                        return newBoundaryLengthSq / (newChartArea * kPi4);
                // Offer no impedance to faces that improve roundness.
                return 0;
        }

        float evaluateStraightnessMetric(Chart *chart, uint32_t firstFace) const
        {
                float l_out = 0.0f, l_in = 0.0f;
                const uint32_t planarRegionId = m_facePlanarRegionId[firstFace];
                uint32_t face = firstFace;
                for (;;) {
                        for (Mesh::FaceEdgeIterator it(m_mesh, face); !it.isDone(); it.advance()) {
                                const float l = m_edgeLengths[it.edge()];
                                if (it.isBoundary()) {
                                        l_out += l;
                                } else if (m_facePlanarRegionId[it.oppositeFace()] != planarRegionId) {
                                        if (m_faceCharts[it.oppositeFace()] != chart->id)
                                                l_out += l;
                                        else
                                                l_in += l;
                                }
                        }
                        face = m_nextPlanarRegionFace[face];
                        if (face == firstFace)
                                break;
                }
                XA_DEBUG_ASSERT(l_in != 0.0f); // Candidate face must be adjacent to chart. @@ This is not true if the input mesh has zero-length edges.
                float ratio = (l_out - l_in) / (l_out + l_in);
                return min(ratio, 0.0f); // Only use the straightness metric to close gaps.
        }

        bool isNormalSeam(uint32_t edge) const
        {
                const uint32_t oppositeEdge = m_mesh->oppositeEdge(edge);
                if (oppositeEdge == UINT32_MAX)
                        return false; // boundary edge
                if (m_mesh->flags() & MeshFlags::HasNormals) {
                        const uint32_t v0 = m_mesh->vertexAt(meshEdgeIndex0(edge));
                        const uint32_t v1 = m_mesh->vertexAt(meshEdgeIndex1(edge));
                        const uint32_t ov0 = m_mesh->vertexAt(meshEdgeIndex0(oppositeEdge));
                        const uint32_t ov1 = m_mesh->vertexAt(meshEdgeIndex1(oppositeEdge));
                        if (v0 == ov1 && v1 == ov0)
                                return false;
                        return !equal(m_mesh->normal(v0), m_mesh->normal(ov1), kNormalEpsilon) || !equal(m_mesh->normal(v1), m_mesh->normal(ov0), kNormalEpsilon);
                }
                const uint32_t f0 = meshEdgeFace(edge);
                const uint32_t f1 = meshEdgeFace(oppositeEdge);
                if (m_facePlanarRegionId[f0] == m_facePlanarRegionId[f1])
                        return false;
                return !equal(m_faceNormals[f0], m_faceNormals[f1], kNormalEpsilon);
        }

        float evaluateNormalSeamMetric(Chart *chart, uint32_t firstFace) const
        {
                float seamFactor = 0.0f, totalLength = 0.0f;
                uint32_t face = firstFace;
                for (;;) {
                        for (Mesh::FaceEdgeIterator it(m_mesh, face); !it.isDone(); it.advance()) {
                                if (it.isBoundary())
                                        continue;
                                if (m_faceCharts[it.oppositeFace()] != chart->id)
                                        continue;
                                float l = m_edgeLengths[it.edge()];
                                totalLength += l;
                                if (!it.isSeam())
                                        continue;
                                // Make sure it's a normal seam.
                                if (isNormalSeam(it.edge())) {
                                        float d;
                                        if (m_mesh->flags() & MeshFlags::HasNormals) {
                                                const Vector3 &n0 = m_mesh->normal(it.vertex0());
                                                const Vector3 &n1 = m_mesh->normal(it.vertex1());
                                                const Vector3 &on0 = m_mesh->normal(m_mesh->vertexAt(meshEdgeIndex0(it.oppositeEdge())));
                                                const Vector3 &on1 = m_mesh->normal(m_mesh->vertexAt(meshEdgeIndex1(it.oppositeEdge())));
                                                const float d0 = clamp(dot(n0, on1), 0.0f, 1.0f);
                                                const float d1 = clamp(dot(n1, on0), 0.0f, 1.0f);
                                                d = (d0 + d1) * 0.5f;
                                        } else {
                                                d = clamp(dot(m_faceNormals[face], m_faceNormals[meshEdgeFace(it.oppositeEdge())]), 0.0f, 1.0f);
                                        }
                                        l *= 1 - d;
                                        seamFactor += l;
                                }
                        }
                        face = m_nextPlanarRegionFace[face];
                        if (face == firstFace)
                                break;
                }
                if (seamFactor <= 0.0f)
                        return 0.0f;
                return seamFactor / totalLength;
        }

        float evaluateTextureSeamMetric(Chart *chart, uint32_t firstFace) const
        {
                float seamLength = 0.0f, totalLength = 0.0f;
                uint32_t face = firstFace;
                for (;;) {
                        for (Mesh::FaceEdgeIterator it(m_mesh, face); !it.isDone(); it.advance()) {
                                if (it.isBoundary())
                                        continue;
                                if (m_faceCharts[it.oppositeFace()] != chart->id)
                                        continue;
                                float l = m_edgeLengths[it.edge()];
                                totalLength += l;
                                if (!it.isSeam())
                                        continue;
                                // Make sure it's a texture seam.
                                if (it.isTextureSeam())
                                        seamLength += l;
                        }
                        face = m_nextPlanarRegionFace[face];
                        if (face == firstFace)
                                break;
                }
                if (seamLength <= 0.0f)
                        return 0.0f; // Avoid division by zero.
                return seamLength / totalLength;
        }

        float computeArea(Chart *chart, uint32_t firstFace) const
        {
                float area = chart->area;
                uint32_t face = firstFace;
                for (;;) {
                        area += m_faceAreas[face];
                        face = m_nextPlanarRegionFace[face];
                        if (face == firstFace)
                                break;
                }
                return area;
        }

        float computeBoundaryLength(Chart *chart, uint32_t firstFace) const
        {
                float boundaryLength = chart->boundaryLength;
                // Add new edges, subtract edges shared with the chart.
                const uint32_t planarRegionId = m_facePlanarRegionId[firstFace];
                uint32_t face = firstFace;
                for (;;) {
                        for (Mesh::FaceEdgeIterator it(m_mesh, face); !it.isDone(); it.advance()) {
                                const float edgeLength = m_edgeLengths[it.edge()];
                                if (it.isBoundary()) {
                                        boundaryLength += edgeLength;
                                } else if (m_facePlanarRegionId[it.oppositeFace()] != planarRegionId) {
                                        if (m_faceCharts[it.oppositeFace()] != chart->id)
                                                boundaryLength += edgeLength;
                                        else
                                                boundaryLength -= edgeLength;
                                }
                        }
                        face = m_nextPlanarRegionFace[face];
                        if (face == firstFace)
                                break;
                }
                return max(0.0f, boundaryLength);  // @@ Hack!
        }

        bool mergeChart(Chart *owner, Chart *chart, float sharedBoundaryLength)
        {
                const uint32_t oldOwnerFaceCount = owner->faces.size();
                const uint32_t chartFaceCount = chart->faces.size();
                owner->faces.push_back(chart->faces);
                for (uint32_t i = 0; i < chartFaceCount; i++) {
                        XA_DEBUG_ASSERT(m_faceCharts[chart->faces[i]] == chart->id);
                        m_faceCharts[chart->faces[i]] = owner->id;
                }
                // Compute basis using best fit normal.
                Basis basis;
                if (!computeChartBasis(owner, &basis)) {
                        owner->faces.resize(oldOwnerFaceCount);
                        for (uint32_t i = 0; i < chartFaceCount; i++)
                                m_faceCharts[chart->faces[i]] = chart->id;
                        return false;
                }
                if (dot(basis.normal, m_faceNormals[owner->faces[0]]) < 0.0f) // Flip normal if oriented in the wrong direction.
                        basis.normal = -basis.normal;
                // Compute orthogonal parameterization and check that it is valid.
                parameterizeChart(owner);
                if (!isChartParameterizationValid(owner)) {
                        owner->faces.resize(oldOwnerFaceCount);
                        for (uint32_t i = 0; i < chartFaceCount; i++)
                                m_faceCharts[chart->faces[i]] = chart->id;
                        return false;
                }
                // Merge chart.
                owner->basis = basis;
                owner->failedPlanarRegions.push_back(chart->failedPlanarRegions);
                // Update adjacencies?
                owner->area += chart->area;
                owner->boundaryLength += chart->boundaryLength - sharedBoundaryLength;
                // Delete chart.
                m_charts[chart->id] = nullptr;
                chart->~Chart();
                XA_FREE(chart);
                return true;
        }

        const Mesh *m_mesh;
        Array<float> m_edgeLengths;
        Array<float> m_faceAreas;
        Array<Vector3> m_faceNormals;
        Array<Vector2> m_texcoords;
        uint32_t m_facesLeft;
        Array<int> m_faceCharts;
        Array<Chart *> m_charts;
        CostQueue m_bestTriangles;
        KISSRng m_rand;
        ChartOptions m_options;
        Array<uint32_t> m_nextPlanarRegionFace;
        Array<uint32_t> m_facePlanarRegionId;
        Array<Vector3> m_tempPoints;
        UniformGrid2 m_boundaryGrid;
#if XA_MERGE_CHARTS
        // mergeCharts
        Array<float> m_sharedBoundaryLengths;
        Array<float> m_sharedBoundaryLengthsNoSeams;
        Array<uint32_t> m_sharedBoundaryEdgeCountNoSeams;
#endif
};

} // namespace segment

namespace param {

class JacobiPreconditioner
{
public:
        JacobiPreconditioner(const sparse::Matrix &M, bool symmetric) : m_inverseDiagonal(M.width())
        {
                XA_ASSERT(M.isSquare());
                for (uint32_t x = 0; x < M.width(); x++) {
                        float elem = M.getCoefficient(x, x);
                        //XA_DEBUG_ASSERT( elem != 0.0f ); // This can be zero in the presence of zero area triangles.
                        if (symmetric) {
                                m_inverseDiagonal[x] = (elem != 0) ? 1.0f / sqrtf(fabsf(elem)) : 1.0f;
                        } else {
                                m_inverseDiagonal[x] = (elem != 0) ? 1.0f / elem : 1.0f;
                        }
                }
        }

        void apply(const FullVector &x, FullVector &y) const
        {
                XA_DEBUG_ASSERT(x.dimension() == m_inverseDiagonal.dimension());
                XA_DEBUG_ASSERT(y.dimension() == m_inverseDiagonal.dimension());
                // @@ Wrap vector component-wise product into a separate function.
                const uint32_t D = x.dimension();
                for (uint32_t i = 0; i < D; i++) {
                        y[i] = m_inverseDiagonal[i] * x[i];
                }
        }

private:
        FullVector m_inverseDiagonal;
};

// Linear solvers.
class Solver
{
public:
        // Solve the symmetric system: At·A·x = At·b
        static bool LeastSquaresSolver(const sparse::Matrix &A, const FullVector &b, FullVector &x, float epsilon = 1e-5f)
        {
                XA_DEBUG_ASSERT(A.width() == x.dimension());
                XA_DEBUG_ASSERT(A.height() == b.dimension());
                XA_DEBUG_ASSERT(A.height() >= A.width()); // @@ If height == width we could solve it directly...
                const uint32_t D = A.width();
                sparse::Matrix At(A.height(), A.width());
                sparse::transpose(A, At);
                FullVector Atb(D);
                sparse::mult(At, b, Atb);
                sparse::Matrix AtA(D);
                sparse::mult(At, A, AtA);
                return SymmetricSolver(AtA, Atb, x, epsilon);
        }

        // See section 10.4.3 in: Mesh Parameterization: Theory and Practice, Siggraph Course Notes, August 2007
        static bool LeastSquaresSolver(const sparse::Matrix &A, const FullVector &b, FullVector &x, const uint32_t *lockedParameters, uint32_t lockedCount, float epsilon = 1e-5f)
        {
                XA_DEBUG_ASSERT(A.width() == x.dimension());
                XA_DEBUG_ASSERT(A.height() == b.dimension());
                XA_DEBUG_ASSERT(A.height() >= A.width() - lockedCount);
                // @@ This is not the most efficient way of building a system with reduced degrees of freedom. It would be faster to do it on the fly.
                const uint32_t D = A.width() - lockedCount;
                XA_DEBUG_ASSERT(D > 0);
                // Compute: b - Al * xl
                FullVector b_Alxl(b);
                for (uint32_t y = 0; y < A.height(); y++) {
                        const uint32_t count = A.getRow(y).size();
                        for (uint32_t e = 0; e < count; e++) {
                                uint32_t column = A.getRow(y)[e].x;
                                bool isFree = true;
                                for (uint32_t i = 0; i < lockedCount; i++) {
                                        isFree &= (lockedParameters[i] != column);
                                }
                                if (!isFree) {
                                        b_Alxl[y] -= x[column] * A.getRow(y)[e].v;
                                }
                        }
                }
                // Remove locked columns from A.
                sparse::Matrix Af(D, A.height());
                for (uint32_t y = 0; y < A.height(); y++) {
                        const uint32_t count = A.getRow(y).size();
                        for (uint32_t e = 0; e < count; e++) {
                                uint32_t column = A.getRow(y)[e].x;
                                uint32_t ix = column;
                                bool isFree = true;
                                for (uint32_t i = 0; i < lockedCount; i++) {
                                        isFree &= (lockedParameters[i] != column);
                                        if (column > lockedParameters[i]) ix--; // shift columns
                                }
                                if (isFree) {
                                        Af.setCoefficient(ix, y, A.getRow(y)[e].v);
                                }
                        }
                }
                // Remove elements from x
                FullVector xf(D);
                for (uint32_t i = 0, j = 0; i < A.width(); i++) {
                        bool isFree = true;
                        for (uint32_t l = 0; l < lockedCount; l++) {
                                isFree &= (lockedParameters[l] != i);
                        }
                        if (isFree) {
                                xf[j++] = x[i];
                        }
                }
                // Solve reduced system.
                bool result = LeastSquaresSolver(Af, b_Alxl, xf, epsilon);
                // Copy results back to x.
                for (uint32_t i = 0, j = 0; i < A.width(); i++) {
                        bool isFree = true;
                        for (uint32_t l = 0; l < lockedCount; l++) {
                                isFree &= (lockedParameters[l] != i);
                        }
                        if (isFree) {
                                x[i] = xf[j++];
                        }
                }
                return result;
        }

private:
        /**
        * Compute the solution of the sparse linear system Ab=x using the Conjugate
        * Gradient method.
        *
        * Solving sparse linear systems:
        * (1)                A·x = b
        *
        * The conjugate gradient algorithm solves (1) only in the case that A is
        * symmetric and positive definite. It is based on the idea of minimizing the
        * function
        *
        * (2)                f(x) = 1/2·x·A·x - b·x
        *
        * This function is minimized when its gradient
        *
        * (3)                df = A·x - b
        *
        * is zero, which is equivalent to (1). The minimization is carried out by
        * generating a succession of search directions p.k and improved minimizers x.k.
        * At each stage a quantity alfa.k is found that minimizes f(x.k + alfa.k·p.k),
        * and x.k+1 is set equal to the new point x.k + alfa.k·p.k. The p.k and x.k are
        * built up in such a way that x.k+1 is also the minimizer of f over the whole
        * vector space of directions already taken, {p.1, p.2, . . . , p.k}. After N
        * iterations you arrive at the minimizer over the entire vector space, i.e., the
        * solution to (1).
        *
        * For a really good explanation of the method see:
        *
        * "An Introduction to the Conjugate Gradient Method Without the Agonizing Pain",
        * Jonhathan Richard Shewchuk.
        *
        **/
        // Conjugate gradient with preconditioner.
        static bool ConjugateGradientSolver(const JacobiPreconditioner &preconditioner, const sparse::Matrix &A, const FullVector &b, FullVector &x, float epsilon)
        {
                XA_DEBUG_ASSERT( A.isSquare() );
                XA_DEBUG_ASSERT( A.width() == b.dimension() );
                XA_DEBUG_ASSERT( A.width() == x.dimension() );
                int i = 0;
                const int D = A.width();
                const int i_max = 4 * D;   // Convergence should be linear, but in some cases, it's not.
                FullVector r(D);    // residual
                FullVector p(D);    // search direction
                FullVector q(D);    //
                FullVector s(D);    // preconditioned
                float delta_0;
                float delta_old;
                float delta_new;
                float alpha;
                float beta;
                // r = b - A·x
                sparse::copy(b, r);
                sparse::sgemv(-1, A, x, 1, r);
                // p = M^-1 · r
                preconditioner.apply(r, p);
                delta_new = sparse::dot(r, p);
                delta_0 = delta_new;
                while (i < i_max && delta_new > epsilon * epsilon * delta_0) {
                        i++;
                        // q = A·p
                        sparse::mult(A, p, q);
                        // alpha = delta_new / p·q
                        const float pdotq = sparse::dot(p, q);
                        if (!isFinite(pdotq) || isNan(pdotq))
                                alpha = 0.0f;
                        else
                                alpha = delta_new / pdotq;
                        // x = alfa·p + x
                        sparse::saxpy(alpha, p, x);
                        if ((i & 31) == 0) { // recompute r after 32 steps
                                                                        // r = b - A·x
                                sparse::copy(b, r);
                                sparse::sgemv(-1, A, x, 1, r);
                        } else {
                                // r = r - alfa·q
                                sparse::saxpy(-alpha, q, r);
                        }
                        // s = M^-1 · r
                        preconditioner.apply(r, s);
                        delta_old = delta_new;
                        delta_new = sparse::dot( r, s );
                        beta = delta_new / delta_old;
                        // p = s + beta·p
                        sparse::scal(beta, p);
                        sparse::saxpy(1, s, p);
                }
                return delta_new <= epsilon * epsilon * delta_0;
        }

        static bool SymmetricSolver(const sparse::Matrix &A, const FullVector &b, FullVector &x, float epsilon = 1e-5f)
        {
                XA_DEBUG_ASSERT(A.height() == A.width());
                XA_DEBUG_ASSERT(A.height() == b.dimension());
                XA_DEBUG_ASSERT(b.dimension() == x.dimension());
                JacobiPreconditioner jacobi(A, true);
                return ConjugateGradientSolver(jacobi, A, b, x, epsilon);
        }
};

// Fast sweep in 3 directions
static bool findApproximateDiameterVertices(Mesh *mesh, uint32_t *a, uint32_t *b)
{
        XA_DEBUG_ASSERT(a != nullptr);
        XA_DEBUG_ASSERT(b != nullptr);
        const uint32_t vertexCount = mesh->vertexCount();
        uint32_t minVertex[3];
        uint32_t maxVertex[3];
        minVertex[0] = minVertex[1] = minVertex[2] = UINT32_MAX;
        maxVertex[0] = maxVertex[1] = maxVertex[2] = UINT32_MAX;
        for (uint32_t v = 1; v < vertexCount; v++) {
                if (mesh->isBoundaryVertex(v)) {
                        minVertex[0] = minVertex[1] = minVertex[2] = v;
                        maxVertex[0] = maxVertex[1] = maxVertex[2] = v;
                        break;
                }
        }
        if (minVertex[0] == UINT32_MAX) {
                // Input mesh has not boundaries.
                return false;
        }
        for (uint32_t v = 1; v < vertexCount; v++) {
                if (!mesh->isBoundaryVertex(v)) {
                        // Skip interior vertices.
                        continue;
                }
                const Vector3 &pos = mesh->position(v);
                if (pos.x < mesh->position(minVertex[0]).x)
                        minVertex[0] = v;
                else if (pos.x > mesh->position(maxVertex[0]).x)
                        maxVertex[0] = v;
                if (pos.y < mesh->position(minVertex[1]).y)
                        minVertex[1] = v;
                else if (pos.y > mesh->position(maxVertex[1]).y)
                        maxVertex[1] = v;
                if (pos.z < mesh->position(minVertex[2]).z)
                        minVertex[2] = v;
                else if (pos.z > mesh->position(maxVertex[2]).z)
                        maxVertex[2] = v;
        }
        float lengths[3];
        for (int i = 0; i < 3; i++) {
                lengths[i] = length(mesh->position(minVertex[i]) - mesh->position(maxVertex[i]));
        }
        if (lengths[0] > lengths[1] && lengths[0] > lengths[2]) {
                *a = minVertex[0];
                *b = maxVertex[0];
        } else if (lengths[1] > lengths[2]) {
                *a = minVertex[1];
                *b = maxVertex[1];
        } else {
                *a = minVertex[2];
                *b = maxVertex[2];
        }
        return true;
}

// Conformal relations from Brecht Van Lommel (based on ABF):

static float vec_angle_cos(const Vector3 &v1, const Vector3 &v2, const Vector3 &v3)
{
        Vector3 d1 = v1 - v2;
        Vector3 d2 = v3 - v2;
        return clamp(dot(d1, d2) / (length(d1) * length(d2)), -1.0f, 1.0f);
}

static float vec_angle(const Vector3 &v1, const Vector3 &v2, const Vector3 &v3)
{
        float dot = vec_angle_cos(v1, v2, v3);
        return acosf(dot);
}

static void triangle_angles(const Vector3 &v1, const Vector3 &v2, const Vector3 &v3, float *a1, float *a2, float *a3)
{
        *a1 = vec_angle(v3, v1, v2);
        *a2 = vec_angle(v1, v2, v3);
        *a3 = kPi - *a2 - *a1;
}

static void setup_abf_relations(sparse::Matrix &A, int row, int id0, int id1, int id2, const Vector3 &p0, const Vector3 &p1, const Vector3 &p2)
{
        // @@ IC: Wouldn't it be more accurate to return cos and compute 1-cos^2?
        // It does indeed seem to be a little bit more robust.
        // @@ Need to revisit this more carefully!
        float a0, a1, a2;
        triangle_angles(p0, p1, p2, &a0, &a1, &a2);
        float s0 = sinf(a0);
        float s1 = sinf(a1);
        float s2 = sinf(a2);
        if (s1 > s0 && s1 > s2) {
                swap(s1, s2);
                swap(s0, s1);
                swap(a1, a2);
                swap(a0, a1);
                swap(id1, id2);
                swap(id0, id1);
        } else if (s0 > s1 && s0 > s2) {
                swap(s0, s2);
                swap(s0, s1);
                swap(a0, a2);
                swap(a0, a1);
                swap(id0, id2);
                swap(id0, id1);
        }
        float c0 = cosf(a0);
        float ratio = (s2 == 0.0f) ? 1.0f : s1 / s2;
        float cosine = c0 * ratio;
        float sine = s0 * ratio;
        // Note  : 2*id + 0 --> u
        //         2*id + 1 --> v
        int u0_id = 2 * id0 + 0;
        int v0_id = 2 * id0 + 1;
        int u1_id = 2 * id1 + 0;
        int v1_id = 2 * id1 + 1;
        int u2_id = 2 * id2 + 0;
        int v2_id = 2 * id2 + 1;
        // Real part
        A.setCoefficient(u0_id, 2 * row + 0, cosine - 1.0f);
        A.setCoefficient(v0_id, 2 * row + 0, -sine);
        A.setCoefficient(u1_id, 2 * row + 0, -cosine);
        A.setCoefficient(v1_id, 2 * row + 0, sine);
        A.setCoefficient(u2_id, 2 * row + 0, 1);
        // Imaginary part
        A.setCoefficient(u0_id, 2 * row + 1, sine);
        A.setCoefficient(v0_id, 2 * row + 1, cosine - 1.0f);
        A.setCoefficient(u1_id, 2 * row + 1, -sine);
        A.setCoefficient(v1_id, 2 * row + 1, -cosine);
        A.setCoefficient(v2_id, 2 * row + 1, 1);
}

static bool computeLeastSquaresConformalMap(Mesh *mesh)
{
        // For this to work properly, mesh should not have colocals that have the same
        // attributes, unless you want the vertices to actually have different texcoords.
        const uint32_t vertexCount = mesh->vertexCount();
        const uint32_t D = 2 * vertexCount;
        const uint32_t N = 2 * mesh->faceCount();
        // N is the number of equations (one per triangle)
        // D is the number of variables (one per vertex; there are 2 pinned vertices).
        if (N < D - 4) {
                return false;
        }
        sparse::Matrix A(D, N);
        FullVector b(N);
        FullVector x(D);
        // Fill b:
        b.fill(0.0f);
        // Fill x:
        uint32_t v0, v1;
        if (!findApproximateDiameterVertices(mesh, &v0, &v1)) {
                // Mesh has no boundaries.
                return false;
        }
        if (mesh->texcoord(v0) == mesh->texcoord(v1)) {
                // LSCM expects an existing parameterization.
                return false;
        }
        for (uint32_t v = 0; v < vertexCount; v++) {
                // Initial solution.
                x[2 * v + 0] = mesh->texcoord(v).x;
                x[2 * v + 1] = mesh->texcoord(v).y;
        }
        // Fill A:
        const uint32_t faceCount = mesh->faceCount();
        for (uint32_t f = 0, t = 0; f < faceCount; f++) {
                const uint32_t vertex0 = mesh->vertexAt(f * 3 + 0);
                const uint32_t vertex1 = mesh->vertexAt(f * 3 + 1);
                const uint32_t vertex2 = mesh->vertexAt(f * 3 + 2);
                setup_abf_relations(A, t, vertex0, vertex1, vertex2, mesh->position(vertex0), mesh->position(vertex1), mesh->position(vertex2));
                t++;
        }
        const uint32_t lockedParameters[] = {
                2 * v0 + 0,
                2 * v0 + 1,
                2 * v1 + 0,
                2 * v1 + 1
        };
        // Solve
        Solver::LeastSquaresSolver(A, b, x, lockedParameters, 4, 0.000001f);
        // Map x back to texcoords:
        for (uint32_t v = 0; v < vertexCount; v++) {
                mesh->texcoord(v) = Vector2(x[2 * v + 0], x[2 * v + 1]);
                XA_DEBUG_ASSERT(!isNan(mesh->texcoord(v).x));
                XA_DEBUG_ASSERT(!isNan(mesh->texcoord(v).y));
        }
        return true;
}

#if XA_RECOMPUTE_CHARTS
struct PiecewiseParam
{
        void reset(const Mesh *mesh, uint32_t faceCount)
        {
                m_mesh = mesh;
                m_faceCount = faceCount;
                const uint32_t vertexCount = m_mesh->vertexCount();
                m_texcoords.resize(vertexCount);
                m_patch.reserve(m_faceCount);
                m_faceAssigned.resize(m_faceCount);
                m_faceAssigned.zeroOutMemory();
                m_faceInvalid.resize(m_faceCount);
                m_faceInPatch.resize(m_faceCount);
                m_vertexInPatch.resize(vertexCount);
                m_faceInCandidates.resize(m_faceCount);
        }

        ConstArrayView<uint32_t> chartFaces() const { return m_patch; }
        const Vector2 *texcoords() const { return m_texcoords.data(); }

        bool computeChart()
        {
                m_patch.clear();
                m_faceInvalid.zeroOutMemory();
                m_faceInPatch.zeroOutMemory();
                m_vertexInPatch.zeroOutMemory();
                // Add the seed face (first unassigned face) to the patch.
                uint32_t seed = UINT32_MAX;
                for (uint32_t f = 0; f < m_faceCount; f++) {
                        if (m_faceAssigned.get(f))
                                continue;
                        seed = f;
                        m_patch.push_back(seed);
                        m_faceInPatch.set(seed);
                        m_faceAssigned.set(seed);
                        Vector2 texcoords[3];
                        orthoProjectFace(seed, texcoords);
                        for (uint32_t i = 0; i < 3; i++) {
                                const uint32_t vertex = m_mesh->vertexAt(seed * 3 + i);
                                m_vertexInPatch.set(vertex);
                                m_texcoords[vertex] = texcoords[i];
                        }
                        break;
                }
                if (seed == UINT32_MAX)
                        return false;
                for (;;) {
                        findCandidates();
                        if (m_candidates.isEmpty())
                                break;
                        for (;;) {
                                // Find the candidate with the lowest cost.
                                float lowestCost = FLT_MAX;
                                uint32_t bestCandidate = UINT32_MAX;
                                for (uint32_t i = 0; i < m_candidates.size(); i++) {
                                        const Candidate &candidate = m_candidates[i];
                                        if (m_faceInvalid.get(candidate.face)) // A candidate face may be invalidated after is was added.
                                                continue;
                                        if (candidate.maxCost < lowestCost) {
                                                lowestCost = candidate.maxCost;
                                                bestCandidate = i;
                                        }
                                }
                                if (bestCandidate == UINT32_MAX)
                                        break;
                                // Compute the position by averaging linked candidates (candidates that share the same free vertex).
                                Vector2 position(0.0f);
                                uint32_t n = 0;
                                for (CandidateIterator it(m_candidates, bestCandidate); !it.isDone(); it.advance()) {
                                        position += it.current().position;
                                        n++;
                                }
                                position *= 1.0f / (float)n;
                                const uint32_t freeVertex = m_candidates[bestCandidate].vertex;
                                XA_DEBUG_ASSERT(!isNan(position.x));
                                XA_DEBUG_ASSERT(!isNan(position.y));
                                m_texcoords[freeVertex] = position;
                                // Check for flipped faces. This is also done when candidates are first added, but the averaged position of the free vertex is different now, so check again.
                                bool invalid = false;
                                for (CandidateIterator it(m_candidates, bestCandidate); !it.isDone(); it.advance()) {
                                        const uint32_t vertex0 = m_mesh->vertexAt(meshEdgeIndex0(it.current().patchEdge));
                                        const uint32_t vertex1 = m_mesh->vertexAt(meshEdgeIndex1(it.current().patchEdge));
                                        const float freeVertexOrient = orientToEdge(m_texcoords[vertex0], m_texcoords[vertex1], position);
                                        if ((it.current().patchVertexOrient < 0.0f && freeVertexOrient < 0.0f) || (it.current().patchVertexOrient > 0.0f && freeVertexOrient > 0.0f)) {
                                                invalid = true;
                                                break;
                                        }
                                }
                                // Check for boundary intersection.
                                if (!invalid) {
                                        m_boundaryGrid.reset(m_texcoords.data(), m_mesh->indices());
                                        // Add edges on the patch boundary to the grid.
                                        // Temporarily adding candidate faces to the patch makes it simpler to detect which edges are on the boundary.
                                        const uint32_t oldPatchSize = m_patch.size();
                                        for (CandidateIterator it(m_candidates, bestCandidate); !it.isDone(); it.advance())
                                                m_patch.push_back(it.current().face);
                                        for (uint32_t i = 0; i < m_patch.size(); i++) {
                                                for (Mesh::FaceEdgeIterator it(m_mesh, m_patch[i]); !it.isDone(); it.advance()) {
                                                        const uint32_t oface = it.oppositeFace();
                                                        if (oface == UINT32_MAX || oface >= m_faceCount || !m_faceInPatch.get(oface))
                                                                m_boundaryGrid.append(it.edge());
                                                }
                                        }
                                        invalid = m_boundaryGrid.intersectSelf(m_mesh->epsilon());
                                        m_patch.resize(oldPatchSize);
                                }
                                if (invalid) {
                                        // Mark all faces of linked candidates as invalid.
                                        for (CandidateIterator it(m_candidates, bestCandidate); !it.isDone(); it.advance())
                                                m_faceInvalid.set(it.current().face);
                                        continue;
                                }
                                // Add faces to the patch.
                                for (CandidateIterator it(m_candidates, bestCandidate); !it.isDone(); it.advance()) {
                                        m_patch.push_back(it.current().face);
                                        m_faceInPatch.set(it.current().face);
                                        m_faceAssigned.set(it.current().face);
                                }
                                // Add vertex to the patch.
                                m_vertexInPatch.set(freeVertex);
                                // Successfully added candidate face(s) to patch.
                                break;
                        }
                }
                return true;
        }

private:
        struct Candidate
        {
                uint32_t face, vertex;
                uint32_t next; // The next candidate with the same vertex.
                Vector2 position;
                float cost;
                float maxCost; // Of all linked candidates.
                uint32_t patchEdge;
                float patchVertexOrient;
        };

        struct CandidateIterator
        {
                CandidateIterator(Array<Candidate> &candidates, uint32_t first) : m_candidates(candidates), m_current(first) {}
                void advance() { if (m_current != UINT32_MAX) m_current = m_candidates[m_current].next; }
                bool isDone() const { return m_current == UINT32_MAX; }
                Candidate &current() { return m_candidates[m_current]; }

        private:
                Array<Candidate> &m_candidates;
                uint32_t m_current;
        };

        const Mesh *m_mesh;
        uint32_t m_faceCount;
        Array<Vector2> m_texcoords;
        Array<Candidate> m_candidates;
        BitArray m_faceInCandidates;
        Array<uint32_t> m_patch;
        BitArray m_faceAssigned; // Face is assigned to a previous chart or the current patch.
        BitArray m_faceInPatch, m_vertexInPatch;
        BitArray m_faceInvalid; // Face cannot be added to the patch - flipped, cost too high or causes boundary intersection.
        UniformGrid2 m_boundaryGrid;

        // Find candidate faces on the patch front.
        void findCandidates()
        {
                m_candidates.clear();
                m_faceInCandidates.zeroOutMemory();
                for (uint32_t i = 0; i < m_patch.size(); i++) {
                        for (Mesh::FaceEdgeIterator it(m_mesh, m_patch[i]); !it.isDone(); it.advance()) {
                                const uint32_t oface = it.oppositeFace();
                                if (oface == UINT32_MAX || oface >= m_faceCount || m_faceAssigned.get(oface) || m_faceInCandidates.get(oface))
                                        continue;
                                // Found an active edge on the patch front.
                                // Find the free vertex (the vertex that isn't on the active edge).
                                // Compute the orientation of the other patch face vertex to the active edge.
                                uint32_t freeVertex = UINT32_MAX;
                                float orient = 0.0f;
                                for (uint32_t j = 0; j < 3; j++) {
                                        const uint32_t vertex = m_mesh->vertexAt(oface * 3 + j);
                                        if (vertex != it.vertex0() && vertex != it.vertex1()) {
                                                freeVertex = vertex;
                                                orient = orientToEdge(m_texcoords[it.vertex0()], m_texcoords[it.vertex1()], m_texcoords[m_mesh->vertexAt(m_patch[i] * 3 + j)]);
                                                break;
                                        }
                                }
                                XA_DEBUG_ASSERT(freeVertex != UINT32_MAX);
                                // If the free vertex is already in the patch, the face is enclosed by the patch. Add the face to the patch - don't need to assign texcoords.
                                if (m_vertexInPatch.get(freeVertex)) {
                                        freeVertex = UINT32_MAX;
                                        m_patch.push_back(oface);
                                        m_faceAssigned.set(oface);
                                        continue;
                                }
                                // Check this here rather than above so faces enclosed by the patch are always added.
                                if (m_faceInvalid.get(oface))
                                        continue;
                                addCandidateFace(it.edge(), orient, oface, it.oppositeEdge(), freeVertex);
                        }
                }
                // Link candidates that share the same vertex.
                for (uint32_t i = 0; i < m_candidates.size(); i++) {
                        if (m_candidates[i].next != UINT32_MAX)
                                continue;
                        uint32_t current = i;
                        for (uint32_t j = i + 1; j < m_candidates.size(); j++) {
                                if (m_candidates[j].vertex == m_candidates[current].vertex) {
                                        m_candidates[current].next = j;
                                        current = j;
                                }
                        }
                }
                // Set max cost for linked candidates.
                for (uint32_t i = 0; i < m_candidates.size(); i++) {
                        float maxCost = 0.0f;
                        for (CandidateIterator it(m_candidates, i); !it.isDone(); it.advance())
                                maxCost = max(maxCost, it.current().cost);
                        for (CandidateIterator it(m_candidates, i); !it.isDone(); it.advance())
                                it.current().maxCost = maxCost;
                }
        }

        void addCandidateFace(uint32_t patchEdge, float patchVertexOrient, uint32_t face, uint32_t edge, uint32_t freeVertex)
        {
                Vector2 texcoords[3];
                orthoProjectFace(face, texcoords);
                // Find corresponding vertices between the patch edge and candidate edge.
                const uint32_t vertex0 = m_mesh->vertexAt(meshEdgeIndex0(patchEdge));
                const uint32_t vertex1 = m_mesh->vertexAt(meshEdgeIndex1(patchEdge));
                uint32_t localVertex0 = UINT32_MAX, localVertex1 = UINT32_MAX, localFreeVertex = UINT32_MAX;
                for (uint32_t i = 0; i < 3; i++) {
                        const uint32_t vertex = m_mesh->vertexAt(face * 3 + i);
                        if (vertex == m_mesh->vertexAt(meshEdgeIndex1(edge)))
                                localVertex0 = i;
                        else if (vertex == m_mesh->vertexAt(meshEdgeIndex0(edge)))
                                localVertex1 = i;
                        else
                                localFreeVertex = i;
                }
                // Scale orthogonal projection to match the patch edge.
                const Vector2 patchEdgeVec = m_texcoords[vertex1] - m_texcoords[vertex0];
                const Vector2 localEdgeVec = texcoords[localVertex1] - texcoords[localVertex0];
                const float len1 = length(patchEdgeVec);
                const float len2 = length(localEdgeVec);
                const float scale = len1 / len2;
                XA_ASSERT(scale > 0.0f);
                for (uint32_t i = 0; i < 3; i++)
                        texcoords[i] *= scale;
                // Translate to the first vertex on the patch edge.
                const Vector2 translate = m_texcoords[vertex0] - texcoords[localVertex0];
                for (uint32_t i = 0; i < 3; i++)
                        texcoords[i] += translate;
                // Compute the angle between the patch edge and the corresponding local edge.
                const float angle = atan2f(patchEdgeVec.y, patchEdgeVec.x) - atan2f(localEdgeVec.y, localEdgeVec.x);
                // Rotate so the patch edge and the corresponding local edge occupy the same space.
                for (uint32_t i = 0; i < 3; i++) {
                        if (i == localVertex0)
                                continue;
                        Vector2 &uv = texcoords[i];
                        uv -= texcoords[localVertex0]; // Rotate around the first vertex.
                        const float c = cosf(angle);
                        const float s = sinf(angle);
                        const float x = uv.x * c - uv.y * s;
                        const float y = uv.y * c + uv.x * s;
                        uv.x = x + texcoords[localVertex0].x;
                        uv.y = y + texcoords[localVertex0].y;
                }
                // Check for local overlap (flipped triangle).
                // The patch face vertex that isn't on the active edge and the free vertex should be oriented on opposite sides to the active edge.
                const float freeVertexOrient = orientToEdge(m_texcoords[vertex0], m_texcoords[vertex1], texcoords[localFreeVertex]);
                if ((patchVertexOrient < 0.0f && freeVertexOrient < 0.0f) || (patchVertexOrient > 0.0f && freeVertexOrient > 0.0f)) {
                        m_faceInvalid.set(face);
                        return;
                }
                const float stretch = computeStretch(m_mesh->position(vertex0), m_mesh->position(vertex1), m_mesh->position(freeVertex), texcoords[0], texcoords[1], texcoords[2]);
                if (stretch >= FLT_MAX) {
                        m_faceInvalid.set(face);
                        return;
                }
                const float cost = fabsf(stretch - 1.0f);
#if 0
                if (cost > 0.25f) {
                        m_faceInvalid.set(face);
                        return;
                }
#endif
                // Add the candidate.
                Candidate candidate;
                candidate.face = face;
                candidate.vertex = freeVertex;
                candidate.position = texcoords[localFreeVertex];
                candidate.next = UINT32_MAX;
                candidate.cost = cost;
                candidate.patchEdge = patchEdge;
                candidate.patchVertexOrient = patchVertexOrient;
                m_candidates.push_back(candidate);
                m_faceInCandidates.set(face);
        }

        void orthoProjectFace(uint32_t face, Vector2 *texcoords) const
        {
                const Vector3 normal = m_mesh->computeFaceNormal(face);
                const Vector3 tangent = normalize(m_mesh->position(m_mesh->vertexAt(face * 3 + 1)) - m_mesh->position(m_mesh->vertexAt(face * 3 + 0)), kEpsilon);
                const Vector3 bitangent = cross(normal, tangent);
                for (uint32_t i = 0; i < 3; i++) {
                        const Vector3 &pos = m_mesh->position(m_mesh->vertexAt(face * 3 + i));
                        texcoords[i] = Vector2(dot(tangent, pos), dot(bitangent, pos));
                }
        }

        float parametricArea(const Vector2 *texcoords) const
        {
                const Vector2 &v1 = texcoords[0];
                const Vector2 &v2 = texcoords[1];
                const Vector2 &v3 = texcoords[2];
                return ((v2.x - v1.x) * (v3.y - v1.y) - (v3.x - v1.x) * (v2.y - v1.y)) * 0.5f;
        }

        float computeStretch(Vector3 p1, Vector3 p2, Vector3 p3, Vector2 t1, Vector2 t2, Vector2 t3) const
        {
                float parametricArea = ((t2.y - t1.y) * (t3.x - t1.x) - (t3.y - t1.y) * (t2.x - t1.x)) * 0.5f;
                if (isZero(parametricArea, kAreaEpsilon))
                        return FLT_MAX;
                if (parametricArea < 0.0f)
                        parametricArea = fabsf(parametricArea);
                const float geometricArea = length(cross(p2 - p1, p3 - p1)) * 0.5f;
                if (parametricArea <= geometricArea)
                        return parametricArea / geometricArea;
                else
                        return geometricArea / parametricArea;
        }

        // Return value is positive if the point is one side of the edge, negative if on the other side.
        float orientToEdge(Vector2 edgeVertex0, Vector2 edgeVertex1, Vector2 point) const
        {
                return (edgeVertex0.x - point.x) * (edgeVertex1.y - point.y) - (edgeVertex0.y - point.y) * (edgeVertex1.x - point.x);
        }
};
#endif

// Estimate quality of existing parameterization.
struct Quality
{
        // computeBoundaryIntersection
        bool boundaryIntersection = false;

        // computeFlippedFaces
        uint32_t totalTriangleCount = 0;
        uint32_t flippedTriangleCount = 0;
        uint32_t zeroAreaTriangleCount = 0;

        // computeMetrics
        float totalParametricArea = 0.0f;
        float totalGeometricArea = 0.0f;
        float stretchMetric = 0.0f;
        float maxStretchMetric = 0.0f;
        float conformalMetric = 0.0f;
        float authalicMetric = 0.0f;

        void computeBoundaryIntersection(const Mesh *mesh, UniformGrid2 &boundaryGrid)
        {
                const Array<uint32_t> &boundaryEdges = mesh->boundaryEdges();
                const uint32_t boundaryEdgeCount = boundaryEdges.size();
                boundaryGrid.reset(mesh->texcoords(), mesh->indices(), boundaryEdgeCount);
                for (uint32_t i = 0; i < boundaryEdgeCount; i++)
                        boundaryGrid.append(boundaryEdges[i]);
                boundaryIntersection = boundaryGrid.intersectSelf(mesh->epsilon());
#if XA_DEBUG_EXPORT_BOUNDARY_GRID
                static int exportIndex = 0;
                char filename[256];
                XA_SPRINTF(filename, sizeof(filename), "debug_boundary_grid_%03d.tga", exportIndex);
                boundaryGrid.debugExport(filename);
                exportIndex++;
#endif
        }

        void computeFlippedFaces(const Mesh *mesh, uint32_t faceCount, Array<uint32_t> *flippedFaces)
        {
                totalTriangleCount = flippedTriangleCount = zeroAreaTriangleCount = 0;
                if (flippedFaces)
                        flippedFaces->clear();
                for (uint32_t f = 0; f < faceCount; f++) {
                        Vector2 texcoord[3];
                        for (int i = 0; i < 3; i++) {
                                const uint32_t v = mesh->vertexAt(f * 3 + i);
                                texcoord[i] = mesh->texcoord(v);
                        }
                        totalTriangleCount++;
                        const float t1 = texcoord[0].x;
                        const float s1 = texcoord[0].y;
                        const float t2 = texcoord[1].x;
                        const float s2 = texcoord[1].y;
                        const float t3 = texcoord[2].x;
                        const float s3 = texcoord[2].y;
                        const float parametricArea = ((s2 - s1) * (t3 - t1) - (s3 - s1) * (t2 - t1)) * 0.5f;
                        if (isZero(parametricArea, kAreaEpsilon)) {
                                zeroAreaTriangleCount++;
                                continue;
                        }
                        if (parametricArea < 0.0f) {
                                // Count flipped triangles.
                                flippedTriangleCount++;
                                if (flippedFaces)
                                        flippedFaces->push_back(f);
                        }
                }
                if (flippedTriangleCount + zeroAreaTriangleCount == totalTriangleCount) {
                        // If all triangles are flipped, then none are.
                        if (flippedFaces)
                                flippedFaces->clear();
                        flippedTriangleCount = 0;
                }
                if (flippedTriangleCount > totalTriangleCount / 2)
                {
                        // If more than half the triangles are flipped, reverse the flipped / not flipped classification.
                        flippedTriangleCount = totalTriangleCount - flippedTriangleCount;
                        if (flippedFaces) {
                                Array<uint32_t> temp;
                                flippedFaces->copyTo(temp);
                                flippedFaces->clear();
                                for (uint32_t f = 0; f < faceCount; f++) {
                                        bool match = false;
                                        for (uint32_t ff = 0; ff < temp.size(); ff++) {
                                                if (temp[ff] == f) {
                                                        match = true;
                                                        break;
                                                }
                                        }
                                        if (!match)
                                                flippedFaces->push_back(f);
                                }
                        }
                }
        }

        void computeMetrics(const Mesh *mesh, uint32_t faceCount)
        {
                totalGeometricArea = totalParametricArea = 0.0f;
                stretchMetric = maxStretchMetric = conformalMetric = authalicMetric = 0.0f;
                for (uint32_t f = 0; f < faceCount; f++) {
                        Vector3 pos[3];
                        Vector2 texcoord[3];
                        for (int i = 0; i < 3; i++) {
                                const uint32_t v = mesh->vertexAt(f * 3 + i);
                                pos[i] = mesh->position(v);
                                texcoord[i] = mesh->texcoord(v);
                        }
                        // Evaluate texture stretch metric. See:
                        // - "Texture Mapping Progressive Meshes", Sander, Snyder, Gortler & Hoppe
                        // - "Mesh Parameterization: Theory and Practice", Siggraph'07 Course Notes, Hormann, Levy & Sheffer.
                        const float t1 = texcoord[0].x;
                        const float s1 = texcoord[0].y;
                        const float t2 = texcoord[1].x;
                        const float s2 = texcoord[1].y;
                        const float t3 = texcoord[2].x;
                        const float s3 = texcoord[2].y;
                        float parametricArea = ((s2 - s1) * (t3 - t1) - (s3 - s1) * (t2 - t1)) * 0.5f;
                        if (isZero(parametricArea, kAreaEpsilon))
                                continue;
                        if (parametricArea < 0.0f)
                                parametricArea = fabsf(parametricArea);
                        const float geometricArea = length(cross(pos[1] - pos[0], pos[2] - pos[0])) / 2;
                        const Vector3 Ss = (pos[0] * (t2 - t3) + pos[1] * (t3 - t1) + pos[2] * (t1 - t2)) / (2 * parametricArea);
                        const Vector3 St = (pos[0] * (s3 - s2) + pos[1] * (s1 - s3) + pos[2] * (s2 - s1)) / (2 * parametricArea);
                        const float a = dot(Ss, Ss); // E
                        const float b = dot(Ss, St); // F
                        const float c = dot(St, St); // G
                                                                                 // Compute eigen-values of the first fundamental form:
                        const float sigma1 = sqrtf(0.5f * max(0.0f, a + c - sqrtf(square(a - c) + 4 * square(b)))); // gamma uppercase, min eigenvalue.
                        const float sigma2 = sqrtf(0.5f * max(0.0f, a + c + sqrtf(square(a - c) + 4 * square(b)))); // gamma lowercase, max eigenvalue.
                        XA_ASSERT(sigma2 > sigma1 || equal(sigma1, sigma2, kEpsilon));
                        // isometric: sigma1 = sigma2 = 1
                        // conformal: sigma1 / sigma2 = 1
                        // authalic: sigma1 * sigma2 = 1
                        const float rmsStretch = sqrtf((a + c) * 0.5f);
                        const float rmsStretch2 = sqrtf((square(sigma1) + square(sigma2)) * 0.5f);
                        XA_DEBUG_ASSERT(equal(rmsStretch, rmsStretch2, 0.01f));
                        XA_UNUSED(rmsStretch2);
                        stretchMetric += square(rmsStretch) * geometricArea;
                        maxStretchMetric = max(maxStretchMetric, sigma2);
                        if (!isZero(sigma1, 0.000001f)) {
                                // sigma1 is zero when geometricArea is zero.
                                conformalMetric += (sigma2 / sigma1) * geometricArea;
                        }
                        authalicMetric += (sigma1 * sigma2) * geometricArea;
                        // Accumulate total areas.
                        totalGeometricArea += geometricArea;
                        totalParametricArea += parametricArea;
                }
                XA_DEBUG_ASSERT(isFinite(totalParametricArea) && totalParametricArea >= 0);
                XA_DEBUG_ASSERT(isFinite(totalGeometricArea) && totalGeometricArea >= 0);
                XA_DEBUG_ASSERT(isFinite(stretchMetric));
                XA_DEBUG_ASSERT(isFinite(maxStretchMetric));
                XA_DEBUG_ASSERT(isFinite(conformalMetric));
                XA_DEBUG_ASSERT(isFinite(authalicMetric));
                if (totalGeometricArea > 0.0f) {
                        const float normFactor = sqrtf(totalParametricArea / totalGeometricArea);
                        stretchMetric = sqrtf(stretchMetric / totalGeometricArea) * normFactor;
                        maxStretchMetric  *= normFactor;
                        conformalMetric = sqrtf(conformalMetric / totalGeometricArea);
                        authalicMetric = sqrtf(authalicMetric / totalGeometricArea);
                }
        }
};

struct ChartWarningFlags
{
        enum Enum
        {
                CloseHolesFailed = 1<<1,
                FixTJunctionsDuplicatedEdge = 1<<2,
                FixTJunctionsFailed = 1<<3,
                TriangulateDuplicatedEdge = 1<<4,
        };
};

struct ChartCtorBuffers
{
        Array<uint32_t> chartMeshIndices;
        Array<uint32_t> unifiedMeshIndices;
        Array<uint32_t> boundaryLoops;
};

/// A chart is a connected set of faces with a certain topology (usually a disk).
class Chart
{
public:
        Chart(ChartCtorBuffers &buffers, const Basis &basis, ConstArrayView<uint32_t> faces, const Mesh *originalMesh, uint32_t meshId, uint32_t chartGroupId, uint32_t chartId) : m_basis(basis), m_mesh(nullptr), m_unifiedMesh(nullptr), m_unmodifiedUnifiedMesh(nullptr), m_type(ChartType::LSCM), m_warningFlags(0), m_closedHolesCount(0), m_fixedTJunctionsCount(0)
        {
                XA_UNUSED(meshId);
                XA_UNUSED(chartGroupId);
                XA_UNUSED(chartId);
                m_faceArray.copyFrom(faces.data, faces.length);
                // Copy face indices.
                m_mesh = XA_NEW_ARGS(MemTag::Mesh, Mesh, originalMesh->epsilon(), m_faceArray.size() * 3, m_faceArray.size());
                m_unifiedMesh = XA_NEW_ARGS(MemTag::Mesh, Mesh, originalMesh->epsilon(), m_faceArray.size() * 3, m_faceArray.size());
                Array<uint32_t> &chartMeshIndices = buffers.chartMeshIndices;
                chartMeshIndices.resize(originalMesh->vertexCount());
                chartMeshIndices.setAll(UINT32_MAX);
                Array<uint32_t> &unifiedMeshIndices = buffers.unifiedMeshIndices;
                unifiedMeshIndices.resize(originalMesh->vertexCount());
                unifiedMeshIndices.setAll(UINT32_MAX);
                // Add vertices.
                const uint32_t faceCount = m_initialFaceCount = m_faceArray.size();
                for (uint32_t f = 0; f < faceCount; f++) {
                        for (uint32_t i = 0; i < 3; i++) {
                                const uint32_t vertex = originalMesh->vertexAt(m_faceArray[f] * 3 + i);
                                const uint32_t unifiedVertex = originalMesh->firstColocal(vertex);
                                if (unifiedMeshIndices[unifiedVertex] == (uint32_t)~0) {
                                        unifiedMeshIndices[unifiedVertex] = m_unifiedMesh->vertexCount();
                                        XA_DEBUG_ASSERT(equal(originalMesh->position(vertex), originalMesh->position(unifiedVertex), originalMesh->epsilon()));
                                        m_unifiedMesh->addVertex(originalMesh->position(vertex));
                                }
                                if (chartMeshIndices[vertex] == (uint32_t)~0) {
                                        chartMeshIndices[vertex] = m_mesh->vertexCount();
                                        m_chartToOriginalMap.push_back(vertex);
                                        m_chartToUnifiedMap.push_back(unifiedMeshIndices[unifiedVertex]);
                                        m_mesh->addVertex(originalMesh->position(vertex), Vector3(0.0f), originalMesh->texcoord(vertex));
                                }
                        }
                }
                // Add faces.
                for (uint32_t f = 0; f < faceCount; f++) {
                        uint32_t indices[3], unifiedIndices[3];
                        for (uint32_t i = 0; i < 3; i++) {
                                const uint32_t vertex = originalMesh->vertexAt(m_faceArray[f] * 3 + i);
                                indices[i] = chartMeshIndices[vertex];
                                unifiedIndices[i] = unifiedMeshIndices[originalMesh->firstColocal(vertex)];
                        }
                        Mesh::AddFaceResult::Enum result = m_mesh->addFace(indices);
                        XA_UNUSED(result);
                        XA_DEBUG_ASSERT(result == Mesh::AddFaceResult::OK);
#if XA_DEBUG
                        // Unifying colocals may create degenerate edges. e.g. if two triangle vertices are colocal.
                        for (int i = 0; i < 3; i++) {
                                const uint32_t index1 = unifiedIndices[i];
                                const uint32_t index2 = unifiedIndices[(i + 1) % 3];
                                XA_DEBUG_ASSERT(index1 != index2);
                        }
#endif
                        result = m_unifiedMesh->addFace(unifiedIndices);
                        XA_UNUSED(result);
                        XA_DEBUG_ASSERT(result == Mesh::AddFaceResult::OK);
                }
                m_mesh->createBoundaries(); // For AtlasPacker::computeBoundingBox
                m_unifiedMesh->createBoundaries();
                if (meshIsPlanar(*m_unifiedMesh))
                        m_type = ChartType::Planar;
                else {
                        m_unifiedMesh->linkBoundaries();
#if XA_DEBUG_EXPORT_OBJ_BEFORE_FIX_TJUNCTION
                        m_unifiedMesh->writeObjFile("debug_before_fix_tjunction.obj");
#endif
                        bool duplicatedEdge = false, failed = false;
                        XA_PROFILE_START(fixChartMeshTJunctions)
                        Mesh *fixedUnifiedMesh = meshFixTJunctions(*m_unifiedMesh, &duplicatedEdge, &failed, &m_fixedTJunctionsCount);
                        XA_PROFILE_END(fixChartMeshTJunctions)
                        if (fixedUnifiedMesh) {
                                if (duplicatedEdge)
                                        m_warningFlags |= ChartWarningFlags::FixTJunctionsDuplicatedEdge;
                                if (failed)
                                        m_warningFlags |= ChartWarningFlags::FixTJunctionsFailed;
                                m_unmodifiedUnifiedMesh = m_unifiedMesh;
                                m_unifiedMesh = fixedUnifiedMesh;
                                m_unifiedMesh->createBoundaries();
                                m_unifiedMesh->linkBoundaries();
                                m_initialFaceCount = m_unifiedMesh->faceCount(); // Fixing t-junctions rewrites faces.
                        }
                        // See if there are any holes that need closing.
                        Array<uint32_t> &boundaryLoops = buffers.boundaryLoops;
                        meshGetBoundaryLoops(*m_unifiedMesh, boundaryLoops);
                        if (boundaryLoops.size() > 1) {
#if XA_DEBUG_EXPORT_OBJ_CLOSE_HOLES_ERROR
                                const uint32_t faceCountBeforeHolesClosed = m_unifiedMesh->faceCount();
#endif
                                // Closing the holes is not always the best solution and does not fix all the problems.
                                // We need to do some analysis of the holes and the genus to:
                                // - Find cuts that reduce genus.
                                // - Find cuts to connect holes.
                                // - Use minimal spanning trees or seamster.
                                XA_PROFILE_START(closeChartMeshHoles)
                                uint32_t holeCount = 0;
#if XA_DEBUG_EXPORT_OBJ_CLOSE_HOLES_ERROR
                                Array<uint32_t> holeFaceCounts;
                                failed = !meshCloseHoles(m_unifiedMesh, boundaryLoops, m_basis.normal, &holeFaceCounts);
#else
                                failed = !meshCloseHoles(m_unifiedMesh, boundaryLoops, m_basis.normal, &holeCount, nullptr);
#endif
                                XA_PROFILE_END(closeChartMeshHoles)
                                m_unifiedMesh->createBoundaries();
                                m_unifiedMesh->linkBoundaries();
                                meshGetBoundaryLoops(*m_unifiedMesh, boundaryLoops);
                                if (failed || boundaryLoops.size() > 1)
                                        m_warningFlags |= ChartWarningFlags::CloseHolesFailed;
                                m_closedHolesCount = holeCount;
#if XA_DEBUG_EXPORT_OBJ_CLOSE_HOLES_ERROR
                                if (m_warningFlags & ChartWarningFlags::CloseHolesFailed) {
                                        char filename[256];
                                        XA_SPRINTF(filename, sizeof(filename), "debug_mesh_%03u_chartgroup_%03u_chart_%03u_close_holes_error.obj", meshId, chartGroupId, chartId);
                                        FILE *file;
                                        XA_FOPEN(file, filename, "w");
                                        if (file) {
                                                m_unifiedMesh->writeObjVertices(file);
                                                fprintf(file, "s off\n");
                                                fprintf(file, "o object\n");
                                                for (uint32_t i = 0; i < faceCountBeforeHolesClosed; i++)
                                                        m_unifiedMesh->writeObjFace(file, i);
                                                uint32_t face = faceCountBeforeHolesClosed;
                                                for (uint32_t i = 0; i < holeFaceCounts.size(); i++) {
                                                        fprintf(file, "s off\n");
                                                        fprintf(file, "o hole%u\n", i);
                                                        for (uint32_t j = 0; j < holeFaceCounts[i]; j++) {
                                                                m_unifiedMesh->writeObjFace(file, face);
                                                                face++;
                                                        }
                                                }
                                                m_unifiedMesh->writeObjBoundaryEges(file);
                                                m_unifiedMesh->writeObjLinkedBoundaries(file);
                                                fclose(file);
                                        }
                                }
#endif
                        }
                }
        }

#if XA_RECOMPUTE_CHARTS
        Chart(ChartCtorBuffers &buffers, const Chart *parent, const Mesh *parentMesh, ConstArrayView<uint32_t> faces, const Vector2 *texcoords, const Mesh *originalMesh, uint32_t meshId, uint32_t chartGroupId, uint32_t chartId) : m_mesh(nullptr), m_unifiedMesh(nullptr), m_unmodifiedUnifiedMesh(nullptr), m_type(ChartType::Piecewise), m_warningFlags(0), m_closedHolesCount(0), m_fixedTJunctionsCount(0)
        {
                XA_UNUSED(meshId);
                XA_UNUSED(chartGroupId);
                XA_UNUSED(chartId);
                const uint32_t faceCount = m_initialFaceCount = faces.length;
                m_faceArray.resize(faceCount);
                for (uint32_t i = 0; i < faceCount; i++)
                        m_faceArray[i] = parent->m_faceArray[faces[i]]; // Map faces to parent chart original mesh.
                // Copy face indices.
                m_mesh = XA_NEW_ARGS(MemTag::Mesh, Mesh, originalMesh->epsilon(), m_faceArray.size() * 3, m_faceArray.size());
                m_unifiedMesh = XA_NEW_ARGS(MemTag::Mesh, Mesh, originalMesh->epsilon(), m_faceArray.size() * 3, m_faceArray.size());
                Array<uint32_t> &chartMeshIndices = buffers.chartMeshIndices;
                chartMeshIndices.resize(originalMesh->vertexCount());
                chartMeshIndices.setAll(UINT32_MAX);
                Array<uint32_t> &unifiedMeshIndices = buffers.unifiedMeshIndices;
                unifiedMeshIndices.resize(originalMesh->vertexCount());
                unifiedMeshIndices.setAll(UINT32_MAX);
                // Add vertices.
                for (uint32_t f = 0; f < faceCount; f++) {
                        for (uint32_t i = 0; i < 3; i++) {
                                const uint32_t vertex = originalMesh->vertexAt(m_faceArray[f] * 3 + i);
                                const uint32_t unifiedVertex = originalMesh->firstColocal(vertex);
                                const uint32_t parentVertex = parentMesh->vertexAt(faces[f] * 3 + i);
                                if (unifiedMeshIndices[unifiedVertex] == (uint32_t)~0) {
                                        unifiedMeshIndices[unifiedVertex] = m_unifiedMesh->vertexCount();
                                        XA_DEBUG_ASSERT(equal(originalMesh->position(vertex), originalMesh->position(unifiedVertex), originalMesh->epsilon()));
                                        m_unifiedMesh->addVertex(originalMesh->position(vertex), Vector3(0.0f), texcoords[parentVertex]);
                                }
                                if (chartMeshIndices[vertex] == (uint32_t)~0) {
                                        chartMeshIndices[vertex] = m_mesh->vertexCount();
                                        m_chartToOriginalMap.push_back(vertex);
                                        m_chartToUnifiedMap.push_back(unifiedMeshIndices[unifiedVertex]);
                                        m_mesh->addVertex(originalMesh->position(vertex), Vector3(0.0f), texcoords[parentVertex]);
                                }
                        }
                }
                // Add faces.
                for (uint32_t f = 0; f < faceCount; f++) {
                        uint32_t indices[3], unifiedIndices[3];
                        for (uint32_t i = 0; i < 3; i++) {
                                const uint32_t vertex = originalMesh->vertexAt(m_faceArray[f] * 3 + i);
                                indices[i] = chartMeshIndices[vertex];
                                unifiedIndices[i] = unifiedMeshIndices[originalMesh->firstColocal(vertex)];
                        }
                        Mesh::AddFaceResult::Enum result = m_mesh->addFace(indices);
                        XA_UNUSED(result);
                        XA_DEBUG_ASSERT(result == Mesh::AddFaceResult::OK);
#if XA_DEBUG
                        // Unifying colocals may create degenerate edges. e.g. if two triangle vertices are colocal.
                        for (int i = 0; i < 3; i++) {
                                const uint32_t index1 = unifiedIndices[i];
                                const uint32_t index2 = unifiedIndices[(i + 1) % 3];
                                XA_DEBUG_ASSERT(index1 != index2);
                        }
#endif
                        result = m_unifiedMesh->addFace(unifiedIndices);
                        XA_UNUSED(result);
                        XA_DEBUG_ASSERT(result == Mesh::AddFaceResult::OK);
                }
                m_mesh->createBoundaries(); // For AtlasPacker::computeBoundingBox
                m_unifiedMesh->createBoundaries();
                m_unifiedMesh->linkBoundaries();
        }
#endif

        ~Chart()
        {
                if (m_mesh) {
                        m_mesh->~Mesh();
                        XA_FREE(m_mesh);
                }
                if (m_unifiedMesh) {
                        m_unifiedMesh->~Mesh();
                        XA_FREE(m_unifiedMesh);
                }
                if (m_unmodifiedUnifiedMesh) {
                        m_unmodifiedUnifiedMesh->~Mesh();
                        XA_FREE(m_unmodifiedUnifiedMesh);
                }
        }

        const Basis &basis() const { return m_basis; }
        ChartType::Enum type() const { return m_type; }
        uint32_t warningFlags() const { return m_warningFlags; }
        uint32_t closedHolesCount() const { return m_closedHolesCount; }
        uint32_t fixedTJunctionsCount() const { return m_fixedTJunctionsCount; }
        const Quality &quality() const { return m_quality; }
        uint32_t initialFaceCount() const { return m_initialFaceCount; }
#if XA_DEBUG_EXPORT_OBJ_INVALID_PARAMETERIZATION
        const Array<uint32_t> &paramFlippedFaces() const { return m_paramFlippedFaces; }
#endif
        uint32_t mapFaceToSourceFace(uint32_t i) const { return m_faceArray[i]; }
        const Mesh *mesh() const { return m_mesh; }
        Mesh *mesh() { return m_mesh; }
        const Mesh *unifiedMesh() const { return m_unifiedMesh; }
        Mesh *unifiedMesh() { return m_unifiedMesh; }
        const Mesh *unmodifiedUnifiedMesh() const { return m_unmodifiedUnifiedMesh; }
        uint32_t mapChartVertexToOriginalVertex(uint32_t i) const { return m_chartToOriginalMap[i]; }

        void evaluateOrthoQuality(UniformGrid2 &boundaryGrid)
        {
                XA_PROFILE_START(parameterizeChartsEvaluateQuality)
                m_quality.computeBoundaryIntersection(m_unifiedMesh, boundaryGrid);
                m_quality.computeFlippedFaces(m_unifiedMesh, m_initialFaceCount, nullptr);
                m_quality.computeMetrics(m_unifiedMesh, m_initialFaceCount);
                XA_PROFILE_END(parameterizeChartsEvaluateQuality)
                // Use orthogonal parameterization if quality is acceptable.
                if (!m_quality.boundaryIntersection && m_quality.totalGeometricArea > 0.0f && m_quality.stretchMetric <= 1.1f && m_quality.maxStretchMetric <= 1.25f)
                        m_type = ChartType::Ortho;
        }

        void evaluateQuality(UniformGrid2 &boundaryGrid)
        {
                XA_PROFILE_START(parameterizeChartsEvaluateQuality)
                m_quality.computeBoundaryIntersection(m_unifiedMesh, boundaryGrid);
#if XA_DEBUG_EXPORT_OBJ_INVALID_PARAMETERIZATION
                m_quality.computeFlippedFaces(m_unifiedMesh, m_initialFaceCount, &m_paramFlippedFaces);
#else
                m_quality.computeFlippedFaces(m_unifiedMesh, m_initialFaceCount, nullptr);
#endif
                // Don't need to call computeMetrics here, that's only used in evaluateOrthoQuality to determine if quality is acceptable enough to use ortho projection.
                XA_PROFILE_END(parameterizeChartsEvaluateQuality)
        }

        // Transfer parameterization from unified mesh to chart mesh.
        void transferParameterization()
        {
                const uint32_t vertexCount = m_mesh->vertexCount();
                for (uint32_t v = 0; v < vertexCount; v++)
                        m_mesh->texcoord(v) = m_unifiedMesh->texcoord(m_chartToUnifiedMap[v]);
        }

        Vector2 computeParametricBounds() const
        {
                Vector2 minCorner(FLT_MAX, FLT_MAX);
                Vector2 maxCorner(-FLT_MAX, -FLT_MAX);
                const uint32_t vertexCount = m_mesh->vertexCount();
                for (uint32_t v = 0; v < vertexCount; v++) {
                        minCorner = min(minCorner, m_mesh->texcoord(v));
                        maxCorner = max(maxCorner, m_mesh->texcoord(v));
                }
                return (maxCorner - minCorner) * 0.5f;
        }

private:
        Basis m_basis;
        Mesh *m_mesh;
        Mesh *m_unifiedMesh;
        Mesh *m_unmodifiedUnifiedMesh; // Unified mesh before fixing t-junctions. Null if no t-junctions were fixed
        ChartType::Enum m_type;
        uint32_t m_warningFlags;
        uint32_t m_initialFaceCount; // Before fixing T-junctions and/or closing holes.
        uint32_t m_closedHolesCount, m_fixedTJunctionsCount;

        // List of faces of the original mesh that belong to this chart.
        Array<uint32_t> m_faceArray;

        // Map vertices of the chart mesh to vertices of the original mesh.
        Array<uint32_t> m_chartToOriginalMap;

        Array<uint32_t> m_chartToUnifiedMap;

        Quality m_quality;
#if XA_DEBUG_EXPORT_OBJ_INVALID_PARAMETERIZATION
        Array<uint32_t> m_paramFlippedFaces;
#endif
};

struct CreateChartTaskArgs
{
        const Mesh *mesh;
        const Basis *basis;
        ConstArrayView<uint32_t> faces;
        uint32_t meshId;
        uint32_t chartGroupId;
        uint32_t chartId;
        ThreadLocal<ChartCtorBuffers> *chartBuffers;
        Chart **chart;
};

static void runCreateChartTask(void *userData)
{
        XA_PROFILE_START(createChartMeshesThread)
        auto args = (CreateChartTaskArgs *)userData;
        *(args->chart) = XA_NEW_ARGS(MemTag::Default, Chart, args->chartBuffers->get(), *(args->basis), args->faces, args->mesh, args->meshId, args->chartGroupId, args->chartId);
        XA_PROFILE_END(createChartMeshesThread)
}

struct ParameterizeChartTaskArgs
{
        Chart *chart;
        ParameterizeFunc func;
        ThreadLocal<UniformGrid2> *boundaryGrid;
};

static void runParameterizeChartTask(void *userData)
{
        auto args = (ParameterizeChartTaskArgs *)userData;
        Mesh *mesh = args->chart->unifiedMesh();
        XA_PROFILE_START(parameterizeChartsOrthogonal)
        {
                // Project vertices to plane.
                const uint32_t vertexCount = mesh->vertexCount();
                const Basis &basis = args->chart->basis();
                for (uint32_t i = 0; i < vertexCount; i++)
                        mesh->texcoord(i) = Vector2(dot(basis.tangent, mesh->position(i)), dot(basis.bitangent, mesh->position(i)));
        }
        XA_PROFILE_END(parameterizeChartsOrthogonal)
        // Computing charts checks for flipped triangles and boundary intersection. Don't need to do that again here if chart is planar.
        if (args->chart->type() != ChartType::Planar)
                args->chart->evaluateOrthoQuality(args->boundaryGrid->get());
        if (args->chart->type() == ChartType::LSCM) {
                XA_PROFILE_START(parameterizeChartsLSCM)
                if (args->func)
                        args->func(&mesh->position(0).x, &mesh->texcoord(0).x, mesh->vertexCount(), mesh->indices(), mesh->indexCount());
                else
                        computeLeastSquaresConformalMap(mesh);
                XA_PROFILE_END(parameterizeChartsLSCM)
                args->chart->evaluateQuality(args->boundaryGrid->get());
        }
        // Transfer parameterization from unified mesh to chart mesh.
        args->chart->transferParameterization();
}

// Set of charts corresponding to mesh faces in the same face group.
class ChartGroup
{
public:
        ChartGroup(uint32_t id, const Mesh *sourceMesh, uint16_t faceGroup) : m_sourceId(sourceMesh->id()), m_id(id), m_isVertexMap(faceGroup == Mesh::kInvalidFaceGroup), m_paramAddedChartsCount(0), m_paramDeletedChartsCount(0)
        {
                // Create new mesh from the source mesh, using faces that belong to this group.
                const uint32_t sourceFaceCount = sourceMesh->faceCount();
                if (!m_isVertexMap) {
                        m_faceToSourceFaceMap.reserve(sourceMesh->faceGroupFaceCount(faceGroup));
                        for (Mesh::GroupFaceIterator it(sourceMesh, faceGroup); !it.isDone(); it.advance())
                                m_faceToSourceFaceMap.push_back(it.face());
                } else {
                        for (uint32_t f = 0; f < sourceFaceCount; f++) {
                                if (sourceMesh->faceGroupAt(f) == faceGroup)
                                        m_faceToSourceFaceMap.push_back(f);
                        }
                }
                // Only initial meshes have face groups and ignored faces. The only flag we care about is HasNormals.
                const uint32_t faceCount = m_faceToSourceFaceMap.size();
                XA_DEBUG_ASSERT(faceCount > 0);
                const uint32_t approxVertexCount = faceCount * 3;
                m_mesh = XA_NEW_ARGS(MemTag::Mesh, Mesh, sourceMesh->epsilon(), approxVertexCount, faceCount, sourceMesh->flags() & MeshFlags::HasNormals);
                m_vertexToSourceVertexMap.reserve(approxVertexCount);
                HashMap<uint32_t> sourceVertexToVertexMap(MemTag::Mesh, approxVertexCount);
                for (uint32_t f = 0; f < faceCount; f++) {
                        const uint32_t face = m_faceToSourceFaceMap[f];
                        for (uint32_t i = 0; i < 3; i++) {
                                const uint32_t vertex = sourceMesh->vertexAt(face * 3 + i);
                                if (sourceVertexToVertexMap.get(vertex) == UINT32_MAX) {
                                        sourceVertexToVertexMap.add(vertex);
                                        m_vertexToSourceVertexMap.push_back(vertex);
                                        Vector3 normal(0.0f);
                                        if (sourceMesh->flags() & MeshFlags::HasNormals)
                                                normal = sourceMesh->normal(vertex);
                                        m_mesh->addVertex(sourceMesh->position(vertex), normal, sourceMesh->texcoord(vertex));
                                }
                        }
                }
                // Add faces.
                for (uint32_t f = 0; f < faceCount; f++) {
                        const uint32_t face = m_faceToSourceFaceMap[f];
                        uint32_t indices[3];
                        for (uint32_t i = 0; i < 3; i++) {
                                const uint32_t vertex = sourceMesh->vertexAt(face * 3 + i);
                                indices[i] = sourceVertexToVertexMap.get(vertex);
                                XA_DEBUG_ASSERT(indices[i] != UINT32_MAX);
                        }
                        // Don't copy flags, it doesn't matter if a face is ignored after this point. All ignored faces get their own vertex map (m_isVertexMap) ChartGroup.
                        // Don't hash edges if m_isVertexMap, they may be degenerate.
                        Mesh::AddFaceResult::Enum result = m_mesh->addFace(indices, false, !m_isVertexMap);
                        XA_UNUSED(result);
                        XA_DEBUG_ASSERT(result == Mesh::AddFaceResult::OK);
                }
                if (!m_isVertexMap) {
                        m_mesh->createColocals();
                        m_mesh->createBoundaries();
                }
#if XA_DEBUG_EXPORT_OBJ_CHART_GROUPS
                char filename[256];
                XA_SPRINTF(filename, sizeof(filename), "debug_mesh_%03u_chartgroup_%03u.obj", m_sourceId, m_id);
                m_mesh->writeObjFile(filename);
#else
                XA_UNUSED(m_id);
#endif
        }

        ~ChartGroup()
        {
                m_mesh->~Mesh();
                XA_FREE(m_mesh);
                for (uint32_t i = 0; i < m_charts.size(); i++) {
                        m_charts[i]->~Chart();
                        XA_FREE(m_charts[i]);
                }
        }

        uint32_t chartCount() const { return m_charts.size(); }
        Chart *chartAt(uint32_t i) const { return m_charts[i]; }
        uint32_t paramAddedChartsCount() const { return m_paramAddedChartsCount; }
        uint32_t paramDeletedChartsCount() const { return m_paramDeletedChartsCount; }
        bool isVertexMap() const { return m_isVertexMap; }
        uint32_t mapFaceToSourceFace(uint32_t face) const { return m_faceToSourceFaceMap[face]; }
        uint32_t mapVertexToSourceVertex(uint32_t i) const { return m_vertexToSourceVertexMap[i]; }
        const Mesh *mesh() const { return m_mesh; }

        /*
        Compute charts using a simple segmentation algorithm.

        LSCM:
        - identify sharp features using local dihedral angles.
        - identify seed faces farthest from sharp features.
        - grow charts from these seeds.

        MCGIM:
        - phase 1: chart growth
          - grow all charts simultaneously using dijkstra search on the dual graph of the mesh.
          - graph edges are weighted based on planarity metric.
          - metric uses distance to global chart normal.
          - terminate when all faces have been assigned.
        - phase 2: seed computation:
          - place new seed of the chart at the most interior face.
          - most interior is evaluated using distance metric only.

        - method repeates the two phases, until the location of the seeds does not change.
          - cycles are detected by recording all the previous seeds and chartification terminates.

        D-Charts:

        - Uniaxial conic metric:
          - N_c = axis of the generalized cone that best fits the chart. (cone can a be cylinder or a plane).
          - omega_c = angle between the face normals and the axis.
          - Fitting error between chart C and tringle t: F(c,t) = (N_c*n_t - cos(omega_c))^2

        - Compactness metrics:
          - Roundness:
                - C(c,t) = pi * D(S_c,t)^2 / A_c
                - S_c = chart seed.
                - D(S_c,t) = length of the shortest path inside the chart betwen S_c and t.
                - A_c = chart area.
          - Straightness:
                - P(c,t) = l_out(c,t) / l_in(c,t)
                - l_out(c,t) = lenght of the edges not shared between C and t.
                - l_in(c,t) = lenght of the edges shared between C and t.

        - Combined metric:
          - Cost(c,t) = F(c,t)^alpha + C(c,t)^beta + P(c,t)^gamma
          - alpha = 1, beta = 0.7, gamma = 0.5

        Our basic approach:
        - Just one iteration of k-means?
        - Avoid dijkstra by greedily growing charts until a threshold is met. Increase threshold and repeat until no faces left.
        - If distortion metric is too high, split chart, add two seeds.
        - If chart size is low, try removing chart.

        Postprocess:
        - If topology is not disk:
          - Fill holes, if new faces fit proxy.
          - Find best cut, otherwise.
        - After parameterization:
          - If boundary self-intersects:
                - cut chart along the closest two diametral boundary vertices, repeat parametrization.
                - what if the overlap is on an appendix? How do we find that out and cut appropiately?
                  - emphasize roundness metrics to prevent those cases.
          - If interior self-overlaps: preserve boundary parameterization and use mean-value map.
        */
        void computeCharts(TaskScheduler *taskScheduler, const ChartOptions &options, segment::Atlas &atlas, ThreadLocal<ChartCtorBuffers> *chartBuffers)
        {
                m_chartOptions = options;
                // This function may be called multiple times, so destroy existing charts.
                for (uint32_t i = 0; i < m_charts.size(); i++) {
                        m_charts[i]->~Chart();
                        XA_FREE(m_charts[i]);
                }
                m_charts.clear();
#if XA_DEBUG_SINGLE_CHART
                Array<uint32_t> chartFaces;
                chartFaces.resize(m_mesh->faceCount());
                for (uint32_t i = 0; i < chartFaces.size(); i++)
                        chartFaces[i] = i;
                Chart *chart = XA_NEW_ARGS(MemTag::Default, Chart, m_mesh, chartFaces, m_sourceId, m_id, 0);
                m_charts.push_back(chart);
#else
                XA_PROFILE_START(buildAtlas)
                atlas.reset(m_sourceId, m_id, m_mesh, options);
                buildAtlas(atlas, options);
                XA_PROFILE_END(buildAtlas)
                const uint32_t chartCount = atlas.chartCount();
                m_charts.resize(chartCount);
                Array<CreateChartTaskArgs> taskArgs;
                taskArgs.resize(chartCount);
                for (uint32_t i = 0; i < chartCount; i++) {
                        CreateChartTaskArgs &args = taskArgs[i];
                        args.basis = &atlas.chartBasis(i);
                        args.faces = atlas.chartFaces(i);
                        args.mesh = m_mesh;
                        args.meshId = m_sourceId;
                        args.chartGroupId = m_id;
                        args.chartId = i;
                        args.chartBuffers = chartBuffers;
                        args.chart = &m_charts[i];
                }
                XA_PROFILE_START(createChartMeshesReal)
                TaskGroupHandle taskGroup = taskScheduler->createTaskGroup(chartCount);
                for (uint32_t i = 0; i < chartCount; i++) {
                        Task task;
                        task.userData = &taskArgs[i];
                        task.func = runCreateChartTask;
                        taskScheduler->run(taskGroup, task);
                }
                taskScheduler->wait(&taskGroup);
                XA_PROFILE_END(createChartMeshesReal)
#endif
#if XA_DEBUG_EXPORT_OBJ_CHARTS
                char filename[256];
                XA_SPRINTF(filename, sizeof(filename), "debug_mesh_%03u_chartgroup_%03u_charts.obj", m_sourceId, m_id);
                FILE *file;
                XA_FOPEN(file, filename, "w");
                if (file) {
                        m_mesh->writeObjVertices(file);
                        for (uint32_t i = 0; i < chartCount; i++) {
                                fprintf(file, "o chart_%04d\n", i);
                                fprintf(file, "s off\n");
                                const Array<uint32_t> &faces = builder.chartFaces(i);
                                for (uint32_t f = 0; f < faces.size(); f++)
                                        m_mesh->writeObjFace(file, faces[f]);
                        }
                        m_mesh->writeObjBoundaryEges(file);
                        m_mesh->writeObjLinkedBoundaries(file);
                        fclose(file);
                }
#endif
        }

#if XA_RECOMPUTE_CHARTS
        void parameterizeCharts(TaskScheduler *taskScheduler, ParameterizeFunc func, ThreadLocal<UniformGrid2> *boundaryGrid, ThreadLocal<ChartCtorBuffers> *chartBuffers, ThreadLocal<PiecewiseParam> *piecewiseParam)
#else
        void parameterizeCharts(TaskScheduler* taskScheduler, ParameterizeFunc func, ThreadLocal<UniformGrid2>* boundaryGrid, ThreadLocal<ChartCtorBuffers>* /*chartBuffers*/)
#endif
        {
                m_paramAddedChartsCount = 0;
                const uint32_t chartCount = m_charts.size();
                Array<ParameterizeChartTaskArgs> taskArgs;
                taskArgs.resize(chartCount);
                TaskGroupHandle taskGroup = taskScheduler->createTaskGroup(chartCount);
                for (uint32_t i = 0; i < chartCount; i++) {
                        ParameterizeChartTaskArgs &args = taskArgs[i];
                        args.chart = m_charts[i];
                        args.func = func;
                        args.boundaryGrid = boundaryGrid;
                        Task task;
                        task.userData = &args;
                        task.func = runParameterizeChartTask;
                        taskScheduler->run(taskGroup, task);
                }
                taskScheduler->wait(&taskGroup);
#if XA_RECOMPUTE_CHARTS
                // Find charts with invalid parameterizations.
                Array<Chart *> invalidCharts;
                for (uint32_t i = 0; i < chartCount; i++) {
                        Chart *chart = m_charts[i];
                        const Quality &quality = chart->quality();
                        if (quality.boundaryIntersection || quality.flippedTriangleCount > 0)
                                invalidCharts.push_back(chart);
                }
                if (invalidCharts.isEmpty())
                        return;
                // Recompute charts with invalid parameterizations.
                PiecewiseParam &pp = piecewiseParam->get();
                for (uint32_t i = 0; i < invalidCharts.size(); i++) {
                        Chart *invalidChart = invalidCharts[i];
                        // Fixing t-junctions rewrites unified mesh faces, and we need to map faces back to input mesh. So use the unmodified unified mesh.
                        const Mesh *invalidMesh = invalidChart->unmodifiedUnifiedMesh();
                        uint32_t faceCount = 0;
                        if (invalidMesh) {
                                faceCount = invalidMesh->faceCount();
                        } else {
                                invalidMesh = invalidChart->unifiedMesh();
                                faceCount = invalidChart->initialFaceCount(); // Not invalidMesh->faceCount(). Don't want faces added by hole closing.
                        }
                        pp.reset(invalidMesh, faceCount);
#if XA_DEBUG_EXPORT_OBJ_RECOMPUTED_CHARTS
                        char filename[256];
                        XA_SPRINTF(filename, sizeof(filename), "debug_mesh_%03u_chartgroup_%03u_recomputed_chart_%03u.obj", m_sourceId, m_id, m_paramAddedChartsCount);
                        FILE *file;
                        XA_FOPEN(file, filename, "w");
                        uint32_t subChartIndex = 0;
#endif
                        for (;;) {
                                if (!pp.computeChart())
                                        break;
                                Chart *chart = XA_NEW_ARGS(MemTag::Default, Chart, chartBuffers->get(), invalidChart, invalidMesh, pp.chartFaces(), pp.texcoords(), m_mesh, m_sourceId, m_id, m_charts.size());
                                m_charts.push_back(chart);
#if XA_DEBUG_EXPORT_OBJ_RECOMPUTED_CHARTS
                                if (file) {
                                        for (uint32_t j = 0; j < invalidMesh->vertexCount(); j++) {
                                                fprintf(file, "v %g %g %g\n", invalidMesh->position(j).x, invalidMesh->position(j).y, invalidMesh->position(j).z);
                                                fprintf(file, "vt %g %g\n", pp.texcoords()[j].x, pp.texcoords()[j].y);
                                        }
                                        fprintf(file, "o chart%03u\n", subChartIndex);
                                        fprintf(file, "s off\n");
                                        for (uint32_t f = 0; f < pp.chartFaces().length; f++) {
                                                fprintf(file, "f ");
                                                const uint32_t face = pp.chartFaces()[f];
                                                for (uint32_t j = 0; j < 3; j++) {
                                                        const uint32_t index = invalidMesh->vertexCount() * subChartIndex + invalidMesh->vertexAt(face * 3 + j) + 1; // 1-indexed
                                                        fprintf(file, "%d/%d/%c", index, index, j == 2 ? '\n' : ' ');
                                                }
                                        }
                                }
                                subChartIndex++;
#endif
                                m_paramAddedChartsCount++;
                        }
#if XA_DEBUG_EXPORT_OBJ_RECOMPUTED_CHARTS
                        if (file)
                                fclose(file);
#endif
                }
                // Remove and delete the invalid charts.
                for (uint32_t i = 0; i < invalidCharts.size(); i++) {
                        Chart *chart = invalidCharts[i];
                        removeChart(chart);
                        chart->~Chart();
                        XA_FREE(chart);
                        m_paramDeletedChartsCount++;
                }
#endif // XA_RECOMPUTE_CHARTS
        }

private:
        void buildAtlas(segment::Atlas &atlas, const ChartOptions &options)
        {
                if (atlas.facesLeft() == 0)
                        return;
                // Create initial charts greedely.
                atlas.placeSeeds(options.maxThreshold * 0.5f);
                if (options.maxIterations == 0) {
                        XA_DEBUG_ASSERT(atlas.facesLeft() == 0);
                        return;
                }
                atlas.relocateSeeds();
                atlas.resetCharts();
                // Restart process growing charts in parallel.
                uint32_t iteration = 0;
                for (;;) {
                        atlas.growCharts(options.maxThreshold);
                        // When charts cannot grow more: fill holes, merge charts, relocate seeds and start new iteration.
                        atlas.fillHoles(options.maxThreshold * 0.5f);
#if XA_MERGE_CHARTS
                        atlas.mergeCharts();
#endif
                        if (++iteration == options.maxIterations)
                                break;
                        if (!atlas.relocateSeeds())
                                break;
                        atlas.resetCharts();
                }
                // Make sure no holes are left!
                XA_DEBUG_ASSERT(atlas.facesLeft() == 0);
        }

        void removeChart(const Chart *chart)
        {
                for (uint32_t i = 0; i < m_charts.size(); i++) {
                        if (m_charts[i] == chart) {
                                m_charts.removeAt(i);
                                return;
                        }
                }
        }

        uint32_t m_sourceId, m_id;
        bool m_isVertexMap;
        Mesh *m_mesh;
        Array<uint32_t> m_faceToSourceFaceMap; // List of faces of the source mesh that belong to this chart group.
        Array<uint32_t> m_vertexToSourceVertexMap; // Map vertices of the mesh to vertices of the source mesh.
        Array<Chart *> m_charts;
        ChartOptions m_chartOptions;
        uint32_t m_paramAddedChartsCount; // Number of new charts added by recomputing charts with invalid parameterizations.
        uint32_t m_paramDeletedChartsCount; // Number of charts with invalid parameterizations that were deleted, after charts were recomputed.
};

struct CreateChartGroupTaskArgs
{
        uint16_t faceGroup;
        uint32_t groupId;
        const Mesh *mesh;
        ChartGroup **chartGroup;
};

static void runCreateChartGroupTask(void *userData)
{
        XA_PROFILE_START(addMeshCreateChartGroupsThread)
        auto args = (CreateChartGroupTaskArgs *)userData;
        *(args->chartGroup) = XA_NEW_ARGS(MemTag::Default, ChartGroup, args->groupId, args->mesh, args->faceGroup);
        XA_PROFILE_END(addMeshCreateChartGroupsThread)
}

struct ComputeChartsTaskArgs
{
        TaskScheduler *taskScheduler;
        ChartGroup *chartGroup;
        ThreadLocal<segment::Atlas> *atlas;
        ThreadLocal<ChartCtorBuffers> *chartBuffers;
        const ChartOptions *options;
        Progress *progress;
};

static void runComputeChartsJob(void *userData)
{
        auto args = (ComputeChartsTaskArgs *)userData;
        if (args->progress->cancel)
                return;
        XA_PROFILE_START(computeChartsThread)
        args->chartGroup->computeCharts(args->taskScheduler, *args->options, args->atlas->get(), args->chartBuffers);
        XA_PROFILE_END(computeChartsThread)
        args->progress->value++;
        args->progress->update();
}

struct ParameterizeChartsTaskArgs
{
        TaskScheduler *taskScheduler;
        ChartGroup *chartGroup;
        ParameterizeFunc func;
        ThreadLocal<UniformGrid2> *boundaryGrid;
        ThreadLocal<ChartCtorBuffers> *chartBuffers;
#if XA_RECOMPUTE_CHARTS
        ThreadLocal<PiecewiseParam> *piecewiseParam;
#endif
        Progress *progress;
};

static void runParameterizeChartsJob(void *userData)
{
        auto args = (ParameterizeChartsTaskArgs *)userData;
        if (args->progress->cancel)
                return;
        XA_PROFILE_START(parameterizeChartsThread)
#if XA_RECOMPUTE_CHARTS
        args->chartGroup->parameterizeCharts(args->taskScheduler, args->func, args->boundaryGrid, args->chartBuffers, args->piecewiseParam);
#else
        args->chartGroup->parameterizeCharts(args->taskScheduler, args->func, args->boundaryGrid, args->chartBuffers);
#endif
        XA_PROFILE_END(parameterizeChartsThread)
        args->progress->value++;
        args->progress->update();
}

/// An atlas is a set of chart groups.
class Atlas
{
public:
        Atlas() : m_meshCount(0), m_chartsComputed(false), m_chartsParameterized(false) {}

        ~Atlas()
        {
                for (uint32_t i = 0; i < m_chartGroups.size(); i++) {
                        m_chartGroups[i]->~ChartGroup();
                        XA_FREE(m_chartGroups[i]);
                }
        }

        bool chartsComputed() const { return m_chartsComputed; }
        bool chartsParameterized() const { return m_chartsParameterized; }
        uint32_t chartGroupCount() const { return m_chartGroups.size(); }
        const ChartGroup *chartGroupAt(uint32_t index) const { return m_chartGroups[index]; }

        uint32_t chartGroupCount(uint32_t mesh) const
        {
                uint32_t count = 0;
                for (uint32_t i = 0; i < m_chartGroups.size(); i++) {
                        if (m_chartGroupSourceMeshes[i] == mesh)
                                count++;
                }
                return count;
        }

        const ChartGroup *chartGroupAt(uint32_t mesh, uint32_t group) const
        {
                for (uint32_t c = 0; c < m_chartGroups.size(); c++) {
                        if (m_chartGroupSourceMeshes[c] != mesh)
                                continue;
                        if (group == 0)
                                return m_chartGroups[c];
                        group--;
                }
                return nullptr;
        }

        // This function is thread safe.
        void addMesh(TaskScheduler *taskScheduler, const Mesh *mesh)
        {
                // Create one chart group per face group.
                // If there's any ignored faces in the mesh, create an extra face group for that (vertex map).
                // Chart group creation is slow since it copies a chunk of the source mesh, so use tasks.
                Array<ChartGroup *> chartGroups;
                chartGroups.resize(mesh->faceGroupCount() + (mesh->ignoredFaceCount() > 0 ? 1 : 0));
                Array<CreateChartGroupTaskArgs> taskArgs;
                taskArgs.resize(chartGroups.size());
                for (uint32_t g = 0; g < chartGroups.size(); g++) {
                        CreateChartGroupTaskArgs &args = taskArgs[g];
                        args.chartGroup = &chartGroups[g];
                        args.faceGroup = uint16_t(g < mesh->faceGroupCount() ? g : Mesh::kInvalidFaceGroup);
                        args.groupId = g;
                        args.mesh = mesh;
                }
                TaskGroupHandle taskGroup = taskScheduler->createTaskGroup(chartGroups.size());
                for (uint32_t g = 0; g < chartGroups.size(); g++) {
                        Task task;
                        task.userData = &taskArgs[g];
                        task.func = runCreateChartGroupTask;
                        taskScheduler->run(taskGroup, task);
                }
                taskScheduler->wait(&taskGroup);
                // Thread-safe append.
                m_addMeshMutex.lock();
                for (uint32_t g = 0; g < chartGroups.size(); g++) {
                        m_chartGroups.push_back(chartGroups[g]);
                        m_chartGroupSourceMeshes.push_back(mesh->id());
                }
                m_meshCount++;
                m_addMeshMutex.unlock();
        }

        // Chart id/index is determined by depth-first hierarchy of mesh -> chart group -> chart.
        // For chart index to be consistent here, chart groups needs to sorted by mesh index. Since addMesh is called by multithreaded tasks, order is indeterminate, so chart groups need to be explicitly sorted after all meshes are added.
        void sortChartGroups()
        {
                Array<ChartGroup *> oldChartGroups;
                oldChartGroups.resize(m_chartGroups.size());
                memcpy(oldChartGroups.data(), m_chartGroups.data(), sizeof(ChartGroup *) * m_chartGroups.size());
                Array<uint32_t> oldChartGroupSourceMeshes;
                oldChartGroupSourceMeshes.resize(m_chartGroupSourceMeshes.size());
                memcpy(oldChartGroupSourceMeshes.data(), m_chartGroupSourceMeshes.data(), sizeof(uint32_t) * m_chartGroupSourceMeshes.size());
                uint32_t current = 0;
                for (uint32_t i = 0; i < m_meshCount; i++) {
                        for (uint32_t j = 0; j < oldChartGroups.size(); j++) {
                                if (oldChartGroupSourceMeshes[j] == i) {
                                        m_chartGroups[current] = oldChartGroups[j];
                                        m_chartGroupSourceMeshes[current] = oldChartGroupSourceMeshes[j];
                                        current++;
                                }
                        }
                }
        }

        bool computeCharts(TaskScheduler *taskScheduler, const ChartOptions &options, ProgressFunc progressFunc, void *progressUserData)
        {
                m_chartsComputed = false;
                m_chartsParameterized = false;
                // Ignore vertex maps.
                uint32_t chartGroupCount = 0;
                for (uint32_t i = 0; i < m_chartGroups.size(); i++) {
                        if (!m_chartGroups[i]->isVertexMap())
                                chartGroupCount++;
                }
                Progress progress(ProgressCategory::ComputeCharts, progressFunc, progressUserData, chartGroupCount);
                ThreadLocal<segment::Atlas> atlas;
                ThreadLocal<ChartCtorBuffers> chartBuffers;
                Array<ComputeChartsTaskArgs> taskArgs;
                taskArgs.reserve(chartGroupCount);
                for (uint32_t i = 0; i < m_chartGroups.size(); i++) {
                        if (!m_chartGroups[i]->isVertexMap()) {
                                ComputeChartsTaskArgs args;
                                args.taskScheduler = taskScheduler;
                                args.chartGroup = m_chartGroups[i];
                                args.atlas = &atlas;
                                args.chartBuffers = &chartBuffers;
                                args.options = &options;
                                args.progress = &progress;
                                taskArgs.push_back(args);
                        }
                }
                // Sort chart groups by mesh indexCount.
                m_chartGroupsRadix = RadixSort();
                Array<float> chartGroupSortData;
                chartGroupSortData.resize(chartGroupCount);
                for (uint32_t i = 0; i < chartGroupCount; i++)
                        chartGroupSortData[i] = (float)taskArgs[i].chartGroup->mesh()->indexCount();
                m_chartGroupsRadix.sort(chartGroupSortData);
                // Larger chart group meshes are added first to reduce the chance of thread starvation.
                TaskGroupHandle taskGroup = taskScheduler->createTaskGroup(chartGroupCount);
                for (uint32_t i = 0; i < chartGroupCount; i++) {
                        Task task;
                        task.userData = &taskArgs[m_chartGroupsRadix.ranks()[chartGroupCount - i - 1]];
                        task.func = runComputeChartsJob;
                        taskScheduler->run(taskGroup, task);
                }
                taskScheduler->wait(&taskGroup);
                if (progress.cancel)
                        return false;
                m_chartsComputed = true;
                return true;
        }

        bool parameterizeCharts(TaskScheduler *taskScheduler, ParameterizeFunc func, ProgressFunc progressFunc, void *progressUserData)
        {
                m_chartsParameterized = false;
                // Ignore vertex maps.
                uint32_t chartGroupCount = 0;
                for (uint32_t i = 0; i < m_chartGroups.size(); i++) {
                        if (!m_chartGroups[i]->isVertexMap())
                                chartGroupCount++;
                }
                Progress progress(ProgressCategory::ParameterizeCharts, progressFunc, progressUserData, chartGroupCount);
                ThreadLocal<UniformGrid2> boundaryGrid; // For Quality boundary intersection.
                ThreadLocal<ChartCtorBuffers> chartBuffers;
#if XA_RECOMPUTE_CHARTS
                ThreadLocal<PiecewiseParam> piecewiseParam;
#endif
                Array<ParameterizeChartsTaskArgs> taskArgs;
                taskArgs.reserve(chartGroupCount);
                for (uint32_t i = 0; i < m_chartGroups.size(); i++) {
                        if (!m_chartGroups[i]->isVertexMap()) {
                                ParameterizeChartsTaskArgs args;
                                args.taskScheduler = taskScheduler;
                                args.chartGroup = m_chartGroups[i];
                                args.func = func;
                                args.boundaryGrid = &boundaryGrid;
                                args.chartBuffers = &chartBuffers;
#if XA_RECOMPUTE_CHARTS
                                args.piecewiseParam = &piecewiseParam;
#endif
                                args.progress = &progress;
                                taskArgs.push_back(args);
                        }
                }
                // Larger chart group meshes are added first to reduce the chance of thread starvation.
                TaskGroupHandle taskGroup = taskScheduler->createTaskGroup(chartGroupCount);
                for (uint32_t i = 0; i < chartGroupCount; i++) {
                        Task task;
                        task.userData = &taskArgs[m_chartGroupsRadix.ranks()[chartGroupCount - i - 1]];
                        task.func = runParameterizeChartsJob;
                        taskScheduler->run(taskGroup, task);
                }
                taskScheduler->wait(&taskGroup);
                if (progress.cancel)
                        return false;
                m_chartsParameterized = true;
                return true;
        }

private:
        std::mutex m_addMeshMutex;
        uint32_t m_meshCount;
        bool m_chartsComputed;
        bool m_chartsParameterized;
        Array<ChartGroup *> m_chartGroups;
        RadixSort m_chartGroupsRadix; // By mesh indexCount.
        Array<uint32_t> m_chartGroupSourceMeshes;
};

} // namespace param

namespace pack {

class AtlasImage
{
public:
        AtlasImage(uint32_t width, uint32_t height) : m_width(width), m_height(height)
        {
                m_data.resize(m_width * m_height);
                memset(m_data.data(), 0, sizeof(uint32_t) * m_data.size());
        }

        void resize(uint32_t width, uint32_t height)
        {
                Array<uint32_t> data;
                data.resize(width * height);
                memset(data.data(), 0, sizeof(uint32_t) * data.size());
                for (uint32_t y = 0; y < min(m_height, height); y++)
                        memcpy(&data[y * width], &m_data[y * m_width], min(m_width, width) * sizeof(uint32_t));
                m_width = width;
                m_height = height;
                data.moveTo(m_data);
        }

        void addChart(uint32_t chartIndex, const BitImage *image, const BitImage *imageBilinear, const BitImage *imagePadding, int atlas_w, int atlas_h, int offset_x, int offset_y)
        {
                const int w = image->width();
                const int h = image->height();
                for (int y = 0; y < h; y++) {
                        const int yy = y + offset_y;
                        if (yy < 0)
                                continue;
                        for (int x = 0; x < w; x++) {
                                const int xx = x + offset_x;
                                if (xx >= 0 && xx < atlas_w && yy < atlas_h) {
                                        const uint32_t dataOffset = xx + yy * m_width;
                                        if (image->get(x, y)) {
                                                XA_DEBUG_ASSERT(m_data[dataOffset] == 0);
                                                m_data[dataOffset] = chartIndex | kImageHasChartIndexBit;
                                        } else if (imageBilinear && imageBilinear->get(x, y)) {
                                                XA_DEBUG_ASSERT(m_data[dataOffset] == 0);
                                                m_data[dataOffset] = chartIndex | kImageHasChartIndexBit | kImageIsBilinearBit;
                                        } else if (imagePadding && imagePadding->get(x, y)) {
                                                XA_DEBUG_ASSERT(m_data[dataOffset] == 0);
                                                m_data[dataOffset] = chartIndex | kImageHasChartIndexBit | kImageIsPaddingBit;
                                        }
                                }
                        }
                }
        }

        void copyTo(uint32_t *dest, uint32_t destWidth, uint32_t destHeight, int padding) const
        {
                for (uint32_t y = 0; y < destHeight; y++)
                        memcpy(&dest[y * destWidth], &m_data[padding + (y + padding) * m_width], destWidth * sizeof(uint32_t));
        }

#if XA_DEBUG_EXPORT_ATLAS_IMAGES
        void writeTga(const char *filename, uint32_t width, uint32_t height) const
        {
                Array<uint8_t> image;
                image.resize(width * height * 3);
                for (uint32_t y = 0; y < height; y++) {
                        if (y >= m_height)
                                continue;
                        for (uint32_t x = 0; x < width; x++) {
                                if (x >= m_width)
                                        continue;
                                const uint32_t data = m_data[x + y * m_width];
                                uint8_t *bgr = &image[(x + y * width) * 3];
                                if (data == 0) {
                                        bgr[0] = bgr[1] = bgr[2] = 0;
                                        continue;
                                }
                                const uint32_t chartIndex = data & kImageChartIndexMask;
                                if (data & kImageIsPaddingBit) {
                                        bgr[0] = 0;
                                        bgr[1] = 0;
                                        bgr[2] = 255;
                                } else if (data & kImageIsBilinearBit) {
                                        bgr[0] = 0;
                                        bgr[1] = 255;
                                        bgr[2] = 0;
                                } else {
                                        const int mix = 192;
                                        srand((unsigned int)chartIndex);
                                        bgr[0] = uint8_t((rand() % 255 + mix) * 0.5f);
                                        bgr[1] = uint8_t((rand() % 255 + mix) * 0.5f);
                                        bgr[2] = uint8_t((rand() % 255 + mix) * 0.5f);
                                }
                        }
                }
                WriteTga(filename, image.data(), width, height);
        }
#endif

private:
        uint32_t m_width, m_height;
        Array<uint32_t> m_data;
};

struct Chart
{
        int32_t atlasIndex;
        uint32_t material;
        uint32_t indexCount;
        const uint32_t *indices;
        float parametricArea;
        float surfaceArea;
        Vector2 *vertices;
        uint32_t vertexCount;
        Array<uint32_t> uniqueVertices;
        bool allowRotate;
        // bounding box
        Vector2 majorAxis, minorAxis, minCorner, maxCorner;
        // Mesh only
        const Array<uint32_t> *boundaryEdges;
        // UvMeshChart only
        Array<uint32_t> faces;

        Vector2 &uniqueVertexAt(uint32_t v) { return uniqueVertices.isEmpty() ? vertices[v] : vertices[uniqueVertices[v]]; }
        uint32_t uniqueVertexCount() const { return uniqueVertices.isEmpty() ? vertexCount : uniqueVertices.size(); }
};

struct AddChartTaskArgs
{
        ThreadLocal<BoundingBox2D> *boundingBox;
        param::Chart *paramChart;
        Chart *chart; // out
};

static void runAddChartTask(void *userData)
{
        XA_PROFILE_START(packChartsAddChartsThread)
        auto args = (AddChartTaskArgs *)userData;
        param::Chart *paramChart = args->paramChart;
        XA_PROFILE_START(packChartsAddChartsRestoreTexcoords)
        paramChart->transferParameterization();
        XA_PROFILE_END(packChartsAddChartsRestoreTexcoords)
        Mesh *mesh = paramChart->mesh();
        Chart *chart = args->chart = XA_NEW(MemTag::Default, Chart);
        chart->atlasIndex = -1;
        chart->material = 0;
        chart->indexCount = mesh->indexCount();
        chart->indices = mesh->indices();
        chart->parametricArea = mesh->computeParametricArea();
        if (chart->parametricArea < kAreaEpsilon) {
                // When the parametric area is too small we use a rough approximation to prevent divisions by very small numbers.
                const Vector2 bounds = paramChart->computeParametricBounds();
                chart->parametricArea = bounds.x * bounds.y;
        }
        chart->surfaceArea = mesh->computeSurfaceArea();
        chart->vertices = mesh->texcoords();
        chart->vertexCount = mesh->vertexCount();
        chart->allowRotate = true;
        chart->boundaryEdges = &mesh->boundaryEdges();
        // Compute bounding box of chart.
        BoundingBox2D &bb = args->boundingBox->get();
        bb.clear();
        for (uint32_t v = 0; v < chart->vertexCount; v++) {
                if (mesh->isBoundaryVertex(v))
                        bb.appendBoundaryVertex(mesh->texcoord(v));
        }
        bb.compute(mesh->texcoords(), mesh->vertexCount());
        chart->majorAxis = bb.majorAxis;
        chart->minorAxis = bb.minorAxis;
        chart->minCorner = bb.minCorner;
        chart->maxCorner = bb.maxCorner;
        XA_PROFILE_END(packChartsAddChartsThread)
}

struct Atlas
{
        ~Atlas()
        {
                for (uint32_t i = 0; i < m_atlasImages.size(); i++) {
                        m_atlasImages[i]->~AtlasImage();
                        XA_FREE(m_atlasImages[i]);
                }
                for (uint32_t i = 0; i < m_bitImages.size(); i++) {
                        m_bitImages[i]->~BitImage();
                        XA_FREE(m_bitImages[i]);
                }
                for (uint32_t i = 0; i < m_charts.size(); i++) {
                        m_charts[i]->~Chart();
                        XA_FREE(m_charts[i]);
                }
        }

        uint32_t getWidth() const { return m_width; }
        uint32_t getHeight() const { return m_height; }
        uint32_t getNumAtlases() const { return m_bitImages.size(); }
        float getTexelsPerUnit() const { return m_texelsPerUnit; }
        const Chart *getChart(uint32_t index) const { return m_charts[index]; }
        uint32_t getChartCount() const { return m_charts.size(); }
        const Array<AtlasImage *> &getImages() const { return m_atlasImages; }
        float getUtilization(uint32_t atlas) const { return m_utilization[atlas]; }

        void addCharts(TaskScheduler *taskScheduler, param::Atlas *paramAtlas)
        {
                // Count charts.
                uint32_t chartCount = 0;
                const uint32_t chartGroupsCount = paramAtlas->chartGroupCount();
                for (uint32_t i = 0; i < chartGroupsCount; i++) {
                        const param::ChartGroup *chartGroup = paramAtlas->chartGroupAt(i);
                        if (chartGroup->isVertexMap())
                                continue;
                        chartCount += chartGroup->chartCount();
                }
                if (chartCount == 0)
                        return;
                // Run one task per chart.
                Array<AddChartTaskArgs> taskArgs;
                taskArgs.resize(chartCount);
                TaskGroupHandle taskGroup = taskScheduler->createTaskGroup(chartCount);
                uint32_t chartIndex = 0;
                ThreadLocal<BoundingBox2D> boundingBox;
                for (uint32_t i = 0; i < chartGroupsCount; i++) {
                        const param::ChartGroup *chartGroup = paramAtlas->chartGroupAt(i);
                        if (chartGroup->isVertexMap())
                                continue;
                        const uint32_t count = chartGroup->chartCount();
                        for (uint32_t j = 0; j < count; j++) {
                                AddChartTaskArgs &args = taskArgs[chartIndex];
                                args.boundingBox = &boundingBox;
                                args.paramChart = chartGroup->chartAt(j);
                                Task task;
                                task.userData = &taskArgs[chartIndex];
                                task.func = runAddChartTask;
                                taskScheduler->run(taskGroup, task);
                                chartIndex++;
                        }
                }
                taskScheduler->wait(&taskGroup);
                // Get task output.
                m_charts.resize(chartCount);
                for (uint32_t i = 0; i < chartCount; i++)
                        m_charts[i] = taskArgs[i].chart;
        }

        void addUvMeshCharts(UvMeshInstance *mesh)
        {
                BitArray vertexUsed(mesh->texcoords.size());
                BoundingBox2D boundingBox;
                for (uint32_t c = 0; c < mesh->mesh->charts.size(); c++) {
                        UvMeshChart *uvChart = mesh->mesh->charts[c];
                        Chart *chart = XA_NEW(MemTag::Default, Chart);
                        chart->atlasIndex = -1;
                        chart->material = uvChart->material;
                        chart->indexCount = uvChart->indices.size();
                        chart->indices = uvChart->indices.data();
                        chart->vertices = mesh->texcoords.data();
                        chart->vertexCount = mesh->texcoords.size();
                        chart->allowRotate = mesh->rotateCharts;
                        chart->boundaryEdges = nullptr;
                        chart->faces.resize(uvChart->faces.size());
                        memcpy(chart->faces.data(), uvChart->faces.data(), sizeof(uint32_t) * uvChart->faces.size());
                        // Find unique vertices.
                        vertexUsed.zeroOutMemory();
                        for (uint32_t i = 0; i < chart->indexCount; i++) {
                                const uint32_t vertex = chart->indices[i];
                                if (!vertexUsed.get(vertex)) {
                                        vertexUsed.set(vertex);
                                        chart->uniqueVertices.push_back(vertex);
                                }
                        }
                        // Compute parametric and surface areas.
                        chart->parametricArea = 0.0f;
                        for (uint32_t f = 0; f < chart->indexCount / 3; f++) {
                                const Vector2 &v1 = chart->vertices[chart->indices[f * 3 + 0]];
                                const Vector2 &v2 = chart->vertices[chart->indices[f * 3 + 1]];
                                const Vector2 &v3 = chart->vertices[chart->indices[f * 3 + 2]];
                                chart->parametricArea += fabsf(triangleArea(v1, v2, v3));
                        }
                        chart->parametricArea *= 0.5f;
                        chart->surfaceArea = chart->parametricArea; // Identical for UV meshes.
                        if (chart->parametricArea < kAreaEpsilon) {
                                // When the parametric area is too small we use a rough approximation to prevent divisions by very small numbers.
                                Vector2 minCorner(FLT_MAX, FLT_MAX);
                                Vector2 maxCorner(-FLT_MAX, -FLT_MAX);
                                for (uint32_t v = 0; v < chart->uniqueVertexCount(); v++) {
                                        minCorner = min(minCorner, chart->uniqueVertexAt(v));
                                        maxCorner = max(maxCorner, chart->uniqueVertexAt(v));
                                }
                                const Vector2 bounds = (maxCorner - minCorner) * 0.5f;
                                chart->parametricArea = bounds.x * bounds.y;
                        }
                        // Compute bounding box of chart.
                        // Using all unique vertices for simplicity, can compute real boundaries if this is too slow.
                        boundingBox.clear();
                        for (uint32_t v = 0; v < chart->uniqueVertexCount(); v++)
                                boundingBox.appendBoundaryVertex(chart->uniqueVertexAt(v));
                        boundingBox.compute();
                        chart->majorAxis = boundingBox.majorAxis;
                        chart->minorAxis = boundingBox.minorAxis;
                        chart->minCorner = boundingBox.minCorner;
                        chart->maxCorner = boundingBox.maxCorner;
                        m_charts.push_back(chart);
                }
        }

        // Pack charts in the smallest possible rectangle.
        bool packCharts(const PackOptions &options, ProgressFunc progressFunc, void *progressUserData)
        {
                if (progressFunc) {
                        if (!progressFunc(ProgressCategory::PackCharts, 0, progressUserData))
                                return false;
                }
                const uint32_t chartCount = m_charts.size();
                XA_PRINT("Packing %u charts\n", chartCount);
                if (chartCount == 0) {
                        if (progressFunc) {
                                if (!progressFunc(ProgressCategory::PackCharts, 100, progressUserData))
                                        return false;
                        }
                        return true;
                }
                // Estimate resolution and/or texels per unit if not specified.
                m_texelsPerUnit = options.texelsPerUnit;
                uint32_t resolution = options.resolution > 0 ? options.resolution + options.padding * 2 : 0;
                const uint32_t maxResolution = m_texelsPerUnit > 0.0f ? resolution : 0;
                if (resolution <= 0 || m_texelsPerUnit <= 0) {
                        if (resolution <= 0 && m_texelsPerUnit <= 0)
                                resolution = 1024;
                        float meshArea = 0;
                        for (uint32_t c = 0; c < chartCount; c++)
                                meshArea += m_charts[c]->surfaceArea;
                        if (resolution <= 0) {
                                // Estimate resolution based on the mesh surface area and given texel scale.
                                const float texelCount = max(1.0f, meshArea * square(m_texelsPerUnit) / 0.75f); // Assume 75% utilization.
                                resolution = max(1u, nextPowerOfTwo(uint32_t(sqrtf(texelCount))));
                        }
                        if (m_texelsPerUnit <= 0) {
                                // Estimate a suitable texelsPerUnit to fit the given resolution.
                                const float texelCount = max(1.0f, meshArea / 0.75f); // Assume 75% utilization.
                                m_texelsPerUnit = sqrtf((resolution * resolution) / texelCount);
                                XA_PRINT("   Estimating texelsPerUnit as %g\n", m_texelsPerUnit);
                        }
                }
                Array<float> chartOrderArray;
                chartOrderArray.resize(chartCount);
                Array<Vector2> chartExtents;
                chartExtents.resize(chartCount);
                float minChartPerimeter = FLT_MAX, maxChartPerimeter = 0.0f;
                for (uint32_t c = 0; c < chartCount; c++) {
                        Chart *chart = m_charts[c];
                        // Compute chart scale
                        float scale = 1.0f;
                        if (chart->parametricArea != 0.0f) {
                                scale = (chart->surfaceArea / chart->parametricArea) * m_texelsPerUnit;
                                XA_ASSERT(isFinite(scale));
                        }
                        // Translate, rotate and scale vertices. Compute extents.
                        Vector2 minCorner(FLT_MAX, FLT_MAX);
                        if (!chart->allowRotate) {
                                for (uint32_t i = 0; i < chart->uniqueVertexCount(); i++)
                                        minCorner = min(minCorner, chart->uniqueVertexAt(i));
                        }
                        Vector2 extents(0.0f);
                        for (uint32_t i = 0; i < chart->uniqueVertexCount(); i++) {
                                Vector2 &texcoord = chart->uniqueVertexAt(i);
                                if (chart->allowRotate) {
                                        const float x = dot(texcoord, chart->majorAxis);
                                        const float y = dot(texcoord, chart->minorAxis);
                                        texcoord.x = x;
                                        texcoord.y = y;
                                        texcoord -= chart->minCorner;
                                } else {
                                        texcoord -= minCorner;
                                }
                                texcoord *= scale;
                                XA_DEBUG_ASSERT(texcoord.x >= 0.0f && texcoord.y >= 0.0f);
                                XA_DEBUG_ASSERT(isFinite(texcoord.x) && isFinite(texcoord.y));
                                extents = max(extents, texcoord);
                        }
                        XA_DEBUG_ASSERT(extents.x >= 0 && extents.y >= 0);
                        // Scale the charts to use the entire texel area available. So, if the width is 0.1 we could scale it to 1 without increasing the lightmap usage and making a better use of it. In many cases this also improves the look of the seams, since vertices on the chart boundaries have more chances of being aligned with the texel centers.
                        if (extents.x > 0.0f && extents.y > 0.0f) {
                                // Block align: align all chart extents to 4x4 blocks, but taking padding and texel center offset into account.
                                const int blockAlignSizeOffset = options.padding * 2 + 1;
                                int width = ftoi_ceil(extents.x);
                                if (options.blockAlign)
                                        width = align(width + blockAlignSizeOffset, 4) - blockAlignSizeOffset;
                                int height = ftoi_ceil(extents.y);
                                if (options.blockAlign)
                                        height = align(height + blockAlignSizeOffset, 4) - blockAlignSizeOffset;
                                for (uint32_t v = 0; v < chart->uniqueVertexCount(); v++) {
                                        Vector2 &texcoord = chart->uniqueVertexAt(v);
                                        texcoord.x = texcoord.x / extents.x * (float)width;
                                        texcoord.y = texcoord.y / extents.y * (float)height;
                                }
                                extents.x = (float)width;
                                extents.y = (float)height;
                        }
                        // Limit chart size, either to PackOptions::maxChartSize or maxResolution (if set), whichever is smaller.
                        // If limiting chart size to maxResolution, print a warning, since that may not be desirable to the user.
                        uint32_t maxChartSize = options.maxChartSize;
                        bool warnChartResized = false;
                        if (maxResolution > 0 && (maxChartSize == 0 || maxResolution < maxChartSize)) {
                                maxChartSize = maxResolution - options.padding * 2; // Don't include padding.
                                warnChartResized = true;
                        }
                        if (maxChartSize > 0) {
                                const float realMaxChartSize = (float)maxChartSize - 1.0f; // Aligning to texel centers increases texel footprint by 1.
                                if (extents.x > realMaxChartSize || extents.y > realMaxChartSize) {
                                        if (warnChartResized)
                                                XA_PRINT("   Resizing chart %u from %gx%g to %ux%u to fit atlas\n", c, extents.x, extents.y, maxChartSize, maxChartSize);
                                        scale = realMaxChartSize / max(extents.x, extents.y);
                                        for (uint32_t i = 0; i < chart->uniqueVertexCount(); i++) {
                                                Vector2 &texcoord = chart->uniqueVertexAt(i);
                                                texcoord = min(texcoord * scale, Vector2(realMaxChartSize));
                                        }
                                }
                        }
                        // Align to texel centers and add padding offset.
                        extents.x = extents.y = 0.0f;
                        for (uint32_t v = 0; v < chart->uniqueVertexCount(); v++) {
                                Vector2 &texcoord = chart->uniqueVertexAt(v);
                                texcoord.x += 0.5f + options.padding;
                                texcoord.y += 0.5f + options.padding;
                                extents = max(extents, texcoord);
                        }
                        if (extents.x > resolution || extents.y > resolution)
                                XA_PRINT("   Chart %u extents are large (%gx%g)\n", c, extents.x, extents.y);
                        chartExtents[c] = extents;
                        chartOrderArray[c] = extents.x + extents.y; // Use perimeter for chart sort key.
                        minChartPerimeter = min(minChartPerimeter, chartOrderArray[c]);
                        maxChartPerimeter = max(maxChartPerimeter, chartOrderArray[c]);
                }
                // Sort charts by perimeter.
                m_radix = RadixSort();
                m_radix.sort(chartOrderArray);
                const uint32_t *ranks = m_radix.ranks();
                // Divide chart perimeter range into buckets.
                const float chartPerimeterBucketSize = (maxChartPerimeter - minChartPerimeter) / 16.0f;
                uint32_t currentChartBucket = 0;
                Array<Vector2i> chartStartPositions; // per atlas
                chartStartPositions.push_back(Vector2i(0, 0));
                // Pack sorted charts.
#if XA_DEBUG_EXPORT_ATLAS_IMAGES
                const bool createImage = true;
#else
                const bool createImage = options.createImage;
#endif
                // chartImage: result from conservative rasterization
                // chartImageBilinear: chartImage plus any texels that would be sampled by bilinear filtering.
                // chartImagePadding: either chartImage or chartImageBilinear depending on options, with a dilate filter applied options.padding times.
                // Rotated versions swap x and y.
                BitImage chartImage, chartImageBilinear, chartImagePadding;
                BitImage chartImageRotated, chartImageBilinearRotated, chartImagePaddingRotated;
                UniformGrid2 boundaryEdgeGrid;
                Array<Vector2i> atlasSizes;
                atlasSizes.push_back(Vector2i(0, 0));
                int progress = 0;
                for (uint32_t i = 0; i < chartCount; i++) {
                        uint32_t c = ranks[chartCount - i - 1]; // largest chart first
                        Chart *chart = m_charts[c];
                        // @@ Add special cases for dot and line charts. @@ Lightmap rasterizer also needs to handle these special cases.
                        // @@ We could also have a special case for chart quads. If the quad surface <= 4 texels, align vertices with texel centers and do not add padding. May be very useful for foliage.
                        // @@ In general we could reduce the padding of all charts by one texel by using a rasterizer that takes into account the 2-texel footprint of the tent bilinear filter. For example,
                        // if we have a chart that is less than 1 texel wide currently we add one texel to the left and one texel to the right creating a 3-texel-wide bitImage. However, if we know that the
                        // chart is only 1 texel wide we could align it so that it only touches the footprint of two texels:
                        //      |   |      <- Touches texels 0, 1 and 2.
                        //    |   |        <- Only touches texels 0 and 1.
                        // \   \ / \ /   /
                        //  \   X   X   /
                        //   \ / \ / \ /
                        //    V   V   V
                        //    0   1   2
                        XA_PROFILE_START(packChartsRasterize)
                        // Resize and clear (discard = true) chart images.
                        // Leave room for padding at extents.
                        chartImage.resize(ftoi_ceil(chartExtents[c].x) + options.padding, ftoi_ceil(chartExtents[c].y) + options.padding, true);
                        if (chart->allowRotate)
                                chartImageRotated.resize(chartImage.height(), chartImage.width(), true);
                        if (options.bilinear) {
                                chartImageBilinear.resize(chartImage.width(), chartImage.height(), true);
                                if (chart->allowRotate)
                                        chartImageBilinearRotated.resize(chartImage.height(), chartImage.width(), true);
                        }
                        // Rasterize chart faces.
                        const uint32_t faceCount = chart->indexCount / 3;
                        for (uint32_t f = 0; f < faceCount; f++) {
                                Vector2 vertices[3];
                                for (uint32_t v = 0; v < 3; v++)
                                        vertices[v] = chart->vertices[chart->indices[f * 3 + v]];
                                DrawTriangleCallbackArgs args;
                                args.chartBitImage = &chartImage;
                                args.chartBitImageRotated = chart->allowRotate ? &chartImageRotated : nullptr;
                                raster::drawTriangle(Vector2((float)chartImage.width(), (float)chartImage.height()), vertices, drawTriangleCallback, &args);
                        }
                        // Expand chart by pixels sampled by bilinear interpolation.
                        if (options.bilinear)
                                bilinearExpand(chart, &chartImage, &chartImageBilinear, chart->allowRotate ? &chartImageBilinearRotated : nullptr, boundaryEdgeGrid);
                        // Expand chart by padding pixels (dilation).
                        if (options.padding > 0) {
                                // Copy into the same BitImage instances for every chart to avoid reallocating BitImage buffers (largest chart is packed first).
                                XA_PROFILE_START(packChartsDilate)
                                if (options.bilinear)
                                        chartImageBilinear.copyTo(chartImagePadding);
                                else
                                        chartImage.copyTo(chartImagePadding);
                                chartImagePadding.dilate(options.padding);
                                if (chart->allowRotate) {
                                        if (options.bilinear)
                                                chartImageBilinearRotated.copyTo(chartImagePaddingRotated);
                                        else
                                                chartImageRotated.copyTo(chartImagePaddingRotated);
                                        chartImagePaddingRotated.dilate(options.padding);
                                }
                                XA_PROFILE_END(packChartsDilate)
                        }
                        XA_PROFILE_END(packChartsRasterize)
                        // Update brute force bucketing.
                        if (options.bruteForce) {
                                if (chartOrderArray[c] > minChartPerimeter && chartOrderArray[c] <= maxChartPerimeter - (chartPerimeterBucketSize * (currentChartBucket + 1))) {
                                        // Moved to a smaller bucket, reset start location.
                                        for (uint32_t j = 0; j < chartStartPositions.size(); j++)
                                                chartStartPositions[j] = Vector2i(0, 0);
                                        currentChartBucket++;
                                }
                        }
                        // Find a location to place the chart in the atlas.
                        BitImage *chartImageToPack, *chartImageToPackRotated;
                        if (options.padding > 0) {
                                chartImageToPack = &chartImagePadding;
                                chartImageToPackRotated = &chartImagePaddingRotated;
                        } else if (options.bilinear) {
                                chartImageToPack = &chartImageBilinear;
                                chartImageToPackRotated = &chartImageBilinearRotated;
                        } else {
                                chartImageToPack = &chartImage;
                                chartImageToPackRotated = &chartImageRotated;
                        }
                        uint32_t currentAtlas = 0;
                        int best_x = 0, best_y = 0;
                        int best_cw = 0, best_ch = 0;
                        int best_r = 0;
                        for (;;)
                        {
                                bool firstChartInBitImage = false;
                                XA_UNUSED(firstChartInBitImage);
                                if (currentAtlas + 1 > m_bitImages.size()) {
                                        // Chart doesn't fit in the current bitImage, create a new one.
                                        BitImage *bi = XA_NEW_ARGS(MemTag::Default, BitImage, resolution, resolution);
                                        m_bitImages.push_back(bi);
                                        atlasSizes.push_back(Vector2i(0, 0));
                                        firstChartInBitImage = true;
                                        if (createImage)
                                                m_atlasImages.push_back(XA_NEW_ARGS(MemTag::Default, AtlasImage, resolution, resolution));
                                        // Start positions are per-atlas, so create a new one of those too.
                                        chartStartPositions.push_back(Vector2i(0, 0));
                                }
                                XA_PROFILE_START(packChartsFindLocation)
                                const bool foundLocation = findChartLocation(chartStartPositions[currentAtlas], options.bruteForce, m_bitImages[currentAtlas], chartImageToPack, chartImageToPackRotated, atlasSizes[currentAtlas].x, atlasSizes[currentAtlas].y, &best_x, &best_y, &best_cw, &best_ch, &best_r, options.blockAlign, maxResolution, chart->allowRotate);
                                XA_PROFILE_END(packChartsFindLocation)
                                XA_DEBUG_ASSERT(!(firstChartInBitImage && !foundLocation)); // Chart doesn't fit in an empty, newly allocated bitImage. Shouldn't happen, since charts are resized if they are too big to fit in the atlas.
                                if (maxResolution == 0) {
                                        XA_DEBUG_ASSERT(foundLocation); // The atlas isn't limited to a fixed resolution, a chart location should be found on the first attempt.
                                        break;
                                }
                                if (foundLocation)
                                        break;
                                // Chart doesn't fit in the current bitImage, try the next one.
                                currentAtlas++;
                        }
                        // Update brute force start location.
                        if (options.bruteForce) {
                                // Reset start location if the chart expanded the atlas.
                                if (best_x + best_cw > atlasSizes[currentAtlas].x || best_y + best_ch > atlasSizes[currentAtlas].y) {
                                        for (uint32_t j = 0; j < chartStartPositions.size(); j++)
                                                chartStartPositions[j] = Vector2i(0, 0);
                                }
                                else {
                                        chartStartPositions[currentAtlas] = Vector2i(best_x, best_y);
                                }
                        }
                        // Update parametric extents.
                        atlasSizes[currentAtlas].x = max(atlasSizes[currentAtlas].x, best_x + best_cw);
                        atlasSizes[currentAtlas].y = max(atlasSizes[currentAtlas].y, best_y + best_ch);
                        // Resize bitImage if necessary.
                        // If maxResolution > 0, the bitImage is always set to maxResolutionIncludingPadding on creation and doesn't need to be dynamically resized.
                        if (maxResolution == 0) {
                                const uint32_t w = (uint32_t)atlasSizes[currentAtlas].x;
                                const uint32_t h = (uint32_t)atlasSizes[currentAtlas].y;
                                if (w > m_bitImages[0]->width() || h > m_bitImages[0]->height()) {
                                        m_bitImages[0]->resize(nextPowerOfTwo(w), nextPowerOfTwo(h), false);
                                        if (createImage)
                                                m_atlasImages[0]->resize(m_bitImages[0]->width(), m_bitImages[0]->height());
                                }
                        } else {
                                XA_DEBUG_ASSERT(atlasSizes[currentAtlas].x <= (int)maxResolution);
                                XA_DEBUG_ASSERT(atlasSizes[currentAtlas].y <= (int)maxResolution);
                        }
                        XA_PROFILE_START(packChartsBlit)
                        addChart(m_bitImages[currentAtlas], chartImageToPack, chartImageToPackRotated, atlasSizes[currentAtlas].x, atlasSizes[currentAtlas].y, best_x, best_y, best_r);
                        XA_PROFILE_END(packChartsBlit)
                        if (createImage) {
                                if (best_r == 0) {
                                        m_atlasImages[currentAtlas]->addChart(c, &chartImage, options.bilinear ? &chartImageBilinear : nullptr, options.padding > 0 ? &chartImagePadding : nullptr, atlasSizes[currentAtlas].x, atlasSizes[currentAtlas].y, best_x, best_y);
                                } else {
                                        m_atlasImages[currentAtlas]->addChart(c, &chartImageRotated, options.bilinear ? &chartImageBilinearRotated : nullptr, options.padding > 0 ? &chartImagePaddingRotated : nullptr, atlasSizes[currentAtlas].x, atlasSizes[currentAtlas].y, best_x, best_y);
                                }
#if XA_DEBUG_EXPORT_ATLAS_IMAGES && XA_DEBUG_EXPORT_ATLAS_IMAGES_PER_CHART
                                for (uint32_t j = 0; j < m_atlasImages.size(); j++) {
                                        char filename[256];
                                        XA_SPRINTF(filename, sizeof(filename), "debug_atlas_image%02u_chart%04u.tga", j, i);
                                        m_atlasImages[j]->writeTga(filename, (uint32_t)atlasSizes[j].x, (uint32_t)atlasSizes[j].y);
                                }
#endif
                        }
                        chart->atlasIndex = (int32_t)currentAtlas;
                        // Modify texture coordinates:
                        //  - rotate if the chart should be rotated
                        //  - translate to chart location
                        //  - translate to remove padding from top and left atlas edges (unless block aligned)
                        for (uint32_t v = 0; v < chart->uniqueVertexCount(); v++) {
                                Vector2 &texcoord = chart->uniqueVertexAt(v);
                                Vector2 t = texcoord;
                                if (best_r) {
                                        XA_DEBUG_ASSERT(chart->allowRotate);
                                        swap(t.x, t.y);
                                }
                                texcoord.x = best_x + t.x;
                                texcoord.y = best_y + t.y;
                                if (!options.blockAlign) {
                                        texcoord.x -= (float)options.padding;
                                        texcoord.y -= (float)options.padding;
                                }
                                XA_ASSERT(texcoord.x >= 0 && texcoord.y >= 0);
                                XA_ASSERT(isFinite(texcoord.x) && isFinite(texcoord.y));
                        }
                        if (progressFunc) {
                                const int newProgress = int((i + 1) / (float)chartCount * 100.0f);
                                if (newProgress != progress) {
                                        progress = newProgress;
                                        if (!progressFunc(ProgressCategory::PackCharts, progress, progressUserData))
                                                return false;
                                }
                        }
                }
                if (options.blockAlign) {
                        if (maxResolution == 0) {
                                m_width = max(0, atlasSizes[0].x);
                                m_height = max(0, atlasSizes[0].y);
                        } else {
                                m_width = m_height = maxResolution;
                        }
                } else {
                        // Remove padding from outer edges.
                        if (maxResolution == 0) {
                                m_width = max(0, atlasSizes[0].x - (int)options.padding * 2);
                                m_height = max(0, atlasSizes[0].y - (int)options.padding * 2);
                        } else {
                                m_width = m_height = maxResolution - (int)options.padding * 2;
                        }
                }
                XA_PRINT("   %dx%d resolution\n", m_width, m_height);
                m_utilization.resize(m_bitImages.size());
                for (uint32_t i = 0; i < m_utilization.size(); i++) {
                        if (m_width == 0 || m_height == 0)
                                m_utilization[i] = 0.0f;
                        else {
                                uint32_t count = 0;
                                for (uint32_t y = 0; y < m_height; y++) {
                                        for (uint32_t x = 0; x < m_width; x++)
                                                count += m_bitImages[i]->get(x, y);
                                }
                                m_utilization[i] = float(count) / (m_width * m_height);
                        }
                        if (m_utilization.size() > 1) {
                                XA_PRINT("   %u: %f%% utilization\n", i, m_utilization[i] * 100.0f);
                        }
                        else {
                                XA_PRINT("   %f%% utilization\n", m_utilization[i] * 100.0f);
                        }
                }
#if XA_DEBUG_EXPORT_ATLAS_IMAGES
                for (uint32_t i = 0; i < m_atlasImages.size(); i++) {
                        char filename[256];
                        XA_SPRINTF(filename, sizeof(filename), "debug_atlas_image%02u.tga", i);
                        m_atlasImages[i]->writeTga(filename, m_width, m_height);
                }
#endif
                if (progressFunc && progress != 100) {
                        if (!progressFunc(ProgressCategory::PackCharts, 100, progressUserData))
                                return false;
                }
                return true;
        }

private:
        // IC: Brute force is slow, and random may take too much time to converge. We start inserting large charts in a small atlas. Using brute force is lame, because most of the space
        // is occupied at this point. At the end we have many small charts and a large atlas with sparse holes. Finding those holes randomly is slow. A better approach would be to
        // start stacking large charts as if they were tetris pieces. Once charts get small try to place them randomly. It may be interesting to try a intermediate strategy, first try
        // along one axis and then try exhaustively along that axis.
        bool findChartLocation(const Vector2i &startPosition, bool bruteForce, const BitImage *atlasBitImage, const BitImage *chartBitImage, const BitImage *chartBitImageRotated, int w, int h, int *best_x, int *best_y, int *best_w, int *best_h, int *best_r, bool blockAligned, uint32_t maxResolution, bool allowRotate)
        {
                const int attempts = 4096;
                if (bruteForce || attempts >= w * h)
                        return findChartLocation_bruteForce(startPosition, atlasBitImage, chartBitImage, chartBitImageRotated, w, h, best_x, best_y, best_w, best_h, best_r, blockAligned, maxResolution, allowRotate);
                return findChartLocation_random(atlasBitImage, chartBitImage, chartBitImageRotated, w, h, best_x, best_y, best_w, best_h, best_r, attempts, blockAligned, maxResolution, allowRotate);
        }

        bool findChartLocation_bruteForce(const Vector2i &startPosition, const BitImage *atlasBitImage, const BitImage *chartBitImage, const BitImage *chartBitImageRotated, int w, int h, int *best_x, int *best_y, int *best_w, int *best_h, int *best_r, bool blockAligned, uint32_t maxResolution, bool allowRotate)
        {
                const int stepSize = blockAligned ? 4 : 1;
                int best_metric = INT_MAX;
                // Try two different orientations.
                for (int r = 0; r < 2; r++) {
                        int cw = chartBitImage->width();
                        int ch = chartBitImage->height();
                        if (r == 1) {
                                if (allowRotate)
                                        swap(cw, ch);
                                else
                                        break;
                        }
                        for (int y = startPosition.y; y <= h + stepSize; y += stepSize) {
                                if (maxResolution > 0 && y > (int)maxResolution - ch)
                                        break;
                                for (int x = (y == startPosition.y ? startPosition.x : 0); x <= w + stepSize; x += stepSize) {
                                        if (maxResolution > 0 && x > (int)maxResolution - cw)
                                                break;
                                        // Early out if metric is not better.
                                        const int extentX = max(w, x + cw), extentY = max(h, y + ch);
                                        const int area = extentX * extentY;
                                        const int extents = max(extentX, extentY);
                                        const int metric = extents * extents + area;
                                        if (metric > best_metric)
                                                continue;
                                        // If metric is the same, pick the one closest to the origin.
                                        if (metric == best_metric && max(x, y) >= max(*best_x, *best_y))
                                                continue;
                                        if (!atlasBitImage->canBlit(r == 1 ? *chartBitImageRotated : *chartBitImage, x, y))
                                                continue;
                                        best_metric = metric;
                                        *best_x = x;
                                        *best_y = y;
                                        *best_w = cw;
                                        *best_h = ch;
                                        *best_r = r;
                                        if (area == w * h)
                                                return true; // Chart is completely inside, do not look at any other location.
                                }
                        }
                }
                return best_metric != INT_MAX;
        }

        bool findChartLocation_random(const BitImage *atlasBitImage, const BitImage *chartBitImage, const BitImage *chartBitImageRotated, int w, int h, int *best_x, int *best_y, int *best_w, int *best_h, int *best_r, int minTrialCount, bool blockAligned, uint32_t maxResolution, bool allowRotate)
        {
                bool result = false;
                const int BLOCK_SIZE = 4;
                int best_metric = INT_MAX;
                for (int i = 0; i < minTrialCount; i++) {
                        int cw = chartBitImage->width();
                        int ch = chartBitImage->height();
                        int r = allowRotate ? m_rand.getRange(1) : 0;
                        if (r == 1)
                                swap(cw, ch);
                        // + 1 to extend atlas in case atlas full. We may want to use a higher number to increase probability of extending atlas.
                        int xRange = w + 1;
                        int yRange = h + 1;
                        // Clamp to max resolution.
                        if (maxResolution > 0) {
                                xRange = min(xRange, (int)maxResolution - cw);
                                yRange = min(yRange, (int)maxResolution - ch);
                        }
                        int x = m_rand.getRange(xRange);
                        int y = m_rand.getRange(yRange);
                        if (blockAligned) {
                                x = align(x, BLOCK_SIZE);
                                y = align(y, BLOCK_SIZE);
                                if (maxResolution > 0 && (x > (int)maxResolution - cw || y > (int)maxResolution - ch))
                                        continue; // Block alignment pushed the chart outside the atlas.
                        }
                        // Early out.
                        int area = max(w, x + cw) * max(h, y + ch);
                        //int perimeter = max(w, x+cw) + max(h, y+ch);
                        int extents = max(max(w, x + cw), max(h, y + ch));
                        int metric = extents * extents + area;
                        if (metric > best_metric) {
                                continue;
                        }
                        if (metric == best_metric && min(x, y) > min(*best_x, *best_y)) {
                                // If metric is the same, pick the one closest to the origin.
                                continue;
                        }
                        if (atlasBitImage->canBlit(r == 1 ? *chartBitImageRotated : *chartBitImage, x, y)) {
                                result = true;
                                best_metric = metric;
                                *best_x = x;
                                *best_y = y;
                                *best_w = cw;
                                *best_h = ch;
                                *best_r = allowRotate ? r : 0;
                                if (area == w * h) {
                                        // Chart is completely inside, do not look at any other location.
                                        break;
                                }
                        }
                }
                return result;
        }

        void addChart(BitImage *atlasBitImage, const BitImage *chartBitImage, const BitImage *chartBitImageRotated, int atlas_w, int atlas_h, int offset_x, int offset_y, int r)
        {
                XA_DEBUG_ASSERT(r == 0 || r == 1);
                const BitImage *image = r == 0 ? chartBitImage : chartBitImageRotated;
                const int w = image->width();
                const int h = image->height();
                for (int y = 0; y < h; y++) {
                        int yy = y + offset_y;
                        if (yy >= 0) {
                                for (int x = 0; x < w; x++) {
                                        int xx = x + offset_x;
                                        if (xx >= 0) {
                                                if (image->get(x, y)) {
                                                        if (xx < atlas_w && yy < atlas_h) {
                                                                XA_DEBUG_ASSERT(atlasBitImage->get(xx, yy) == false);
                                                                atlasBitImage->set(xx, yy);
                                                        }
                                                }
                                        }
                                }
                        }
                }
        }

        void bilinearExpand(const Chart *chart, BitImage *source, BitImage *dest, BitImage *destRotated, UniformGrid2 &boundaryEdgeGrid) const
        {
                boundaryEdgeGrid.reset(chart->vertices, chart->indices);
                if (chart->boundaryEdges) {
                        const uint32_t edgeCount = chart->boundaryEdges->size();
                        for (uint32_t i = 0; i < edgeCount; i++)
                                boundaryEdgeGrid.append((*chart->boundaryEdges)[i]);
                } else {
                        for (uint32_t i = 0; i < chart->indexCount; i++)
                                boundaryEdgeGrid.append(i);
                }
                const int xOffsets[] = { -1, 0, 1, -1, 1, -1, 0, 1 };
                const int yOffsets[] = { -1, -1, -1, 0, 0, 1, 1, 1 };
                for (uint32_t y = 0; y < source->height(); y++) {
                        for (uint32_t x = 0; x < source->width(); x++) {
                                // Copy pixels from source.
                                if (source->get(x, y))
                                        goto setPixel;
                                // Empty pixel. If none of of the surrounding pixels are set, this pixel can't be sampled by bilinear interpolation.
                                {
                                        uint32_t s = 0;
                                        for (; s < 8; s++) {
                                                const int sx = (int)x + xOffsets[s];
                                                const int sy = (int)y + yOffsets[s];
                                                if (sx < 0 || sy < 0 || sx >= (int)source->width() || sy >= (int)source->height())
                                                        continue;
                                                if (source->get((uint32_t)sx, (uint32_t)sy))
                                                        break;
                                        }
                                        if (s == 8)
                                                continue;
                                }
                                {
                                        // If a 2x2 square centered on the pixels centroid intersects the triangle, this pixel will be sampled by bilinear interpolation.
                                        // See "Precomputed Global Illumination in Frostbite (GDC 2018)" page 95
                                        const Vector2 centroid((float)x + 0.5f, (float)y + 0.5f);
                                        const Vector2 squareVertices[4] = {
                                                Vector2(centroid.x - 1.0f, centroid.y - 1.0f),
                                                Vector2(centroid.x + 1.0f, centroid.y - 1.0f),
                                                Vector2(centroid.x + 1.0f, centroid.y + 1.0f),
                                                Vector2(centroid.x - 1.0f, centroid.y + 1.0f)
                                        };
                                        for (uint32_t j = 0; j < 4; j++) {
                                                if (boundaryEdgeGrid.intersect(squareVertices[j], squareVertices[(j + 1) % 4], 0.0f))
                                                        goto setPixel;
                                        }
                                }
                                continue;
                        setPixel:
                                dest->set(x, y);
                                if (destRotated)
                                        destRotated->set(y, x);
                        }
                }
        }

        struct DrawTriangleCallbackArgs
        {
                BitImage *chartBitImage, *chartBitImageRotated;
        };

        static bool drawTriangleCallback(void *param, int x, int y)
        {
                auto args = (DrawTriangleCallbackArgs *)param;
                args->chartBitImage->set(x, y);
                if (args->chartBitImageRotated)
                        args->chartBitImageRotated->set(y, x);
                return true;
        }

        Array<AtlasImage *> m_atlasImages;
        Array<float> m_utilization;
        Array<BitImage *> m_bitImages;
        Array<Chart *> m_charts;
        RadixSort m_radix;
        uint32_t m_width = 0;
        uint32_t m_height = 0;
        float m_texelsPerUnit = 0.0f;
        KISSRng m_rand;
};

} // namespace pack
} // namespace internal

struct Context
{
        Atlas atlas;
        uint32_t meshCount = 0;
        internal::Progress *addMeshProgress = nullptr;
        internal::TaskGroupHandle addMeshTaskGroup;
        internal::param::Atlas paramAtlas;
        ProgressFunc progressFunc = nullptr;
        void *progressUserData = nullptr;
        internal::TaskScheduler *taskScheduler;
        internal::Array<internal::UvMesh *> uvMeshes;
        internal::Array<internal::UvMeshInstance *> uvMeshInstances;
};

Atlas *Create()
{
        Context *ctx = XA_NEW(internal::MemTag::Default, Context);
        memset(&ctx->atlas, 0, sizeof(Atlas));
        ctx->taskScheduler = XA_NEW(internal::MemTag::Default, internal::TaskScheduler);
        return &ctx->atlas;
}

static void DestroyOutputMeshes(Context *ctx)
{
        if (!ctx->atlas.meshes)
                return;
        for (int i = 0; i < (int)ctx->atlas.meshCount; i++) {
                Mesh &mesh = ctx->atlas.meshes[i];
                for (uint32_t j = 0; j < mesh.chartCount; j++) {
                        if (mesh.chartArray[j].faceArray)
                                XA_FREE(mesh.chartArray[j].faceArray);
                }
                if (mesh.chartArray)
                        XA_FREE(mesh.chartArray);
                if (mesh.vertexArray)
                        XA_FREE(mesh.vertexArray);
                if (mesh.indexArray)
                        XA_FREE(mesh.indexArray);
        }
        if (ctx->atlas.meshes)
                XA_FREE(ctx->atlas.meshes);
        ctx->atlas.meshes = nullptr;
}

void Destroy(Atlas *atlas)
{
        XA_DEBUG_ASSERT(atlas);
        Context *ctx = (Context *)atlas;
        if (atlas->utilization)
                XA_FREE(atlas->utilization);
        if (atlas->image)
                XA_FREE(atlas->image);
        DestroyOutputMeshes(ctx);
        if (ctx->addMeshProgress) {
                ctx->addMeshProgress->cancel = true;
                AddMeshJoin(atlas); // frees addMeshProgress
        }
        ctx->taskScheduler->~TaskScheduler();
        XA_FREE(ctx->taskScheduler);
        for (uint32_t i = 0; i < ctx->uvMeshes.size(); i++) {
                internal::UvMesh *mesh = ctx->uvMeshes[i];
                for (uint32_t j = 0; j < mesh->charts.size(); j++) {
                        mesh->charts[j]->~UvMeshChart();
                        XA_FREE(mesh->charts[j]);
                }
                mesh->~UvMesh();
                XA_FREE(mesh);
        }
        for (uint32_t i = 0; i < ctx->uvMeshInstances.size(); i++) {
                internal::UvMeshInstance *mesh = ctx->uvMeshInstances[i];
                mesh->~UvMeshInstance();
                XA_FREE(mesh);
        }
        ctx->~Context();
        XA_FREE(ctx);
#if XA_DEBUG_HEAP
        internal::ReportLeaks();
#endif
}

struct AddMeshTaskArgs
{
        Context *ctx;
        internal::Mesh *mesh;
};

static void runAddMeshTask(void *userData)
{
        XA_PROFILE_START(addMeshThread)
        auto args = (AddMeshTaskArgs *)userData; // Responsible for freeing this.
        internal::Mesh *mesh = args->mesh;
        internal::Progress *progress = args->ctx->addMeshProgress;
        if (progress->cancel)
                goto cleanup;
        {
                XA_PROFILE_START(addMeshCreateColocals)
                mesh->createColocals();
                XA_PROFILE_END(addMeshCreateColocals)
        }
        if (progress->cancel)
                goto cleanup;
        {
                XA_PROFILE_START(addMeshCreateFaceGroups)
                mesh->createFaceGroups();
                XA_PROFILE_END(addMeshCreateFaceGroups)
        }
        if (progress->cancel)
                goto cleanup;
#if XA_DEBUG_EXPORT_OBJ_SOURCE_MESHES
        char filename[256];
        XA_SPRINTF(filename, sizeof(filename), "debug_mesh_%03u.obj", mesh->id());
        FILE *file;
        XA_FOPEN(file, filename, "w");
        if (file) {
                mesh->writeObjVertices(file);
                // groups
                uint32_t numGroups = 0;
                for (uint32_t i = 0; i < mesh->faceCount(); i++) {
                        if (mesh->faceGroupAt(i) != Mesh::kInvalidFaceGroup)
                                numGroups = internal::max(numGroups, mesh->faceGroupAt(i) + 1);
                }
                for (uint32_t i = 0; i < numGroups; i++) {
                        fprintf(file, "o group_%04d\n", i);
                        fprintf(file, "s off\n");
                        for (uint32_t f = 0; f < mesh->faceCount(); f++) {
                                if (mesh->faceGroupAt(f) == i)
                                        mesh->writeObjFace(file, f);
                        }
                }
                fprintf(file, "o group_ignored\n");
                fprintf(file, "s off\n");
                for (uint32_t f = 0; f < mesh->faceCount(); f++) {
                        if (mesh->faceGroupAt(f) == Mesh::kInvalidFaceGroup)
                                mesh->writeObjFace(file, f);
                }
                mesh->writeObjBoundaryEges(file);
                fclose(file);
        }
#endif
        {
                XA_PROFILE_START(addMeshCreateChartGroupsReal)
                args->ctx->paramAtlas.addMesh(args->ctx->taskScheduler, mesh); // addMesh is thread safe
                XA_PROFILE_END(addMeshCreateChartGroupsReal)
        }
        if (progress->cancel)
                goto cleanup;
        progress->value++;
        progress->update();
cleanup:
        mesh->~Mesh();
        XA_FREE(mesh);
        args->~AddMeshTaskArgs();
        XA_FREE(args);
        XA_PROFILE_END(addMeshThread)
}

static internal::Vector3 DecodePosition(const MeshDecl &meshDecl, uint32_t index)
{
        XA_DEBUG_ASSERT(meshDecl.vertexPositionData);
        XA_DEBUG_ASSERT(meshDecl.vertexPositionStride > 0);
        return *((const internal::Vector3 *)&((const uint8_t *)meshDecl.vertexPositionData)[meshDecl.vertexPositionStride * index]);
}

static internal::Vector3 DecodeNormal(const MeshDecl &meshDecl, uint32_t index)
{
        XA_DEBUG_ASSERT(meshDecl.vertexNormalData);
        XA_DEBUG_ASSERT(meshDecl.vertexNormalStride > 0);
        return *((const internal::Vector3 *)&((const uint8_t *)meshDecl.vertexNormalData)[meshDecl.vertexNormalStride * index]);
}

static internal::Vector2 DecodeUv(const MeshDecl &meshDecl, uint32_t index)
{
        XA_DEBUG_ASSERT(meshDecl.vertexUvData);
        XA_DEBUG_ASSERT(meshDecl.vertexUvStride > 0);
        return *((const internal::Vector2 *)&((const uint8_t *)meshDecl.vertexUvData)[meshDecl.vertexUvStride * index]);
}

static uint32_t DecodeIndex(IndexFormat::Enum format, const void *indexData, int32_t offset, uint32_t i)
{
        XA_DEBUG_ASSERT(indexData);
        if (format == IndexFormat::UInt16)
                return uint16_t((int32_t)((const uint16_t *)indexData)[i] + offset);
        return uint32_t((int32_t)((const uint32_t *)indexData)[i] + offset);
}

AddMeshError::Enum AddMesh(Atlas *atlas, const MeshDecl &meshDecl, uint32_t meshCountHint)
{
        XA_DEBUG_ASSERT(atlas);
        if (!atlas) {
                XA_PRINT_WARNING("AddMesh: atlas is null.\n");
                return AddMeshError::Error;
        }
        Context *ctx = (Context *)atlas;
        if (!ctx->uvMeshes.isEmpty()) {
                XA_PRINT_WARNING("AddMesh: Meshes and UV meshes cannot be added to the same atlas.\n");
                return AddMeshError::Error;
        }
#if XA_PROFILE
        if (ctx->meshCount == 0)
                internal::s_profile.addMeshReal = clock();
#endif
        // Don't know how many times AddMesh will be called, so progress needs to adjusted each time.
        if (!ctx->addMeshProgress) {
                ctx->addMeshProgress = XA_NEW_ARGS(internal::MemTag::Default, internal::Progress, ProgressCategory::AddMesh, ctx->progressFunc, ctx->progressUserData, 1);
        }
        else {
                ctx->addMeshProgress->setMaxValue(internal::max(ctx->meshCount + 1, meshCountHint));
        }
        XA_PROFILE_START(addMeshCopyData)
        const bool hasIndices = meshDecl.indexCount > 0;
        const uint32_t indexCount = hasIndices ? meshDecl.indexCount : meshDecl.vertexCount;
        XA_PRINT("Adding mesh %d: %u vertices, %u triangles\n", ctx->meshCount, meshDecl.vertexCount, indexCount / 3);
        // Expecting triangle faces.
        if ((indexCount % 3) != 0)
                return AddMeshError::InvalidIndexCount;
        if (hasIndices) {
                // Check if any index is out of range.
                for (uint32_t i = 0; i < indexCount; i++) {
                        const uint32_t index = DecodeIndex(meshDecl.indexFormat, meshDecl.indexData, meshDecl.indexOffset, i);
                        if (index >= meshDecl.vertexCount)
                                return AddMeshError::IndexOutOfRange;
                }
        }
        uint32_t meshFlags = internal::MeshFlags::HasFaceGroups | internal::MeshFlags::HasIgnoredFaces;
        if (meshDecl.vertexNormalData)
                meshFlags |= internal::MeshFlags::HasNormals;
        internal::Mesh *mesh = XA_NEW_ARGS(internal::MemTag::Mesh, internal::Mesh, meshDecl.epsilon, meshDecl.vertexCount, indexCount / 3, meshFlags, ctx->meshCount);
        for (uint32_t i = 0; i < meshDecl.vertexCount; i++) {
                internal::Vector3 normal(0.0f);
                internal::Vector2 texcoord(0.0f);
                if (meshDecl.vertexNormalData)
                        normal = DecodeNormal(meshDecl, i);
                if (meshDecl.vertexUvData)
                        texcoord = DecodeUv(meshDecl, i);
                mesh->addVertex(DecodePosition(meshDecl, i), normal, texcoord);
        }
        for (uint32_t i = 0; i < indexCount / 3; i++) {
                uint32_t tri[3];
                for (int j = 0; j < 3; j++)
                        tri[j] = hasIndices ? DecodeIndex(meshDecl.indexFormat, meshDecl.indexData, meshDecl.indexOffset, i * 3 + j) : i * 3 + j;
                bool ignore = false;
                // Check for degenerate or zero length edges.
                for (int j = 0; j < 3; j++) {
                        const uint32_t index1 = tri[j];
                        const uint32_t index2 = tri[(j + 1) % 3];
                        if (index1 == index2) {
                                ignore = true;
                                XA_PRINT("   Degenerate edge: index %d, index %d\n", index1, index2);
                                break;
                        }
                        const internal::Vector3 &pos1 = mesh->position(index1);
                        const internal::Vector3 &pos2 = mesh->position(index2);
                        if (internal::length(pos2 - pos1) <= 0.0f) {
                                ignore = true;
                                XA_PRINT("   Zero length edge: index %d position (%g %g %g), index %d position (%g %g %g)\n", index1, pos1.x, pos1.y, pos1.z, index2, pos2.x, pos2.y, pos2.z);
                                break;
                        }
                }
                // Ignore faces with any nan vertex attributes.
                if (!ignore) {
                        for (int j = 0; j < 3; j++) {
                                const internal::Vector3 &pos = mesh->position(tri[j]);
                                if (internal::isNan(pos.x) || internal::isNan(pos.y) || internal::isNan(pos.z)) {
                                        XA_PRINT("   NAN position in face: %d\n", i);
                                        ignore = true;
                                        break;
                                }
                                if (meshDecl.vertexNormalData) {
                                        const internal::Vector3 &normal = mesh->normal(tri[j]);
                                        if (internal::isNan(normal.x) || internal::isNan(normal.y) || internal::isNan(normal.z)) {
                                                XA_PRINT("   NAN normal in face: %d\n", i);
                                                ignore = true;
                                                break;
                                        }
                                }
                                if (meshDecl.vertexUvData) {
                                        const internal::Vector2 &uv = mesh->texcoord(tri[j]);
                                        if (internal::isNan(uv.x) || internal::isNan(uv.y)) {
                                                XA_PRINT("   NAN texture coordinate in face: %d\n", i);
                                                ignore = true;
                                                break;
                                        }
                                }
                        }
                }
                const internal::Vector3 &a = mesh->position(tri[0]);
                const internal::Vector3 &b = mesh->position(tri[1]);
                const internal::Vector3 &c = mesh->position(tri[2]);
                // Check for zero area faces.
                float area = 0.0f;
                if (!ignore) {
                        area = internal::length(internal::cross(b - a, c - a)) * 0.5f;
                        if (area <= internal::kAreaEpsilon) {
                                ignore = true;
                                XA_PRINT("   Zero area face: %d, indices (%d %d %d), area is %f\n", i, tri[0], tri[1], tri[2], area);
                        }
                }
                if (!ignore) {
                        if (internal::equal(a, b, meshDecl.epsilon) || internal::equal(a, c, meshDecl.epsilon) || internal::equal(b, c, meshDecl.epsilon)) {
                                ignore = true;
                                XA_PRINT("   Degenerate face: %d, area is %f\n", i, area);
                        }
                }
                if (meshDecl.faceIgnoreData && meshDecl.faceIgnoreData[i])
                        ignore = true;
                mesh->addFace(tri[0], tri[1], tri[2], ignore);
        }
        XA_PROFILE_END(addMeshCopyData)
        if (ctx->addMeshTaskGroup.value == UINT32_MAX)
                ctx->addMeshTaskGroup = ctx->taskScheduler->createTaskGroup();
        AddMeshTaskArgs *taskArgs = XA_NEW(internal::MemTag::Default, AddMeshTaskArgs); // The task frees this.
        taskArgs->ctx = ctx;
        taskArgs->mesh = mesh;
        internal::Task task;
        task.userData = taskArgs;
        task.func = runAddMeshTask;
        ctx->taskScheduler->run(ctx->addMeshTaskGroup, task);
        ctx->meshCount++;
        return AddMeshError::Success;
}

void AddMeshJoin(Atlas *atlas)
{
        XA_DEBUG_ASSERT(atlas);
        if (!atlas) {
                XA_PRINT_WARNING("AddMeshJoin: atlas is null.\n");
                return;
        }
        Context *ctx = (Context *)atlas;
        if (!ctx->addMeshProgress)
                return;
        ctx->taskScheduler->wait(&ctx->addMeshTaskGroup);
        ctx->addMeshProgress->~Progress();
        XA_FREE(ctx->addMeshProgress);
        ctx->addMeshProgress = nullptr;
        ctx->paramAtlas.sortChartGroups();
#if XA_PROFILE
        XA_PRINT("Added %u meshes\n", ctx->meshCount);
        internal::s_profile.addMeshReal = clock() - internal::s_profile.addMeshReal;
#endif
        XA_PROFILE_PRINT_AND_RESET("   Total (real): ", addMeshReal)
        XA_PROFILE_PRINT_AND_RESET("      Copy data: ", addMeshCopyData)
        XA_PROFILE_PRINT_AND_RESET("   Total (thread): ", addMeshThread)
        XA_PROFILE_PRINT_AND_RESET("      Create colocals: ", addMeshCreateColocals)
        XA_PROFILE_PRINT_AND_RESET("      Create face groups: ", addMeshCreateFaceGroups)
        XA_PROFILE_PRINT_AND_RESET("      Create chart groups (real): ", addMeshCreateChartGroupsReal)
        XA_PROFILE_PRINT_AND_RESET("      Create chart groups (thread): ", addMeshCreateChartGroupsThread)
        XA_PRINT_MEM_USAGE
}

struct EdgeKey
{
        EdgeKey() {}
        EdgeKey(const EdgeKey &k) : v0(k.v0), v1(k.v1) {}
        EdgeKey(uint32_t v0, uint32_t v1) : v0(v0), v1(v1) {}
        bool operator==(const EdgeKey &k) const { return v0 == k.v0 && v1 == k.v1; }

        uint32_t v0;
        uint32_t v1;
};

AddMeshError::Enum AddUvMesh(Atlas *atlas, const UvMeshDecl &decl)
{
        XA_DEBUG_ASSERT(atlas);
        if (!atlas) {
                XA_PRINT_WARNING("AddUvMesh: atlas is null.\n");
                return AddMeshError::Error;
        }
        Context *ctx = (Context *)atlas;
        if (ctx->meshCount > 0) {
                XA_PRINT_WARNING("AddUvMesh: Meshes and UV meshes cannot be added to the same atlas.\n");
                return AddMeshError::Error;
        }
        const bool decoded = (decl.indexCount <= 0);
        const uint32_t indexCount = decoded ? decl.vertexCount : decl.indexCount;
        XA_PRINT("Adding UV mesh %d: %u vertices, %u triangles\n", ctx->uvMeshes.size(), decl.vertexCount, indexCount / 3);
        // Expecting triangle faces.
        if ((indexCount % 3) != 0)
                return AddMeshError::InvalidIndexCount;
        if (!decoded) {
                // Check if any index is out of range.
                for (uint32_t i = 0; i < indexCount; i++) {
                        const uint32_t index = DecodeIndex(decl.indexFormat, decl.indexData, decl.indexOffset, i);
                        if (index >= decl.vertexCount)
                                return AddMeshError::IndexOutOfRange;
                }
        }
        internal::UvMeshInstance *meshInstance = XA_NEW(internal::MemTag::Default, internal::UvMeshInstance);
        meshInstance->texcoords.resize(decl.vertexCount);
        for (uint32_t i = 0; i < decl.vertexCount; i++) {
                internal::Vector2 texcoord = *((const internal::Vector2 *)&((const uint8_t *)decl.vertexUvData)[decl.vertexStride * i]);
                // Set nan values to 0.
                if (internal::isNan(texcoord.x) || internal::isNan(texcoord.y))
                        texcoord.x = texcoord.y = 0.0f;
                meshInstance->texcoords[i] = texcoord;
        }
        meshInstance->rotateCharts = decl.rotateCharts;
        // See if this is an instance of an already existing mesh.
        internal::UvMesh *mesh = nullptr;
        for (uint32_t m = 0; m < ctx->uvMeshes.size(); m++) {
                if (memcmp(&ctx->uvMeshes[m]->decl, &decl, sizeof(UvMeshDecl)) == 0) {
                        meshInstance->mesh = mesh = ctx->uvMeshes[m];
                        break;
                }
        }
        if (!mesh) {
                // Copy geometry to mesh.
                meshInstance->mesh = mesh = XA_NEW(internal::MemTag::Default, internal::UvMesh);
                mesh->decl = decl;
                mesh->indices.resize(decl.indexCount);
                for (uint32_t i = 0; i < indexCount; i++)
                        mesh->indices[i] = decoded ? i : DecodeIndex(decl.indexFormat, decl.indexData, decl.indexOffset, i);
                mesh->vertexToChartMap.resize(decl.vertexCount);
                for (uint32_t i = 0; i < mesh->vertexToChartMap.size(); i++)
                        mesh->vertexToChartMap[i] = UINT32_MAX;
                // Calculate charts (incident faces).
                internal::HashMap<internal::Vector2> vertexToFaceMap(internal::MemTag::Default, indexCount); // Face is index / 3
                const uint32_t faceCount = indexCount / 3;
                for (uint32_t i = 0; i < indexCount; i++)
                        vertexToFaceMap.add(meshInstance->texcoords[mesh->indices[i]]);
                internal::BitArray faceAssigned(faceCount);
                faceAssigned.zeroOutMemory();
                for (uint32_t f = 0; f < faceCount; f++) {
                        if (faceAssigned.get(f))
                                continue;
                        // Found an unassigned face, create a new chart.
                        internal::UvMeshChart *chart = XA_NEW(internal::MemTag::Default, internal::UvMeshChart);
                        chart->material = decl.faceMaterialData ? decl.faceMaterialData[f] : 0;
                        // Walk incident faces and assign them to the chart.
                        faceAssigned.set(f);
                        chart->faces.push_back(f);
                        for (;;) {
                                bool newFaceAssigned = false;
                                const uint32_t faceCount2 = chart->faces.size();
                                for (uint32_t f2 = 0; f2 < faceCount2; f2++) {
                                        const uint32_t face = chart->faces[f2];
                                        for (uint32_t i = 0; i < 3; i++) {
                                                const internal::Vector2 &texcoord = meshInstance->texcoords[meshInstance->mesh->indices[face * 3 + i]];
                                                uint32_t mapIndex = vertexToFaceMap.get(texcoord);
                                                while (mapIndex != UINT32_MAX) {
                                                        const uint32_t face2 = mapIndex / 3; // 3 vertices added per face.
                                                        // Materials must match.
                                                        if (!faceAssigned.get(face2) && (!decl.faceMaterialData || decl.faceMaterialData[face] == decl.faceMaterialData[face2])) {
                                                                faceAssigned.set(face2);
                                                                chart->faces.push_back(face2);
                                                                newFaceAssigned = true;
                                                        }
                                                        mapIndex = vertexToFaceMap.getNext(mapIndex);
                                                }
                                        }
                                }
                                if (!newFaceAssigned)
                                        break;
                        }
                        for (uint32_t i = 0; i < chart->faces.size(); i++) {
                                for (uint32_t j = 0; j < 3; j++) {
                                        const uint32_t vertex = meshInstance->mesh->indices[chart->faces[i] * 3 + j];
                                        chart->indices.push_back(vertex);
                                        mesh->vertexToChartMap[vertex] = mesh->charts.size();
                                }
                        }
                        mesh->charts.push_back(chart);
                }
                ctx->uvMeshes.push_back(mesh);
        } else {
                XA_PRINT("   instance of a previous UV mesh\n");
        }
        XA_PRINT("   %u charts\n", meshInstance->mesh->charts.size());
        ctx->uvMeshInstances.push_back(meshInstance);
        return AddMeshError::Success;
}

void ComputeCharts(Atlas *atlas, ChartOptions chartOptions)
{
        if (!atlas) {
                XA_PRINT_WARNING("ComputeCharts: atlas is null.\n");
                return;
        }
        Context *ctx = (Context *)atlas;
        if (!ctx->uvMeshInstances.isEmpty()) {
                XA_PRINT_WARNING("ComputeCharts: This function should not be called with UV meshes.\n");
                return;
        }
        AddMeshJoin(atlas);
        if (ctx->meshCount == 0) {
                XA_PRINT_WARNING("ComputeCharts: No meshes. Call AddMesh first.\n");
                return;
        }
        XA_PRINT("Computing charts\n");
        uint32_t chartCount = 0, chartsWithHolesCount = 0, holesCount = 0, chartsWithTJunctionsCount = 0, tJunctionsCount = 0;
        XA_PROFILE_START(computeChartsReal)
        if (!ctx->paramAtlas.computeCharts(ctx->taskScheduler, chartOptions, ctx->progressFunc, ctx->progressUserData)) {
                XA_PRINT("   Cancelled by user\n");
                return;
        }
        XA_PROFILE_END(computeChartsReal)
        // Count charts and print warnings.
        for (uint32_t i = 0; i < ctx->meshCount; i++) {
                for (uint32_t j = 0; j < ctx->paramAtlas.chartGroupCount(i); j++) {
                        const internal::param::ChartGroup *chartGroup = ctx->paramAtlas.chartGroupAt(i, j);
                        if (chartGroup->isVertexMap())
                                continue;
                        for (uint32_t k = 0; k < chartGroup->chartCount(); k++) {
                                const internal::param::Chart *chart = chartGroup->chartAt(k);
#if XA_PRINT_CHART_WARNINGS
                                if (chart->warningFlags() & internal::param::ChartWarningFlags::CloseHolesFailed)
                                        XA_PRINT_WARNING("   Chart %u (mesh %u, group %u, id %u): failed to close holes\n", chartCount, i, j, k);
                                if (chart->warningFlags() & internal::param::ChartWarningFlags::FixTJunctionsDuplicatedEdge)
                                        XA_PRINT_WARNING("   Chart %u (mesh %u, group %u, id %u): fixing t-junctions created non-manifold geometry\n", chartCount, i, j, k);
                                if (chart->warningFlags() & internal::param::ChartWarningFlags::FixTJunctionsFailed)
                                        XA_PRINT_WARNING("   Chart %u (mesh %u, group %u, id %u): fixing t-junctions failed\n", chartCount, i, j, k);
                                if (chart->warningFlags() & internal::param::ChartWarningFlags::TriangulateDuplicatedEdge)
                                        XA_PRINT_WARNING("   Chart %u (mesh %u, group %u, id %u): triangulation created non-manifold geometry\n", chartCount, i, j, k);
#endif
                                holesCount += chart->closedHolesCount();
                                if (chart->closedHolesCount() > 0)
                                        chartsWithHolesCount++;
                                tJunctionsCount += chart->fixedTJunctionsCount();
                                if (chart->fixedTJunctionsCount() > 0)
                                        chartsWithTJunctionsCount++;
                                chartCount++;
                        }
                }
        }
        if (holesCount > 0)
                XA_PRINT("   Closed %u holes in %u charts\n", holesCount, chartsWithHolesCount);
        if (tJunctionsCount > 0)
                XA_PRINT("   Fixed %u t-junctions in %u charts\n", tJunctionsCount, chartsWithTJunctionsCount);
        XA_PRINT("   %u charts\n", chartCount);
        XA_PROFILE_PRINT_AND_RESET("   Total (real): ", computeChartsReal)
        XA_PROFILE_PRINT_AND_RESET("   Total (thread): ", computeChartsThread)
        XA_PROFILE_PRINT_AND_RESET("      Build atlas: ", buildAtlas)
        XA_PROFILE_PRINT_AND_RESET("         Init: ", buildAtlasInit)
        XA_PROFILE_PRINT_AND_RESET("         Place seeds: ", buildAtlasPlaceSeeds)
        XA_PROFILE_PRINT_AND_RESET("         Relocate seeds: ", buildAtlasRelocateSeeds)
        XA_PROFILE_PRINT_AND_RESET("         Reset charts: ", buildAtlasResetCharts)
        XA_PROFILE_PRINT_AND_RESET("         Grow charts: ", buildAtlasGrowCharts)
        XA_PROFILE_PRINT_AND_RESET("         Merge charts: ", buildAtlasMergeCharts)
        XA_PROFILE_PRINT_AND_RESET("         Fill holes: ", buildAtlasFillHoles)
        XA_PROFILE_PRINT_AND_RESET("      Create chart meshes (real): ", createChartMeshesReal)
        XA_PROFILE_PRINT_AND_RESET("      Create chart meshes (thread): ", createChartMeshesThread)
        XA_PROFILE_PRINT_AND_RESET("         Fix t-junctions: ", fixChartMeshTJunctions)
        XA_PROFILE_PRINT_AND_RESET("         Close holes: ", closeChartMeshHoles)
        XA_PRINT_MEM_USAGE
}

void ParameterizeCharts(Atlas *atlas, ParameterizeFunc func)
{
        if (!atlas) {
                XA_PRINT_WARNING("ParameterizeCharts: atlas is null.\n");
                return;
        }
        Context *ctx = (Context *)atlas;
        if (!ctx->uvMeshInstances.isEmpty()) {
                XA_PRINT_WARNING("ParameterizeCharts: This function should not be called with UV meshes.\n");
                return;
        }
        if (!ctx->paramAtlas.chartsComputed()) {
                XA_PRINT_WARNING("ParameterizeCharts: ComputeCharts must be called first.\n");
                return;
        }
        atlas->atlasCount = 0;
        atlas->height = 0;
        atlas->texelsPerUnit = 0;
        atlas->width = 0;
        if (atlas->utilization) {
                XA_FREE(atlas->utilization);
                atlas->utilization = nullptr;
        }
        if (atlas->image) {
                XA_FREE(atlas->image);
                atlas->image = nullptr;
        }
        DestroyOutputMeshes(ctx);
        XA_PRINT("Parameterizing charts\n");
        XA_PROFILE_START(parameterizeChartsReal)
        if (!ctx->paramAtlas.parameterizeCharts(ctx->taskScheduler, func, ctx->progressFunc, ctx->progressUserData)) {
                XA_PRINT("   Cancelled by user\n");
                        return;
        }
        XA_PROFILE_END(parameterizeChartsReal)
        uint32_t chartCount = 0, orthoChartsCount = 0, planarChartsCount = 0, lscmChartsCount = 0, piecewiseChartsCount = 0, chartsAddedCount = 0, chartsDeletedCount = 0;
        for (uint32_t i = 0; i < ctx->meshCount; i++) {
                for (uint32_t j = 0; j < ctx->paramAtlas.chartGroupCount(i); j++) {
                        const internal::param::ChartGroup *chartGroup = ctx->paramAtlas.chartGroupAt(i, j);
                        if (chartGroup->isVertexMap())
                                continue;
                        for (uint32_t k = 0; k < chartGroup->chartCount(); k++) {
                                const internal::param::Chart *chart = chartGroup->chartAt(k);
                                if (chart->type() == ChartType::Planar)
                                        planarChartsCount++;
                                else if (chart->type() == ChartType::Ortho)
                                        orthoChartsCount++;
                                else if (chart->type() == ChartType::LSCM)
                                        lscmChartsCount++;
                                else if (chart->type() == ChartType::Piecewise)
                                        piecewiseChartsCount++;
                        }
                        chartCount += chartGroup->chartCount();
                        chartsAddedCount += chartGroup->paramAddedChartsCount();
                        chartsDeletedCount += chartGroup->paramDeletedChartsCount();
                }
        }
        XA_PRINT("   %u planar charts, %u ortho charts, %u LSCM charts, %u piecewise charts\n", planarChartsCount, orthoChartsCount, lscmChartsCount, piecewiseChartsCount);
        if (chartsDeletedCount > 0) {
                XA_PRINT("   %u charts with invalid parameterizations replaced with %u new charts\n", chartsDeletedCount, chartsAddedCount);
                XA_PRINT("   %u charts\n", chartCount);
        }
        uint32_t chartIndex = 0, invalidParamCount = 0;
        for (uint32_t i = 0; i < ctx->meshCount; i++) {
                for (uint32_t j = 0; j < ctx->paramAtlas.chartGroupCount(i); j++) {
                        const internal::param::ChartGroup *chartGroup = ctx->paramAtlas.chartGroupAt(i, j);
                        if (chartGroup->isVertexMap())
                                continue;
                        for (uint32_t k = 0; k < chartGroup->chartCount(); k++) {
                                const internal::param::Chart *chart = chartGroup->chartAt(k);
                                const internal::param::Quality &quality = chart->quality();
#if XA_DEBUG_EXPORT_OBJ_CHARTS_AFTER_PARAMETERIZATION
                                {
                                        char filename[256];
                                        XA_SPRINTF(filename, sizeof(filename), "debug_chart_%03u_after_parameterization.obj", chartIndex);
                                        chart->unifiedMesh()->writeObjFile(filename);
                                }
#endif
                                bool invalid = false;
                                const char *type = "LSCM";
                                if (chart->type() == ChartType::Planar)
                                        type = "planar";
                                else if (chart->type() == ChartType::Ortho)
                                        type = "ortho";
                                else if (chart->type() == ChartType::Piecewise)
                                        type = "piecewise";
                                if (quality.boundaryIntersection) {
                                        invalid = true;
                                        XA_PRINT_WARNING("   Chart %u (mesh %u, group %u, id %u) (%s): invalid parameterization, self-intersecting boundary.\n", chartIndex, i, j, k, type);
                                }
                                if (quality.flippedTriangleCount > 0) {
                                        invalid = true;
                                        XA_PRINT_WARNING("   Chart %u  (mesh %u, group %u, id %u) (%s): invalid parameterization, %u / %u flipped triangles.\n", chartIndex, i, j, k, type, quality.flippedTriangleCount, quality.totalTriangleCount);
                                }
                                if (invalid)
                                        invalidParamCount++;
#if XA_DEBUG_EXPORT_OBJ_INVALID_PARAMETERIZATION
                                if (invalid) {
                                        char filename[256];
                                        XA_SPRINTF(filename, sizeof(filename), "debug_chart_%03u_invalid_parameterization.obj", chartIndex);
                                        const internal::Mesh *mesh = chart->unifiedMesh();
                                        FILE *file;
                                        XA_FOPEN(file, filename, "w");
                                        if (file) {
                                                mesh->writeObjVertices(file);
                                                fprintf(file, "s off\n");
                                                fprintf(file, "o object\n");
                                                for (uint32_t f = 0; f < mesh->faceCount(); f++)
                                                        mesh->writeObjFace(file, f);
                                                if (!chart->paramFlippedFaces().isEmpty()) {
                                                        fprintf(file, "o flipped_faces\n");
                                                        for (uint32_t f = 0; f < chart->paramFlippedFaces().size(); f++)
                                                                mesh->writeObjFace(file, chart->paramFlippedFaces()[f]);
                                                }
                                                mesh->writeObjBoundaryEges(file);
                                                mesh->writeObjLinkedBoundaries(file);
                                                fclose(file);
                                        }
                                }
#endif
                                chartIndex++;
                        }
                }
        }
        if (invalidParamCount > 0)
                XA_PRINT_WARNING("   %u charts with invalid parameterizations\n", invalidParamCount);
        XA_PROFILE_PRINT_AND_RESET("   Total (real): ", parameterizeChartsReal)
        XA_PROFILE_PRINT_AND_RESET("   Total (thread): ", parameterizeChartsThread)
        XA_PROFILE_PRINT_AND_RESET("      Orthogonal: ", parameterizeChartsOrthogonal)
        XA_PROFILE_PRINT_AND_RESET("      LSCM: ", parameterizeChartsLSCM)
        XA_PROFILE_PRINT_AND_RESET("      Evaluate quality: ", parameterizeChartsEvaluateQuality)
        XA_PRINT_MEM_USAGE
}

void PackCharts(Atlas *atlas, PackOptions packOptions)
{
        // Validate arguments and context state.
        if (!atlas) {
                XA_PRINT_WARNING("PackCharts: atlas is null.\n");
                return;
        }
        Context *ctx = (Context *)atlas;
        if (ctx->meshCount == 0 && ctx->uvMeshInstances.isEmpty()) {
                XA_PRINT_WARNING("PackCharts: No meshes. Call AddMesh or AddUvMesh first.\n");
                return;
        }
        if (ctx->uvMeshInstances.isEmpty()) {
                if (!ctx->paramAtlas.chartsComputed()) {
                        XA_PRINT_WARNING("PackCharts: ComputeCharts must be called first.\n");
                        return;
                }
                if (!ctx->paramAtlas.chartsParameterized()) {
                        XA_PRINT_WARNING("PackCharts: ParameterizeCharts must be called first.\n");
                        return;
                }
        }
        if (packOptions.texelsPerUnit < 0.0f) {
                XA_PRINT_WARNING("PackCharts: PackOptions::texelsPerUnit is negative.\n");
                packOptions.texelsPerUnit = 0.0f;
        }
        // Cleanup atlas.
        DestroyOutputMeshes(ctx);
        if (atlas->utilization) {
                XA_FREE(atlas->utilization);
                atlas->utilization = nullptr;
        }
        if (atlas->image) {
                XA_FREE(atlas->image);
                atlas->image = nullptr;
        }
        atlas->meshCount = 0;
        // Pack charts.
        XA_PROFILE_START(packChartsAddCharts)
        internal::pack::Atlas packAtlas;
        if (!ctx->uvMeshInstances.isEmpty()) {
                for (uint32_t i = 0; i < ctx->uvMeshInstances.size(); i++)
                        packAtlas.addUvMeshCharts(ctx->uvMeshInstances[i]);
        }
        else
                packAtlas.addCharts(ctx->taskScheduler, &ctx->paramAtlas);
        XA_PROFILE_END(packChartsAddCharts)
        XA_PROFILE_START(packCharts)
        if (!packAtlas.packCharts(packOptions, ctx->progressFunc, ctx->progressUserData))
                return;
        XA_PROFILE_END(packCharts)
        // Populate atlas object with pack results.
        atlas->atlasCount = packAtlas.getNumAtlases();
        atlas->chartCount = packAtlas.getChartCount();
        atlas->width = packAtlas.getWidth();
        atlas->height = packAtlas.getHeight();
        atlas->texelsPerUnit = packAtlas.getTexelsPerUnit();
        if (atlas->atlasCount > 0) {
                atlas->utilization = XA_ALLOC_ARRAY(internal::MemTag::Default, float, atlas->atlasCount);
                for (uint32_t i = 0; i < atlas->atlasCount; i++)
                        atlas->utilization[i] = packAtlas.getUtilization(i);
        }
        if (packOptions.createImage) {
                atlas->image = XA_ALLOC_ARRAY(internal::MemTag::Default, uint32_t, atlas->atlasCount * atlas->width * atlas->height);
                for (uint32_t i = 0; i < atlas->atlasCount; i++)
                        packAtlas.getImages()[i]->copyTo(&atlas->image[atlas->width * atlas->height * i], atlas->width, atlas->height, packOptions.blockAlign ? 0 : packOptions.padding);
        }
        XA_PROFILE_PRINT_AND_RESET("   Total: ", packCharts)
        XA_PROFILE_PRINT_AND_RESET("      Add charts (real): ", packChartsAddCharts)
        XA_PROFILE_PRINT_AND_RESET("      Add charts (thread): ", packChartsAddChartsThread)
        XA_PROFILE_PRINT_AND_RESET("         Restore texcoords: ", packChartsAddChartsRestoreTexcoords)
        XA_PROFILE_PRINT_AND_RESET("      Rasterize: ", packChartsRasterize)
        XA_PROFILE_PRINT_AND_RESET("      Dilate (padding): ", packChartsDilate)
        XA_PROFILE_PRINT_AND_RESET("      Find location: ", packChartsFindLocation)
        XA_PROFILE_PRINT_AND_RESET("      Blit: ", packChartsBlit)
        XA_PRINT_MEM_USAGE
        XA_PRINT("Building output meshes\n");
        XA_PROFILE_START(buildOutputMeshes)
        int progress = 0;
        if (ctx->progressFunc) {
                if (!ctx->progressFunc(ProgressCategory::BuildOutputMeshes, 0, ctx->progressUserData))
                        return;
        }
        if (ctx->uvMeshInstances.isEmpty())
                atlas->meshCount = ctx->meshCount;
        else
                atlas->meshCount = ctx->uvMeshInstances.size();
        atlas->meshes = XA_ALLOC_ARRAY(internal::MemTag::Default, Mesh, atlas->meshCount);
        memset(atlas->meshes, 0, sizeof(Mesh) * atlas->meshCount);
        if (ctx->uvMeshInstances.isEmpty()) {
                uint32_t chartIndex = 0;
                for (uint32_t i = 0; i < ctx->meshCount; i++) {
                        Mesh &outputMesh = atlas->meshes[i];
                        // Count and alloc arrays. Ignore vertex mapped chart groups in Mesh::chartCount, since they're ignored faces.
                        for (uint32_t cg = 0; cg < ctx->paramAtlas.chartGroupCount(i); cg++) {
                                const internal::param::ChartGroup *chartGroup = ctx->paramAtlas.chartGroupAt(i, cg);
                                if (chartGroup->isVertexMap()) {
                                        outputMesh.vertexCount += chartGroup->mesh()->vertexCount();
                                        outputMesh.indexCount += chartGroup->mesh()->faceCount() * 3;
                                } else {
                                        for (uint32_t c = 0; c < chartGroup->chartCount(); c++) {
                                                const internal::param::Chart *chart = chartGroup->chartAt(c);
                                                outputMesh.vertexCount += chart->mesh()->vertexCount();
                                                outputMesh.indexCount += chart->mesh()->faceCount() * 3;
                                                outputMesh.chartCount++;
                                        }
                                }
                        }
                        outputMesh.vertexArray = XA_ALLOC_ARRAY(internal::MemTag::Default, Vertex, outputMesh.vertexCount);
                        outputMesh.indexArray = XA_ALLOC_ARRAY(internal::MemTag::Default, uint32_t, outputMesh.indexCount);
                        outputMesh.chartArray = XA_ALLOC_ARRAY(internal::MemTag::Default, Chart, outputMesh.chartCount);
                        XA_PRINT("   mesh %u: %u vertices, %u triangles, %u charts\n", i, outputMesh.vertexCount, outputMesh.indexCount / 3, outputMesh.chartCount);
                        // Copy mesh data.
                        uint32_t firstVertex = 0, meshChartIndex = 0;
                        for (uint32_t cg = 0; cg < ctx->paramAtlas.chartGroupCount(i); cg++) {
                                const internal::param::ChartGroup *chartGroup = ctx->paramAtlas.chartGroupAt(i, cg);
                                if (chartGroup->isVertexMap()) {
                                        const internal::Mesh *mesh = chartGroup->mesh();
                                        // Vertices.
                                        for (uint32_t v = 0; v < mesh->vertexCount(); v++) {
                                                Vertex &vertex = outputMesh.vertexArray[firstVertex + v];
                                                vertex.atlasIndex = -1;
                                                vertex.chartIndex = -1;
                                                vertex.uv[0] = vertex.uv[1] = 0.0f;
                                                vertex.xref = chartGroup->mapVertexToSourceVertex(v);
                                        }
                                        // Indices.
                                        for (uint32_t f = 0; f < mesh->faceCount(); f++) {
                                                const uint32_t indexOffset = chartGroup->mapFaceToSourceFace(f) * 3;
                                                for (uint32_t j = 0; j < 3; j++)
                                                        outputMesh.indexArray[indexOffset + j] = firstVertex + mesh->vertexAt(f * 3 + j);
                                        }
                                        firstVertex += mesh->vertexCount();
                                } else {
                                        for (uint32_t c = 0; c < chartGroup->chartCount(); c++) {
                                                const internal::param::Chart *chart = chartGroup->chartAt(c);
                                                const internal::Mesh *mesh = chart->mesh();
                                                // Vertices.
                                                for (uint32_t v = 0; v < mesh->vertexCount(); v++) {
                                                        Vertex &vertex = outputMesh.vertexArray[firstVertex + v];
                                                        vertex.atlasIndex = packAtlas.getChart(chartIndex)->atlasIndex;
                                                        XA_DEBUG_ASSERT(vertex.atlasIndex >= 0);
                                                        vertex.chartIndex = (int32_t)chartIndex;
                                                        const internal::Vector2 &uv = mesh->texcoord(v);
                                                        vertex.uv[0] = internal::max(0.0f, uv.x);
                                                        vertex.uv[1] = internal::max(0.0f, uv.y);
                                                        vertex.xref = chartGroup->mapVertexToSourceVertex(chart->mapChartVertexToOriginalVertex(v));
                                                }
                                                // Indices.
                                                for (uint32_t f = 0; f < mesh->faceCount(); f++) {
                                                        const uint32_t indexOffset = chartGroup->mapFaceToSourceFace(chart->mapFaceToSourceFace(f)) * 3;
                                                        for (uint32_t j = 0; j < 3; j++)
                                                                outputMesh.indexArray[indexOffset + j] = firstVertex + mesh->vertexAt(f * 3 + j);
                                                }
                                                // Charts.
                                                Chart *outputChart = &outputMesh.chartArray[meshChartIndex];
                                                const int32_t atlasIndex = packAtlas.getChart(chartIndex)->atlasIndex;
                                                XA_DEBUG_ASSERT(atlasIndex >= 0);
                                                outputChart->atlasIndex = (uint32_t)atlasIndex;
                                                outputChart->type = chart->type();
                                                outputChart->faceCount = mesh->faceCount();
                                                outputChart->faceArray = XA_ALLOC_ARRAY(internal::MemTag::Default, uint32_t, outputChart->faceCount);
                                                for (uint32_t f = 0; f < outputChart->faceCount; f++)
                                                        outputChart->faceArray[f] = chartGroup->mapFaceToSourceFace(chart->mapFaceToSourceFace(f));
                                                outputChart->material = 0;
                                                meshChartIndex++;
                                                chartIndex++;
                                                firstVertex += mesh->vertexCount();
                                        }
                                }
                        }
                        XA_DEBUG_ASSERT(outputMesh.vertexCount == firstVertex);
                        XA_DEBUG_ASSERT(outputMesh.chartCount == meshChartIndex);
                        if (ctx->progressFunc) {
                                const int newProgress = int((i + 1) / (float)atlas->meshCount * 100.0f);
                                if (newProgress != progress) {
                                        progress = newProgress;
                                        if (!ctx->progressFunc(ProgressCategory::BuildOutputMeshes, progress, ctx->progressUserData))
                                                return;
                                }
                        }
                }
        } else {
                uint32_t chartIndex = 0;
                for (uint32_t m = 0; m < ctx->uvMeshInstances.size(); m++) {
                        Mesh &outputMesh = atlas->meshes[m];
                        const internal::UvMeshInstance *mesh = ctx->uvMeshInstances[m];
                        // Alloc arrays.
                        outputMesh.vertexCount = mesh->texcoords.size();
                        outputMesh.indexCount = mesh->mesh->indices.size();
                        outputMesh.chartCount = mesh->mesh->charts.size();
                        outputMesh.vertexArray = XA_ALLOC_ARRAY(internal::MemTag::Default, Vertex, outputMesh.vertexCount);
                        outputMesh.indexArray = XA_ALLOC_ARRAY(internal::MemTag::Default, uint32_t, outputMesh.indexCount);
                        outputMesh.chartArray = XA_ALLOC_ARRAY(internal::MemTag::Default, Chart, outputMesh.chartCount);
                        XA_PRINT("   UV mesh %u: %u vertices, %u triangles, %u charts\n", m, outputMesh.vertexCount, outputMesh.indexCount / 3, outputMesh.chartCount);
                        // Copy mesh data.
                        // Vertices.
                        for (uint32_t v = 0; v < mesh->texcoords.size(); v++) {
                                Vertex &vertex = outputMesh.vertexArray[v];
                                vertex.uv[0] = mesh->texcoords[v].x;
                                vertex.uv[1] = mesh->texcoords[v].y;
                                vertex.xref = v;
                                const uint32_t meshChartIndex = mesh->mesh->vertexToChartMap[v];
                                if (meshChartIndex == UINT32_MAX) {
                                        // Vertex doesn't exist in any chart.
                                        vertex.atlasIndex = -1;
                                        vertex.chartIndex = -1;
                                } else {
                                        const internal::pack::Chart *chart = packAtlas.getChart(chartIndex + meshChartIndex);
                                        vertex.atlasIndex = chart->atlasIndex;
                                        vertex.chartIndex = (int32_t)chartIndex + meshChartIndex;
                                }
                        }
                        // Indices.
                        memcpy(outputMesh.indexArray, mesh->mesh->indices.data(), mesh->mesh->indices.size() * sizeof(uint32_t));
                        // Charts.
                        for (uint32_t c = 0; c < mesh->mesh->charts.size(); c++) {
                                Chart *outputChart = &outputMesh.chartArray[c];
                                const internal::pack::Chart *chart = packAtlas.getChart(chartIndex);
                                XA_DEBUG_ASSERT(chart->atlasIndex >= 0);
                                outputChart->atlasIndex = (uint32_t)chart->atlasIndex;
                                outputChart->faceCount = chart->faces.size();
                                outputChart->faceArray = XA_ALLOC_ARRAY(internal::MemTag::Default, uint32_t, outputChart->faceCount);
                                outputChart->material = chart->material;
                                for (uint32_t f = 0; f < outputChart->faceCount; f++)
                                        outputChart->faceArray[f] = chart->faces[f];
                                chartIndex++;
                        }
                        if (ctx->progressFunc) {
                                const int newProgress = int((m + 1) / (float)atlas->meshCount * 100.0f);
                                if (newProgress != progress) {
                                        progress = newProgress;
                                        if (!ctx->progressFunc(ProgressCategory::BuildOutputMeshes, progress, ctx->progressUserData))
                                                return;
                                }
                        }
                }
        }
        if (ctx->progressFunc && progress != 100)
                ctx->progressFunc(ProgressCategory::BuildOutputMeshes, 100, ctx->progressUserData);
        XA_PROFILE_END(buildOutputMeshes)
        XA_PROFILE_PRINT_AND_RESET("   Total: ", buildOutputMeshes)
        XA_PRINT_MEM_USAGE
}

void Generate(Atlas *atlas, ChartOptions chartOptions, ParameterizeFunc paramFunc, PackOptions packOptions)
{
        if (!atlas) {
                XA_PRINT_WARNING("Generate: atlas is null.\n");
                return;
        }
        Context *ctx = (Context *)atlas;
        if (!ctx->uvMeshInstances.isEmpty()) {
                XA_PRINT_WARNING("Generate: This function should not be called with UV meshes.\n");
                return;
        }
        if (ctx->meshCount == 0) {
                XA_PRINT_WARNING("Generate: No meshes. Call AddMesh first.\n");
                return;
        }
        ComputeCharts(atlas, chartOptions);
        ParameterizeCharts(atlas, paramFunc);
        PackCharts(atlas, packOptions);
}

void SetProgressCallback(Atlas *atlas, ProgressFunc progressFunc, void *progressUserData)
{
        if (!atlas) {
                XA_PRINT_WARNING("SetProgressCallback: atlas is null.\n");
                return;
        }
        Context *ctx = (Context *)atlas;
        ctx->progressFunc = progressFunc;
        ctx->progressUserData = progressUserData;
}

void SetAlloc(ReallocFunc reallocFunc, FreeFunc freeFunc)
{
        internal::s_realloc = reallocFunc;
        internal::s_free = freeFunc;
}

void SetPrint(PrintFunc print, bool verbose)
{
        internal::s_print = print;
        internal::s_printVerbose = verbose;
}

const char *StringForEnum(AddMeshError::Enum error)
{
        if (error == AddMeshError::Error)
                return "Unspecified error";
        if (error == AddMeshError::IndexOutOfRange)
                return "Index out of range";
        if (error == AddMeshError::InvalidIndexCount)
                return "Invalid index count";
        return "Success";
}

const char *StringForEnum(ProgressCategory::Enum category)
{
        if (category == ProgressCategory::AddMesh)
                return "Adding mesh(es)";
        if (category == ProgressCategory::ComputeCharts)
                return "Computing charts";
        if (category == ProgressCategory::ParameterizeCharts)
                return "Parameterizing charts";
        if (category == ProgressCategory::PackCharts)
                return "Packing charts";
        if (category == ProgressCategory::BuildOutputMeshes)
                return "Building output meshes";
        return "";
}

} // namespace xatlas
