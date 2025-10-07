/**
 * Interface between OpenNL and the AMGCL solver.
 * Works both on the CPU and the GPU.
 */

#include <geogram/NL/nl.h>

extern "C" {
#include <geogram/NL/nl_amgcl.h>
#include <geogram/NL/nl_context.h>
#include <geogram/NL/nl_cuda.h>
}

#include <geogram/basic/logger.h>
#include <geogram/basic/stopwatch.h>
#include <geogram/basic/command_line.h>
#include <geogram/NL/nl.h>
#include <geogram/NL/nl_matrix.h>

#ifdef NL_WITH_AMGCL

/*************************************************************************/

// switch off some warnings else it complains too much when
// compiling AMGCL code.
#ifdef __GNUC__
#ifndef __ICC
#pragma GCC diagnostic ignored "-Wsign-conversion"
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wdouble-promotion"
#pragma GCC diagnostic ignored "-Wfloat-conversion"
#pragma GCC diagnostic ignored "-Wsign-compare"
#endif
#endif

#ifdef __clang__
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wshadow-field-in-constructor"
#pragma GCC diagnostic ignored "-Wsource-uses-openmp"
#pragma GCC diagnostic ignored "-Wcast-qual"
#pragma GCC diagnostic ignored "-Wzero-as-null-pointer-constant"
#pragma GCC diagnostic ignored "-Wconditional-uninitialized"
#pragma GCC diagnostic ignored "-Wunused-template"
#endif

#ifdef GEO_COMPILER_MSVC
#pragma warning( disable : 4244 4018 4458 4267 4701 )
#endif

//#define WITH_BOOST
// AMGCL has a runtime that lets you dynamically select
// the preconditioner, iterative solver etc..., however
// it depends on BOOST (a super heavy dependency), so we
// use hardwired settings here to avoid this dependency.


#ifdef WITH_BOOST
# include <boost/property_tree/ptree.hpp>
# include <boost/property_tree/json_parser.hpp>
# include <amgcl/preconditioner/runtime.hpp>
# include <amgcl/solver/runtime.hpp>
#else
# define AMGCL_NO_BOOST
# include <amgcl/amg.hpp>
# include <amgcl/coarsening/smoothed_aggregation.hpp>
# include <amgcl/relaxation/spai0.hpp>
# include <amgcl/solver/cg.hpp>

#endif

#include <type_traits>
#include <amgcl/make_solver.hpp>
#include <amgcl/adapter/zero_copy.hpp>
#include <amgcl/solver/skyline_lu.hpp> // for cuda backend

# include <amgcl/preconditioner/dummy.hpp>


#ifdef AMGCL_PROFILING
#include <amgcl/profiler.hpp>
namespace amgcl {
    profiler<> prof;
}
#endif

/*************************************************************************/

// Types for column index and row pointers, as expected
// by AMGCL. Note: AMGCL prefers signed indices.
// In standard mode,  32-bit indices everywhere
// In GARGANTUA mode, 64-bit indices for row pointers
//               and  32-bit indices for column indices
// (in GARGANTUA mode, NNZ can be larger than 4 billions).
typedef int colind_t;
#ifdef GARGANTUA
typedef ptrdiff_t rowptr_t;
#else
typedef int rowptr_t;
#endif


/*************************************************************************/
/**************** AMGCL backend using OpenNL CUDA interface **************/
/*************************************************************************/

/**
 * \brief Adapters around OpenNL CUDA interface used by AMGCL backend
 */

namespace amgcl2nl {
    typedef double   value_type;
    typedef colind_t col_type;
    typedef rowptr_t ptr_type;
    typedef colind_t index_type;

    /**
     * \brief a vector, stored in device memory (default) or pinned host memory.
     */
    struct vector {
	typedef double value_type;

	/**
	 * \brief vector constructor from size
	 * \param[in] mem_type one of NL_HOST_MEMORY, NL_DEVICE_MEMORY
	 * \details vector is initialized to zero
	 */
	vector(index_type n, NLmemoryType mem_type = NL_DEVICE_MEMORY) {
	    mem_type_ = mem_type;
	    n_ = n;
	    data_ = NL_NEW_VECTOR(nlCUDABlas(), mem_type_, n_);
	    temp_ = nullptr;
	    clear(); // TODO: check whether it is necessary to clear.
	}

	/**
	 * \brief vector constructor from data on host and size
	 * \param[in] mem_type one of NL_HOST_MEMORY, NL_DEVICE_MEMORY
	 */
	vector(
	    const value_type* x_on_host, index_type n,
	    NLmemoryType mem_type = NL_DEVICE_MEMORY
	) {
	    mem_type_ = mem_type;
	    n_ = n;
	    data_ = NL_NEW_VECTOR(nlCUDABlas(), mem_type_, n_);
	    temp_ = nullptr;
	    copy_from_host(x_on_host, n);
	}

	/**
	 * \brief vector destructor
	 */
	~vector() {
	    if(data_ != nullptr) {
		NL_DELETE_VECTOR(nlCUDABlas(), mem_type_, n_, data_);
	    }
	    n_ = 0;
	    data_ = nullptr;
	}

	/**
	 * \brief Forbids copy
	 */
	vector(const vector& rhs) = delete;

	/**
	 * \brief Forbids copy
	 */
	vector& operator=(const vector& rhs) = delete;

	/**
	 * \brief Gets the size of the vector
	 * \return the number of elements in the vector
	 */
	index_type size() const {
	    return n_;
	}

	/**
	 * \brief Gets the memory type where vector data resides
	 * \return one of NL_HOST_MEMORY, NL_DEVICE_MEMORY
	 */
	NLmemoryType mem_type() const {
	    return mem_type_;
	}

	double& operator[](index_type i) {
	    nl_debug_assert(i < n_);
	    nl_debug_assert(mem_type_ == NL_HOST_MEMORY);
	    return data_[i];
	}

	const double& operator[](index_type i) const {
	    nl_debug_assert(i < n_);
	    nl_debug_assert(mem_type_ == NL_HOST_MEMORY);
	    return data_[i];
	}

	/**
	 * \brief Gets a const device pointer to the data
	 * \return a pointer to the stored data on the GPU
	 */
	const double* data() const {
	    return data_;
	}

	/**
	 * \brief Gets a device pointer to the data
	 * \return a pointer to the stored data on the GPU
	 */
	double* data() {
	    return data_;
	}

	/**
	 * \brief Gets the temporary vector.
	 * \details Each vector optionally has a temporary work
	 *  space of the same size as the vector. First call to
	 *  this function creates the temorary space. If temporary
	 *  space was created, it is deallocated when the vector
	 *  is destroyed.
	 * \return a shared_ptr to the temporary vector.
	 */
	std::shared_ptr<vector> temp() const {
	    if(temp_ == nullptr) {
		temp_ = std::shared_ptr<vector>(new vector(n_));
	    }
	    return temp_;
	}

	/**
	 * \brief Sets all coefficients to zero
	 */
	void clear() {
	    NLBlas_t blas = nlCUDABlas();
	    blas->Memset(blas, data_, mem_type_, 0, bytes());
	}

	/**
	 * \brief Gets the size of the allocated memory
	 * \return the size of the allocated memory on the GPU, in bytes
	 */
	size_t bytes() const {
	    return sizeof(value_type) * size_t(n_);
	}

	void copy_from_host(const value_type* x_on_host, index_type n) {
	    nl_arg_used(n);
	    nl_debug_assert(n == n_);
	    NLBlas_t blas = nlCUDABlas();
	    blas->Memcpy(
		blas,
		data_, mem_type_,
		const_cast<double*>(x_on_host), NL_HOST_MEMORY,
		bytes()
	    );
	}

	void copy_from_host(const std::vector<value_type>& x_on_host) {
	    copy_from_host(x_on_host.data(), index_type(x_on_host.size()));
	}

	void copy_to_host(value_type* x_on_host, index_type n) const {
	    nl_arg_used(n);
	    nl_debug_assert(n == n_);
	    NLBlas_t blas = nlCUDABlas();
	    blas->Memcpy(
		blas,
		x_on_host, NL_HOST_MEMORY,
		data_, mem_type_,
		bytes()
	    );
	}

	void copy_to_host(std::vector<value_type>& x_on_host) const {
	    copy_to_host(x_on_host.data(), index_type(x_on_host.size()));
	}

	void copy_from_device(const value_type* x_on_device, index_type n) {
	    nl_arg_used(n);
	    nl_debug_assert(n == n_);
	    NLBlas_t blas = nlCUDABlas();
	    blas->Memcpy(
		blas,
		data_, mem_type_,
		const_cast<double*>(x_on_device), NL_DEVICE_MEMORY,
		bytes()
	    );
	}

	void copy_from(const vector& rhs) {
	    nl_debug_assert(rhs.n_ == n_);
	    if(rhs.mem_type() == NL_DEVICE_MEMORY) {
		copy_from_device(rhs.data_, rhs.n_);
	    } else {
		copy_from_host(rhs.data_, rhs.n_);
	    }
	}

	/**
	 * \brief computes the dot product between two vectors
	 */
	static double dot(const vector& x, const vector& y) {
	    nl_debug_assert(x.size() == y.size());
	    nl_debug_assert(x.mem_type() == NL_DEVICE_MEMORY);
	    nl_debug_assert(y.mem_type() == NL_DEVICE_MEMORY);
	    NLBlas_t blas = nlCUDABlas();
	    return blas->Ddot(blas,x.n_,x.data_,1,y.data_,1);
	}


	/**
	 * \brief \f$ x \leftarrow a x \f$
	 */
	static void scal(double a, const vector& x) {
	    nl_debug_assert(x.mem_type() == NL_DEVICE_MEMORY);
	    NLBlas_t blas = nlCUDABlas();
	    blas->Dscal(blas, x.n_, a, x.data_, 1);
	}

	/**
	 * \brief \f$ y \leftarrow a x + y \f$
	 */
	static void axpy(double a, const vector& x, vector& y) {
	    nl_debug_assert(x.size() == y.size());
	    nl_debug_assert(x.mem_type() == NL_DEVICE_MEMORY);
	    nl_debug_assert(y.mem_type() == NL_DEVICE_MEMORY);
	    NLBlas_t blas = nlCUDABlas();
	    blas->Daxpy(blas, x.n_, a, x.data_, 1, y.data_, 1);
	}

	/**
	 * \brief \f$ y \leftarrow M x + y \f$
	 * \details \p M is a diagonal matrix stored in a vector
	 */
	static void mul(const vector& M, const vector& x, vector& y) {
	    nl_debug_assert(x.size() == M.size() && y.size() == M.size());
	    nl_debug_assert(M.mem_type() == NL_DEVICE_MEMORY);
	    nl_debug_assert(x.mem_type() == NL_DEVICE_MEMORY);
	    nl_debug_assert(y.mem_type() == NL_DEVICE_MEMORY);
	    NLBlas_t blas = nlCUDABlas();
	    blas->Dmul(blas,M.size(),M.data_,x.data_,y.data_);
	}

    private:
	double* data_;
	index_type n_;
	NLmemoryType mem_type_;
	mutable std::shared_ptr<vector> temp_; // temporary vector for vmul()
    };

    typedef vector matrix_diagonal;

    /**
     * \brief a matrix, stored in device memory (GPU)
     */
    class matrix {
    public:
	typedef double value_type;

	/**
	 * \brief matrix constructor from CRS representation
	 * \param[in] m number of rows
	 * \param[in] n number of columns
	 * \param[in] rowptr CRS row pointers, on host
	 * \param[in] colind CRS column indices, on host
	 * \param[in] val CRS coefficient values, on host
	 */
	matrix(
	    NLuint m, NLuint n,
	    const NLuint_big* rowptr, const NLuint* colind, const double* val
	) {
	    // Create an NLCRSMatrix with rowptr, colind and val arrays
	    NLCRSMatrix CRS;
	    memset(&CRS, 0, sizeof(NLCRSMatrix));
	    CRS.type = NL_MATRIX_CRS;
	    CRS.symmetric_storage = NL_FALSE;
	    CRS.m = m;
	    CRS.n = n;
	    CRS.rowptr = const_cast<NLuint_big*>(rowptr);
	    CRS.colind = const_cast<NLuint*>(colind);
	    CRS.val = const_cast<double*>(val);
	    CRS.nslices = 0;
	    CRS.sliceptr = nullptr;
	    ptr_type nnz = rowptr[CRS.m];
	    bytes_ = nnz * (sizeof(value_type) + sizeof(col_type)) +
		(CRS.m+1) * sizeof(ptr_type);
	    impl_ = nlCUDAMatrixNewFromCRSMatrix(NLMatrix(&CRS));
	}

	/**
	 * \brief matrix destructor
	 */
	~matrix() {
	    nlDeleteMatrix(impl_);
	    impl_ = nullptr;
	}

	/**
	 * \brief Forbids copy
	 */
	matrix(const matrix& rhs) = delete;

	/**
	 * \brief Forbids copy
	 */
	matrix& operator=(const matrix& rhs) = delete;

	/**
	 * \brief sparse matrix-vector product
	 * \details \f$ y \leftarrow alpha A x + beta y \f$
	 */
	void SpMV(
	    const vector& x, vector& y, double alpha=1.0, double beta=0.0
	) const {
	    nlCUDAMatrixSpMV(impl_, x.data(), y.data(), alpha, beta);
	}

	/**
	 * \brief Gets the size of the allocated memory
	 * \return the size of the allocated memory on the GPU, in bytes
	 */
	size_t bytes() const {
	    return bytes_;
	}

    private:
	NLMatrix impl_;
	size_t bytes_;
    };

    /**
     * Wrapper around solver::skyline_lu for use with the NLCUDA backend.
     * Inspired from AMGCL cuda wrapper
     * Copies the rhs to the host memory, solves the problem using the host CPU,
     * then copies the solution back to the compute device.
     */
    struct cuda_skyline_lu : amgcl::solver::skyline_lu<value_type> {
	typedef amgcl::solver::skyline_lu<value_type> Base;

	template <class Matrix, class Params> cuda_skyline_lu(
	    const Matrix &A, const Params&
	): Base(*A),
	   rhs_on_host_(amgcl::backend::rows(*A), NL_HOST_MEMORY),
	   x_on_host_(amgcl::backend::rows(*A), NL_HOST_MEMORY) {
	}

	void operator()(const vector &rhs_on_device, vector &x_on_device) const {
	    rhs_on_host_.copy_from(rhs_on_device);
	    static_cast<const Base*>(this)->operator()(
		rhs_on_host_, x_on_host_
	    );
	    x_on_device.copy_from(x_on_host_);
	}

	mutable amgcl2nl::vector rhs_on_host_;
	mutable amgcl2nl::vector x_on_host_;
    };
}

namespace amgcl { namespace backend {

    /**
     * \brief nlcuda backend for AMGCL
     */
    struct nlcuda {
	typedef amgcl2nl::value_type value_type;
	typedef amgcl2nl::col_type col_type;
	typedef amgcl2nl::ptr_type ptr_type;
	typedef amgcl2nl::index_type index_type;

	typedef amgcl2nl::matrix matrix;
	typedef amgcl2nl::matrix_diagonal matrix_diagonal;
	typedef amgcl2nl::vector vector;

	typedef amgcl2nl::cuda_skyline_lu direct_solver;

	struct provides_row_iterator : std::false_type {};

	typedef amgcl::detail::empty_params params;

	static std::string name() { return "nlcuda"; }

	typedef builtin<value_type, col_type, ptr_type> builtin_backend;

	/**
	 * \brief copies a matrix from the builtin backend to this backend
	 * \param[in] A a shared_ptr to the builtin matrix
	 * \param[in] param a const reference to the parameters, unused in
	 *  this backend
	 * \return a shared_ptr to the matrix, of type amgcl2nl::matrix
	 */
	static std::shared_ptr<matrix> copy_matrix(
	    std::shared_ptr<typename builtin_backend::matrix> A,
	    const params& param
	) {
	    nl_arg_used(param);
	    const typename builtin_backend::matrix &a = *A;

	    // Sanity checks: signedness can differ, but sizes should
	    // be the same !
	    static_assert(sizeof(NLuint_big) == sizeof(ptr_type));
	    static_assert(sizeof(NLuint) == sizeof(col_type));
	    static_assert(sizeof(double) == sizeof(value_type));

	    return std::shared_ptr<matrix>(
		new matrix(
		    NLuint(a.nrows), NLuint(a.ncols),
		    reinterpret_cast<const NLuint_big*>(a.ptr),
		    reinterpret_cast<NLuint*>(a.col),
		    a.val
		)
	    );
	}

	/**
	 * \brief copies a vector from the builtin backend to this backend
	 * \param[in] x a reference to the builtin vector
	 * \param[in] param a const reference to the parameters, unused in
	 *  this backend
	 * \return a shared_ptr to the vector, of type amgcl2nl::vector
	 */
	static std::shared_ptr<vector> copy_vector(
	    typename builtin_backend::vector const &x, const params& param
	) {
	    nl_arg_used(param);
	    return std::make_shared<vector>(x.data(), x.size());
	}

	/**
	 * \brief copies a vector from the builtin backend to CUDA
	 * \param[in] x a shared_ptr to the builtin vector
	 * \param[in] param a const reference to the parameters, unused in
	 *  this backend
	 * \return a shared_ptr to the vector, of type amgcl2nl::vector
	 */
	static std::shared_ptr<vector> copy_vector(
	    std::shared_ptr< typename builtin_backend::vector > x,
	    const params &param
	) {
	    return copy_vector(*x, param);
	}

	/**
	 * \brief creates a vector in this backend, in CUDA memory
	 * \param[in] size the size of this vector (number of components)
	 * \param[in] param a const reference to the parameters, unused in
	 *  this backend
	 * \return a shared_ptr to the vector, of type amgcl2nl::vector
	 */
	static std::shared_ptr<vector> create_vector(
	    size_t size, const params& param
	) {
	    nl_arg_used(param);
	    return std::make_shared<vector>(size);
	}

	/**
	 * \brief creates a direct solver
	 * \param[in] A a shared_ptr to a matrix in the builtin backend
	 * \param[in] param a const reference to the parameters, unused in
	 *  this backend
	 * \return a shared_ptr to a amgcl2nl::cuda_skyline_lu, a direct
	 *  direct solver that can solve linear systems with vectors
	 *  stored in this backend, in GPU memory (copies data back and
	 *  forth).
	 */
	static std::shared_ptr<direct_solver> create_solver(
	    std::shared_ptr<typename builtin_backend::matrix > A,
	    const params &prm
	) {
	    return std::make_shared<direct_solver>(A, prm);
	}
    };

   /****************************************************************/

   // AMGCL backend using OpenNL's CUDA interface: specializations

   // In AMGCL, these are templated structs with a single static function
   // because one cannot use partial specialization with functions

    /**
     * \brief gets the size of a vector in bytes
     */
    template <> struct bytes_impl< amgcl2nl::vector > {
	static size_t get(const amgcl2nl::vector &v) {
	    return v.bytes();
	}
    };

    /**
     * \brief \f$ y \leftarrow \alpha A x + \beta y \f$
     */
    template <> struct spmv_impl<
	double,	amgcl2nl::matrix, amgcl2nl::vector, double, amgcl2nl::vector
    > {
	static void apply(
	    double alpha, const amgcl2nl::matrix& A, const amgcl2nl::vector& x,
	    double beta, amgcl2nl::vector& y
	) {
	    A.SpMV(x,y,alpha,beta);
	}
    };

    /**
     * \brief \f$ r \leftarrow rhs - A x \f$
     */
    template <> struct residual_impl<
	amgcl2nl::matrix, amgcl2nl::vector, amgcl2nl::vector, amgcl2nl::vector
    > {
	static void apply(
	    const amgcl2nl::vector &rhs, const amgcl2nl::matrix &A,
	    const amgcl2nl::vector &x, amgcl2nl::vector &r
	) {
	    // r <- rhs - A x
	    r.copy_from(rhs);
	    A.SpMV(x,r,-1.0,1.0);
	}
    };

    /**
     * \brief \f$ x \leftarrow 0 \f$
     */
    template <> struct clear_impl<amgcl2nl::vector> {
	static void apply(amgcl2nl::vector& x) {
	    x.clear();
	}
    };

    /**
     * \brief computes the dot product \f$ x \cdot y \f$
     */
    template <> struct inner_product_impl<amgcl2nl::vector, amgcl2nl::vector> {
	static double get(const amgcl2nl::vector& x, const amgcl2nl::vector& y) {
	    return amgcl2nl::vector::dot(x,y);
	}
    };

    /**
     * \brief \f$ y \leftarrow a x + b y \f$
     */
    template <> struct axpby_impl<
	double, amgcl2nl::vector, double, amgcl2nl::vector
    > {
	static void apply(
	    double a, const amgcl2nl::vector& x,
	    double b, amgcl2nl::vector& y
	) {
	    if(b != 1.0){
		amgcl2nl::vector::scal(b,y);
	    }
	    amgcl2nl::vector::axpy(a,x,y);
	}
    };

    /**
     * \brief \f$ z \leftarrow a*x + b*y + c*z \f$
     * \details Unused here (the combination of solver and preconditioner
     *  used here does not instanciate this function). Kept just in case.
     */
    template <> struct axpbypcz_impl<
	double,	amgcl2nl::vector, double, amgcl2nl::vector,
	double, amgcl2nl::vector
    > {
	static void apply(
            double a, const amgcl2nl::vector &x,
            double b, const amgcl2nl::vector &y,
            double c,       amgcl2nl::vector &z
	) {
	    // z <- a*x + b*y + c*z
	    if(c != 1.0) {
		amgcl2nl::vector::scal(c,z);
	    }
	    amgcl2nl::vector::axpy(a,x,z);
	    amgcl2nl::vector::axpy(b,y,z);
	}
    };

    /**
     * \brief \f$ z \leftarrow a M y + b z \f$
     * \details M is a diagonal matrix stored in a vector
     */
    template <> struct vmul_impl<
	double,	amgcl2nl::matrix_diagonal, amgcl2nl::vector,
	double,	amgcl2nl::vector
    > {
	static void apply(
	    double a, const amgcl2nl::matrix_diagonal &M,
	    const amgcl2nl::vector &y, double b, amgcl2nl::vector &z
	) {
	    // z <- a * M * y + b * z

	    if(b != 0.0) {
		// tmp <- z
		M.temp()->copy_from(z);
	    }

	    // z <- a * M * y
	    amgcl2nl::vector::mul(M,y,z);
	    if(a != 1.0) {
		amgcl2nl::vector::scal(a,z);
	    }

	    if(b != 0.0) {
		// z <- b * tmp + z
		amgcl2nl::vector::axpy(b,*M.temp(),z);
	    }
	}
    };

    /**
     * \brief copies a vector from nlcuda backend to nlcuda backend
     * \details order of arguments: from, to
     */
    template <> struct copy_impl<amgcl2nl::vector, amgcl2nl::vector> {
	static void apply(const amgcl2nl::vector &x, amgcl2nl::vector &y) {
	    y.copy_from(x);
	}
    };

    /**
     * \brief copies a vector from builtin backend to nlcuda backend
     * \details order of arguments: from, to
     */
    template <> struct copy_impl<std::vector<double>,amgcl2nl::vector> {
	static void apply(const std::vector<double> &x, amgcl2nl::vector &y) {
	    y.copy_from_host(x.data(),x.size());
	}
    };

    /**
     * \brief copies a vector from nlcuda backend to builtin backend
     * \details order of arguments: from, to
     */
    template <> struct copy_impl<amgcl2nl::vector,std::vector<double> > {
	static void apply(const amgcl2nl::vector &x, std::vector<double> &y) {
	    x.copy_to_host(y.data(),y.size());
	}
    };

}}

/*************************************************************************/
/***** Generic function to call AMGCL on OpenNL system (CPU and GPU) *****/
/*************************************************************************/

/**
 * \brief Wrapper around solver to use rhs and x directly (CPU) or copy
 *  them to/from GPU memory.
 * \details Nothing in this version, it is specialized below.
 */
template <class Backend> struct solve_linear_system_impl {
};

/**
 * \brief Wrapper around solver that uses rhs and x directly, on the CPU.
 */
template <> struct solve_linear_system_impl<
    amgcl::backend::builtin<double, colind_t, rowptr_t>
> {
    template <class Solver> static std::tuple<size_t, double> apply(
	Solver& solve, size_t n, double* b, double* x
    ) {
        return solve(
            amgcl::make_iterator_range(b, b + n),
            amgcl::make_iterator_range(x, x + n)
        );
    }
};

/**
 * \brief Wrapper around solver that copy rhs and x to/from GPU memory.
 */
template <> struct solve_linear_system_impl<amgcl::backend::nlcuda> {
    template <class Solver> static std::tuple<size_t, double> apply(
	Solver& solve, size_t n, double* b, double* x
    ) {
	amgcl2nl::vector b_cuda(b, n);
	amgcl2nl::vector x_cuda(x, n);
	std::tuple<size_t, double> result = solve(b_cuda,x_cuda);
	x_cuda.copy_to_host(x,n);
	return result;
    }
};


/**
 * \brief Solves the linear system in the current OpenNL context using AMGCL
 * \tparam Backend a AMGCL backend, can be one of
 *   - amgcl::backend::builtin<double, colind_t, rowptr_t> (CPU)
 *   - amgcl::backend::nlcuda (GPU)
 *  to support other backends, one needs to write a specialization
 *  of solve_linear_system_impl
 */
template <class Backend> NLboolean nlSolveAMGCL_generic() {

#ifdef WITH_BOOST
    typedef amgcl::runtime::preconditioner<Backend> Precond;
    typedef amgcl::runtime::solver::wrapper<Backend> IterativeSolver;
    typedef amgcl::make_solver<Precond,IterativeSolver> Solver;
    typedef boost::property_tree::ptree Params;
#else
    typedef amgcl::amg<
	Backend,
	amgcl::coarsening::smoothed_aggregation, amgcl::relaxation::spai0
    > Precond;
    typedef amgcl::solver::cg<Backend> IterativeSolver;
    typedef amgcl::make_solver<Precond,IterativeSolver> Solver;
    typedef typename Solver::params Params;
#endif

    // Get linear system to solve from OpenNL context
    NLContextStruct* ctxt = (NLContextStruct*)nlGetCurrent();

    if(ctxt->verbose) {
        GEO::Logger::out("AMGCL") << "calling AMGCL solver (built in geogram) "
				  << "(" << Backend::name() << ")"
				  << std::endl;
    }

    nlBlasResetStats(nlCUDABlas());


    if(ctxt->M->type == NL_MATRIX_SPARSE_DYNAMIC) {
        if(ctxt->verbose) {
            GEO::Logger::out("AMGCL") << "Compressing matrix" << std::endl;
        }
        nlMatrixCompress(&ctxt->M);
    }

    nl_assert(ctxt->M->type == NL_MATRIX_CRS);
    NLCRSMatrix* M = (NLCRSMatrix*)(ctxt->M);
    size_t n = size_t(M->m);
    NLdouble* b = ctxt->b;
    NLdouble* x = ctxt->x;

    Params prm;

    // Initialize AMGCL parameters from OpenNL context.
    // solver.type: "cg" for symmetric, else the other ones ("bicgstab"...)
    // solver.tol: converged as soon as || Ax - b || / || b || < solver.tol
    // solver.maxiter: or stop if using more than solver.maxiter
    // solver.verbose: display || Ax - b || / || b || every 5 iterations
#ifdef WITH_BOOST
    prm.put("solver.type", "cg");
    prm.put("solver.tol", float(ctxt->threshold));
    prm.put("solver.maxiter", int(ctxt->max_iterations));
    prm.put("solver.verbose", int(ctxt->verbose));
#else
    prm.solver.tol = float(ctxt->threshold);
    prm.solver.maxiter = int(ctxt->max_iterations);
    prm.solver.verbose = int(ctxt->verbose);
#endif

    // using the zero-copy interface of AMGCL
    auto M_amgcl = amgcl::adapter::zero_copy_direct(
        size_t(n), (rowptr_t*)M->rowptr, (colind_t *)M->colind, M->val
    );

    GEO::Stopwatch* Wbuild = new GEO::Stopwatch("AMGCL build", ctxt->verbose);

    if(nlExtensionIsInitialized_CUDA()) {
	nlBlasResetStats(nlCUDABlas());
    }

    Solver solver(M_amgcl,prm);

    if(ctxt->verbose) {
	GEO::Logger::out("AMGCL build") << solver << std::endl;
	if(nlExtensionIsInitialized_CUDA()) {
	    nlBlasShowStats(nlCUDABlas());
	}
    }

    delete Wbuild;

    if(ctxt->verbose) {
	GEO::Logger::out("AMGCL solve") << "Start..." << std::endl;
    }

    // Start timer when running iterative solver
    // (do not count construction in gflops stats)
    nlCurrentContext->start_time = nlCurrentTime();

    // There can be several linear systems to solve in OpenNL
    for(int k=0; k<ctxt->nb_systems; ++k) {

        if(ctxt->no_variables_indirection) {
            x = (double*)ctxt->variable_buffer[k].base_address;
            nl_assert(ctxt->variable_buffer[k].stride == sizeof(double));
        }

	std::tie(ctxt->used_iterations, ctxt->error) =
	    solve_linear_system_impl<Backend>::apply(solver,n,b,x);

        b += n;
        x += n;
    }

    nlCurrentContext->flops += nlCUDABlas()->flops;

    return NL_TRUE;
}

/*******************************************************************************/

NLboolean nlSolveAMGCL() {
    typedef amgcl::backend::builtin<double, colind_t, rowptr_t> CPU;
    typedef amgcl::backend::nlcuda GPU;

    // Cute, no ? :-)
    return nlExtensionIsInitialized_CUDA() ?
	nlSolveAMGCL_generic<GPU>() :
	nlSolveAMGCL_generic<CPU>() ;

    // Usually I do not like templates, because templates promise customization,
    // but in general, when abstraction is done through templates, customization
    // is either impossible or one has to pay agonizing pain for it.
    // BUT with its well-designed, easy-to-understand, not too deep abstraction
    // hierarchy, AMGCL is a noticeable exception !)
}

/******************************************************************************/

#else

nlBoolean nlSolveAMGCL() {
    GEO::Logger::out("AMGCL") << "Not supported" << std::endl;
    return NL_FALSE;
}

#endif
