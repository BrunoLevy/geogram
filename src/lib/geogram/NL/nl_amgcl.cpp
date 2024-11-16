/**
 * Interface between Warpdrive and the AMGCL solver.
 */

#include <geogram/NL/nl.h>

extern "C" {
#include <geogram/NL/nl_amgcl.h>
#include <geogram/NL/nl_context.h>
#include <geogram/NL/nl_cuda.h>
}

extern "C" {
    NLboolean nlSolveAMGCL(void);
}


#include <geogram/basic/logger.h>
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

static NLboolean nlSolveAMGCL_CPU(void) {

    typedef amgcl::backend::builtin<double, colind_t, rowptr_t> Backend;

#ifdef WITH_BOOST

    typedef amgcl::runtime::preconditioner<Backend> Precond;
    typedef amgcl::runtime::solver::wrapper<Backend> IterativeSolver;
    typedef amgcl::make_solver<Precond,IterativeSolver> Solver;
    typedef boost::property_tree::ptree Params;

#else

    typedef amgcl::amg<
	Backend,
	amgcl::coarsening::smoothed_aggregation,
	amgcl::relaxation::spai0
	> Precond;

    typedef amgcl::solver::cg<Backend> IterativeSolver;
    typedef amgcl::make_solver<Precond,IterativeSolver> Solver;
    typedef Solver::params Params;

#endif

    /********************************/

    if(
        GEO::CmdLine::get_arg_bool("OT:verbose") ||
        GEO::CmdLine::get_arg_bool("OT:benchmark")
    ) {
        GEO::Logger::out("AMGCL") << "calling AMGCL solver" << std::endl;
    }

    // Get linear system to solve from OpenNL context
    NLContextStruct* ctxt = (NLContextStruct*)nlGetCurrent();

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
    if(ctxt->verbose) {
        GEO::Logger::out("AMGCL") << "Building AMGCL matrix (zero copy)" << std::endl;
    }
    auto M_amgcl = amgcl::adapter::zero_copy_direct(
        size_t(n), (rowptr_t*)M->rowptr, (colind_t *)M->colind, M->val
    );

    if(ctxt->verbose) {
        GEO::Logger::out("AMGCL") << "Sorting matrix" << std::endl;
    }
    amgcl::backend::sort_rows(*M_amgcl);

    // Declare the solver
    if(ctxt->verbose) {
        GEO::Logger::out("AMGCL") << "Building solver" << std::endl;
    }
    Solver solver(M_amgcl,prm);


    // There can be several linear systems to solve in OpenNL
    for(int k=0; k<ctxt->nb_systems; ++k) {

        if(ctxt->no_variables_indirection) {
            x = (double*)ctxt->variable_buffer[k].base_address;
            nl_assert(
                ctxt->variable_buffer[k].stride == sizeof(double)
            );
        }

        if(ctxt->verbose) {
            GEO::Logger::out("AMGCL") << "Calling solver" << std::endl;
        }

        // Call the solver and copy used iterations and last
        // relative residual to OpenNL context.
        std::tie(ctxt->used_iterations, ctxt->error) = solver(
            amgcl::make_iterator_range(b, b + n),
            amgcl::make_iterator_range(x, x + n)
        );

        b += n;
        x += n;
    }

    return NL_TRUE;
}

/*************************************************************************/

namespace nlcuda_adapters {
    typedef double   value_type;
    typedef colind_t col_type;
    typedef rowptr_t ptr_type;
    typedef colind_t index_type;

    /**
     * \brief a matrix, stored in device memory
     */
    struct matrix {
	matrix() : impl_(nullptr) {
	}

	matrix(NLMatrix M) : impl_(M) {
	}

	~matrix() {
	    nlDeleteMatrix(impl_);
	    impl_ = nullptr;
	}

	matrix(const matrix& rhs) = delete;
	matrix& operator=(const matrix& rhs) = delete;

	NLMatrix impl_;
    };

    /**
     * \brief a diagonal matrix, stored in device memory
     */
    typedef matrix matrix_diagonal;

    /**
     * \brief a vector, stored in device memory
     */
    struct vector {

	vector() : n_(0), data_(nullptr) {
	}

	vector(index_type n) {
	    n_ = n;
	    // TODO: should we clear it ?
	    data_ = NL_NEW_VECTOR(
		nlCUDABlas(), NL_DEVICE_MEMORY, n_
	    );
	    clear();
	}

	vector(const value_type* x_on_host, index_type n) {
	    n_ = n;
	    data_ = NL_NEW_VECTOR(
		nlCUDABlas(), NL_DEVICE_MEMORY, n_
	    );
	    copy_from_host(x_on_host, n);
	}

	~vector() {
	    if(data_ != nullptr) {
		NL_DELETE_VECTOR(
		    nlCUDABlas(), NL_DEVICE_MEMORY, n_, data_
		);
		n_ = 0;
		data_ = nullptr;
	    }
	}

	vector(const vector& rhs) = delete;
	vector& operator=(const vector& rhs) = delete;

	void clear() {
	    // TODO: smarter method for clearing a vector
	    double* tmp = new double[n_];
	    memset(tmp, 0, bytes());
	    copy_from_host(tmp, n_);
	    delete[] tmp;
	}

	size_t bytes() const {
	    return sizeof(value_type) * size_t(n_);
	}

	void copy_from_host(const value_type* x_on_host, index_type n) {
	    nl_assert(n == n_);
	    NLBlas_t blas = nlCUDABlas();
	    blas->Memcpy(
		blas,
		const_cast<double*>(x_on_host), NL_HOST_MEMORY,
		data_, NL_DEVICE_MEMORY,
		bytes()
	    );
	}

	void copy_from_host(const std::vector<value_type>& x_on_host) {
	    copy_from_host(x_on_host.data(), index_type(x_on_host.size()));
	}

	void copy_to_host(value_type* x_on_host, index_type n) const {
	    nl_assert(n == n_);
	    NLBlas_t blas = nlCUDABlas();
	    blas->Memcpy(
		blas,
		data_, NL_DEVICE_MEMORY,
		x_on_host, NL_HOST_MEMORY,
		bytes()
	    );
	}

	void copy_to_host(std::vector<value_type>& x_on_host) const {
	    copy_to_host(x_on_host.data(), index_type(x_on_host.size()));
	}

	void copy_from_device(const value_type* x_on_device, index_type n) {
	    nl_assert(n == n_);
	    NLBlas_t blas = nlCUDABlas();
	    blas->Memcpy(
		blas,
		const_cast<double*>(x_on_device), NL_DEVICE_MEMORY,
		data_, NL_DEVICE_MEMORY,
		bytes()
	    );
	}

	void copy_from(const vector& rhs) {
	    nl_assert(rhs.n_ == n_);
	    copy_from_device(rhs.data_, rhs.n_);
	}

	index_type n_;
	double* data_;
    };

    /**
     * Wrapper around solver::skyline_lu for use with the NLCUDA backend.
     * Inspired from AMGCL cuda wrapper
     * Copies the rhs to the host memory, solves the problem using the host CPU,
     * then copies the solution back to the compute device(s).
     */
    struct cuda_skyline_lu : amgcl::solver::skyline_lu<value_type> {
	typedef amgcl::solver::skyline_lu<value_type> Base;

	template <class Matrix, class Params>
	cuda_skyline_lu(const Matrix &A, const Params&):
	    Base(*A),
	    rhs_on_host_(amgcl::backend::rows(*A)),
	    x_on_host_(amgcl::backend::rows(*A)) {
	}

	void operator()(const vector &rhs, vector &x) const {
	    rhs.copy_to_host(rhs_on_host_);
	    static_cast<const Base*>(this)->operator()(rhs_on_host_, x_on_host_);
	    x.copy_from_host(x_on_host_);
	}

	mutable std::vector<value_type> rhs_on_host_;
	mutable std::vector<value_type> x_on_host_;
    };
}

namespace amgcl { namespace backend {
    /**
     * \brief nlcuda backend for AMGCL
     */
    struct nlcuda {
	typedef nlcuda_adapters::value_type value_type;
	typedef nlcuda_adapters::col_type col_type;
	typedef nlcuda_adapters::ptr_type ptr_type;
	typedef nlcuda_adapters::index_type index_type;

	typedef nlcuda_adapters::matrix matrix;
	typedef nlcuda_adapters::matrix matrix_diagonal;
	typedef nlcuda_adapters::vector vector;

	typedef nlcuda_adapters::cuda_skyline_lu direct_solver;

	struct provides_row_iterator : std::false_type {};

	typedef amgcl::detail::empty_params params;

	static std::string name() { return "nlcuda"; }

	typedef builtin<value_type, col_type, ptr_type> builtin_backend;

	static std::shared_ptr<matrix> copy_matrix(
	    std::shared_ptr<typename builtin_backend::matrix> A, const params&
	) {
	    const typename builtin_backend::matrix &a = *A;
	    ptr_type* rowptr = const_cast<ptr_type*>(a.ptr);
	    col_type* colind = const_cast<col_type*>(a.col);
	    value_type* val = const_cast<value_type*>(a.val);

	    // Sanity checks: signedness differ, but sizes should
	    // be the same !
	    static_assert(sizeof(NLuint_big) == sizeof(ptr_type));
	    static_assert(sizeof(NLuint) == sizeof(col_type));
	    static_assert(sizeof(double) == sizeof(value_type));

	    // Create NLCRSMatrix with rowptr, colind and val arrays
	    // pointing to arrays in AMGCL matrix. Zero copy.
	    NLCRSMatrix CRS;
	    memset(&CRS, 0, sizeof(NLCRSMatrix));
	    CRS.type = NL_MATRIX_CRS;
	    CRS.symmetric_storage = NL_FALSE;
	    CRS.m = NLuint(a.nrows);
	    CRS.n = NLuint(a.ncols);
	    CRS.rowptr = reinterpret_cast<NLuint_big*>(rowptr);
	    CRS.colind = reinterpret_cast<NLuint*>(colind);
	    CRS.val = val;
	    CRS.nslices = 0;
	    CRS.sliceptr = nullptr;

	    return std::shared_ptr<matrix>(
		new matrix(nlCUDAMatrixNewFromCRSMatrix(NLMatrix(&CRS)))
	    );
	}

	static std::shared_ptr<vector> copy_vector(
	    typename builtin_backend::vector const &x, const params&
	) {
	    return std::make_shared<vector>(x.data(), x.size());
	}

	static std::shared_ptr<vector> copy_vector(
	    std::shared_ptr< typename builtin_backend::vector > x,
	    const params &prm
	) {
	    return copy_vector(*x, prm);
	}

	static std::shared_ptr<vector> create_vector(
	    size_t size, const params&
	) {
	    return std::make_shared<vector>(size);
	}

	static std::shared_ptr<direct_solver> create_solver(
	    std::shared_ptr<typename builtin_backend::matrix > A,
	    const params &prm
	) {
	    return std::make_shared<direct_solver>(A, prm);
	}
    };

   /****************************************************************/

    template <> struct spmv_impl<
	double,
	nlcuda_adapters::matrix,
	nlcuda_adapters::vector,
	double, nlcuda_adapters::vector
    > {
	static void apply(
	    double alpha,
	    const nlcuda_adapters::matrix& A,
	    const nlcuda_adapters::vector& x,
	    double beta,
	    nlcuda_adapters::vector& y
	) {
	    nlCUDAMatrixSpMV(A.impl_, x.data_, y.data_, alpha, beta);
	}
    };

    template <> struct residual_impl<
	nlcuda_adapters::matrix,
	nlcuda_adapters::vector,
	nlcuda_adapters::vector,
	nlcuda_adapters::vector
    > {
	static void apply(
	    const nlcuda_adapters::vector &rhs,
	    const nlcuda_adapters::matrix &A,
	    const nlcuda_adapters::vector &x,
	    nlcuda_adapters::vector &r
	) {
	    // r = rhs - A * x;
	    r.copy_from(rhs);
	    nlCUDAMatrixSpMV(A.impl_, x.data_, r.data_, -1.0, 1.0);
	}
    };

    template <> struct clear_impl<nlcuda_adapters::vector> {
	static void apply(nlcuda_adapters::vector& x) {
	    x.clear();
	}
    };

    template <> struct inner_product_impl<
	nlcuda_adapters::vector,
	nlcuda_adapters::vector
	> {
	static double get(
	    const nlcuda_adapters::vector& x,
	    const nlcuda_adapters::vector& y
	) {
	    nl_assert(x.n_ == y.n_);
	    NLBlas_t blas = nlCUDABlas();
	    return blas->Ddot(blas,x.n_,x.data_,1,y.data_,1);
	}
    };

    template <> struct axpby_impl<
	double,
	nlcuda_adapters::vector,
	double,
	nlcuda_adapters::vector
	> {
	static void apply(
	    double a,
	    const nlcuda_adapters::vector& x,
	    double b,
	    const nlcuda_adapters::vector& y
	) {
	    nl_assert(x.n_ == y.n_);
	    NLBlas_t blas = nlCUDABlas();
	    if(b != 1.0) {
		blas->Dscal(blas, x.n_, b, y.data_, 1);
	    }
	    blas->Daxpy(blas, x.n_, a, x.data_, 1, y.data_, 1);
	}
    };

    template <> struct axpbypcz_impl<
	double,
	nlcuda_adapters::vector,
	double,
	nlcuda_adapters::vector,
	double,
	nlcuda_adapters::vector
	> {
	static void apply(
            double a, const nlcuda_adapters::vector &x,
            double b, const nlcuda_adapters::vector &y,
            double c,       nlcuda_adapters::vector &z
	) {
	    nl_assert(x.n_ == y.n_ && y.n_ && z.n_);
	    NLBlas_t blas = nlCUDABlas();

	    if(c != 1.0) {
		blas->Dscal(blas, x.n_, c, z.data_, 1);
	    }

	    blas->Daxpy(blas, x.n_, a, x.data_, 1, z.data_, 1);
	    blas->Daxpy(blas, x.n_, b, y.data_, 1, z.data_, 1);
	}
    };

    template <> struct vmul_impl<
	double,
	nlcuda_adapters::vector,
	nlcuda_adapters::vector,
	double,
	nlcuda_adapters::vector
	> {
	static void apply(
	    double a,
	    const nlcuda_adapters::vector &x,
	    const nlcuda_adapters::vector &y,
	    double b,
	    nlcuda_adapters::vector &z
	) {
	    // TODO
	    nl_assert(false);
	}
    };

    template <> struct copy_impl<
	nlcuda_adapters::vector,
	nlcuda_adapters::vector
	> {
	static void apply(
	    const nlcuda_adapters::vector &x,
	    nlcuda_adapters::vector &y
	) {
	    y.copy_from(x);
	}
    };

}}




/*************************************************************************/

static NLboolean nlSolveAMGCL_GPU(void) {


    return NL_FALSE;
}

/*************************************************************************/

NLboolean nlSolveAMGCL() {
    return nlExtensionIsInitialized_CUDA() ?
	nlSolveAMGCL_GPU() : nlSolveAMGCL_CPU();
}

/*************************************************************************/

#else

nlBoolean nlSolveAMGCL() {
    GEO::Logger::out("AMGCL") << "Not supported" << std::endl;
    return NL_FALSE;
}

#endif
