/**
 * Interface between Warpdrive and the AMGCL solver.
 */

#include <geogram/basic/logger.h>
#include <geogram/basic/command_line.h>
#include <geogram/NL/nl.h>
extern "C" {
  #include <geogram/NL/nl_context.h>
}


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

#ifdef AMGCL_PROFILING
#include <amgcl/profiler.hpp>
namespace amgcl {
    profiler<> prof;
}
#endif

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


/*************************************************************************/

/**
 * \brief Solves the current linear system in OpenNL using AMGCL.
 * \details Compatible with OpenNL solver protocol, can be used
 *   in lieu of default's OpenNL solver, using nlSetFunc().
 *   This function is inspired by the zero_copy example shipped 
 *   with AMGCL
 * 
 * \see nlSetAMGCL()
 */
bool nlSolveAMGCL() {

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
    
    geo_assert(ctxt->M->type == NL_MATRIX_CRS);
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
    // (note: in WarpDrive there will be always a single one).
    for(int k=0; k<ctxt->nb_systems; ++k) {
	
	if(ctxt->no_variables_indirection) {
	    x = (double*)ctxt->variable_buffer[k].base_address;
	    geo_assert(
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

    return true;
}

extern "C" {
   /**
    * \brief Replaces the solver in current OpenNL context with AMGCL.
    */
   void nlSetAMGCL() {
       nlSetFunction(NL_FUNC_SOLVER, (NLfunc)nlSolveAMGCL);
   }
}

