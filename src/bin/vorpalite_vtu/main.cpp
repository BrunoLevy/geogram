/* Vorpaline - geogram demo program */

#include <geogram/basic/command_line.h>
#include <geogram/basic/command_line_args.h>
#include <geogram/basic/common.h>
#include <geogram/basic/file_system.h>
#include <geogram/basic/geometry_nd.h>
#include <geogram/basic/logger.h>
#include <geogram/basic/process.h>
#include <geogram/basic/progress.h>
#include <geogram/basic/stopwatch.h>

#include <geogram/mesh/mesh.h>
#include <geogram/mesh/mesh_decimate.h>
#include <geogram/mesh/mesh_frame_field.h>
#include <geogram/mesh/mesh_geometry.h>
#include <geogram/mesh/mesh_io.h>
#include <geogram/mesh/mesh_preprocessing.h>
#include <geogram/mesh/mesh_remesh.h>
#include <geogram/mesh/mesh_reorder.h>
#include <geogram/mesh/mesh_surface_intersection.h>
#include <geogram/mesh/mesh_tetrahedralize.h>

#include <geogram/delaunay/LFS.h>
#include <geogram/points/co3ne.h>
#include <geogram/voronoi/CVT.h>
#include <geogram/voronoi/RVD_mesh_builder.h>

#include <geogram/third_party/PoissonRecon/poisson_geogram.h>

#include <algorithm>
#include <typeinfo>

#include <vtkCellArray.h>
#include <vtkCellData.h>
#include <vtkCleanUnstructuredGrid.h>
#include <vtkDoubleArray.h>
#include <vtkPoints.h>
#include <vtkPolygon.h>
#include <vtkSmartPointer.h>
#include <vtkUnstructuredGrid.h>
#include <vtkXMLUnstructuredGridWriter.h>

namespace {

using namespace GEO;

/**
 * @brief Save the Voronoi restricted cells to VTK as an unstructured grid.
 *
 * @param RVD_mesh
 * @param voronoi_seeds
 * @param filename
 * @param ioflags
 *
 */
bool save_RVD_cells_to_vtu(Mesh &RVD_mesh, RestrictedVoronoiDiagram *RVD,
                           const std::string &filename,
                           const MeshIOFlags &ioflags) {

  // TODO: For now, we do not use delaunay but we could to check that two cells
  // sharing a face actually share one after the cleaning operation.
  // Of course this is only if not shrinking cells.

  // Display the name of the file to save
  if (ioflags.verbose()) {
    Logger::out("I/O") << "Saving file " << filename << "..." << std::endl;
  }

  // Check if we have the permission to write a file there
  if (!FileSystem::can_write_directory(
          FileSystem::dir_name(FileSystem::absolute_path(filename)), true)) {
    Logger::err("I/O") << "Failed to open \"" << filename << "\" for writing"
                       << std::endl;
    return false;
  }

  // Reference to the output mesh
  Mesh &M(RVD_mesh);
  // Reference to the input voronoi seeds (mesh)
  // Mesh &V(voronoi_seeds);

  // Here happens the magic
  std::string mtl_filename;
  std::string mtl_filename_fullpath;

  // Need to check if we need that
  std::ofstream out(filename.c_str());
  if (!out) {
    Logger::err("I/O") << "Could not create file \'" << filename << "\'"
                       << std::endl;
    return false;
  }

  // ----------------------------------------
  // 1. Create points
  // ----------------------------------------
  vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();

  for (index_t v = 0; v < M.vertices.nb(); ++v) {

    if (M.vertices.single_precision()) {
      const float *p = M.vertices.single_precision_point_ptr(v);
      points->InsertNextPoint(p[0], (M.vertices.dimension() > 1 ? p[1] : 0.0),
                              (M.vertices.dimension() > 2 ? p[2] : 0.0));
    } else {
      const double *p = M.vertices.point_ptr(v);
      points->InsertNextPoint(p[0], (M.vertices.dimension() > 1 ? p[1] : 0.0),
                              (M.vertices.dimension() > 2 ? p[2] : 0.0));
    }
  }

  // ----------------------------------------
  // 2. Create an unstructured grid
  // ----------------------------------------
  auto uGrid = vtkSmartPointer<vtkUnstructuredGrid>::New();

  uGrid->SetPoints(points);

  // ---------------------------------------
  // 3. Create cells => This is working only because faces are correctly ordered
  // ---------------------------------------

  // Reference of points for the current cell
  std::vector<vtkIdType> cellPointIds;
  // Reference of faces for the current cell
  std::vector<vtkIdType> facesStream;
  // Number of faces on the current cell
  vtkIdType numFaces{0};

  // Reference to the seed matching each facet
  Attribute<index_t> facet_region_(RVD_mesh.facets.attributes(), "region");
  // Mapping between cells and seed
  std::vector<std::size_t> cell_id_to_seed{};

  // Id of the current seed.
  unsigned int seedId{facet_region_[0]};
  // Id of the previous seed.
  unsigned int seedIdPrev{facet_region_[0]};

  // Iterate faces => faces are ordered by seed (n being the number of seeds)
  // e.g, {s0_f0, s0_f1, s0_f2, s0_f3, s0_f4, s1_f0, s1_f2, ... sn_f5}
  for (index_t f : M.facets) {

    // Get the id of the corresponding seed
    seedId = facet_region_[f];

    // If the seedId do not match the previous one, terminate the current cell
    // and add it to the grid
    if (seedId != seedIdPrev) {
      // Add the cell to the grid
      uGrid->InsertNextCell(VTK_POLYHEDRON, cellPointIds.size(),
                            cellPointIds.data(), numFaces, facesStream.data());
      // Add the seed to the mapping
      cell_id_to_seed.push_back(seedIdPrev);

      // Start a new cell by
      // - Setting the number of faces to zero
      numFaces = 0;
      // - Emptying the cell points ids and the faceStrem
      cellPointIds.clear();
      facesStream.clear();
    }

    // Initiate a face
    std::vector<vtkIdType> facePts;

    // Iterate the coners of the face, i.e., vertices
    for (index_t c = M.facets.corners_begin(f); c < M.facets.corners_end(f);
         ++c) {
      // Add the vertex ID to the list of points composing the face
      facePts.push_back(M.facet_corners.vertex(c));
      // Do the same for the cell
      cellPointIds.push_back(M.facet_corners.vertex(c));
    }
    // Add the number of points composing the face
    facesStream.push_back(facePts.size());
    // Insert the vertices ids
    facesStream.insert(facesStream.end(), facePts.begin(), facePts.end());

    numFaces++;
    // Update tje previous seed
    seedIdPrev = seedId;

    // If the last face is reached =>
    // end the current cell and add it to the grid
    if (f == M.facets.nb() - 1) {
      // Add the cell to the grid
      uGrid->InsertNextCell(VTK_POLYHEDRON, cellPointIds.size(),
                            cellPointIds.data(), numFaces, facesStream.data());
      cell_id_to_seed.push_back(seedId);
    }
  }

  // ---------------------------------------
  // 5. Clean duplicate points
  // ---------------------------------------
  vtkSmartPointer<vtkCleanUnstructuredGrid> cleaner =
      vtkSmartPointer<vtkCleanUnstructuredGrid>::New();

  cleaner->SetInputData(uGrid);
  cleaner->SetTolerance(
      CmdLine::get_arg_double("RVD_cells:vtu_clean_tolerance"));
  cleaner->Update();

  vtkSmartPointer<vtkUnstructuredGrid> cleanedGrid = cleaner->GetOutput();

  // ---------------------------------------
  // 6. Add seed id and coordinates (x, y, z)
  // ---------------------------------------

  vtkNew<vtkIntArray> seedIds;

  seedIds->SetName("original_voronoi_seed_id");
  seedIds->SetNumberOfComponents(1);
  seedIds->SetNumberOfValues(RVD->delaunay()->nb_vertices());

  vtkNew<vtkDoubleArray> cellValues;

  cellValues->SetName("voronoi_seed_coords");
  cellValues->SetNumberOfComponents(3);
  cellValues->SetNumberOfTuples(RVD->delaunay()->nb_vertices());

  auto seedCoords = RVD->delaunay()->vertices_ptr();

  // Iterate the voronoi seeds
  for (index_t v = 0; v < RVD->delaunay()->nb_vertices(); ++v) {

    // Insert tuple for ONE CELL
    seedIds->SetValue(v, cell_id_to_seed[v]);
    cellValues->SetTuple3(v, seedCoords[cell_id_to_seed[v] * 3],
                          seedCoords[cell_id_to_seed[v] * 3 + 1],
                          seedCoords[cell_id_to_seed[v] * 3 + 2]);
  }
  // Add the data to the grid
  cleanedGrid->GetCellData()->AddArray(seedIds);
  cleanedGrid->GetCellData()->AddArray(cellValues);

  // ---------------------------------------
  // 6. Write to disk
  // ---------------------------------------
  vtkSmartPointer<vtkXMLUnstructuredGridWriter> writer =
      vtkSmartPointer<vtkXMLUnstructuredGridWriter>::New();

  writer->SetFileName(filename.c_str());
  writer->SetInputData(cleanedGrid);
  writer->Write();

  return true;
}

/**
 * \brief Removes the small connected components from a mesh.
 * \param[in,out] M the mesh
 * \param[in] min_comp_area connected components smaller than
 *   this threshold are discarded
 */
void remove_small_components(Mesh &M, double min_comp_area) {
  if (min_comp_area == 0.0) {
    return;
  }
  index_t nb_f_removed = M.facets.nb();
  remove_small_connected_components(M, min_comp_area);
  nb_f_removed -= M.facets.nb();
  if (nb_f_removed != 0) {
    double radius = bbox_diagonal(M);
    double epsilon = CmdLine::get_arg_percent("pre:epsilon", radius);
    mesh_repair(M, MESH_REPAIR_DEFAULT, epsilon);
  }
}

/**
 * \brief The callback called for each RVD polyhedron. Constructs a
 *  mesh with the boundary of all cells.
 * \details Its member functions are called for each RVD polyhedron,
 *  i.e. the intersections between the volumetric mesh tetrahedra and
 *  the Voronoi cells. Based on set_simplify_xxx(), a smaller number of
 *  polyhedra can be generated.
 */
class SaveRVDCells : public RVDPolyhedronCallback {
public:
  /**
   * \brief SaveRVDCells constructor.
   * \param[out] output_mesh a reference to the generated mesh
   */
  SaveRVDCells(Mesh &output_mesh) : output_mesh_(output_mesh) {
    my_vertex_map_ = nullptr;

    // If set, then only one polyhedron per (connected component of) restricted
    // Voronoi cell is generated.
    set_simplify_internal_tet_facets(
        CmdLine::get_arg_bool("RVD_cells:simplify_tets"));

    // If set, then only one polygon per Voronoi facet is generated.
    set_simplify_voronoi_facets(
        CmdLine::get_arg_bool("RVD_cells:simplify_voronoi"));

    // If set, then the intersection between a Voronoi cell and the boundary
    // surface is replaced with a single polygon whenever possible (i.e. when
    // its topology is a disk and when it has at least 3 corners).
    set_simplify_boundary_facets(
        CmdLine::get_arg_bool("RVD_cells:simplify_boundary"),
        CmdLine::get_arg_double("RVD_cells:simplify_boundary_angle_threshold"));

    // If set, then the intersections are available as Mesh objects through the
    // function process_polyhedron_mesh(). Note that this is implied by
    // simplify_voronoi_facets or simplify_boundary.
    if (CmdLine::get_arg_double("RVD_cells:shrink") != 0.0) {
      set_use_mesh(true);
    }
  }

  ~SaveRVDCells() override {
    delete my_vertex_map_;
    my_vertex_map_ = nullptr;
  }

  /**
   * \brief Called at the beginning of RVD traversal.
   */
  void begin() override {
    RVDPolyhedronCallback::begin();
    output_mesh_.clear();
    output_mesh_.vertices.set_dimension(3);
  }

  /**
   * \brief Called at the end of RVD traversal.
   */
  void end() override {
    RVDPolyhedronCallback::end();
    output_mesh_.facets.connect();
  }

  /**
   * \brief Called at the beginning of each RVD polyhedron.
   * \param[in] seed , tetrahedron the (seed,tetrahedron) pair that
   *  defines the RVD polyhedron, as the intersection between the Voronoi
   *  cell of the seed and the tetrahedron.
   */
  void begin_polyhedron(index_t seed, index_t tetrahedron) override {
    geo_argused(tetrahedron);

    //   The RVDVertexMap is used to map the symbolic representation of vertices
    // to indices. Here we reset indexing for each new cell, so that vertices
    // shared by the faces of two different cells will be duplicated. We do that
    // because we construct the boundary of the cells in a surfacic mesh (for
    // visualization purposes). Client code that has a data structure for
    // polyhedral volumetric mesh will not want to reset indexing (and will
    // comment-out the following three lines). It will also construct the
    // RVDVertexMap in the constructor.

    // keep track of the seed
    current_seed_id_ = seed;

    delete my_vertex_map_;
    my_vertex_map_ = new RVDVertexMap;
    my_vertex_map_->set_first_vertex_index(output_mesh_.vertices.nb());
  }

  /**
   * \brief Called at the beginning of each RVD polyhedron.
   * \param[in] facet_seed if the facet is on a Voronoi bisector,
   *  the index of the Voronoi seed on the other side of the bisector,
   *  else index_t(-1)
   * \param[in] facet_tet if the facet is on a tethedral facet, then
   *  the index of the tetrahedron on the other side, else index_t(-1)
   */
  void begin_facet(index_t facet_seed, index_t facet_tet) override {
    geo_argused(facet_seed);
    geo_argused(facet_tet);
    current_facet_.resize(0);
  }

  void vertex(const double *geometry,
              const GEOGen::SymbolicVertex &symb) override {
    // Find the index of the vertex associated with its symbolic representation.
    index_t vid = my_vertex_map_->find_or_create_vertex(seed(), symb);

    // If the vertex does not exist in the mesh, create it.
    if (vid >= output_mesh_.vertices.nb()) {
      output_mesh_.vertices.create_vertex(geometry);
    }

    // Memorize the current facet.
    current_facet_.push_back(vid);
  }

  void end_facet() override {
    // Create the facet from the memorized indices.
    index_t f = output_mesh_.facets.nb();
    output_mesh_.facets.create_polygon(current_facet_.size());
    for (index_t i = 0; i < current_facet_.size(); ++i) {
      output_mesh_.facets.set_vertex(f, i, current_facet_[i]);
    }
    // link the facet to its seed
    facet_regions_.push_back(current_seed_id_);
  }

  void end_polyhedron() override {
    // Nothing to do.
  }

  void process_polyhedron_mesh() override {
    // This function is called for each cell if set_use_mesh(true) was called.
    // It is the case if simplify_voronoi_facets(true) or
    // simplify_boundary_facets(true) was called.
    //   Note1: most users will not need to overload this function (advanded use
    //   only).
    //   Note2: mesh_ is managed internally by RVDPolyhedronCallback class, as
    //   an
    // intermediary representation to store the cell before calling the
    // callbacks. It is distinct from the output_mesh_ constructed by the
    // callbacks.

    //   The current cell represented by a Mesh can be
    // filtered/modified/post-processed (member variable mesh_)
    // here, before calling base class's implementation.
    //   As an example, we shrink the cells. More drastic modifications/
    // transformations of the mesh can be done (see base class's implementation
    // in geogram/voronoi/RVD_polyhedron_callback.cpp).
    double shrink = CmdLine::get_arg_double("RVD_cells:shrink");
    if (shrink != 0.0 && mesh_.vertices.nb() != 0) {
      vec3 center(0.0, 0.0, 0.0);
      for (index_t v = 0; v < mesh_.vertices.nb(); ++v) {
        center += vec3(mesh_.vertices.point_ptr(v));
      }
      center = (1.0 / double(mesh_.vertices.nb())) * center;
      for (index_t v = 0; v < mesh_.vertices.nb(); ++v) {
        vec3 p(mesh_.vertices.point_ptr(v));
        p = shrink * center + (1.0 - shrink) * p;
        mesh_.vertices.point_ptr(v)[0] = p.x;
        mesh_.vertices.point_ptr(v)[1] = p.y;
        mesh_.vertices.point_ptr(v)[2] = p.z;
      }
    }

    //  The default implementation simplifies Voronoi facets
    // and boundary mesh facets based on the boolean flags
    // defined by set_simplify_xxx(). Then it calls the callbacks
    // for each mesh facet.
    RVDPolyhedronCallback::process_polyhedron_mesh();
  }

  // Getter for the region associated with each facet
  index_t get_facet_region(const index_t facet_id) const {
    return facet_regions_[facet_id];
  }

private:
  // Keep track of the voronoi seed
  index_t current_seed_id_{0};
  // Vector storing the seed associated with each facet
  vector<index_t> facet_regions_{};
  vector<index_t> current_facet_;
  Mesh &output_mesh_;
  RVDVertexMap *my_vertex_map_;
};

void compute_RVD_cells(RestrictedVoronoiDiagram *RVD, Mesh &RVD_mesh) {
  SaveRVDCells callback(RVD_mesh);
  RVD->for_each_polyhedron(callback, true, true,
                           CmdLine::get_arg_bool("RVD_cells:parallel"));

  // CVT.RVD()->delaunay()->vertex_ptr()
  // Link facets to seeds
  if (RVD_mesh.facets.nb() != 0) {
    Attribute<index_t> facet_region_attr(RVD_mesh.facets.attributes(),
                                         "region");
    for (index_t f = 0; f < RVD_mesh.facets.nb(); ++f) {
      facet_region_attr[f] = callback.get_facet_region(f);
    }
  }
}

// This is what I need to edit
/**
 * \brief Generates a polyhedral mesh.
 * \param[in] input_filename name of the input file, can be
 *   either a closed surfacic mesh or a tetrahedral mesh
 * \param[in] output_filename name of the output file
 * \retval 0 on success
 * \retval non-zero value otherwise
 */
int polyhedral_mesher(const std::string &input_filename,
                      std::string output_filename) {
  Mesh M_in;
  Mesh M_out;
  Mesh M_points;

  Logger::div("Polyhedral meshing");

  if (!mesh_load(input_filename, M_in)) {
    return 1;
  }

  if (M_in.cells.nb() == 0) {
    Logger::out("Poly") << "Mesh is not a volume" << std::endl;
    Logger::out("Poly") << "Trying to tetrahedralize" << std::endl;
    if (!mesh_tetrahedralize(M_in)) {
      return 1;
    }
    M_in.cells.compute_borders();
  }

  index_t dim = M_in.vertices.dimension();
  index_t spec_dim = CmdLine::get_arg_uint("poly:embedding_dim");
  if (spec_dim != 0 && spec_dim <= dim) {
    dim = spec_dim;
  }

  CentroidalVoronoiTesselation CVT(&M_in, coord_index_t(dim));
  CVT.set_volumetric(true);

  Logger::div("Generate random samples");

  CVT.compute_initial_sampling(CmdLine::get_arg_uint("RVD_cells:nb_pts"));

  Logger::div("Optimize sampling");

  try {
    index_t nb_iter = CmdLine::get_arg_uint("opt:nb_Lloyd_iter");
    ProgressTask progress("Lloyd", nb_iter);
    CVT.set_progress_logger(&progress);
    CVT.Lloyd_iterations(nb_iter);
  } catch (const TaskCanceled &) {
  }

  try {
    index_t nb_iter = CmdLine::get_arg_uint("opt:nb_Newton_iter");
    ProgressTask progress("Newton", nb_iter);
    CVT.set_progress_logger(&progress);
    CVT.Newton_iterations(nb_iter);
  } catch (const TaskCanceled &) {
  }

  CVT.set_progress_logger(nullptr);

  CVT.RVD()->set_exact_predicates(true);

  compute_RVD_cells(CVT.RVD(), M_out);

  save_RVD_cells_to_vtu(M_out, CVT.RVD(), output_filename, MeshIOFlags());

  return 0;
}

} // namespace

int main(int argc, char **argv) {
  using namespace GEO;

  GEO::initialize(GEO::GEOGRAM_INSTALL_ALL);

  try {

    Stopwatch total("Total time");

    CmdLine::import_arg_group("standard");
    CmdLine::import_arg_group("pre");
    CmdLine::import_arg_group("algo");
    CmdLine::import_arg_group("opt");
    CmdLine::import_arg_group("poly");

    CmdLine::declare_arg_group("RVD_cells", "RVD cells simplification flags");
    CmdLine::declare_arg("RVD_cells:simplify_tets", true,
                         "Simplify tets intersections");
    CmdLine::declare_arg("RVD_cells:simplify_voronoi", true,
                         "Simplify Voronoi facets");
    CmdLine::declare_arg("RVD_cells:simplify_boundary", true,
                         "Simplify boundary facets. The default is `true`.");
    CmdLine::declare_arg("RVD_cells:simplify_boundary_angle_threshold", 10.0,
                         "Angle below which boundary facets are simplified. "
                         "Only applies if simplify_boundary is `true`.");
    CmdLine::declare_arg("RVD_cells:shrink", 0.0,
                         "Shrink factor for computed cells");
    CmdLine::declare_arg("RVD_cells:vtu_clean_tolerance", 1.0e-6,
                         "Toleance when cleaning the VTU mesh before saving "
                         "it. The default is 1e-6.");
    CmdLine::declare_arg(
        "RVD_cells:nb_pts", 10000,
        "Number of seeds to use if initial seeds are not provided.");

    CmdLine::declare_arg(
        "RVD_cells:parallel", false,
        "Whether to run the RVD step in parallel (multi-thread).");

    std::vector<std::string> filenames;

    if (!CmdLine::parse(argc, argv, filenames, "inputfile <outputfile>")) {
      return 1;
    }

    std::string input_filename = filenames[0];
    std::string output_filename =
        filenames.size() >= 2 ? filenames[1] : std::string("out.meshb");
    Logger::out("I/O") << "Output = " << output_filename << std::endl;
    CmdLine::set_arg("input", input_filename);
    CmdLine::set_arg("output", output_filename);

    if (CmdLine::get_arg_bool("poly")) {
      return polyhedral_mesher(input_filename, output_filename);
    }

  } catch (const std::exception &e) {
    std::cerr << "Received an exception: " << e.what() << std::endl;
    return 1;
  }

  return 0;
}
