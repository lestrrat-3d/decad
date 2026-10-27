# decad file layout: output and repository

Tessellation and export files, then examples, test kits, tooling and CI.
The rules for rows live in `agent-docs/layout.md`.

## Layout

### Output

| Path | Responsibility |
|---|---|
| `tessellate.go` | `Mesh`, `Body.Tessellate`, its mesh cache and proof withholding, and the call into `internal/tessdispatch/`. See tessellation design, surface §10.1. |
| `tessellate_verification.go` | `Verification`, `WithVerification` and mesh proof publication. See tessellation §1, §4. |
| `mesh_body.go` | Public `MeshBody`, `MeshReport` and `NewMeshBody`: the imported triangle mesh and its bounds, area, volume and point readings. `internal/meshbody/` reads the triangles. See mesh-import design. |
| `solid_from_mesh.go` | Public `SolidFromMesh`: gates, the exact crossing audit and mesh audit, exact plane groups, then `facetedrecord.BuildWithProof` and one commit. See mesh-solid design. |
| `mesh_import.go` | Public `ReadSTL`, `ReadOBJ`, `ReadThreeMF`, `ReadSTEP`, their options and `ErrMalformedMesh`, `ErrMeshTooLarge`: limits, then `internal/meshparse/` parses and `NewMeshBody` reads the result. See mesh-import design. |
| `mesh_repair.go` | Public `RepairMesh`, its options and `MeshRepairReport`: option gates, then `internal/meshrepair/` mends and `NewMeshBody` reads the result. See mesh-import design §7. |
| `mesh_refine.go` | Public `RefineMesh`: gates, then `internal/meshrefine/` splits long edges and `NewMeshBody` reads the result. See mesh-import design §11. |
| `internal/meshrefine/` | Midpoint splitting of the long edges of an indexed triangle mesh. See mesh-import design §11. |
| `mesh_scale.go` | Public `ScaleMesh` and `ScaleBodyMesh`: gates, per-axis stretch about an anchor, reversed winding under an odd number of negative factors, then `NewMeshBody`. See mesh-import design §10. |
| `subd.go` | Public `SubDMesh` and `SubDCage`: gates, then `internal/subd/` refines the cage and `NewMeshBody` reads the result. See subd design. |
| `lattice.go` | Public `GyroidLattice`, `SchwarzPLattice`, `DiamondLattice` and `LatticeSpec`: gates, then `internal/lattice/` samples and cuts the field and `NewMeshBody` reads the result. See lattice design. |
| `internal/orthoshell/` | Grid erosion of an orthogonal polyhedron by a cube, the removed-face slab and the wall mesh, for `Shell`. See orthogonal-shell design. |
| `internal/lattice/` | Gyroid field sampled on a grid and cut by marching tetrahedra into a closed triangle mesh. See lattice design. |
| `mesh_reduce.go` | Public `ReduceMesh`, `ReduceMeshWithin` and `MeshReduceReport`: gates, then `internal/meshreduce/` clusters vertices and `NewMeshBody` reads the result. See mesh-import design §9. |
| `internal/meshreduce/` | Vertex-clustering reduction of an indexed triangle mesh. See mesh-import design §9. |
| `internal/meshrepair/` | Weld, degenerate-triangle drop, shell orientation and boundary-loop fill of an indexed triangle mesh. See mesh-import design §7. |
| `internal/meshparse/` | STL, OBJ and 3MF parsing into welded vertices and triangles, with size limits. See mesh-import design. |
| `internal/meshbody/` | Exact edge classification, shell count, signed volume and area interval of an imported mesh. See mesh-import design. |
| `export/` | STL, OBJ, 3MF, glTF, PNG, DXF and AP214 writers, and mesh writers for a `MeshBody`. See `agent-docs/step-export-design.md`, `agent-docs/3mf-export-design.md`, `agent-docs/gltf-export-design.md`, `agent-docs/png-export-design.md`, `agent-docs/dxf-export-design.md`. |
| `internal/meshslice/` | Cuts a triangle mesh with a plane and chains the cut into closed outlines by the mesh edges it crosses. See section-outline design. |
| `internal/drawing/` | Feature-edge projection of a mesh with depth-buffer hidden-line runs, and the even-odd hatch of a section, for `export.DXF` and `export.DXFSection`; plain millimetre data. See dxf-export design. |
| `internal/raster/` | Orthographic flat-shaded triangle rasterizer, projection and depth buffer, for `export.PNG` and `export.DXF`; plain millimetre data. See png-export design. |
| `linkage/` | The generic linkage model, its loop and drive solver, and the linkage and joint-box checks over the motion engine. `decad` aliases it. See `agent-docs/linkage-check-design.md`. |
| `report/` | Result vocabulary generic over body, face, edge and vertex references: measurements, outcome enums, contact, sweep and Verify records. `decad` aliases it. See api-design §6.4. |

### Repository

| Path | Responsibility |
|---|---|
| `examples/` | Executable Go examples (`Example_…`, `go test`-verified `// Output:` blocks). Never `package main`. |
| `examples/operation_diagnostics_example_test.go` | Builds an exact body, reads its empty warning list, and branches on a wrapped selection diagnostic. See operation-diagnostics-design. |
| `dynamics/` | Rigid-body worlds and their scheduled step. See `agent-docs/multibody-dynamics-design.md`. |
| `apitest/` | Tests of the exported API alone. See `apitest/doc.go`. |
| `decadtest/` | The public test kit. See `decadtest/doc.go`. |
| `payload_class.go` | Installs the reader behind `decadtest.PayloadClass`, so tests can tell a mesh Boolean result from an analytic one. See faceted-source-retention §10. |
| `_gallery/` | Nested module for README images, landing and linkage clips, dynamics scenes; excludes SolidLens. See `main.go`. |
| `_shardgen/` | Nested module packing root and `apitest` tests into race shards. See `main.go`. |
| `.github/workflows/` | `ci.yml`: lint, tests, tidy, vulnerability; `focused-go-test.yml`: capped named test; `refresh-gallery-images.yml`: image refresh branches; `codeql.yml`: CodeQL; `test-shards*.txt`: race shards. |
