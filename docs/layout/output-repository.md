# decad file layout: output and repository

Tessellation and export files, then examples, test kits, tooling and CI.
The rules for rows live in `docs/layout.md`.

## Layout

### Output

| Path | Responsibility |
|---|---|
| `tessellate.go` | `Mesh`, `Body.Tessellate`, prism/cup mesh assembly and dispatch. See tessellation design. |
| `tessellate_patch.go` | Chords and triangulates a recorded `PatchProfile` sheet. See tessellation §5.1. |
| `tessellate_brep.go` | Meshes a brep body and proves its occupied volume. See general-boolean §4.4. |
| `tessellate_brep_band.go` | Adapts route L band geometry to shared wall samples and emits mesh patches. See modify-general DG3. |
| `tessellate_brep_fillet.go` | Adapts fillet band ring geometry, strips and proof terms. See loop-fillet §7.1. |
| `tessellate_brep_partial_fillet.go` | Adapts `internal/partialband/` layouts to selected cap-edge fillet meshes. |
| `tessellate_brep_partial_chamfer.go` | Emits two selected planar chamfer strips over their shared contour vertices. See modify-general §4.3a. |
| `tessellate_all_edge_chamfer.go` | Meshes the box all-edge bevels and shared circular bore with occupied-volume proof. See modify-general §4.3b. |
| `tessellate_verification.go` | `Verification`, `WithVerification` and mesh proof publication. See tessellation §1. |
| `tessellate_revolve.go` | Assembles revolve cells and caps. See tessellation §8–§10. |
| `tessellate_sweep_composite.go` | Assembles composite Sweep meshes from shared station plans. See sweep Table D. |
| `tessellate_revolve_volume.go` | Adapts the revolve volume proof and its fitted-spline meridian charge. See tessellation §11. |
| `tessellate_stitch.go` | Stitch mesh adapters. See tessellation §2 and surface §10.1. |
| `tessellate_body_patch.go` | Triangulates planar PatchEdges sheets and proves face bounds. See tessellation §2. |
| `tessellate_body_patch_prism.go` | Reuses circular surface-extrusion wall stations to mesh one or both PatchEdges fills with shared rim indices and verified face bounds. |
| `tessellate_chain.go` | A chain ribbon's exact-quad mesh. See surface §13.4. |
| `tessellate_chain_sweep.go` | Shared-station mesh assembly and face selection for the cardinal composite chain sheet. See sweep §15.8. |
| `tessellate_chain_arc.go` | Certified angular-cell mesh for one-line single-arc chain sheets. See sweep §15.9. |
| `tessellate_coil.go` | A coil's held-shell mesh with its area and volume proofs. See helix §8. |
| `tessellate_capblend.go` | The cap-loop chamfer mesh. See tessellation reach §7. |
| `loft_capband_proof.go` | Bounds a matching loft's reflex cap bands for occupied-volume proof. See loft §18. |
| `tessellate_draft.go` | A draft body's mesh through its band view. See draft §9.1. |
| `tessellate_draft_two_sided.go` | Sewn two-slab draft mesh. See draft §9.1. |
| `export/` | STL, OBJ, 3MF and AP214 writers. See `docs/step-export-design.md`, `docs/3mf-export-design.md`. |

### Repository

| Path | Responsibility |
|---|---|
| `examples/` | Executable Go examples (`Example_decad_…`, `go test`-verified `// Output:` blocks). Never `package main`. |
| `dynamics/` | Rigid-body worlds and their scheduled step. See `docs/multibody-dynamics-design.md`. |
| `apitest/` | Tests of the exported API alone. See `apitest/doc.go`. |
| `decadtest/` | The public test kit. See `decadtest/doc.go`. |
| `_gallery/` | Nested module for README images, landing and linkage clips, dynamics scenes; excludes SolidLens. See `main.go`. |
| `_shardgen/` | Nested module packing root and `apitest` tests into race shards. See `main.go`. |
| `.github/workflows/` | `ci.yml`: lint, tests, tidy, vulnerability; `focused-go-test.yml`: capped named test; `refresh-gallery-images.yml`: image refresh branches; `codeql.yml`: CodeQL; `test-shards*.txt`: race shards. |
