# decad file layout: output and repository

Tessellation and export files, then examples, test kits, tooling and CI.
The rules for rows live in `docs/layout.md`.

## Layout

### Output

| Path | Responsibility |
|---|---|
| `tessellate.go` | `Mesh`, `Body.Tessellate`, prism mesh handle adapter and dispatch. See tessellation design. |
| `tessellate_brep.go` | Meshes a brep body and proves its occupied volume. See general-boolean §4.4. |
| `tessellate_brep_band.go` | Adapts route L bands to shared wall samples and delegates fillet rings and strips to `internal/brepfilletmesh/`. See modify-general DG3. |
| `tessellate_brep_partial_fillet.go` | Selects `internal/partialband/` layouts and places their boundary vertices for `internal/brepfilletmesh/`. |
| `tessellate_brep_partial_chamfer.go` | Emits two selected planar chamfer strips over their shared contour vertices. See modify-general §4.3a. |
| `tessellate_verification.go` | `Verification`, `WithVerification` and mesh proof publication. See tessellation §1. |
| `tessellate_sweep_composite.go` | Builds composite Sweep span handles and runs the body audit around internal station assembly. See sweep Table D. |
| `tessellate_body_patch.go` | Adapts PatchEdges source mesh handles and public refusal names to `internal/patchmesh/`. See surface §10.1. |
| `tessellate_chain.go` | A chain ribbon's exact-quad mesh. See surface §13.4. |
| `tessellate_coil.go` | A coil's held-shell mesh with its area and volume proofs. See helix §8. |
| `tessellate_capblend.go` | Adapts cap-blend mesh nodes to public handles and retains the shared BRep band emitter and Loft proof. See tessellation reach §7. |
| `loft_capband_proof.go` | Adapts a matching loft's cap-band payload and mesh to the numeric occupied-volume proof in `internal/tessellation`. See loft §18. |
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
