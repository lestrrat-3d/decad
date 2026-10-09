# decad file layout: output and repository

Tessellation and export files, then examples, test kits, tooling and CI.
The rules for rows live in `docs/layout.md`.

## Layout

### Output

| Path | Responsibility |
|---|---|
| `tessellate.go` | `Mesh`, `Body.Tessellate`, prism/cup mesh assembly and dispatch. See tessellation design. |
| `tessellate_stacked.go` | Stacked-slab mesh and volume proof. See stacked-prism §5. |
| `tessellate_brep.go` | Meshes a brep body and proves its occupied volume. See general-boolean §4.4. |
| `tessellate_brep_band.go` | Route L bands in a brep mesh. See modify-general DG3. |
| `tessellate_verification.go` | `Verification`, `WithVerification` and mesh proof publication. See tessellation §1. |
| `tessellate_revolve.go` | Assembles revolve cells and caps. See tessellation §8–§10. |
| `tessellate_revolve_volume.go` | Adapts the revolve volume proof. See tessellation §11. |
| `tessellate_stitch.go` | Stitch mesh adapters. See tessellation §2 and surface §10.1. |
| `tessellate_chain.go` | A chain ribbon's exact-quad mesh. See surface §13.4. |
| `tessellate_coil.go` | A coil's held-shell mesh with its area and volume proofs. See helix §8. |
| `tessellate_capblend.go` | The cap-loop chamfer mesh. See tessellation reach §7. |
| `tessellate_draft.go` | A draft body's mesh through its band view. See draft §9.1. |
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
| `.github/workflows/` | `ci.yml`: lint, tests, tidy, vulnerability; `codeql.yml`: CodeQL; `test-shards*.txt`: race shards. |
