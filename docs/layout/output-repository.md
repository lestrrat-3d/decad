# decad file layout: output and repository

Tessellation and export files, then examples, test kits, tooling and CI.
The rules for rows live in `docs/layout.md`.

## Layout

### Output

| Path | Responsibility |
|---|---|
| `tessellate.go` | `Mesh`, `Body.Tessellate`, prism mesh handle adapter and dispatch, and the PatchEdges source mesh adapter. See tessellation design, surface §10.1. |
| `tessellate_verification.go` | `Verification`, `WithVerification` and mesh proof publication. See tessellation §1, §4. |
| `tessellate_sweep_composite.go` | Builds composite Sweep span handles and runs the body audit around internal station assembly. See sweep Table D. |
| `tessellate_coil.go` | A coil's held-shell mesh with its area and volume proofs. See helix §8. |
| `tessellate_capblend.go` | Adapts the cap-blend mesh build to public handles and runs the Loft proof, including the cap-band volume bound. See tessellation reach §7. |
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
