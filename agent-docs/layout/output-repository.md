# decad file layout: output and repository

Tessellation and export files, then examples, test kits, tooling and CI.
The rules for rows live in `agent-docs/layout.md`.

## Layout

### Output

| Path | Responsibility |
|---|---|
| `tessellate.go` | `Mesh`, `Body.Tessellate`, its mesh cache and proof withholding, and the call into `internal/tessdispatch/`. See tessellation design, surface §10.1. |
| `tessellate_verification.go` | `Verification`, `WithVerification` and mesh proof publication. See tessellation §1, §4. |
| `export/` | STL, OBJ, 3MF and AP214 writers. See `agent-docs/step-export-design.md`, `agent-docs/3mf-export-design.md`. |
| `linkage/` | The generic linkage model, its loop and drive solver, and the linkage and joint-box checks over the motion engine. `decad` aliases it. See `agent-docs/linkage-check-design.md`. |
| `report/` | Result vocabulary generic over body, face, edge and vertex references: measurements, outcome enums, contact, sweep and Verify records. `decad` aliases it. See api-design §6.4. |

### Repository

| Path | Responsibility |
|---|---|
| `examples/` | Executable Go examples (`Example_decad_…`, `go test`-verified `// Output:` blocks). Never `package main`. |
| `dynamics/` | Rigid-body worlds and their scheduled step. See `agent-docs/multibody-dynamics-design.md`. |
| `apitest/` | Tests of the exported API alone. See `apitest/doc.go`. |
| `decadtest/` | The public test kit. See `decadtest/doc.go`. |
| `payload_class.go` | Installs the reader behind `decadtest.PayloadClass`, so tests can tell a mesh Boolean result from an analytic one. See faceted-source-retention §10. |
| `_gallery/` | Nested module for README images, landing and linkage clips, dynamics scenes; excludes SolidLens. See `main.go`. |
| `_shardgen/` | Nested module packing root and `apitest` tests into race shards. See `main.go`. |
| `.github/workflows/` | `ci.yml`: lint, tests, tidy, vulnerability; `focused-go-test.yml`: capped named test; `refresh-gallery-images.yml`: image refresh branches; `codeql.yml`: CodeQL; `test-shards*.txt`: race shards. |
