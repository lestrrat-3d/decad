# Lean proofs

`_proofs/` holds machine-checked Lean 4 proofs about decad's bounded arithmetic, plus a Go checker
that fails when a covered Go function changes without its theorem being re-read. The proofs are about
Lean models of Go functions under the assumptions in section 5. They are not proofs about compiled Go.

Question router (navigation only):

| Question | Section |
|---|---|
| Where files live | Directory layout |
| Lean/Mathlib pins, options, file conventions | Lean project |
| Which theorems exist and which Go they cover | Theorems |
| What the proofs assume and do NOT prove | Assumptions |
| Manifest format, hash, checker | Trace manifest, Checker |
| CI workflow | CI workflow |
| Adding a theorem for a new Go function | Recipe: cover a new Go function |
| Upgrading Lean or Mathlib | Upgrade Lean and Mathlib |
| Running everything locally | Run locally |

## Directory layout

```
_proofs/
  lakefile.toml          Lake config: package, lean_lib Decad, leanOptions, Mathlib require
  lean-toolchain         leanprover/lean4:v4.36.0, equal to Mathlib's own at the pinned commit
  lake-manifest.json     exact commit of Mathlib and every transitive Lean package
  Decad.lean             root module; `lake exe mk_all` writes it from every Decad/**.lean
  Decad/Float/Model.lean     FloatModel structure; up_ge, rn_nonneg
  Decad/Float/Grid.lean      FloatModel instance on the grid δℤ
  Decad/Bounded/Upper.lean   upRound, provenUp, productUpper, absSum and their bound theorems
  Decad/Bounded/Mul.lean     mul_enclosure, boundedMul_bound_sound
  Decad/Rational/LazyRound.lean  floor/ceiling, outward, error, sign, scale, relative error, lazyRound model
  go.mod                 module github.com/lestrrat-3d/decad/_proofs; standard library only
  trace.json             theorem to Go function manifest
  tracecheck/            package main: manifest checker and its tests
```

- New area → new directory `Decad/<Area>/`. One topic per file.
- `go.mod` sits at `_proofs/` (depth 2), so `scripts/tidy-dependent-go-modules.sh` tidies it.
- `_proofs` starts with `_`, so the root module, root `go vet`, `golangci-lint` and `go list ./...` never see it.
  `.lake/` starts with `.`, so `go list ./...` inside `_proofs` skips it too.
- `.gitignore` covers `/_proofs/.lake/`. NEVER commit `.lake/`. A checkout that builds the proofs holds about
  7 GB there; remove `_proofs/.lake` when done.

## Lean project

### Pinning

- `lean-toolchain` is `leanprover/lean4:v4.36.0`. It MUST equal `.lake/packages/mathlib/lean-toolchain`.
- `lakefile.toml` requires Mathlib by tag `v4.36.0`. `lake-manifest.json` pins the commit
  (`fc4f59e577feb1a4956671bc04ac18aaaf8cd842`). `lake build` never updates the manifest.

### Build is warning-clean

- ALWAYS build with `lake build --wfail`. CI passes `--wfail` too. It fails on every warning, including
  `declaration uses sorry`.
- Fix a deprecation by using the replacement the warning names. NEVER silence it.
- Remove an unused hypothesis. NEVER prefix it with `_`.
- NEVER add `set_option ... false` to a file to silence a linter. A linter that is wrong for this project is
  disabled once in `lakefile.toml` with a comment. `lakefile.toml` disables `linter.style.header` (Mathlib's
  copyright block rule) and `linter.style.longLine` (Mathlib's 100-column rule; this repository wraps at about 120).

### File conventions

Every file:

```lean
module

public import Mathlib            -- or public import Decad.<...>

/-!
# <Title>

<What this file models, one paragraph. Name the Go functions by package-qualified name.>
-/

public section

namespace Decad.<Area>
...
end Decad.<Area>

end
```

- `module` + `public import` + `public section` are required. Without them Lake warns that `Mathlib` is
  designed for the module system, and `--wfail` fails on it.
- A model definition another file unfolds MUST be `@[expose]`. Without it `unfold` in the importing file fails.
- The `/-! # Title -/` docstring is required by Mathlib's header linter.
- NEVER write `axiom`, `sorry`, `admit` or `native_decide`. CI's axiom audit fails on all four.
- Every assumption is a hypothesis of a theorem or a field of a model structure (`FloatModel`).
- `lake exe mk_all` rewrites `Decad.lean` with the header `module  -- shake: keep-all --deprecated_module: ignore`.
  Keep that line as it writes it; `lake exe mk_all --check` compares against it.

### Naming and docstrings

- Defs are `lowerCamelCase`; theorems are `snake_case` embedding def names. A Lean def mirroring Go
  `ProductUpper` is `productUpper`.
- A def mirroring a Go function has a docstring `Model of \`pkg.Func\`.` plus any deliberate difference
  (for example "finite operands only; the +Inf refusal branch is not modeled").
- A top-level theorem docstring states the guarantee in words, the Go functions it covers by package-qualified
  name, and the assumption ids its hypotheses carry (for example "Hypothesis `hr` is A4"). File paths and
  hashes live only in `trace.json`.

## Theorems

All theorems build with no `sorry`, and `axiom-audit` accepts only `propext`, `Classical.choice`, `Quot.sound`.

| Lean (namespace `Decad.…`) | File | Go covered |
|---|---|---|
| `Float.FloatModel`, `up_ge`, `rn_nonneg` | Float/Model | the float model itself |
| `Float.gridModel` : `FloatModel` | Float/Grid | none; shows the `FloatModel` axioms are satisfiable |
| `Bounded.upRound`, `upRound_mem`, `add_up` | Bounded/Upper | `proofbound.UpRound` |
| `Bounded.provenUp` | Bounded/Upper | `proofbound.ProvenUpRound`, `proof.ProvenUpRound` |
| `Bounded.productUpper`, `productUpper_mem`, `productUpper_ge` | Bounded/Upper | `proofbound.ProductUpper` |
| `Bounded.absSum`, `absSum_ge` | Bounded/Upper | `proofbound.AbsSumUpper` |
| `Bounded.mul_enclosure`, `boundedMul_bound_sound` | Bounded/Mul | `proofbound.BoundedMul`, `MeasuredScalar` |
| `Rational.down_is_floor`, `up_is_ceil`, `round_outward`, `round_error`, `round_sign` | Rational/LazyRound | `proof.lazyRound` |
| `Rational.scale_range`, `round_rel_error`, `lazyRoundModel`, `lazyRound_sound` | Rational/LazyRound | `proof.lazyRound`, `LazyRoundDown`, `LazyRoundUp` |

- `Float.gridModel`: `F = {δ·k | k : ℤ}`, `rn x = δ·round(x/δ)`, `nu f = f + δ`, any `δ > 0`. It does not model
  binary64. Without an instance, an inconsistent `FloatModel` would make every theorem vacuously true.
- `boundedMul_bound_sound` takes `M : FloatModel`, a float `r`, non-negative bounds and `|v·w − held| ≤ r`. Its
  conclusion is `|x·y − held| ≤ absSum M [productUpper |v| Eb, productUpper |w| Ea, productUpper Ea Eb, r] 0`.
- `scale_range`: with `a = Nat.size |n|`, `b = Nat.size d` (Go `BitLen`, A8), `n ≠ 0`, `d > 0`,
  `s = bits − (a − b)`: `2^(bits−1) < |n/d·2^s| < 2^(bits+1)`.
- `round_rel_error`: `2^(bits−1) < |x·2^s|` ⇒ the floor and ceiling results are each within `2^(1−bits)·|x|` of
  `x`, strictly. This is the relative-error claim in `LazyRoundDown`'s doc comment.
- `lazyRoundModel (n d : ℤ) (bits : ℕ) (up : Bool) : ℚ` mirrors `lazyRound` branch for branch: the early return
  (`n = 0` or `size n + size d ≤ 4·bits`), the shift, truncating `Int.tdiv`/`Int.tmod` for `QuoRem` (A8), the
  one-step correction, and `m / 2^s`.
- `lazyRound_sound`: for `d > 0` and `bits ≥ 1` the result is at most `n/d` (down) or at least `n/d` (up). When
  the early return does not fire it is also nonzero with the sign of `n/d` and within `2^(1−bits)·|n/d|`.

## Assumptions

The proofs do NOT prove the rows below. A change that breaks a row can make a proved bound false while every
theorem still builds.

| Id | Assumption | Lean form | Go evidence | Covered Go entries |
|---|---|---|---|---|
| A1 | Each modeled float `+`/`*` on finite operands without overflow returns the round-to-nearest-even value: Go IEEE 754 binary64, and no fused multiply-add on the modeled path | `rn`, `rn_mem`, `rn_near` | Code reading. Go may fuse `x*y+z` across statements and inlined calls on arm64/ppc64/s390x. Dated check (Go 1.27.4, `GOARCH=arm64 -tags race -gcflags=-S ./internal/proofbound/`): no `FMADDD`/`FMSUBD` in `ProductUpper`, `BoundedMul`, `AbsSumUpper`, `UpRound`; `NextUp` is called, not inlined | `proofbound` `AbsSumUpper`, `BoundedMul`, `ProductUpper`, `ProvenUpRound`, `UpRound`; `proof.ProvenUpRound` |
| A2 | `proof.NextUp(f)` is the least float above finite `f` (both build files) | `nu`, `nu_mem`, `lt_nu`, `nu_min` | `TestFloatStepMatchesMath` (`internal/proof/float_step_test.go`) against `math.Nextafter` | `float_step.go NextUp`, `float_step_plain.go NextUp` |
| A3 | Floats are multiples of `2^-1074` = `math.SmallestNonzeroFloat64`, so a float sum is 0 or at least `δ` in magnitude | `δ`, `δ_pos`, `δ_mem`, `add_grid`, `zero_mem`, `neg_mem` | IEEE 754 encoding; code reading of `proof.ProvenUpRound` | `rounding.go ProvenUpRound` |
| A4 | `proof.MulRoundError(a, b, a*b)` returns a float ≥ `|a·b − held|` (FMA residual exact under the `Ilogb` gate; else `RationalFloatError`) | hypotheses `hr`, `hrF` of `boundedMul_bound_sound` | Gate derivation in `ExactFloatSquare`'s doc comment; `internal/proof/rounding_test.go` | `rounding.go MulRoundError`, `RationalFloatError`, `FloatRat` |
| A5 | `proof.Abs(x) = |x|` for finite `x` (both build files) | `|·|` in `absSum` | Code reading; `float_step_test.go` | `float_step.go Abs`, `float_step_plain.go Abs`, `proofbound.AbsSumUpper` |
| A6 | Input bounds are finite and non-negative; operand values finite | hypotheses `hEa0`, `hEb0`; finiteness implicit | `BoundedScalar` construction sites; not checked | `bounded.go BoundedMul`, `MeasuredScalar` |
| A7 | No overflow: every modeled intermediate is finite. `+Inf` results and the `+Inf` refusal branches are not modeled | omitted | `ProductUpper`'s `+Inf` branch; code reading | `bounded.go ProductUpper` |
| A8 | `math/big`: `QuoRem` truncates (`n = d·q + r`, `|r| < d`, `r` has `n`'s sign); `Lsh` multiplies by `2^k`; `BitLen` is `Nat.size` of the absolute value; `lazyFrac(n, d)` represents `n/d` | `Int.tdiv`, `Int.tmod`, `Nat.size` in `lazyRoundModel` | Go `math/big` docs; `lazy_round_test.go` | `lazy_rat.go lazyFrac`, `LazyRoundDown`, `LazyRoundUp`, `lazyRound` |

- Adding a row = new id `A<n+1>`. NEVER renumber. Delete a row only when no manifest entry cites it.
- `FloatModel` is the seam for discharging A1 to A3: a concrete binary64 model instantiating it proves its
  fields, and downstream theorems take `M : FloatModel` and need no edit.
- The checker fails when a manifest entry cites an id with no row starting `| A<n> |` in this file.

## Trace manifest

`_proofs/trace.json`. Every decad function on a proved path is an entry: the function a theorem models, and
each decad function it calls whose behavior the proof relies on (as `assumes`). Standard-library calls are
covered by an assumption row, not an entry.

```json
{
  "entries": [
    {
      "file": "internal/proofbound/bounded.go",
      "func": "BoundedMul",
      "role": "models",
      "theorems": ["Decad/Bounded/Mul.lean:Decad.Bounded.boundedMul_bound_sound"],
      "assumptions": ["A1", "A4", "A6"],
      "sha256": "<64 hex>"
    }
  ]
}
```

| Field | Rule |
|---|---|
| `file` | Repo-relative path. Key is (`file`, `func`), so same-named functions in build-tag twins are separate entries. |
| `func` | `Name` for a function, `Type.Name` for a method (receiver type without `*`). |
| `role` | `models`: a Lean def mirrors it; needs at least 1 theorem. `assumes`: an assumption row states its behavior; needs at least 1 assumption. |
| `theorems` | `<file under _proofs>:<fully qualified name>`. |
| `assumptions` | Assumption ids. |
| `sha256` | See Hash definition. |

- Entries are sorted by (`file`, `func`) and have no duplicate keys. The file is pretty-printed with a 2-space
  indent and a trailing newline.
- Each covered function's doc comment ends with exactly this line:

```go
// Lean proofs depend on this function; see _proofs/trace.json.
```

- An undocumented covered function gets one line starting with its name that states what the code does.

## Checker

`_proofs/tracecheck` is a standard-library-only `package main`. Its tests use `testing` without testify, so
the module has no requirements and no `go.sum`.

### Hash definition

`sha256` of the exact source bytes from the `func` keyword through the closing `}`:
`src[fset.Position(fd.Pos()).Offset : fset.Position(fd.End()).Offset]` for the `*ast.FuncDecl`.

- It includes the receiver, signature, body and comments inside the body.
- It excludes the doc comment, so editing the pointer line or the doc text never changes the hash.
- It is whitespace-sensitive. Repo Go is gofmt'd, so this only matters on real edits.
- `go/parser` ignores build tags, so both files of a build-tag twin parse.

### Checks

All must pass. Each failure is one line naming file, func and the next action.

| # | Check | Failure line starts with |
|---|---|---|
| 1 | Manifest parses; fields valid; sorted; no duplicate keys | `<file> <func>: ...` |
| 2 | `file` exists; `func` found exactly once; hash matches | `<file> <func>: source changed; re-check <theorems> ...` |
| 3 | Each theorem's Lean file exists and has a line matching `^\s*(theorem\|lemma)\s+<last name component>\b` | `<file> <func>: theorem <name> not declared in _proofs/<lean file>` |
| 4 | Each assumption id has a row starting `\| A<n> \|` in this file | `<file> <func>: assumption A<n> has no row` |
| 5 | Each entry's Go doc comment contains `_proofs/trace.json` | `<file> <func>: doc comment lacks the pointer` |
| 6 | Reverse: every function in a repo `.go` file (skipping `_`/`.`-prefixed directories and `_test.go`) whose doc comment contains `_proofs/trace.json` is an entry | `<file> <func>: doc comment points to ... but the manifest has no entry` |
| 7 | Each entry's `file` appears single-quoted in `.github/workflows/proofs.yml` | `<file> <func>: '<file>' is missing from .github/workflows/proofs.yml` |

### CLI

Run from `_proofs`:

- `go run ./tracecheck` runs the checks and exits 0 or 1. `-repo <dir>` sets the repository root (default `..`).
- `go run ./tracecheck -update` rewrites the `sha256` of every existing entry whose function is found, prints
  each change, then runs every check. It NEVER adds or removes entries.
- Run `-update` ONLY after re-reading each affected theorem against the changed Go code. The PR diff of
  `trace.json` shows reviewers which theorems the change touched.

## CI workflow

`.github/workflows/proofs.yml` is separate from `ci.yml`, which does not run it. It is NOT a required check: a
path-filtered required check stays pending on PRs it skips. **NEVER merge a PR whose `Proofs` workflow failed**,
even when `CI gate` is green.

- Triggers: `pull_request` and `push` to `main` for the `paths` list (`_proofs/**`, the workflow, this file and
  each covered Go file), and `workflow_dispatch`. The `paths` list is a YAML anchor shared by both triggers.
  Check 7 reads the file as text, so a duplicated list passes as well.
- Job `trace` (`proofs trace`, 5-minute cap): checkout, `actions/setup-go` with `go-version-file: _proofs/go.mod`,
  then `go vet ./...`, `go test ./...` and `go run ./tracecheck` in `_proofs`.
- Job `lean` (`proofs build`, 30-minute cap): checkout, then `leanprover/lean-action` pinned by full commit SHA
  with the version in a trailing comment, as every action in this repository is. Dependabot's `github-actions`
  entry updates the pin.

| lean-action input | Value | Reason |
|---|---|---|
| `lake-package-directory` | `_proofs` | The Lake project is not at the repository root. |
| `use-github-cache` | `false` | The repository's Actions cache is at its quota; a `.lake` entry would evict `ci.yml`'s Go build caches. `lake exe cache get` downloads Mathlib's artifacts from Mathlib's servers instead. |
| `use-mathlib-cache` | `true` | Runs `lake exe cache get`. |
| `mk_all-check` | `true` | Fails when a `Decad/**.lean` file is not imported by `Decad.lean`, so `lake build` cannot skip it. |
| `build`, `build-args` | `true`, `--wfail` | Warnings fail the build. |
| `axiom-audit`, `axiom-audit-root` | `true`, `Decad` | Fails when a `Decad.*` declaration depends on an axiom outside `propext`, `Classical.choice`, `Quot.sound`: catches `sorry`, `admit`, `native_decide` and any `axiom`. |
| `test`, `lint`, `leanchecker` | `false` | No Lake test or lint driver is configured. `leanchecker` replays declarations the kernel already checked. |

lean-action runs elan's `elan-init.sh` from elan's `master` branch, which the action SHA does not pin. The Lean
version is pinned by `lean-toolchain`.

| Failure | Meaning | Action |
|---|---|---|
| `trace` red, "source changed" | A covered Go function changed | Re-check the theorem; fix the proof or the code; then `-update` |
| `trace` red, workflow path missing | A new entry's file is not in `paths` | Add it to the `paths` list in `proofs.yml` |
| `lean` red at build | Proof broken or new warning | Fix the Lean file |
| `lean` red at mk_all | New file not imported | `lake exe mk_all` in `_proofs`, commit `Decad.lean` |
| `lean` red at axiom-audit | `sorry`/`axiom`/`native_decide` reached a `Decad` declaration | Remove it |
| `lean` red at cache get | Mathlib cache server down, or artifacts for the pin no longer served | Rerun; if the artifacts are gone, upgrade Mathlib |

A workflow edit needs a token with the `workflow` scope to merge.

## Recipe: cover a new Go function

1. Read the Go function. List every decad function it calls on the path the guarantee depends on.
2. Write an `@[expose]` Lean def mirroring its control flow in `Decad/<Area>/<Topic>.lean`. Use ℚ/ℤ for exact
   arithmetic and `M : FloatModel` for float arithmetic. A fact Lean cannot see (hardware, standard library, a
   callee you do not model) becomes a hypothesis or a model-structure field, plus a new assumption row.
   NEVER an `axiom`.
3. Prove the end-to-end theorem stating the published guarantee about the def. Write its docstring per the
   naming rules above.
4. `lake exe mk_all` (updates `Decad.lean`), then `lake build --wfail`.
5. Add `trace.json` entries: `models` for the function, `assumes` for each relied-on callee. Add the pointer
   line to each. Put placeholder hashes of 64 zeros, run `go run ./tracecheck -update`, then
   `go run ./tracecheck`.
6. A new Go file → add it to the `paths` list in `proofs.yml`.
7. Run the axiom audit (Run locally).

A non-float area (exact predicates, geometry) adds its own model structure the way `FloatModel` does, only when
its assumptions differ. Exact integer or rational code needs no model structure.

## Upgrade Lean and Mathlib

One PR per upgrade, nothing else in it.

1. Pick a Mathlib release tag `vX` (Mathlib tags match Lean releases).
2. Set `rev = "vX"` in `lakefile.toml`; in `_proofs` run `lake update mathlib`.
3. Copy `.lake/packages/mathlib/lean-toolchain` to `_proofs/lean-toolchain`.
4. `lake exe cache get`, then `lake build --wfail`. Fix deprecations and broken proofs; NEVER silence them.
5. Run the axiom audit and `go run ./tracecheck`.
6. Commit `lakefile.toml`, `lean-toolchain`, `lake-manifest.json` and the Lean fixes.

- NEVER run `lake update` without step 2's pinned `rev`; it moves every transitive package.
- Dependabot does not update Lean packages.
- Upgrade when a needed Mathlib lemma is newer, or when `cache get` stops serving the pin.

## Run locally

Put `~/.elan/bin` on `PATH`. In `_proofs`:

| Step | Command |
|---|---|
| Download Mathlib artifacts | `lake exe cache get` |
| Build, warnings fatal | `lake build --wfail` |
| Imports complete | `lake exe mk_all --check` |
| Trace checker tests | `go vet ./... && go test ./...` |
| Manifest check | `go run ./tracecheck` |

Axiom audit, as lean-action runs it: clone `https://github.com/leanprover-community/axiom-audit` at tag `v0.1.2`
into a scratch directory, check `HEAD` is `46024e005996495c65ef609368e11ab39c4222e3`, copy
`_proofs/lean-toolchain` over its `lean-toolchain`, run `lake build` there, then in `_proofs` run
`lake env <clone>/.lake/build/bin/axiom-audit --allow propext,Classical.choice,Quot.sound --root Decad`.
