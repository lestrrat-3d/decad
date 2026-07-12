# Roadmap

The state of decad's design. Two parts: what is **settled** (contracts that new
code must not violate) and what is **open** (decisions that must be made in a
design doc before the corresponding code exists).

## Settled

These hold regardless of how the kernel turns out.

| Contract | Rule |
|---|---|
| **Layering** | `decad -> sketch -> r3`. NEVER the reverse. decad may import both; neither imports decad. |
| **2D is not decad's job** | Profile closure, DOF, constraint conflicts, sketch validity → ask `sketch`. NEVER re-derive a 2D answer here. |
| **Coordinates are not decad's job** | Vectors, frames, local↔world transforms → `r3`. NEVER hand-roll a transform or a matrix solve. |
| **Shapes ARE decad's job** | Solids, surfaces, meshes, features, topology. `r3` excludes them by charter; this is the layer that owns them. |
| **Headless** | No GUI, no viewer, no interactive loop. The API is the only interface. |
| **Verification is the product** | Every modeling capability must be answerable to a machine: not "it rendered", but "it is watertight, its volume is X, it does not interfere". |
| **Correctness is observable** | Every capability ships with a test asserting on computed geometry — coordinates, volumes, residuals — never merely "it ran". |
| **Curated dependencies** | stdlib + `sketch` + `r3` + `testify` (test-only). Adding a module requires recording the decision in `CLAUDE.md`. |

## Open

Nothing below is decided. Each needs a design doc in `docs/` before it is built.

### The kernel representation — the fork everything else hangs off

The central unanswered question: **what is a `Body`?**

* **B-rep** (exact faces/edges/vertices, analytic surfaces) — what Fusion
  actually is, so verification results transfer most faithfully. Booleans on
  exact surfaces are the hardest thing in this repository, and the reason
  commercial kernels are decades old.
* **Mesh** (triangles) — tractable, and enough to answer volume, watertightness,
  bbox and interference. But a mesh has already thrown away the analytic
  surfaces, so tangency/fillet questions get approximate answers, and
  agent-authored parts are exactly where exactness matters.
* **Hybrid** — exact feature tree as the document, mesh as the evaluated form
  for analysis. Likely, but it must be designed, not defaulted into.

Everything downstream (features, booleans, exports, verification depth) is
determined by this choice, so it is the first design doc to write.

### Features

Which modeling operations exist, and their exact semantics: extrude, revolve,
sweep, loft, fillet/chamfer, shell, pattern, boolean union/subtract/intersect.
Open: whether the document is a **feature tree** (re-evaluated on parameter
change, like Fusion's timeline — which would make decad parametric end-to-end,
inheriting `sketch`'s parametric dimensions) or a flat pile of bodies.

### The sketch seam

How a `sketch.Profile` becomes 3D input. `sketch` hands over an ordered
`[]BoundaryEdge` loop of whole-or-fragment entities plus holes, in plane-local
2D; the plane's `r3.Frame` lifts it to world. Open: how curved boundaries
(spline, NURBS, conic) cross the seam — exactly, or tessellated, and if
tessellated, who chooses the tolerance.

### Verification — the north star

The 3D analog of `sketch.Verify`: one non-mutating call returning a report an
agent can gate on. Candidate signals, none yet specified:

* watertight / manifold / self-intersection
* volume, surface area, centroid, moments of inertia, bounding box
* minimum wall thickness, draft, undercut
* interference between bodies, and clearance
* a `Trustworthy()` verdict aggregating them — the single bit an agent branches on

### Exchange formats

STL, OBJ, 3MF, STEP; JSON save/load of the document. STEP is the one that would
actually round-trip into Fusion, and is also the most work.

### Fusion correspondence

The point of the exercise is that a verified decad model **translates** to CAD
code. Open: whether decad emits the Fusion script directly, or only proves the
geometry and leaves the agent to write it.
