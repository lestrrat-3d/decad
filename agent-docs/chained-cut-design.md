# Chained Cuts: Fine Partner Chord and Fine Twist Refinement

A tube bored by several twisted `Sweep` tools, then cut by prism windows,
chained on one body. Companion to `agent-docs/twist-sweep-cut-design.md` (the
refinement this extends), `agent-docs/faceted-vertex-bounds-design.md` §3 and
§5 (the bound composition and the chain-depth gate), `agent-docs/api-design.md`
§8 "The chain depth" and `agent-docs/general-boolean-design.md` (the mesh path).

## 1. The problem

The case is a tube (outer radius 18.5 mm, inner 12.5 mm, 37.5 mm
tall) bored by twisted rectangular tools (15.4 x 4.15 mm, 7.27 degrees of twist
per mm, 10.9 mm of axis, 79 degrees) along two axes 80 degrees apart, then cut
by two slanted prism windows. Every `Cut` after the first was refused:

> cut's target holds its mesh, where this pair meets, only within a bound of
> 1.33 mm, coarser than this pair's chord tolerance 0.385 mm

A first cut that reached its inner end 0.9 mm from the tube's inner wall was
refused as well, by the contact gate.

## 2. Cause

Three separate terms stack.

1. **The tool sets the pair tolerance.** A twisted tool that is already
   stationed (any angle above one radian) holds its mesh within `Delta` =
   0.39 to 0.43 mm: the stationed build aims at 4% of the profile radius.
   `booloperand.heldPrimitiveFloor` raises the pair's chord tolerance to that
   floor, 0.385 mm against a diameter-derived 0.0012 mm.
2. **The tolerance chords the partner.** Both operands were meshed at the pair
   tolerance, so the tube's two cylinders were chorded to a 0.3 mm sagitta.
   The contact gate's slack (`deltaA + deltaB`, tessellation §11 step 4) was
   then 0.7 to 0.9 mm, wider than the clearance of the tool's inner end. The
   cylinders' volume allowance (`delta * area`) was 200 mm3, larger than all
   four bores together.
3. **A rim vertex amplifies, and a facet takes its worst corner.** A vertex the
   cut creates carries `(delta_target + delta_tool) / sin theta`
   (vertex-bounds §3.2), 0.7 to 1.33 mm here. A facet's bound is the largest of
   its three corners (§2). A chorded cylinder wall is a strip as tall as the
   tube and 0.17 mm wide, so the strip a rim vertex lands in becomes triangles
   that run the whole 37.5 mm and carry the rim's bound along all of it.
   The next tool's pair touches those strips wherever it crosses the same
   angular range, even 14 mm higher, and §5's gate refuses: the strip's bound
   exceeds the pair's tolerance, which only the next tool's floor sets.

Windows after bores meet the same strips against a tolerance of
`2e-5 * diameter` (1.4e-3 mm), so a window anywhere over a rimmed strip is
refused.

## 3. Choice

The gate is policy, not a proof: a facet's bound is proven wherever it is
admitted, and `WithChainTolerance` already admits a larger one with every bound
still carried. The fix therefore shrinks the terms that make the bounds large
and the touched set wide, and leaves the gate as it is.

| Option | Verdict |
|---|---|
| Restate the tube finer (4.1) | Taken. Slack falls from 0.9 mm to 0.05 mm, the cylinders' volume allowance from 200 mm3 to about 10 mm3, and a tool that clears the wall by less than 0.9 mm is admitted. |
| Refine the tool finer (4.2) | Taken. The tool's own `Delta` is the largest term left, and it sets the tool's volume allowance and the rim bounds. |
| Keep the cylinder analytic through the cut | Not taken. The result of a mesh Boolean is a `Faceted` body of flat triangles by design (vertex-bounds §2.1); an analytic wall is a different payload class and a different route. |
| A multi-tool `Cut(target, tools...)` | Not taken. It needs a new public signature and a proof that the tools are pairwise apart, and it removes only the tool-on-tool rim contacts. With 4.1 and 4.2 the four bores no longer touch one another's rims, so it buys nothing here. |
| Subdivide a chorded cylinder's strips along the axis | Not taken. It would keep a rim's bound local to its neighbourhood and admit a window over a rimmed strip. It changes every prism and revolve tessellation and multiplies their facet counts; §6 states what is left without it. |

## 4. Design

### 4.1 A partner is chorded at its own tolerance

`meshbool.OperandChord(pairTol, diameter, floor, restating)` is the chord
request for one operand of a pair. An operand that restates a held mesh keeps
the pair tolerance, as before. Every other operand is requested at
`max(diameter * ChordFactor, its own floor)`, never above the pair tolerance.

`booleaneval.evaluate` and `Evaluate`'s shifted retry use it for both operands,
and `verifyrun.newVerifyMeshCache` uses it for each shared mesh, so `Verify`
meshes a body as the construction did. The pair tolerance itself, the gate, the
volume proofs and `dPair` are unchanged. The request is finer only when another
operand's held floor raised the pair tolerance, so a pair of two analytic
operands is meshed exactly as before.

Soundness. A mesh publishes the sagitta it took, and every proof reads that
published figure, never the request (tessellation §2). A finer request makes
the published figure smaller. The tangency gate's slack is a sum of published
displacements, so it falls with them, and the gate still admits a meet only on
a witness deeper than the slack. No gate changes.

Cost. The tube's outer cylinder goes from about 17 to about 180 segments. A coil
or mitred-sweep cut of a larger part meshes its partner finer by the same
ratio, up to `MaxFacetPairTestsPerCall`, which still refuses an excess.

### 4.2 A twisted sweep refines to a finer, fixed target

`twistsweep.Payload.RefinedDelta` and `RefinedMesh` (twist-sweep-cut §3) now

- apply to a sweep that already holds a stationed mesh, not only a one-cell
  sweep, so a twist of more than one radian refines too;
- aim at `refineTargetScale` = 0.25% of the profile's largest L1 radius, a
  tenth of the stationed build's 4% target;
- use `refinedCellCount`: the fewest cells that reach the target, else the
  most the ceilings allow (`maxRefinedTwistCells` = 512 and the facet-pair
  ceiling). A refinement that is no closer than the sweep's own `Delta` is
  still not offered.

For the sleeve tool this is about 220 cells and a `Delta` of about 0.025 mm,
against 0.43 mm.
The proofs are the stationed build's own and are unchanged
(twist-sweep-cut §3, Soundness): `RefinedMesh` refuses when the mesh proves
more than `RefinedDelta` promised.

## 5. What is admitted

Measured on the sleeve (apitest `TestCutChainedTwistedBoresAndWindows`):

- Four one-sided bore cuts (stations 8.6 to 19.5 mm and -19.5 to -8.6 mm on
  each axis) are admitted by default, then two narrow slanted prism windows
  placed in the angular gaps the bores leave (centred on the +Y and -Y
  sides).
- The result is a solid, `Tessellate(VerifyAll)` passes, and the volume lies
  within its certified bound of the tube minus an independently integrated
  overlap.
- Certified figures for the chain: volume 20020 mm3 with bound 104 mm3 (0.5% of
  the tube; 26 mm3 per bore), boundary bound 0.051 mm (`Mesh.Bound`). A bore
  clearance of 0.2 mm is proven four times over by the boundary bound.
- A window wide enough to reach a bore's rim is refused by default with the
  chain-depth message (it meets rimmed strips, §2 item 3) and admitted with
  `WithChainTolerance(0.1 mm)`, with the same volume check
  (`TestCutChainedWindowOnBoreRims`).

Refused, soundly:

- A prism window over a rimmed strip, as above, without
  `WithChainTolerance`.
- One through-sweep per bore axis (285 degrees, 39 mm): the refinement reaches only
  the cell ceiling (`Delta` = 0.054 mm) and the second cut still touches the
  first one's rim at 0.088 mm.
- A true crossing or gap under the new slack (about 0.05 mm).

## 6. Not done

Bore cuts that follow a window cut, and windows that follow bore cuts and sit
over a bored strip, still need `WithChainTolerance`. Cutting the windows
first avoids it: their rims carry about 0.004 mm, far under a bore pair's
tolerance. A real fix keeps a rim's bound local to a facet neighbourhood, by
axial subdivision of a chorded wall or by a bound that weights corners at the
touched point (vertex-bounds §2). Either is its own change.

## 7. Size

About 60 lines of Go in `internal/twistsweep/refine.go` and
`build_stationed.go`, 20 in `internal/meshbool/chord.go` and
`internal/booleaneval/eval.go`, 5 in `internal/verifyrun/pairs.go`, and
`apitest/chained_twist_cut_test.go`.
