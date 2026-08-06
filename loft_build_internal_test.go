package decad

import (
	"context"
	"fmt"
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// Fixtures. pt builds a plane-local point; squareLoop a 4-segment LineSeg
// square loop, CCW or CW; planeAt a plane record with the standard XY basis
// at the given world origin.

func pt(u, v float64) Point2 { return Point2{U: u, V: v} }

func squareLoop(cx, cy, half float64, ccw bool) LoopRecord {
	corners := []Point2{
		pt(cx-half, cy-half), pt(cx+half, cy-half), pt(cx+half, cy+half), pt(cx-half, cy+half),
	}
	if !ccw {
		corners = []Point2{corners[0], corners[3], corners[2], corners[1]}
	}
	segs := make([]CurveSegment, 4)
	for i := range corners {
		segs[i] = LineSeg{Start: corners[i], End: corners[(i+1)%4], TStart: 0, TEnd: 1}
	}
	return LoopRecord{Segments: segs}
}

func triangleLoop() LoopRecord {
	corners := []Point2{pt(0, 0), pt(1, 0), pt(0.5, 1)}
	segs := make([]CurveSegment, 3)
	for i := range corners {
		segs[i] = LineSeg{Start: corners[i], End: corners[(i+1)%3], TStart: 0, TEnd: 1}
	}
	return LoopRecord{Segments: segs}
}

func unitSquareProfile() ProfileRecord {
	return ProfileRecord{Outer: squareLoop(0.5, 0.5, 0.5, true)}
}

func planeAt(origin r3.Vec) PlaneRecord {
	return PlaneRecord{Origin: origin, U: r3.NewVec(1, 0, 0), V: r3.NewVec(0, 1, 0)}
}

func mustFrame(t *testing.T, pr PlaneRecord) r3.Frame {
	t.Helper()
	f, err := r3.NewFrame(pr.Origin, pr.U, pr.V)
	require.NoError(t, err)
	return f
}

// boxLoftPayloadOn is the untwisted unit-square-to-unit-square loft with p0
// recorded on the z=z0 plane and p1 on z=z1. Both spellings describe the same
// unit box; which one is p0 decides whether §5's whole-shell orientation step
// has to reverse the raw winding.
func boxLoftPayloadOn(t *testing.T, z0, z1 float64) loftPayload {
	t.Helper()
	p := unitSquareProfile()
	pl0 := planeAt(r3.NewVec(0, 0, z0))
	pl1 := planeAt(r3.NewVec(0, 0, z1))
	return loftPayload{
		profile0: p, profile1: p,
		plane0: pl0, plane1: pl1,
		frame0: mustFrame(t, pl0), frame1: mustFrame(t, pl1),
		xform: r3.Identity(),
	}
}

// boxLoftPayload is fixture U of docs/loft-design.md §13, whose own raw
// (pre-flip) triangle winding is already outward, so it also exercises the
// "no flip needed" path.
func boxLoftPayload(t *testing.T) loftPayload {
	t.Helper()
	return boxLoftPayloadOn(t, 0, 1)
}

func evalLoftFixture(t *testing.T, pl loftPayload) *Body {
	t.Helper()
	budget := newWorkBudget(t.Context())
	body, err := evalLoft(t.Context(), New(), StepRef(0), pl, budget, newFreeformWork(), newFreeformWork())
	require.NoError(t, err)
	return body
}

// TestEvalLoftUnitBoxTopology proves the shape of Table B for the box
// fixture: 10 faces (8 wall triangles + 2 caps), every edge bounding exactly
// two faces, the roles exactly side(0,j,k) plus capStart/capEnd, and every
// wall face's surface a Plane.
func TestEvalLoftUnitBoxTopology(t *testing.T) {
	body := evalLoftFixture(t, boxLoftPayload(t))

	faces := body.Faces()
	require.Len(t, faces, 10, "8 wall triangles + capStart + capEnd")

	edges := body.Edges()
	require.Len(t, edges, 16, "4 bottom rim + 4 top rim + 4 diagonal + 4 rung")
	for _, e := range edges {
		require.Lenf(t, e.Faces(), 2, "every loft edge must bound exactly two faces, got %d", len(e.Faces()))
	}

	require.Len(t, body.Vertices(), 8)

	var capRoles []string
	var wallRoles []string
	for _, f := range faces {
		roles := f.Origins()
		require.Len(t, roles, 1)
		role := roles[0].Role
		if role == roleCapStart || role == roleCapEnd {
			capRoles = append(capRoles, role)
			continue
		}
		wallRoles = append(wallRoles, role)
		_, isPlane := f.Surface().(Plane)
		require.True(t, isPlane, "every wall face's surface must be a Plane")
	}
	require.ElementsMatch(t, []string{roleCapStart, roleCapEnd}, capRoles)

	var wantWallRoles []string
	for j := range 4 {
		wantWallRoles = append(wantWallRoles, fmt.Sprintf("side(0,%d,0)", j), fmt.Sprintf("side(0,%d,1)", j))
	}
	require.ElementsMatch(t, wantWallRoles, wallRoles)
}

// TestEvalLoftRolesUseTheGivenStepRef proves every role FeatureRef the build
// mints — the body's own origin and every face's role — carries the StepRef
// evalLoft was actually called with, not a hardcoded one.
func TestEvalLoftRolesUseTheGivenStepRef(t *testing.T) {
	const ref = StepRef(7)
	budget := newWorkBudget(t.Context())
	body, err := evalLoft(t.Context(), New(), ref, boxLoftPayload(t), budget, newFreeformWork(), newFreeformWork())
	require.NoError(t, err)

	require.Equal(t, ref, body.Origin().Step)
	for _, f := range body.Faces() {
		for _, o := range f.Origins() {
			require.Equal(t, ref, o.Step)
		}
	}
}

// TestEvalLoftUnitBoxMeasurements proves §8's closed forms against the box's
// own known-exact answers: unit volume, centroid at the cube's own center,
// exact bounds, and an Area that is never Exact even though every individual
// wall triangle's own area is exactly representable (0.5 mm² each) — the
// summation loop's own slop still keeps the composed bound positive.
func TestEvalLoftUnitBoxMeasurements(t *testing.T) {
	body := evalLoftFixture(t, boxLoftPayload(t))

	require.InDelta(t, 1.0, body.volume.Value.Base(), 1e-12)
	require.Equal(t, 0.0, body.volume.Bound.Base())
	require.Equal(t, Exact, body.volume.Exactness)

	require.InDelta(t, 0.5, body.centroid.Value.X, 1e-12)
	require.InDelta(t, 0.5, body.centroid.Value.Y, 1e-12)
	require.InDelta(t, 0.5, body.centroid.Value.Z, 1e-12)
	require.Equal(t, Exact, body.centroid.Exactness)

	require.Equal(t, r3.NewVec(0, 0, 0), body.bounds.Min)
	require.Equal(t, r3.NewVec(1, 1, 1), body.bounds.Max)
	require.Equal(t, Exact, body.bounds.Exactness)
	require.Equal(t, 0.0, body.bounds.Bound.Base())

	require.InDelta(t, 6.0, body.area.Value.Base(), 1e-9)
	require.Equal(t, Approximate, body.area.Exactness)
	require.Greater(t, body.area.Bound.Base(), 0.0)
}

// TestEvalLoftFaceAreasSumToBodyArea proves every face's own Area().Value
// sums to Body.Area().Value within the composed bound — the capblend
// regression shape (a face left with an unset area would silently overstate
// the body's own coverage).
func TestEvalLoftFaceAreasSumToBodyArea(t *testing.T) {
	body := evalLoftFixture(t, boxLoftPayload(t))

	sum := 0.0
	for _, f := range body.Faces() {
		a, err := f.Area()
		require.NoError(t, err)
		sum += a.Value.Base()
	}
	require.InDelta(t, body.area.Value.Base(), sum, body.area.Bound.Base()+1e-9)
}

// TestEvalLoftWholeShellOrientationCorrectsSign proves the whole-shell
// orientation rule (§5): building the SAME box with capStart and capEnd
// swapped (p0 at z=1, p1 at z=0) produces raw triangle winding whose signed
// tetrahedron sum is negative before the flip, yet the published Volume is
// still the correct positive 1 mm³ once the whole-shell flip has run.
func TestEvalLoftWholeShellOrientationCorrectsSign(t *testing.T) {
	body := evalLoftFixture(t, boxLoftPayloadOn(t, 1, 0))
	require.InDelta(t, 1.0, body.volume.Value.Base(), 1e-12)
	require.Equal(t, Exact, body.volume.Exactness)
}

// TestEvalLoftLoopWalksFollowTheFaceNormal proves §5's whole-shell
// orientation step reaches the PUBLISHED DIRECTED BOUNDARY and not merely the
// triangle set each face's Plane is rebuilt from. Volume, area and convexity
// all read the flipped triangles and so agree either way; only the loop walk
// — Loop.CoEdges, CoEdge.Start/End/IsForward — can witness a shell whose
// faces were reversed while their walks were not.
//
// Both spellings of the same unit box are checked, because the defect this
// pins is invisible on the one whose raw winding already came out outward.
// For every face: the walk is a real closed chain, the face's own normal
// genuinely points away from the box centre, and the walk's Newell normal
// agrees with it — decad's material-on-the-left convention.
func TestEvalLoftLoopWalksFollowTheFaceNormal(t *testing.T) {
	// The unit box occupies [0,1]^3 under either spelling, so this is its
	// interior centre in both.
	center := r3.NewVec(0.5, 0.5, 0.5)

	for _, tc := range []struct {
		name   string
		z0, z1 float64
	}{
		{name: "raw winding already outward", z0: 0, z1: 1},
		{name: "raw winding reversed by the whole-shell step", z0: 1, z1: 0},
	} {
		t.Run(tc.name, func(t *testing.T) {
			body := evalLoftFixture(t, boxLoftPayloadOn(t, tc.z0, tc.z1))

			faces := body.Faces()
			require.Len(t, faces, 10, "8 wall triangles + capStart + capEnd")
			for _, f := range faces {
				role := f.Origins()[0].Role
				loops := f.Loops()
				require.Lenf(t, loops, 1, "face %s has no hole in this fixture", role)

				co := loops[0].CoEdges()
				require.GreaterOrEqualf(t, len(co), 3, "face %s", role)

				walk := make([]r3.Vec, len(co))
				for k, ce := range co {
					require.Samef(t, ce.End(), co[(k+1)%len(co)].Start(),
						"face %s: coedge %d must end where coedge %d starts", role, k, (k+1)%len(co))
					walk[k] = ce.Start().Position().Value
				}

				// Newell's normal of the closed walk (twice the signed area
				// vector), and the walk's own centroid.
				area2 := r3.NewVec(0, 0, 0)
				centroid := r3.NewVec(0, 0, 0)
				for k, a := range walk {
					b := walk[(k+1)%len(walk)]
					area2 = area2.Add(a.Cross(b))
					centroid = centroid.Add(a)
				}
				centroid = centroid.Scale(1 / float64(len(walk)))

				n, err := f.NormalAt(centroid)
				require.NoError(t, err)

				require.Greaterf(t, centroid.Sub(center).Dot(n.Value), 0.0,
					"face %s: its own normal must point away from the box centre", role)
				require.Greaterf(t, area2.Dot(n.Value), 0.0,
					"face %s: its loop walk must run material-on-the-left about its own outward normal", role)
			}
		})
	}
}

// TestEvalLoftJunctionConvexity hand-verifies §5's junction rule on the box
// fixture. A box's vertical edges (the rungs) are genuine 90-degree convex
// corners; its diagonals split a FLAT wall quad (no twist at all in this
// fixture) and are therefore flat, non-convex edges per §5's own stated rule
// ("a zero result retains a flat rung or diagonal as a decided non-convex
// edge"). Rim edges take the outer/hole rule: all four outer rims (both cap
// boundaries) are convex.
func TestEvalLoftJunctionConvexity(t *testing.T) {
	body := evalLoftFixture(t, boxLoftPayload(t))

	rungs, diagonals, rims := 0, 0, 0
	for _, e := range body.Edges() {
		startZ := e.Start().Position().Value.Z
		endZ := e.End().Position().Value.Z
		length, err := e.Length()
		require.NoError(t, err)
		mm := length.Value.Base()
		switch {
		case startZ == endZ: // a rim: both endpoints on the same cap plane
			rims++
		case mm > 0.99 && mm < 1.01: // a rung: box height 1, spans both caps
			rungs++
			require.True(t, e.IsConvex(), "a box's vertical edge is a genuine convex corner")
		case mm > 1.41 && mm < 1.43: // a diagonal: sqrt(2) across a unit face, spans both caps
			diagonals++
			require.False(t, e.IsConvex(), "an untwisted box's wall diagonal is flat, not convex")
		default:
			t.Fatalf("unclassified edge: length %v, z %v -> %v", mm, startZ, endZ)
		}
	}
	require.Equal(t, 4, rungs)
	require.Equal(t, 4, diagonals)
	require.Equal(t, 8, rims, "4 bottom rim + 4 top rim")
}

// TestEvalLoftHoleRimIsConcave proves the rim rule's other half: a hole's
// rim edges are concave while the outer rim's are convex, exactly as a
// prism's already are (topology.go's Edge.IsConvex doc).
func TestEvalLoftHoleRimIsConcave(t *testing.T) {
	p := ProfileRecord{
		Outer: squareLoop(0.5, 0.5, 0.5, true),
		Holes: []LoopRecord{squareLoop(0.5, 0.5, 0.2, false)},
	}
	pl0 := planeAt(r3.NewVec(0, 0, 0))
	pl1 := planeAt(r3.NewVec(0, 0, 1))
	pl := loftPayload{
		profile0: p, profile1: p,
		plane0: pl0, plane1: pl1,
		frame0: mustFrame(t, pl0), frame1: mustFrame(t, pl1),
		xform: r3.Identity(),
	}
	body := evalLoftFixture(t, pl)

	var outerRimConvex, holeRimConvex []bool
	for _, f := range body.Faces() {
		role := f.Origins()[0].Role
		if role != roleCapStart {
			continue
		}
		for _, l := range f.Loops() {
			for _, ce := range l.CoEdges() {
				length, err := ce.Edge().Length()
				require.NoError(t, err)
				if length.Value.Base() >= 0.99 { // an outer-square rim edge (length 1)
					outerRimConvex = append(outerRimConvex, ce.Edge().IsConvex())
				} else { // a hole-square rim edge (length 0.4)
					holeRimConvex = append(holeRimConvex, ce.Edge().IsConvex())
				}
			}
		}
	}
	require.NotEmpty(t, outerRimConvex)
	require.NotEmpty(t, holeRimConvex)
	for _, c := range outerRimConvex {
		require.True(t, c, "the outer loop's rim is convex")
	}
	for _, c := range holeRimConvex {
		require.False(t, c, "a hole's rim is concave")
	}
}

// TestLoftPairingsDefaultOffsetIsZero and
// TestLoftPairingsAlignmentRotatesCorrespondence prove Table P4/P6 directly
// against the pairing's own output: with no alignment, vertex k of loop0
// pairs with vertex k of loop1; with an offset, it pairs with vertex
// (k+offset) mod n — asserted on the built correspondence's own coordinates.
func TestLoftPairingsDefaultOffsetIsZero(t *testing.T) {
	p := unitSquareProfile()
	offsets := []int{0}
	pairs, err := loftPairings(p, p, offsets, newFreeformWork(), newFreeformWork())
	require.NoError(t, err)
	require.Equal(t, pt(0, 0), pairs[0].w[0])
}

func TestLoftPairingsAlignmentRotatesCorrespondence(t *testing.T) {
	p := unitSquareProfile()
	offsets := []int{1}
	pairs, err := loftPairings(p, p, offsets, newFreeformWork(), newFreeformWork())
	require.NoError(t, err)
	// loop0 segment 0 (V_0, at local (0,0)) now pairs with loop1 segment 1,
	// whose own recorded start is local (1,0) — the far endpoint of rung R_0
	// moves from W[0]=(0,0) to W[1]=(1,0).
	require.Equal(t, pt(1, 0), pairs[0].w[0])
	require.Equal(t, pt(0, 0), pairs[0].v[0])
}

// TestLoftPairingsTwoHolesPairByPosition proves P1: two holes recorded in
// SWAPPED slice order between the two profiles pair by POSITION in Holes,
// never by size or nearest match. p0's small hole (Holes[0]) pairs with
// p1's LARGE hole (also recorded at Holes[0]) — a nearest-size or
// nearest-position matcher would pick the other correspondence.
func TestLoftPairingsTwoHolesPairByPosition(t *testing.T) {
	smallHole := squareLoop(0.2, 0.2, 0.05, false)
	largeHole := squareLoop(0.8, 0.8, 0.15, false)

	p0 := ProfileRecord{Outer: squareLoop(0.5, 0.5, 0.5, true), Holes: []LoopRecord{smallHole, largeHole}}
	// p1 records the identical two hole loops but in SWAPPED slice order.
	p1 := ProfileRecord{Outer: squareLoop(0.5, 0.5, 0.5, true), Holes: []LoopRecord{largeHole, smallHole}}

	offsets := []int{0, 0, 0}
	pairs, err := loftPairings(p0, p1, offsets, newFreeformWork(), newFreeformWork())
	require.NoError(t, err)

	require.Len(t, pairs, 3) // outer + 2 holes
	// Hole loop 1 (index 1+0): p0's own small hole (v) pairs with p1's
	// Holes[0], the LARGE hole (w) — pure positional pairing.
	require.Equal(t, smallHole.Segments[0].(LineSeg).Start.U, pairs[1].v[0].U)
	require.InDelta(t, 0.65, pairs[1].w[0].U, 1e-9, "p1's Holes[0] is the large hole, centered at 0.8 with half-width 0.15")
}

// TestEvalLoftCollinearSplitKeepsTwoFacesPerCell proves that one outer side
// deliberately split into two collinear LineSegs still retains exactly two
// Plane faces per wall cell (no adjacent-coplanar-side canonicalization,
// §5's explicit exemption), and that the resulting flat rung and both flat
// diagonals at the split report IsConvex() == false.
func TestEvalLoftCollinearSplitKeepsTwoFacesPerCell(t *testing.T) {
	// Outer loop: the unit square's bottom side (0,0)->(1,0) split into two
	// collinear segments at u=0.5, followed by the square's other three
	// sides — 5 segments total, still forming the same square boundary.
	loop := LoopRecord{Segments: []CurveSegment{
		LineSeg{Start: pt(0, 0), End: pt(0.5, 0), TStart: 0, TEnd: 1},
		LineSeg{Start: pt(0.5, 0), End: pt(1, 0), TStart: 0, TEnd: 1},
		LineSeg{Start: pt(1, 0), End: pt(1, 1), TStart: 0, TEnd: 1},
		LineSeg{Start: pt(1, 1), End: pt(0, 1), TStart: 0, TEnd: 1},
		LineSeg{Start: pt(0, 1), End: pt(0, 0), TStart: 0, TEnd: 1},
	}}
	p := ProfileRecord{Outer: loop}
	pl0 := planeAt(r3.NewVec(0, 0, 0))
	pl1 := planeAt(r3.NewVec(0, 0, 1))
	pl := loftPayload{
		profile0: p, profile1: p,
		plane0: pl0, plane1: pl1,
		frame0: mustFrame(t, pl0), frame1: mustFrame(t, pl1),
		xform: r3.Identity(),
	}
	body := evalLoftFixture(t, pl)

	faces := body.Faces()
	require.Len(t, faces, 12, "2*5 wall triangles + 2 caps: no merge across the split")

	wallPlanes := 0
	for _, f := range faces {
		role := f.Origins()[0].Role
		if role == roleCapStart || role == roleCapEnd {
			continue
		}
		_, ok := f.Surface().(Plane)
		require.True(t, ok)
		wallPlanes++
	}
	require.Equal(t, 10, wallPlanes)

	// The split point (u=0.5, v=0) introduces one new rung — (0.5,0,0) to
	// (0.5,0,1) — and its two neighboring cells' own diagonals — (0,0,0) to
	// (0.5,0,1), and (0.5,0,0) to (1,0,1). All three lie within the SAME flat
	// y=0 face the rest of the untwisted box's bottom wall already occupies
	// (this fixture introduces no twist at all), so all three are decided
	// non-convex edges, not real corners.
	near := func(v r3.Vec, x, y, z float64) bool {
		return math.Abs(v.X-x) < 1e-9 && math.Abs(v.Y-y) < 1e-9 && math.Abs(v.Z-z) < 1e-9
	}
	endpoints := func(e *Edge) (r3.Vec, r3.Vec) {
		return e.Start().Position().Value, e.End().Position().Value
	}
	matches := func(e *Edge, ax, ay, az, bx, by, bz float64) bool {
		p, q := endpoints(e)
		return (near(p, ax, ay, az) && near(q, bx, by, bz)) || (near(q, ax, ay, az) && near(p, bx, by, bz))
	}
	wantFlat := 0
	for _, e := range body.Edges() {
		switch {
		case matches(e, 0.5, 0, 0, 0.5, 0, 1): // the split rung
		case matches(e, 0, 0, 0, 0.5, 0, 1): // cell 0's own diagonal
		case matches(e, 0.5, 0, 0, 1, 0, 1): // cell 1's own diagonal
		default:
			continue
		}
		require.False(t, e.IsConvex(), "an untwisted box's split-cell rung/diagonal must be flat")
		wantFlat++
	}
	require.Equal(t, 3, wantFlat, "the split rung and both neighboring cells' own diagonals")
}

// TestLoftGateDiameterIsTheVertexDiameter proves design O1's bodyGateDiameter
// arm: a loft body's tolerance-gate diameter is the exact diameter of its
// own held vertex set — for the unit box, the space diagonal sqrt(3).
func TestLoftGateDiameterIsTheVertexDiameter(t *testing.T) {
	body := evalLoftFixture(t, boxLoftPayload(t))
	d, ok, err := bodyGateDiameter(t.Context(), body)
	require.NoError(t, err)
	require.True(t, ok)
	require.InDelta(t, 1.7320508075688772, d, 1e-12) // sqrt(3)
}

// TestLoftPlacedGateDiameterShrinksByTwiceDelta proves docs/loft-design.md
// §12 PR 2a's bodyGateDiameter arm: a placed loft's own diameter reads the
// held vertex-set diameter shrunk by 2*delta, never the raw held reading —
// understating the reference tightens the tolerance gate rather than
// loosening it.
func TestLoftPlacedGateDiameterShrinksByTwiceDelta(t *testing.T) {
	unplaced := evalLoftFixture(t, boxLoftPayload(t))
	unplacedD, ok, err := bodyGateDiameter(t.Context(), unplaced)
	require.NoError(t, err)
	require.True(t, ok)

	pl := unplaced.payload.(loftPayload)
	rot, err := r3.Rotation(r3.NewVec(1, 1, 1), units.Degrees(29))
	require.NoError(t, err)

	placedBody, err := pl.placed(t.Context(), New(), StepRef(1), rot)
	require.NoError(t, err)
	placedPl := placedBody.payload.(loftPayload)
	require.Greater(t, placedPl.delta, 0.0)

	placedD, ok, err := bodyGateDiameter(t.Context(), placedBody)
	require.NoError(t, err)
	require.True(t, ok)
	require.InDelta(t, unplacedD-2*placedPl.delta, placedD, 1e-12)
}

// TestLoftPlacedGateDiameterRoundsTheShrinkOutward proves the shrink's own
// DIRECTION, which the InDelta assertion above cannot see. 2*delta is exact (a
// power-of-two scaling), so `d - 2*delta` is the one rounding in the arm, and
// round-to-nearest can land it ABOVE the exact difference — a reference larger
// than the proven one, loosening the gate the shrink exists to tighten. Each
// translation below is a measured instance of that: the bare subtraction
// overshoots the exact value by a fraction of an ulp at 2^30, 1e6 and 1e9,
// while at 2^36 it happens to round down on its own (which is exactly why one
// witness translation proves nothing). The comparison is over math/big.Rat
// against the payload's OWN float d and delta taken exactly, so it judges the
// rounding and nothing else.
func TestLoftPlacedGateDiameterRoundsTheShrinkOutward(t *testing.T) {
	for _, dx := range []float64{1 << 30, 1e6, 1e9, 1 << 36} {
		t.Run(fmt.Sprintf("dx=%g", dx), func(t *testing.T) {
			move, err := r3.Translation(r3.NewVec(dx, 0, 0))
			require.NoError(t, err)

			placedBody, err := boxLoftPayloadOn(t, 0, 3).placed(t.Context(), New(), StepRef(1), move)
			require.NoError(t, err)
			placedPl := placedBody.payload.(loftPayload)
			require.Greater(t, placedPl.delta, 0.0)

			held, ok, err := pointSetDiameterContext(t.Context(), placedPl.verts)
			require.NoError(t, err)
			require.True(t, ok)

			exact := new(big.Rat).Sub(
				new(big.Rat).SetFloat64(held),
				new(big.Rat).Mul(big.NewRat(2, 1), new(big.Rat).SetFloat64(placedPl.delta)),
			)

			got, ok, err := bodyGateDiameter(t.Context(), placedBody)
			require.NoError(t, err)
			require.True(t, ok)
			require.LessOrEqual(t, new(big.Rat).SetFloat64(got).Cmp(exact), 0,
				"the reported reference %.20g must not exceed the exact d - 2*delta %s", got, exact.FloatString(20))
		})
	}
}

// TestEvalLoftCancellation proves a context cancelled before the build even
// starts returns ctx.Err() rather than any evaluator sentinel.
func TestEvalLoftCancellation(t *testing.T) {
	ctx, cancel := context.WithCancel(t.Context())
	cancel()
	budget := newWorkBudget(ctx)
	_, err := evalLoft(ctx, New(), StepRef(0), boxLoftPayload(t), budget, newFreeformWork(), newFreeformWork())
	require.ErrorIs(t, err, context.Canceled)
}

// --- Table S gate tests ---

func TestValidateLoftRecordsHoleCountMismatch(t *testing.T) {
	p0 := ProfileRecord{Outer: squareLoop(0.5, 0.5, 0.5, true), Holes: []LoopRecord{squareLoop(0.5, 0.5, 0.1, false)}}
	p1 := unitSquareProfile()
	pl0, pl1 := planeAt(r3.NewVec(0, 0, 0)), planeAt(r3.NewVec(0, 0, 1))
	_, err := validateLoftRecords(p0, p1, pl0, pl1, nil, newFreeformWork(), newFreeformWork())
	require.ErrorIs(t, err, ErrUnsupported, "S1: hole-count mismatch")
}

func TestValidateLoftRecordsSegmentCountMismatch(t *testing.T) {
	p0 := unitSquareProfile()
	p1 := ProfileRecord{Outer: triangleLoop()}
	pl0, pl1 := planeAt(r3.NewVec(0, 0, 0)), planeAt(r3.NewVec(0, 0, 1))
	_, err := validateLoftRecords(p0, p1, pl0, pl1, nil, newFreeformWork(), newFreeformWork())
	require.ErrorIs(t, err, ErrUnsupported, "S2: segment-count mismatch")
}

func squareLoopWithFirstSegment(seg CurveSegment) LoopRecord {
	base := squareLoop(0.5, 0.5, 0.5, true)
	segs := append([]CurveSegment{seg}, base.Segments[1:]...)
	return LoopRecord{Segments: segs}
}

func TestValidateLoftRecordsCurvedPairIsUnsupported(t *testing.T) {
	p0 := unitSquareProfile()
	p1 := ProfileRecord{Outer: squareLoopWithFirstSegment(ArcSeg{
		Center: pt(0.5, -1), Start: pt(0, 0), End: pt(1, 0), TStart: 0, TEnd: 1,
	})}
	pl0, pl1 := planeAt(r3.NewVec(0, 0, 0)), planeAt(r3.NewVec(0, 0, 1))
	_, err := validateLoftRecords(p0, p1, pl0, pl1, nil, newFreeformWork(), newFreeformWork())
	require.ErrorIs(t, err, ErrUnsupported, "S3: a mixed LineSeg/ArcSeg pair")
}

func TestValidateLoftRecordsSameKindCircularPairIsUnsupported(t *testing.T) {
	circle := func() CurveSegment {
		return CircleSeg{Center: pt(0.5, 0.5), Radius: units.Millimeters(0.5), CCW: true, TStart: 0, TEnd: 1}
	}
	p0 := ProfileRecord{Outer: squareLoopWithFirstSegment(circle())}
	p1 := ProfileRecord{Outer: squareLoopWithFirstSegment(circle())}
	pl0, pl1 := planeAt(r3.NewVec(0, 0, 0)), planeAt(r3.NewVec(0, 0, 1))
	_, err := validateLoftRecords(p0, p1, pl0, pl1, nil, newFreeformWork(), newFreeformWork())
	require.ErrorIs(t, err, ErrUnsupported, "S3: a same-kind CircleSeg pair is still refused in increment 1")
}

func TestValidateLoftRecordsMalformedAlignment(t *testing.T) {
	p := unitSquareProfile()
	pl0, pl1 := planeAt(r3.NewVec(0, 0, 0)), planeAt(r3.NewVec(0, 0, 1))

	_, err := validateLoftRecords(p, p, pl0, pl1, []int{0, 1}, newFreeformWork(), newFreeformWork())
	require.ErrorIs(t, err, ErrDegenerate, "S4: wrong-length alignment (2 offsets for 1 loop)")

	_, err = validateLoftRecords(p, p, pl0, pl1, []int{5}, newFreeformWork(), newFreeformWork())
	require.ErrorIs(t, err, ErrDegenerate, "S4: an offset outside [0, 4)")
}

func TestValidateLoftRecordsCoincidentPlanes(t *testing.T) {
	p := unitSquareProfile()
	pl0 := planeAt(r3.NewVec(0, 0, 0))

	_, err := validateLoftRecords(p, p, pl0, pl0, nil, newFreeformWork(), newFreeformWork())
	require.ErrorIs(t, err, ErrDegenerate, "S5: identical planes")

	rotated := PlaneRecord{Origin: r3.NewVec(0, 0, 0), U: r3.NewVec(0, 1, 0), V: r3.NewVec(-1, 0, 0)}
	_, err = validateLoftRecords(p, p, pl0, rotated, nil, newFreeformWork(), newFreeformWork())
	require.ErrorIs(t, err, ErrDegenerate, "S5: the same geometric plane under a rotated U/V basis")
}

func TestValidateLoftRecordsDistinctPlanesPass(t *testing.T) {
	p := unitSquareProfile()
	pl0, pl1 := planeAt(r3.NewVec(0, 0, 0)), planeAt(r3.NewVec(0, 0, 1))
	offsets, err := validateLoftRecords(p, p, pl0, pl1, nil, newFreeformWork(), newFreeformWork())
	require.NoError(t, err)
	require.Equal(t, []int{0}, offsets)
}

// TestEvalLoftCollapsedTriangleIsDegenerate is S6, reached through the full
// evalLoft pipeline: plane0 and plane1 share the same origin but different
// (non-coincident, so S5 passes) bases, which puts corner 0 of BOTH profiles
// at the exact same world point — collapsing that corner's two incident wall
// triangles to zero area.
func TestEvalLoftCollapsedTriangleIsDegenerate(t *testing.T) {
	p := unitSquareProfile()
	pl0 := PlaneRecord{Origin: r3.NewVec(0, 0, 0), U: r3.NewVec(1, 0, 0), V: r3.NewVec(0, 1, 0)}
	pl1 := PlaneRecord{Origin: r3.NewVec(0, 0, 0), U: r3.NewVec(0, 1, 0), V: r3.NewVec(0, 0, 1)}
	pl := loftPayload{
		profile0: p, profile1: p,
		plane0: pl0, plane1: pl1,
		frame0: mustFrame(t, pl0), frame1: mustFrame(t, pl1),
		xform: r3.Identity(),
	}
	budget := newWorkBudget(t.Context())
	_, err := evalLoft(t.Context(), New(), StepRef(0), pl, budget, newFreeformWork(), newFreeformWork())
	require.ErrorIs(t, err, ErrDegenerate, "S6: a corner shared by both profiles collapses its incident wall triangles")
}

// TestEvalLoftOverTwistedCorrespondenceCrosses is S7: p1's own square is
// recorded through a MIRRORED frame (U negated) at the same local indices as
// p0's, which reverses p1's world-space winding and makes the ruled walls
// self-cross — docs/loft-design.md §13's own worked S7 fixture, restated
// through the public payload rather than raw triangles.
func TestEvalLoftOverTwistedCorrespondenceCrosses(t *testing.T) {
	p := unitSquareProfile()
	pl0 := planeAt(r3.NewVec(0, 0, 0))
	pl1 := PlaneRecord{Origin: r3.NewVec(1, 0, 1), U: r3.NewVec(-1, 0, 0), V: r3.NewVec(0, 1, 0)}
	pl := loftPayload{
		profile0: p, profile1: p,
		plane0: pl0, plane1: pl1,
		frame0: mustFrame(t, pl0), frame1: mustFrame(t, pl1),
		xform: r3.Identity(),
	}
	budget := newWorkBudget(t.Context())
	_, err := evalLoft(t.Context(), New(), StepRef(0), pl, budget, newFreeformWork(), newFreeformWork())
	require.ErrorIs(t, err, ErrDegenerate, "S7: a mirrored correspondence self-crosses")
}

// manyGonLoop builds a regular n-gon loop centered at (cx, cy) with the
// given radius, CCW.
func manyGonLoop(cx, cy, radius float64, n int) LoopRecord {
	pts := make([]Point2, n)
	for i := range n {
		theta := 2 * math.Pi * float64(i) / float64(n)
		pts[i] = pt(cx+radius*math.Cos(theta), cy+radius*math.Sin(theta))
	}
	segs := make([]CurveSegment, n)
	for i := range pts {
		segs[i] = LineSeg{Start: pts[i], End: pts[(i+1)%n], TStart: 0, TEnd: 1}
	}
	return LoopRecord{Segments: segs}
}

// TestEvalLoftAuditRefusesOverBudget is S8: a synthetic profile pair sized so
// the audit's own facet-pair count exceeds the fixed ceiling refuses before
// any pair result is trusted, wired end to end through evalLoft.
func TestEvalLoftAuditRefusesOverBudget(t *testing.T) {
	const n = 1200 // triangle count grows to roughly 4n-4, comfortably past 4001
	p := ProfileRecord{Outer: manyGonLoop(0, 0, 10, n)}
	pl0 := planeAt(r3.NewVec(0, 0, 0))
	pl1 := planeAt(r3.NewVec(0, 0, 1))
	pl := loftPayload{
		profile0: p, profile1: p,
		plane0: pl0, plane1: pl1,
		frame0: mustFrame(t, pl0), frame1: mustFrame(t, pl1),
		xform: r3.Identity(),
	}
	budget := newWorkBudget(t.Context())
	_, err := evalLoft(t.Context(), New(), StepRef(0), pl, budget, newFreeformWork(), newFreeformWork())
	require.ErrorIs(t, err, ErrUnsupported, "S8: the facet-pair ceiling")
}
