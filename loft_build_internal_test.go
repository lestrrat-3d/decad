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
// resolveLoftLoopWalks resolves every loop of p (Outer, then Holes in order)
// into its own per-segment walk slice, on a fresh freeformWork per loop — the
// shape validateLoftRecords returns and loftPairings consumes.
func resolveLoftLoopWalks(t *testing.T, p ProfileRecord) [][]segmentWalk {
	t.Helper()
	loops := append([]LoopRecord{p.Outer}, p.Holes...)
	walks := make([][]segmentWalk, len(loops))
	for i, loop := range loops {
		work := newFreeformWork()
		w := make([]segmentWalk, len(loop.Segments))
		for j, seg := range loop.Segments {
			var err error
			w[j], err = walkOf(seg, work)
			require.NoError(t, err)
		}
		walks[i] = w
	}
	return walks
}

func TestLoftPairingsDefaultOffsetIsZero(t *testing.T) {
	p := unitSquareProfile()
	offsets := []int{0}
	walks := resolveLoftLoopWalks(t, p)
	pairs, sectionDelta, err := loftPairings(p, p, offsets, walks, walks, 0, nil, nil)
	require.NoError(t, err)
	require.Equal(t, pt(0, 0), pairs[0].w[0])
	require.Zero(t, sectionDelta, "a LineSeg-only pairing carries no curve to depart from")
}

func TestLoftPairingsAlignmentRotatesCorrespondence(t *testing.T) {
	p := unitSquareProfile()
	offsets := []int{1}
	walks := resolveLoftLoopWalks(t, p)
	pairs, _, err := loftPairings(p, p, offsets, walks, walks, 0, nil, nil)
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
	pairs, _, err := loftPairings(p0, p1, offsets, resolveLoftLoopWalks(t, p0), resolveLoftLoopWalks(t, p1), 0, nil, nil)
	require.NoError(t, err)

	require.Len(t, pairs, 3) // outer + 2 holes
	// Hole loop 1 (index 1+0): p0's own small hole (v) pairs with p1's
	// Holes[0], the LARGE hole (w) — pure positional pairing.
	require.Equal(t, smallHole.Segments[0].(LineSeg).Start.U, pairs[1].v[0].U)
	require.InDelta(t, 0.65, pairs[1].w[0].U, 1e-9, "p1's Holes[0] is the large hole, centered at 0.8 with half-width 0.15")
}

// TestLoftWalkResolutionChargesOncePerSegment pins the A10 plan's Task 1: a
// loft build resolves each recorded segment's walk exactly ONCE, never twice
// (walkOf neither memoizes nor is free to call again — it charges the
// free-form work budget on every call).
//
// The charge is pinned AT THE GATE because a full free-form BUILD is not
// reachable: S3 refuses every non-LineSeg pair, so no input runs evalLoft to
// completion with a nonzero free-form charge, and a LineSeg-only build
// charges zero whether its segments are walked once or twice (walkOf's
// LineSeg arm charges nothing), which would separate nothing.
// validateLoftRecords with a FitSplineSeg as p0's first segment does charge,
// and refuses at S3 immediately after, so its counter reads exactly what one
// segment's walk costs.
func TestLoftWalkResolutionChargesOncePerSegment(t *testing.T) {
	fit := FitSplineSeg{
		Fit:    []Point2{pt(0, 0), pt(1, 1), pt(2, 0), pt(3, 1), pt(4, 0)},
		TStart: 0, TEnd: 1,
	}

	// The reference: what ONE walkOf(fit) costs on a fresh counter.
	single := &freeformWork{}
	_, err := walkOf(fit, single)
	require.NoError(t, err)
	require.Greater(t, single.spent, uint64(0), "a FitSplineSeg's own walk must charge the free-form counter")

	const k = 4
	loopWork := &freeformWork{}
	walks := make([]segmentWalk, k)
	for i := range walks {
		walks[i], err = walkOf(fit, loopWork)
		require.NoError(t, err)
	}
	require.Equal(t, k*single.spent, loopWork.spent,
		"walkOf charges the same amount every call, so k calls read k reference charges")

	// The production gate's own charge, on the same segment. p0's segment 0
	// is the FitSplineSeg: the gate walks it, then S3 refuses it. Walking it
	// a second time instead of threading the resolved walk onward is exactly
	// the difference between one reference charge here and two.
	pl0, pl1 := planeAt(r3.NewVec(0, 0, 0)), planeAt(r3.NewVec(0, 0, 1))
	work0, work1 := newFreeformWork(), newFreeformWork()
	err = validateLoftRecordsErr(
		ProfileRecord{Outer: squareLoopWithFirstSegment(fit)}, unitSquareProfile(),
		pl0, pl1, nil, work0, work1)
	require.ErrorIs(t, err, ErrUnsupported, "S3: a FitSplineSeg is not a LineSeg")
	require.Equal(t, single.spent, work0.spent,
		"the gate walks that segment ONCE: its counter reads a single reference charge, not two")
	require.Zero(t, work1.spent, "S3 refuses on p0's segment 0 before p1's own segment is walked")

	// loftPairings, handed those already-resolved walks, spends nothing
	// further.
	before := loopWork.spent
	loop := LoopRecord{Segments: make([]CurveSegment, k)}
	for i := range loop.Segments {
		loop.Segments[i] = fit
	}
	profile := ProfileRecord{Outer: loop}
	// A free-form pairing has no station rule yet (loftCellStations' own
	// default case, unreached from any real build since S3 refuses it
	// first) — this is asserted below for the charge, not the correspondence.
	_, _, err = loftPairings(profile, profile, []int{0}, [][]segmentWalk{walks}, [][]segmentWalk{walks}, 0, loopWork, loopWork)
	require.Error(t, err)
	require.Equal(t, before, loopWork.spent, "loftPairings must spend no further free-form work")
}

// TestLoftPairingsConsumesTheGateResolvedWalks pins Task 1's other half on
// the ADMITTED path: loftPairings publishes the coordinates of the walks
// validateLoftRecords already resolved, rather than resolving the segments
// again. loftPairings is handed p0's record and both walk sets, and never
// sees p1's record at all, so every w coordinate it publishes can only come
// from the walks1 slice the gate returned — the two profiles are deliberately
// disjoint squares, so a pairing that re-resolved from the one record it does
// hold would publish p0's coordinates in w's place.
func TestLoftPairingsConsumesTheGateResolvedWalks(t *testing.T) {
	p0 := unitSquareProfile()                               // corners (0,0), (1,0), (1,1), (0,1)
	p1 := ProfileRecord{Outer: squareLoop(10, 20, 2, true)} // corners (8,18), (12,18), (12,22), (8,22)
	pl0, pl1 := planeAt(r3.NewVec(0, 0, 0)), planeAt(r3.NewVec(0, 0, 1))

	offsets, walks0, walks1, err := validateLoftRecords(p0, p1, pl0, pl1, []int{1}, newFreeformWork(), newFreeformWork())
	require.NoError(t, err)
	require.Equal(t, []int{1}, offsets)
	require.Len(t, walks0, 1)
	require.Len(t, walks1, 1)

	pairs, _, err := loftPairings(p0, p1, offsets, walks0, walks1, 0, nil, nil)
	require.NoError(t, err)
	require.Len(t, pairs, 1)

	n := len(p0.Outer.Segments)
	require.Len(t, pairs[0].v, n)
	require.Len(t, pairs[0].w, n)
	for j := range n {
		k := (j + offsets[0]) % n
		require.Equal(t, pt(walks0[0][j].startU, walks0[0][j].startV), pairs[0].v[j],
			"v[%d] is walks0[0][%d]'s own start point", j, j)
		require.Equal(t, pt(walks1[0][k].startU, walks1[0][k].startV), pairs[0].w[j],
			"w[%d] is walks1[0][%d]'s own start point", j, k)
	}
	// The same claim as literal coordinates: v runs p0's own corners from
	// (0,0), and w runs p1's corners rotated by the offset, so w[0] is p1's
	// SECOND corner (12,18) — a coordinate p0's record does not contain.
	require.Equal(t, pt(0, 0), pairs[0].v[0])
	require.Equal(t, pt(12, 18), pairs[0].w[0])
}

// TestValidateLoftRecordsS3PrecedesAWalkOfErrorLaterInTheOtherProfile pins
// the walk-resolution seam's own precedence: validateLoftRecords' per-segment
// loop walks p0's segment j, tests S3, THEN walks p1's segment k — the
// ORIGINAL interleaved order, unchanged by threading the resolved walks
// through (Task 1). A record whose FIRST profile fails S3 at its very first
// segment must therefore report that refusal even when the SECOND profile
// carries a later segment walkOf itself cannot resolve at all (a malformed
// CircleSeg here, ErrDegenerate) — a combination sketch's own authentication
// never produces but a decoded recipe can (docs/recipe-replay-design.md).
// Resolving a whole loop ahead of the S3 test — the shape this test guards
// against — would let that later walkOf error surface first instead.
func TestValidateLoftRecordsS3PrecedesAWalkOfErrorLaterInTheOtherProfile(t *testing.T) {
	p0 := ProfileRecord{Outer: squareLoopWithFirstSegment(ArcSeg{
		Center: pt(0.5, -1), Start: pt(0, 0), End: pt(1, 0), TStart: 0, TEnd: 1,
	})}

	base := squareLoop(0.5, 0.5, 0.5, true)
	segs := append([]CurveSegment{}, base.Segments...)
	// CCW true with TStart > TEnd contradicts the range order — walkOf's own
	// CircleSeg arm refuses it with ErrDegenerate, never reached here.
	segs[2] = CircleSeg{Center: pt(0.5, 0.5), Radius: units.Millimeters(0.5), CCW: true, TStart: 1, TEnd: 0}
	p1 := ProfileRecord{Outer: LoopRecord{Segments: segs}}

	pl0, pl1 := planeAt(r3.NewVec(0, 0, 0)), planeAt(r3.NewVec(0, 0, 1))
	err := validateLoftRecordsErr(p0, p1, pl0, pl1, nil, newFreeformWork(), newFreeformWork())
	require.ErrorIs(t, err, ErrUnsupported, "S3 on the first profile's own segment 0 must win")
	require.NotErrorIs(t, err, ErrDegenerate)
	require.Contains(t, err.Error(), "first profile")
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
// arm: a loft body's tolerance-gate diameter is its own held vertex set's
// diameter, as the shared reader publishes it — for the unit box, the space
// diagonal sqrt(3), rounded toward zero like every other witness maximum
// (verification §3).
//
// The bit-identity assertion is the one that pins §12 PR 2a's zero-delta fast
// path. An unplaced loft carries delta 0, so subtracting 2*delta changes
// nothing while rounding the difference toward zero still costs one ulp, and
// the InDelta comparison below is far too loose to see that: the reference
// must equal the held vertex-set diameter EXACTLY, not merely to 1e-12.
func TestLoftGateDiameterIsTheVertexDiameter(t *testing.T) {
	body := evalLoftFixture(t, boxLoftPayload(t))
	d, ok, err := bodyGateDiameter(t.Context(), body)
	require.NoError(t, err)
	require.True(t, ok)
	require.InDelta(t, 1.7320508075688772, d, 1e-12) // sqrt(3)

	pl := body.payload.(loftPayload)
	require.Zero(t, pl.delta, "an unplaced loft's vertices are exact")
	require.Zero(t, pl.sectionDelta, "S3 admits only LineSeg pairs, so every wall cell's chord IS the recorded segment")
	held, ok, err := pointSetDiameterContext(t.Context(), pl.verts)
	require.NoError(t, err)
	require.True(t, ok)
	require.Equal(t, held, d, "an unplaced loft's reference must be the held diameter bit-for-bit")
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

// TestLoftCollapsedGateDiameterIsRefusedFirst pins the antecedence
// docs/loft-design.md §12 states for bodyGateDiameter's shrink: a placement
// whose delta reaches half the held diameter is the regime where the shrunk
// reference collapses to zero and the arm answers no diameter at all, and no
// such placement ever reaches the gate, because S12 refuses it first. The
// divergence theorem bounds a closed boundary's own volume by d*A/3, so a
// delta at or above d/2 puts sweptVolumeAllow's delta*A at 3/2 of the held
// volume or more — exactly S12's non-positive clearance. Each fixture below
// asserts it sits in the collapse regime BEFORE asserting the refusal, so a
// fixture that drifted out of that regime fails rather than passing on an
// unrelated refusal.
func TestLoftCollapsedGateDiameterIsRefusedFirst(t *testing.T) {
	for _, tc := range []struct {
		half, height, dx float64
	}{
		{half: 1e-6, height: 1e-6, dx: 1e10},
		{half: 1e-4, height: 1e-4, dx: 1e12},
	} {
		t.Run(fmt.Sprintf("half=%g/dx=%g", tc.half, tc.dx), func(t *testing.T) {
			p := ProfileRecord{Outer: squareLoop(0, 0, tc.half, true)}
			pl0, pl1 := planeAt(r3.NewVec(0, 0, 0)), planeAt(r3.NewVec(0, 0, tc.height))
			pl := loftPayload{
				profile0: p, profile1: p,
				plane0: pl0, plane1: pl1,
				frame0: mustFrame(t, pl0), frame1: mustFrame(t, pl1),
				xform: r3.Identity(),
			}

			held := evalLoftFixture(t, pl).payload.(loftPayload)
			d, ok, err := pointSetDiameterContext(t.Context(), held.verts)
			require.NoError(t, err)
			require.True(t, ok)

			maxInputAbs := 0.0
			for _, v := range held.verts {
				maxInputAbs = max(maxInputAbs, vecMaxAbs(v))
			}
			move, err := r3.Translation(r3.NewVec(tc.dx, 0, 0))
			require.NoError(t, err)
			delta := rigidRoundAllow(maxInputAbs, vecMaxAbs(move.Translation()))
			require.GreaterOrEqual(t, 2*delta, d,
				"the fixture must sit in the collapse regime the doc's antecedence claim covers")

			_, err = pl.placed(t.Context(), New(), StepRef(1), move)
			require.ErrorIs(t, err, ErrUnsupported)
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

// validateLoftRecordsErr is validateLoftRecords with only the error kept, for
// the gate tests below that assert a refusal and don't need the resolved
// offsets or walks.
func validateLoftRecordsErr(p0, p1 ProfileRecord, pl0, pl1 PlaneRecord, alignment []int, work0, work1 *freeformWork) error {
	_, _, _, err := validateLoftRecords(p0, p1, pl0, pl1, alignment, work0, work1) //nolint:dogsled // only the error matters here.
	return err
}

func TestValidateLoftRecordsHoleCountMismatch(t *testing.T) {
	p0 := ProfileRecord{Outer: squareLoop(0.5, 0.5, 0.5, true), Holes: []LoopRecord{squareLoop(0.5, 0.5, 0.1, false)}}
	p1 := unitSquareProfile()
	pl0, pl1 := planeAt(r3.NewVec(0, 0, 0)), planeAt(r3.NewVec(0, 0, 1))
	err := validateLoftRecordsErr(p0, p1, pl0, pl1, nil, newFreeformWork(), newFreeformWork())
	require.ErrorIs(t, err, ErrUnsupported, "S1: hole-count mismatch")
}

func TestValidateLoftRecordsSegmentCountMismatch(t *testing.T) {
	p0 := unitSquareProfile()
	p1 := ProfileRecord{Outer: triangleLoop()}
	pl0, pl1 := planeAt(r3.NewVec(0, 0, 0)), planeAt(r3.NewVec(0, 0, 1))
	err := validateLoftRecordsErr(p0, p1, pl0, pl1, nil, newFreeformWork(), newFreeformWork())
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
	err := validateLoftRecordsErr(p0, p1, pl0, pl1, nil, newFreeformWork(), newFreeformWork())
	require.ErrorIs(t, err, ErrUnsupported, "S3: a mixed LineSeg/ArcSeg pair")
}

func TestValidateLoftRecordsSameKindCircularPairIsUnsupported(t *testing.T) {
	circle := func() CurveSegment {
		return CircleSeg{Center: pt(0.5, 0.5), Radius: units.Millimeters(0.5), CCW: true, TStart: 0, TEnd: 1}
	}
	p0 := ProfileRecord{Outer: squareLoopWithFirstSegment(circle())}
	p1 := ProfileRecord{Outer: squareLoopWithFirstSegment(circle())}
	pl0, pl1 := planeAt(r3.NewVec(0, 0, 0)), planeAt(r3.NewVec(0, 0, 1))
	err := validateLoftRecordsErr(p0, p1, pl0, pl1, nil, newFreeformWork(), newFreeformWork())
	require.ErrorIs(t, err, ErrUnsupported, "S3: a same-kind CircleSeg pair is still refused in increment 1")
}

func TestValidateLoftRecordsMalformedAlignment(t *testing.T) {
	p := unitSquareProfile()
	pl0, pl1 := planeAt(r3.NewVec(0, 0, 0)), planeAt(r3.NewVec(0, 0, 1))

	err := validateLoftRecordsErr(p, p, pl0, pl1, []int{0, 1}, newFreeformWork(), newFreeformWork())
	require.ErrorIs(t, err, ErrDegenerate, "S4: wrong-length alignment (2 offsets for 1 loop)")

	err = validateLoftRecordsErr(p, p, pl0, pl1, []int{5}, newFreeformWork(), newFreeformWork())
	require.ErrorIs(t, err, ErrDegenerate, "S4: an offset outside [0, 4)")
}

func TestValidateLoftRecordsCoincidentPlanes(t *testing.T) {
	p := unitSquareProfile()
	pl0 := planeAt(r3.NewVec(0, 0, 0))

	err := validateLoftRecordsErr(p, p, pl0, pl0, nil, newFreeformWork(), newFreeformWork())
	require.ErrorIs(t, err, ErrDegenerate, "S5: identical planes")

	rotated := PlaneRecord{Origin: r3.NewVec(0, 0, 0), U: r3.NewVec(0, 1, 0), V: r3.NewVec(-1, 0, 0)}
	err = validateLoftRecordsErr(p, p, pl0, rotated, nil, newFreeformWork(), newFreeformWork())
	require.ErrorIs(t, err, ErrDegenerate, "S5: the same geometric plane under a rotated U/V basis")
}

func TestValidateLoftRecordsDistinctPlanesPass(t *testing.T) {
	p := unitSquareProfile()
	pl0, pl1 := planeAt(r3.NewVec(0, 0, 0)), planeAt(r3.NewVec(0, 0, 1))
	offsets, walks0, walks1, err := validateLoftRecords(p, p, pl0, pl1, nil, newFreeformWork(), newFreeformWork())
	require.NoError(t, err)
	require.Equal(t, []int{0}, offsets)
	require.Len(t, walks0, 1)
	require.Len(t, walks1, 1)
	require.Len(t, walks0[0], 4, "the unit square loop has 4 segments")
	require.Len(t, walks1[0], 4)
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

// --- capPolygonAreaRat: docs/loft-design.md §8's cap-polygon shoelace ---

// assembleLoftFixture runs evalLoft's own pairing/assembly prefix
// (validateLoftRecords, loftPairings, assembleLoft) and stops there, so a
// test can inspect the assembled cap polygon (pts0/loopIdx0, pts1/loopIdx1)
// directly rather than only the published body.
func assembleLoftFixture(t *testing.T, pl loftPayload) loftAssembly {
	t.Helper()
	offsets, walks0, walks1, err := validateLoftRecords(pl.profile0, pl.profile1, pl.plane0, pl.plane1, pl.alignment, newFreeformWork(), newFreeformWork())
	require.NoError(t, err)
	target, err := loftChordTarget(pl.profile0, pl.profile1, walks0, walks1)
	require.NoError(t, err)
	pairs, _, err := loftPairings(pl.profile0, pl.profile1, offsets, walks0, walks1, target, newFreeformWork(), newFreeformWork())
	require.NoError(t, err)
	a, err := assembleLoft(t.Context(), pairs, pl.frame0, pl.frame1, pl.plane0, pl.xform)
	require.NoError(t, err)
	return a
}

// triangleAreaRat2D is the plain 2D triangle signed-area formula — a
// square-root-free rational, unlike a 3D triangle's cross-product norm —
// used by the tests below as an INDEPENDENT check that capPolygonAreaRat's
// shoelace sum really does equal the sum of the SAME triangulation's own
// triangle areas.
func triangleAreaRat2D(pts []Point2, tri [3]int) *big.Rat {
	a, b, c := pts[tri[0]], pts[tri[1]], pts[tri[2]]
	ua, va := mustRatOf(a.U), mustRatOf(a.V)
	ub, vb := mustRatOf(b.U), mustRatOf(b.V)
	uc, vc := mustRatOf(c.U), mustRatOf(c.V)
	sum := new(big.Rat).Mul(ua, new(big.Rat).Sub(vb, vc))
	sum.Add(sum, new(big.Rat).Mul(ub, new(big.Rat).Sub(vc, va)))
	sum.Add(sum, new(big.Rat).Mul(uc, new(big.Rat).Sub(va, vb)))
	return sum.Quo(sum, big.NewRat(2, 1))
}

// TestCapPolygonAreaRatMatchesMomentsOnUntrimmedLineSeg is the untrimmed
// half of docs/loft-design.md §8's cap-area rule: on an untrimmed LineSeg
// profile (every segment's TStart/TEnd is the natural 0/1, so lerp2 and
// moments.go's own ratLerp both return the record's own endpoint verbatim,
// with no rounding on either side) the shoelace of the cap polygon
// assembleLoft actually built must equal moments.go's own region-level exact
// rational EXACTLY — a big.Rat.Cmp, not a float comparison, since both sides
// are genuinely the same rational here.
func TestCapPolygonAreaRatMatchesMomentsOnUntrimmedLineSeg(t *testing.T) {
	pl := boxLoftPayload(t)
	a := assembleLoftFixture(t, pl)

	ig, err := pl.profile0.integralsTo(momentAreaOrder)
	require.NoError(t, err)
	require.False(t, ig.exactDead)
	require.True(t, ig.exact.complete())

	got := capPolygonAreaRat(a.pts0, a.loopIdx0)
	require.Equalf(t, 0, ig.exact.area.Cmp(got),
		"untrimmed LineSeg: shoelace %s must equal moments.go's own region rational %s exactly",
		got.RatString(), ig.exact.area.RatString())
}

// trimmedLineTriangleProfile is a triangle whose first segment is a TRIMMED
// fragment of a longer virtual line: Start=(0,0), End=(10,3), TStart=0.1 —
// a fractional parameter that walkOf's lerp2 (float64) and moments.go's own
// ratLerp (exact math/big.Rat) evaluate to two DIFFERENT values, by exactly
// the lerp rounding docs/loft-design.md §8 owns. The test below compares
// those two rationals against each other rather than pinning either as a
// literal, so its assertion stays host-portable. Starting the trimmed
// segment AT THE ORIGIN is deliberate: moments.go's own Green's-theorem
// term for that segment is then EXACTLY zero in exact rational arithmetic
// (three points through the origin are collinear with it however the trim
// divides them), while the shoelace term over the FLOAT-rounded walked
// point is not — so this fixture isolates the lerp-rounding difference from
// every other source of difference. The other two segments are untrimmed
// (TStart 0, TEnd 1) so the loop closes on ordinary recorded corners, and
// the closing segment's End is the walked point's own float64 value so the
// loop is a clean triangle.
func trimmedLineTriangleProfile() ProfileRecord {
	segs := []CurveSegment{
		LineSeg{Start: pt(0, 0), End: pt(10, 3), TStart: 0.1, TEnd: 1},
		LineSeg{Start: pt(10, 3), End: pt(5, 7), TStart: 0, TEnd: 1},
		LineSeg{Start: pt(5, 7), End: pt(1, 0.30000000000000004), TStart: 0, TEnd: 1},
	}
	return ProfileRecord{Outer: LoopRecord{Segments: segs}}
}

// TestCapPolygonAreaRatMatchesTrianglesOnTrimmedLineSeg is the counterpart
// of the untrimmed case above, and the acceptance line for the case where
// the two rationals genuinely disagree. On a TRIMMED LineSeg profile the
// corner assembleLoft walks into the cap's own triangles is NOT the corner
// moments.go's record-level integral reads (this file's
// trimmedLineTriangleProfile doc comment), and the cap reading has to follow
// the walked one: the published cap Area equals, EXACTLY, the sum of the
// SAME triangulation's own triangle areas (an independent square-root-free
// 2D check), and the built Face.Area() the caller actually reads carries
// that same value.
//
// That the two rationals differ at all is asserted rather than assumed, so a
// fixture that quietly stopped exercising the trimmed path would fail here
// instead of passing vacuously. The difference is logged and held to
// rounding scale; neither rational is ever pinned as a literal.
func TestCapPolygonAreaRatMatchesTrianglesOnTrimmedLineSeg(t *testing.T) {
	p := trimmedLineTriangleProfile()
	pl0 := planeAt(r3.NewVec(0, 0, 0))
	pl1 := planeAt(r3.NewVec(0, 0, 1))
	pl := loftPayload{
		profile0: p, profile1: p,
		plane0: pl0, plane1: pl1,
		frame0: mustFrame(t, pl0), frame1: mustFrame(t, pl1),
		xform: r3.Identity(),
	}

	a := assembleLoftFixture(t, pl)
	polyRat := capPolygonAreaRat(a.pts0, a.loopIdx0)

	// The other rational this cap could have been read from: moments.go's
	// own region-level integral of the record, independent of whatever
	// assembleLoft actually walked.
	ig, err := p.integralsTo(momentAreaOrder)
	require.NoError(t, err)
	require.False(t, ig.exactDead)
	require.True(t, ig.exact.complete())
	recordRat := ig.exact.area

	require.NotEqualf(t, 0, recordRat.Cmp(polyRat),
		"a trimmed LineSeg must leave moments.go's record-level area %s and the assembled cap polygon's own shoelace %s different, or this fixture no longer exercises the trimmed path",
		recordRat.RatString(), polyRat.RatString())
	diff := new(big.Rat).Sub(recordRat, polyRat)
	diffFloat, _ := diff.Float64()
	t.Logf("trimmed LineSeg cap: moments.go area = %s, assembled shoelace area = %s, difference = %s (%.3e mm^2)",
		recordRat.RatString(), polyRat.RatString(), diff.RatString(), diffFloat)
	// The difference is a lerp-rounding artifact at the coordinate's own
	// float64 ULP scale (~1e-16 relative to coordinates of order 1-10),
	// never a structural mismatch — bounded loosely so the assertion stays
	// host-portable (never pin an ULP-scale literal, CLAUDE.md's host
	// portability note).
	require.Lessf(t, math.Abs(diffFloat), 1e-9, "the difference must be a rounding-scale artifact, not a structural one")

	// polyRat is EXACTLY the sum of the SAME triangulation's own triangle
	// areas (the square-root-free 2D formula, so this comparison is exact
	// rather than a proven-bound enclosure).
	tris0, err := triangulate2DContext(t.Context(), a.pts0, a.loopIdx0)
	require.NoError(t, err)
	require.NotEmpty(t, tris0)
	triSum := new(big.Rat)
	for _, tri := range tris0 {
		triSum.Add(triSum, triangleAreaRat2D(a.pts0, tri))
	}
	require.Equalf(t, 0, polyRat.Cmp(triSum),
		"published cap area %s must equal the sum of its own triangulation's triangle areas %s exactly",
		polyRat.RatString(), triSum.RatString())

	// End to end: the built Face the caller actually reads carries the
	// SAME value, not merely the helper this test called directly.
	body := evalLoftFixture(t, pl)
	var capStart *Face
	for _, f := range body.Faces() {
		if f.Origins()[0].Role == roleCapStart {
			capStart = f
			break
		}
	}
	require.NotNil(t, capStart, "no capStart face in the built body")
	areaM, err := capStart.Area()
	require.NoError(t, err)
	wantFloat, _ := polyRat.Float64()
	require.Equal(t, wantFloat, areaM.Value.Base(), "the built capStart face must publish the same shoelace-derived area")
}

// holeSquareProfile is a unit square carrying one clockwise square hole, and
// twoHoleSquareProfile the same square carrying two disjoint ones. A hole's
// clockwise walk is what makes a per-loop shoelace sum SUBTRACT it
// (docs/sketch-seam-design.md, ProfileRecord.Area's own doc comment), so
// these are the fixtures that exercise capPolygonAreaRat's multi-loop arm at
// all: on a single-loop profile the loop over loopIdx runs exactly once and
// hole netting is never reached.
func holeSquareProfile() ProfileRecord {
	return ProfileRecord{
		Outer: squareLoop(0.5, 0.5, 0.5, true),
		Holes: []LoopRecord{squareLoop(0.5, 0.5, 0.2, false)},
	}
}

func twoHoleSquareProfile() ProfileRecord {
	return ProfileRecord{
		Outer: squareLoop(0.5, 0.5, 0.5, true),
		Holes: []LoopRecord{
			squareLoop(0.3, 0.3, 0.1, false),
			squareLoop(0.7, 0.7, 0.1, false),
		},
	}
}

// loftPayloadFor is the general unplaced-payload builder the cap-area table
// uses: two profiles on two standard-basis planes at the given world origins.
func loftPayloadFor(t *testing.T, p0, p1 ProfileRecord, o0, o1 r3.Vec) loftPayload {
	t.Helper()
	pl0, pl1 := planeAt(o0), planeAt(o1)
	return loftPayload{
		profile0: p0, profile1: p1,
		plane0: pl0, plane1: pl1,
		frame0: mustFrame(t, pl0), frame1: mustFrame(t, pl1),
		xform: r3.Identity(),
	}
}

// placedHoleLoftPayload is the hole-bearing square lofted under a genuine
// rigid placement, so the table also covers the delta > 0 arm of
// loftMassAccumulator.area (its perturbAreaSum term, docs/loft-design.md §12
// PR 2a). The cap polygon itself is PLANE-LOCAL and so is untouched by the
// placement (assembleLoft's pts0/pts1 are the pairs' own (U, V) points), which
// is exactly why a placed row still admits the same exact-rational equality.
func placedHoleLoftPayload(t *testing.T) loftPayload {
	t.Helper()
	pl := loftPayloadFor(t, holeSquareProfile(), holeSquareProfile(), r3.NewVec(0, 0, 0), r3.NewVec(0, 0, 1))
	rot, err := r3.Rotation(r3.NewVec(1, 2, 3), units.Degrees(37))
	require.NoError(t, err)
	shift, err := r3.Translation(r3.NewVec(12, -5, 3))
	require.NoError(t, err)
	xform, err := rot.Then(shift)
	require.NoError(t, err)
	pl.xform = xform
	return pl
}

// TestCapPolygonAreaRatNetsEveryLoop is capPolygonAreaRat's MULTI-LOOP
// acceptance line: the shoelace must walk every loop loopIdx records, so a
// hole's clockwise walk nets its own area back out of the cap reading.
//
// Both halves are computed inside this one process and compared to each
// other; NO float literal is pinned anywhere in this test, and none may be
// added. A placed loft's vertices, its delta and every Bound derived from
// them move by an ULP between amd64 and arm64, because r3's
// Transform.ApplyDir has the x*y+z shape Go contracts into a fused
// multiply-add on arm64 and not on amd64. CI runs ubuntu, macOS and windows,
// so a pinned constant here would be a permanently red build on one of them.
//
// The two halves:
//
//   - capPolygonAreaRat over the polygon assembleLoft ACTUALLY assembled must
//     equal, as an exact rational, moments.go's own region-level integral of
//     the same record — and that integral nets holes out per loop already
//     (integrateMomentRecordWithPoll walks Outer then every hole). Dropping a
//     hole loop from the shoelace breaks this comparison on every
//     hole-bearing row.
//   - The published Measurement must be the same one either rational
//     produces: value, proven bound and Exactness all compared field by
//     field, through the SAME accumulator the evaluator itself feeds, so the
//     check covers the publication path and not just the helper.
//
// Every row's profiles are untrimmed LineSeg loops (TStart 0, TEnd 1), which
// is what makes the first comparison an exact rational equality rather than a
// bounded one: walkOf's lerp2 and moments.go's ratLerp both return the
// record's own endpoint verbatim, with no rounding on either side (this
// file's trimmedLineTriangleProfile doc comment owns the trimmed case).
func TestCapPolygonAreaRatNetsEveryLoop(t *testing.T) {
	zero, up := r3.NewVec(0, 0, 0), r3.NewVec(0, 0, 1)
	for _, tc := range []struct {
		name  string
		build func(*testing.T) loftPayload
	}{
		{
			name:  "unit box, p0 below p1",
			build: func(t *testing.T) loftPayload { return boxLoftPayloadOn(t, 0, 1) },
		},
		{
			// The other z-spelling of the same box: p0 above p1, so §5's
			// whole-shell orientation step reverses the raw winding.
			name:  "unit box, p0 above p1",
			build: func(t *testing.T) loftPayload { return boxLoftPayloadOn(t, 1, 0) },
		},
		{
			name: "frustum, unit square to quarter square",
			build: func(t *testing.T) loftPayload {
				small := ProfileRecord{Outer: squareLoop(0.5, 0.5, 0.25, true)}
				return loftPayloadFor(t, unitSquareProfile(), small, zero, up)
			},
		},
		{
			// A skewed pair: the two planes are laterally offset, so no wall
			// triangle is vertical and every cap point is walked into a
			// differently sheared triangle.
			name: "skewed pair, laterally offset planes",
			build: func(t *testing.T) loftPayload {
				return loftPayloadFor(t, unitSquareProfile(), unitSquareProfile(), zero, r3.NewVec(0.4, 0.3, 1))
			},
		},
		{
			name: "triangle",
			build: func(t *testing.T) loftPayload {
				tri := ProfileRecord{Outer: triangleLoop()}
				return loftPayloadFor(t, tri, tri, zero, up)
			},
		},
		{
			name: "one hole",
			build: func(t *testing.T) loftPayload {
				return loftPayloadFor(t, holeSquareProfile(), holeSquareProfile(), zero, up)
			},
		},
		{
			name: "two holes",
			build: func(t *testing.T) loftPayload {
				return loftPayloadFor(t, twoHoleSquareProfile(), twoHoleSquareProfile(), zero, up)
			},
		},
		{
			name:  "placed, one hole",
			build: placedHoleLoftPayload,
		},
	} {
		t.Run(tc.name, func(t *testing.T) {
			pl := tc.build(t)
			a := assembleLoftFixture(t, pl)

			got0 := capPolygonAreaRat(a.pts0, a.loopIdx0)
			got1 := capPolygonAreaRat(a.pts1, a.loopIdx1)

			ig0, err := pl.profile0.integralsTo(momentAreaOrder)
			require.NoError(t, err)
			require.False(t, ig0.exactDead)
			require.True(t, ig0.exact.complete())
			ig1, err := pl.profile1.integralsTo(momentAreaOrder)
			require.NoError(t, err)
			require.False(t, ig1.exactDead)
			require.True(t, ig1.exact.complete())

			require.Equalf(t, 0, ig0.exact.area.Cmp(got0),
				"capStart: the assembled polygon's shoelace %s must equal moments.go's own hole-netted region rational %s exactly",
				got0.RatString(), ig0.exact.area.RatString())
			require.Equalf(t, 0, ig1.exact.area.Cmp(got1),
				"capEnd: the assembled polygon's shoelace %s must equal moments.go's own hole-netted region rational %s exactly",
				got1.RatString(), ig1.exact.area.RatString())

			mass := newLoftMassAccumulator(pl.xform.Apply(pl.plane0.Origin), a.delta)
			for k, tri := range a.tris {
				mass.add(a.verts[tri[0]], a.verts[tri[1]], a.verts[tri[2]], k < a.walls)
			}
			want := mass.area(ig0.exact.area, ig1.exact.area)
			area := mass.area(got0, got1)
			require.Equal(t, want.Value.Base(), area.Value.Base(), "published area value")
			require.Equal(t, want.Bound.Base(), area.Bound.Base(), "published area bound")
			require.Equal(t, want.Exactness, area.Exactness, "published area exactness")
		})
	}
}
