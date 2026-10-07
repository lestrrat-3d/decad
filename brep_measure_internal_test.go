package decad

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file is the brep consumers' share of docs/general-boolean-design.md
// §4.5 that reads through Verify: the undercut and minimum-radius surveys.
// The fixtures are hand-built records (internalCrossDrilledBrep), a prism's
// face view committed as a brep body (internalBrepBody), and S1 as the public
// class B Cut builds it (internalCrossDrilled).

// internalHalfDiscPrism extrudes a half disc of radius 10 about the origin
// (an arc counter-clockwise from (10, 0) to (−10, 0) closed by its diameter)
// 5 mm: one convex partial cylinder wall, one planar wall, two caps whose
// loops are an arc and a line.
func internalHalfDiscPrism(t *testing.T, doc *Document) *Body {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	c, a, b := s.CreatePoint(0, 0), s.CreatePoint(10, 0), s.CreatePoint(-10, 0)
	s.Fix(c)
	s.Fix(a)
	s.Fix(b)
	s.CreateArc(c, a, b)
	s.CreateLine(b, a)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	body, err := doc.Extrude(s, s.Profiles()[0], Distance{D: units.Millimeters(5), Dir: Along})
	require.NoError(t, err)
	return body
}

// requireRoles asserts the faces carry exactly the given brep roles.
func requireRoles(t *testing.T, body *Body, faces []*Face, want ...string) {
	t.Helper()
	got := []string{}
	for _, f := range faces {
		for _, o := range f.Origins() {
			if o.producer == body.origin.producer {
				got = append(got, o.Role)
			}
		}
	}
	require.ElementsMatch(t, want, got)
}

// TestBrepUndercutSurveyReadsEveryFace runs Verify's pull survey over S1's
// cross-drilled box, whose faces sit in three permuted frames. A straight
// pull up (+z) hooks only the hole's wall: the floor's normal is exactly
// antiparallel and separates, and every other face is parallel to the pull.
// A pull along the hole (+y) hooks nothing: the y = 0 face is exactly
// antiparallel and the hole's wall is parallel to it. A pull tilted to
// (1, 0, 1) hooks the x = 0 face, the floor and the hole's wall. Shown to
// fail with the survey's brepPayload arm deleted (coverage read
// unavailable) and with brepUndercuts' planar sign inverted (the tilted
// pull read the x = 40 face and the top instead).
func TestBrepUndercutSurveyReadsEveryFace(t *testing.T) {
	t.Parallel()
	for _, tc := range []struct {
		name  string
		pull  r3.Vec
		roles []string
	}{
		{"straight up", r3.NewVec(0, 0, 1), []string{"wall(6)"}},
		{"along the hole", r3.NewVec(0, 1, 0), nil},
		{"tilted", r3.NewVec(1, 0, 1), []string{"face(0)", "face(2)", "wall(6)"}},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			doc := New()
			body := internalCommitBrep(t, doc, internalCrossDrilledBrep(t))
			rep, err := doc.Verify(t.Context(), WithPullDirection(tc.pull))
			require.NoError(t, err)
			br, err := rep.ForBody(body)
			require.NoError(t, err)
			require.Equal(t, CoverageComplete, br.Undercut.Coverage)
			requireRoles(t, body, br.Undercut.Faces, tc.roles...)
		})
	}
}

// TestBrepConcaveRadiusReadsConcaveWalls runs Verify's concave-radius survey
// over brep bodies: S1's hole wall (a whole circle walked clockwise with the
// material outside it) measures 3 mm within its bound; the half disc's
// convex arc wall is no candidate, so the survey proves no concave feature;
// and S1 carrying a section displacement on one face is undecided. Shown to
// fail with brepMinRadius' clockwise test deleted (the half disc measured
// its convex 10 mm wall) and with its displacement refusal deleted (the
// displaced record measured).
func TestBrepConcaveRadiusReadsConcaveWalls(t *testing.T) {
	t.Parallel()
	t.Run("hole", func(t *testing.T) {
		doc := New()
		body := internalCommitBrep(t, doc, internalCrossDrilledBrep(t))
		rep, err := doc.Verify(t.Context(), WithConcaveRadius())
		require.NoError(t, err)
		br, err := rep.ForBody(body)
		require.NoError(t, err)
		require.Equal(t, ScalarMeasured, br.ConcaveRadius.Outcome)
		got := br.ConcaveRadius.Minimum.Measurement
		require.LessOrEqual(t, math.Abs(got.Value.Base()-3), got.Bound.Base())
	})
	t.Run("convex arc", func(t *testing.T) {
		doc := New()
		body := internalBrepBody(t, internalHalfDiscPrism(t, doc))
		out, ok := brepMinRadius(body.payload.(brepPayload))
		require.True(t, ok)
		require.True(t, out.ok)
		require.Nil(t, out.reading, "no wall curves away from the material")
	})
	t.Run("displaced", func(t *testing.T) {
		bp := internalCrossDrilledBrep(t)
		bp.faces[6].delta = 1e-12
		_, ok := brepMinRadius(bp)
		require.False(t, ok)
	})
}

// TestBrepArcWallHasTheAnalyticSTEPShape pins the face shape export's
// analytic STEP arm admits with Arc3 edges (supportsAnalyticPartialWall,
// supportsAnalyticPlanarLoop), read off a brep body: the half disc's face
// view carries one cylinder face bounded by a single loop alternating two
// arcs about its axis and two lines along it, and two planar faces whose
// loops carry an arc. No public boolean builds a brep with an arc wall yet
// (class B's keyway takes the mesh path), so export's own arc tests run on
// the prism that states the same faces; its class B tests write S1 and B1.
func TestBrepArcWallHasTheAnalyticSTEPShape(t *testing.T) {
	t.Parallel()
	doc := New()
	body := internalBrepBody(t, internalHalfDiscPrism(t, doc))
	walls, arcPlanes := 0, 0
	for _, face := range body.Faces() {
		switch surface := face.Surface().(type) {
		case Cylinder:
			walls++
			require.Len(t, face.Loops(), 1)
			coedges := face.Loops()[0].CoEdges()
			require.Len(t, coedges, 4)
			arcs := 0
			for i, ce := range coedges {
				switch c := ce.Edge().Curve().(type) {
				case Arc3:
					arcs++
					require.True(t, c.Axis == surface.Axis || c.Axis == surface.Axis.Scale(-1))
					if i > 0 {
						_, prevArc := coedges[i-1].Edge().Curve().(Arc3)
						require.False(t, prevArc, "arcs and lines alternate")
					}
				case Line3:
					run := ce.Edge().End().Position().Value.Sub(ce.Edge().Start().Position().Value)
					require.Equal(t, r3.Vec{}, run.Cross(surface.Axis))
				default:
					t.Fatalf("a partial wall edge is %T", c)
				}
			}
			require.Equal(t, 2, arcs)
		case Plane:
			for _, ce := range face.Loops()[0].CoEdges() {
				if _, ok := ce.Edge().Curve().(Arc3); ok {
					arcPlanes++
					break
				}
			}
		}
	}
	require.Equal(t, 1, walls)
	require.Equal(t, 2, arcPlanes)
}

// TestBrepClearanceReadsItsCarriers runs Verify's clearance kernel against
// S1's cross-drilled box through its bodyGeom arm (addBrepFaces): a 5 mm
// box standing 5 mm off the x = 40 wall, and a Ø2 pin threaded through the
// Ø6 hole along its axis, 2 mm from the hole's cylinder all round. Both
// gaps are proven: each Clearance row contains its closed form within its
// bound. Shown to fail with newBodyGeomBudget's brepPayload arm deleted (no
// clearance row for either pair).
func TestBrepClearanceReadsItsCarriers(t *testing.T) {
	t.Parallel()
	doc := New()
	brep := internalCommitBrep(t, doc, internalCrossDrilledBrep(t))
	box := internalBoxBody(t, doc, 45, 0, 50, 20, 20)
	pin := internalPinThroughS1(t, doc)

	_, ok := newBodyGeom(brep)
	require.True(t, ok, "the brep body has a carrier model")

	rep, err := doc.Verify(t.Context(), WithClearances())
	require.NoError(t, err)
	require.Empty(t, rep.Interferences)
	gaps := map[*Body]Measurement{}
	for _, row := range rep.Clearances {
		switch brep {
		case row.A:
			gaps[row.B] = row.Gap
		case row.B:
			gaps[row.A] = row.Gap
		}
	}
	for other, want := range map[*Body]float64{box: 5, pin: 2} {
		gap, found := gaps[other]
		require.True(t, found, "the brep body has a proven clearance row")
		require.LessOrEqual(t, math.Abs(gap.Value.Base()-want), gap.Bound.Base()+1e-12)
	}
}

// internalPinThroughS1 commits a Ø2 pin threaded along y through S1's hole
// axis (20, ·, 10), spanning y from −5 to 25: 2 mm from the Ø6 hole's wall
// all round.
func internalPinThroughS1(t *testing.T, doc *Document) *Body {
	t.Helper()
	xz, err := r3.NewFrame(r3.Vec{}, r3.NewVec(1, 0, 0), r3.NewVec(0, 0, 1))
	require.NoError(t, err)
	pinPayload := prismPayload{
		profile: ProfileRecord{Outer: LoopRecord{Segments: []CurveSegment{
			CircleSeg{Center: Point2{U: 20, V: 10}, Radius: units.Millimeters(1), CCW: true, TStart: 0, TEnd: 1},
		}}},
		// The frame's normal is −y: z ∈ [−25, 5] spans y ∈ [−5, 25].
		frame: xz, z0: -25, z1: 5, xform: r3.Identity(),
	}
	pin, err := evalPrismContext(t.Context(), doc, doc.nextProducerID(), pinPayload, freeform.NewFreeformWork())
	require.NoError(t, err)
	doc.commit(pin)
	return pin
}

// TestBrepConsumersOnTheClassBCutResult runs the same consumers over S1 as a
// public Cut builds it (class B, classb.go), not the hand-built record:
// a pull up hooks only the hole's cylinder; a pull along (1, 0, 1) hooks the
// cylinder and the two planes facing −x and −z; the concave-radius survey
// measures the hole's 3 mm; and the clearance rows read 5 mm to a box off
// the x = 40 wall and 2 mm to a pin threaded through the hole. Shown to fail
// with the survey's and newBodyGeomBudget's brepPayload arms deleted
// (coverage unavailable, radius unavailable, no clearance rows).
func TestBrepConsumersOnTheClassBCutResult(t *testing.T) {
	t.Parallel()
	doc, s1 := internalCrossDrilled(t)
	_, isBrep := s1.payload.(brepPayload)
	require.True(t, isBrep, "the public Cut builds S1 as a brep body")
	box := internalBoxBody(t, doc, 45, 0, 50, 20, 20)
	pin := internalPinThroughS1(t, doc)

	rep, err := doc.Verify(t.Context(), WithPullDirection(r3.NewVec(0, 0, 1)), WithConcaveRadius(), WithClearances())
	require.NoError(t, err)
	br, err := rep.ForBody(s1)
	require.NoError(t, err)
	require.Equal(t, CoverageComplete, br.Undercut.Coverage)
	require.Len(t, br.Undercut.Faces, 1)
	_, isCylinder := br.Undercut.Faces[0].Surface().(Cylinder)
	require.True(t, isCylinder, "a pull up hooks only the hole")
	require.Equal(t, ScalarMeasured, br.ConcaveRadius.Outcome)
	radius := br.ConcaveRadius.Minimum.Measurement
	require.LessOrEqual(t, math.Abs(radius.Value.Base()-3), radius.Bound.Base())
	gaps := map[*Body]Measurement{}
	for _, row := range rep.Clearances {
		switch s1 {
		case row.A:
			gaps[row.B] = row.Gap
		case row.B:
			gaps[row.A] = row.Gap
		}
	}
	for other, want := range map[*Body]float64{box: 5, pin: 2} {
		gap, found := gaps[other]
		require.True(t, found, "S1 has a proven clearance row")
		require.LessOrEqual(t, math.Abs(gap.Value.Base()-want), gap.Bound.Base()+1e-12)
	}

	rep, err = doc.Verify(t.Context(), WithPullDirection(r3.NewVec(1, 0, 1)))
	require.NoError(t, err)
	br, err = rep.ForBody(s1)
	require.NoError(t, err)
	require.Equal(t, CoverageComplete, br.Undercut.Coverage)
	require.Len(t, br.Undercut.Faces, 3)
	planes := 0
	for _, f := range br.Undercut.Faces {
		if _, ok := f.Surface().(Plane); !ok {
			continue
		}
		at := f.Loops()[0].CoEdges()[0].Start().Position().Value
		n, err := f.NormalAt(at)
		require.NoError(t, err)
		require.True(t, n.Value.Sub(r3.NewVec(-1, 0, 0)).Len() < 1e-12 || n.Value.Sub(r3.NewVec(0, 0, -1)).Len() < 1e-12,
			"a hooked plane faces −x or −z, got %v", n.Value)
		planes++
	}
	require.Equal(t, 2, planes)
}
