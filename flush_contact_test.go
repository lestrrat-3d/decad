package decad

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/booleaneval"
	"github.com/lestrrat-3d/decad/internal/booloperand"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// flushArcPrism is flushPolygonPrism with the side from (1, 6) to (-1, 6) drawn as a true arc about (0, 6).
func flushArcPrism(t *testing.T, doc *Document, p flushPlacement, origin, u, v, dir r3.Vec, pts [][2]float64,
	length float64) *Body {
	t.Helper()
	world := sketch.NewWorld()
	frame, err := r3.NewFrame(p.apply(origin), p.apply(u), p.apply(v))
	require.NoError(t, err)
	plane, err := world.CreatePlaneFromFrame(frame)
	require.NoError(t, err)
	s, err := world.CreateSketch(plane)
	require.NoError(t, err)
	verts := make([]*sketch.Point, len(pts))
	for i, q := range pts {
		verts[i] = s.CreatePoint(q[0], q[1])
		s.Fix(verts[i])
	}
	c := s.CreatePoint(0, 6)
	s.Fix(c)
	for i := range verts {
		j := (i + 1) % len(verts)
		if pts[i] == [2]float64{1, 6} && pts[j] == [2]float64{-1, 6} {
			s.CreateArc(c, verts[i], verts[j])
			continue
		}
		s.CreateLine(verts[i], verts[j])
	}
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	require.Len(t, s.Profiles(), 1)
	body, err := Extrude(doc, s, s.Profiles()[0], Distance{D: units.Millimeters(length), Dir: p.toward(u, v, dir)}).Do(t.Context())
	require.NoError(t, err)
	return body
}

// flushArcUnion is the Union of the host box [-10, 10] × [0, 16] × [-10, 0] and the chord-round tongue closed at the
// top by a true arc of radius 1 about (0, 6), drawn on the plane y = offset and extruded 4 toward +y, under p.
func flushArcUnion(t *testing.T, doc *Document, p flushPlacement, offset float64) *Body {
	t.Helper()
	x, y, z := r3.Vec{X: 1}, r3.Vec{Y: 1}, r3.Vec{Z: 1}
	host := flushPolygonPrism(t, doc, p, r3.Vec{}, x, y, z.Scale(-1), [][2]float64{{-10, 0}, {10, 0}, {10, 16}, {-10, 16}}, 10)
	tongue := flushArcPrism(t, doc, p, r3.Vec{Y: offset}, x, z, y, chordRoundTongue(), 4)
	joined, err := Union(host, tongue).Do(t.Context())
	require.NoError(t, err)
	return joined
}

// flushArcVolume is the volume of the exact R1 solid: the host and the part of the tongue above z = 0, whose profile is
// the polygon above z = 0 plus the half disc of radius 1.
func flushArcVolume() float64 {
	return 3200 + 4*(areaAbove(chordRoundTongue())+math.Pi/2)
}

// requireFlushCheck asserts the design's flush check: a mesh-path result with no gap shift, no Faceted face, and one
// face with one outer loop on the plane coord(axis) = at.
func requireFlushCheck(t *testing.T, b *Body, axis int, at float64) {
	t.Helper()
	fp := mergeFaceted(t, b)
	requireNoGapShift(t, b)
	for _, f := range b.Faces() {
		_, faceted := f.Surface().(Faceted)
		require.False(t, faceted, "every face keeps its analytic tag")
	}
	on := facesOnPlane(fp, b, axis, at)
	require.Len(t, on, 1, "one face on the flush plane")
	face := b.Faces()[on[0]]
	require.Len(t, face.Loops(), 1)
	require.True(t, face.Loops()[0].IsOuter())
}

func TestFlushContactArcCap(t *testing.T) {
	t.Parallel()
	for _, p := range flushPlacements {
		t.Run(p.name, func(t *testing.T) {
			t.Parallel()
			joined := flushArcUnion(t, New(), p, 0)
			requireFlushCheck(t, joined, p.axis(), 0)
			require.Len(t, joined.Faces(), 14)
			requireVolumeNear(t, joined, flushArcVolume())
		})
	}
}

func TestFlushContactArcCapExactOnly(t *testing.T) {
	t.Parallel()
	joined := flushArcUnion(t, New(ExactOnly()), flushPlacements[0], 0)
	require.Len(t, joined.Faces(), 14)
	requireFlushCheck(t, joined, 1, 0)
}

// filletedRectPrism extrudes the rectangle x in [-10, 10], z in [-10, 0], drawn on the plane y = 0 with its corner
// (-10, -10) rounded by a sketch fillet of radius 2, by 16 toward +y.
func filletedRectPrism(t *testing.T, doc *Document) *Body {
	t.Helper()
	world := sketch.NewWorld()
	frame, err := r3.NewFrame(r3.Vec{}, r3.Vec{X: 1}, r3.Vec{Z: 1})
	require.NoError(t, err)
	plane, err := world.CreatePlaneFromFrame(frame)
	require.NoError(t, err)
	s, err := world.CreateSketch(plane)
	require.NoError(t, err)
	corners := [][2]float64{{-10, -10}, {10, -10}, {10, 0}, {-10, 0}}
	pts := make([]*sketch.Point, len(corners))
	for i, c := range corners {
		pts[i] = s.CreatePoint(c[0], c[1])
	}
	lines := make([]*sketch.Line, len(pts))
	for i := range pts {
		lines[i] = s.CreateLine(pts[i], pts[(i+1)%len(pts)])
	}
	f, err := sketch.CreateFillet(s, lines[3], lines[0], pts[0], 2)
	require.NoError(t, err)
	for _, p := range pts[1:] {
		s.Fix(p)
	}
	s.AddConstraint(sketch.NewVertical(f.L1), sketch.NewHorizontal(f.L2))
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	require.Len(t, s.Profiles(), 1)
	body, err := Extrude(doc, s, s.Profiles()[0], Distance{D: units.Millimeters(16), Dir: flushPlacements[0].toward(r3.Vec{X: 1}, r3.Vec{Z: 1}, r3.Vec{Y: 1})}).Do(t.Context())
	require.NoError(t, err)
	return body
}

// flushChainHull is R1's arc tongue profile drawn on y = -1, extruded 9 toward +y and cut back flush to y = 0 by a box.
func flushChainHull(t *testing.T, doc *Document) *Body {
	t.Helper()
	x, y, z := r3.Vec{X: 1}, r3.Vec{Y: 1}, r3.Vec{Z: 1}
	long := flushArcPrism(t, doc, flushPlacements[0], r3.Vec{Y: -1}, x, z, y, chordRoundTongue(), 9)
	back := internalBoxBodyAtZ(t, doc, -5, -3, 5, 0, -3, 11)
	hull, err := Cut(long, back).Do(t.Context())
	require.NoError(t, err)
	requireNoGapShift(t, hull)
	return hull
}

// flushEndBlock is the block x in [-3, 3], y in [3, 3.5], z in [z0, 8].
func flushEndBlock(t *testing.T, doc *Document, z0 float64) *Body {
	t.Helper()
	return internalBoxBodyAtZ(t, doc, -3, 3, 3, 3.5, z0, 8-z0)
}

func requireVolumeWithin(t *testing.T, got Measurement, want, bound float64) {
	t.Helper()
	require.LessOrEqual(t, math.Abs(got.Value.Base()-want), got.Bound.Base()+bound)
}

// flushChainTongue is the chain hull with the end block cut out flush on its foot tops.
func flushChainTongue(t *testing.T, doc *Document) *Body {
	t.Helper()
	tongue, err := Cut(flushChainHull(t, doc), flushEndBlock(t, doc, -0.5)).Do(t.Context())
	require.NoError(t, err)
	requireNoGapShift(t, tongue)
	return tongue
}

// flushChainHost is the Union of the filleted prism and a co-directional prism drawn on y = 8.
func flushChainHost(t *testing.T, doc *Document) *Body {
	t.Helper()
	x, y, z := r3.Vec{X: 1}, r3.Vec{Y: 1}, r3.Vec{Z: 1}
	back := flushPolygonPrism(t, doc, flushPlacements[0], r3.Vec{Y: 8}, x, z, y,
		[][2]float64{{-10, -10}, {10, -10}, {10, -5}, {-10, -5}}, 12)
	host, err := Union(filletedRectPrism(t, doc), back).Do(t.Context())
	require.NoError(t, err)
	return host
}

func TestFlushContactChain(t *testing.T) {
	t.Parallel()
	doc := New()
	// A Boolean retires its operands, so each reading builds its own copies.
	host, tongue := flushChainHost(t, doc), flushChainTongue(t, doc)
	vHost, err := host.Volume()
	require.NoError(t, err)
	vTongue, err := tongue.Volume()
	require.NoError(t, err)
	joined, err := Union(host, tongue).Do(t.Context())
	require.NoError(t, err)
	requireFlushCheck(t, joined, 1, 0)
	common, err := Intersect(flushChainHost(t, doc), flushChainTongue(t, doc)).Do(t.Context())
	require.NoError(t, err)
	vCommon, err := common.Volume()
	require.NoError(t, err)
	vJoined, err := joined.Volume()
	require.NoError(t, err)
	want := vHost.Value.Base() + vTongue.Value.Base() - vCommon.Value.Base()
	requireVolumeWithin(t, vJoined, want, vHost.Bound.Base()+vTongue.Bound.Base()+vCommon.Bound.Base())
}

func TestFlushContactKeepsApart(t *testing.T) {
	t.Parallel()
	for _, offset := range []float64{math.Ldexp(1, -40), 1e-3} {
		joined := flushArcUnion(t, New(), flushPlacements[0], offset)
		fp := mergeFaceted(t, joined)
		require.Len(t, facesOnPlane(fp, joined, 1, 0), 1, "offset %g: the host face", offset)
		require.Len(t, facesOnPlane(fp, joined, 1, offset), 1, "offset %g: the tongue's cap", offset)
		for _, g := range fp.Groups {
			if g.Source != nil {
				require.Empty(t, g.Source.Members, "offset %g: no group is merged", offset)
			}
		}
		requireVolumeNear(t, joined, flushArcVolume())
	}
}

// flushDiscUnion is the Union of R1's host and a disc of radius 1 about the origin drawn on y = 0 and extruded 4 toward
// +y: its chords cross the host's edge z = 0.
func flushDiscUnion(t *testing.T, doc *Document) (*Body, error) {
	t.Helper()
	x, y, z := r3.Vec{X: 1}, r3.Vec{Y: 1}, r3.Vec{Z: 1}
	p := flushPlacements[0]
	host := flushPolygonPrism(t, doc, p, r3.Vec{}, x, y, z.Scale(-1), [][2]float64{{-10, 0}, {10, 0}, {10, 16}, {-10, 16}}, 10)
	world := sketch.NewWorld()
	frame, err := r3.NewFrame(r3.Vec{}, x, z)
	require.NoError(t, err)
	plane, err := world.CreatePlaneFromFrame(frame)
	require.NoError(t, err)
	s, err := world.CreateSketch(plane)
	require.NoError(t, err)
	c := s.CreatePoint(0, 0)
	s.Fix(c)
	s.CreateCircle(c, 1)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	disc, err := Extrude(doc, s, s.Profiles()[0], Distance{D: units.Millimeters(4), Dir: p.toward(x, z, y)}).Do(t.Context())
	require.NoError(t, err)
	return Union(host, disc).Do(t.Context())
}

func requireGapShift(t *testing.T, b *Body) {
	t.Helper()
	shifted := false
	for _, d := range b.OperationDiagnostics() {
		shifted = shifted || d.Code == OpDiagGapShiftApplied
	}
	require.True(t, shifted, "the Boolean takes the gap shift")
}

func TestFlushContactArcCrossingRefused(t *testing.T) {
	t.Parallel()
	joined, err := flushDiscUnion(t, New())
	require.NoError(t, err)
	requireGapShift(t, joined)
	_, err = flushDiscUnion(t, New(ExactOnly()))
	var be *BooleanError
	require.ErrorAs(t, err, &be)
	require.Equal(t, BooleanUnsupportedContact, be.Code)
}

func TestFlushContactTurnedRefused(t *testing.T) {
	t.Parallel()
	doc := New()
	x, y, z := r3.Vec{X: 1}, r3.Vec{Y: 1}, r3.Vec{Z: 1}
	p := flushPlacements[0]
	host := flushPolygonPrism(t, doc, p, r3.Vec{}, x, y, z.Scale(-1), [][2]float64{{-10, 0}, {10, 0}, {10, 16}, {-10, 16}}, 10)
	tongue := flushArcPrism(t, doc, p, r3.Vec{}, x, z, y, chordRoundTongue(), 4)
	// A turn of 37° and back leaves a placement that is not the identity; 30° and back composes to it exactly.
	for _, deg := range []float64{37, -37} {
		turn, err := r3.Rotation(z, units.Degrees(deg))
		require.NoError(t, err)
		tongue, err = Transform(t.Context(), tongue, turn)
		require.NoError(t, err)
	}
	joined, err := Union(host, tongue).Do(t.Context())
	require.NoError(t, err)
	requireGapShift(t, joined)
	for _, g := range mergeFaceted(t, joined).Groups {
		if g.Source != nil {
			require.Empty(t, g.Source.Members, "no group is merged")
		}
	}
}

func TestFlushContactCutFlushEnd(t *testing.T) {
	t.Parallel()
	doc := New()
	flush, err := Cut(flushChainHull(t, doc), flushEndBlock(t, doc, -0.5)).Do(t.Context())
	require.NoError(t, err)
	requireNoGapShift(t, flush)
	fp := mergeFaceted(t, flush)
	for t2, tri := range fp.Tris {
		on, inside := true, true
		for _, v := range tri {
			p := fp.Verts[v]
			on = on && p.Z == -0.5
			inside = inside && p.Y > 3 && p.Y < 3.5
		}
		require.False(t, on && inside, "facet %d lies on z = -0.5 inside the block", t2)
	}
	lifted := -0.5 + math.Ldexp(1, -40)
	slab, err := Cut(flushChainHull(t, doc), flushEndBlock(t, doc, lifted)).Do(t.Context())
	require.NoError(t, err)
	require.NotEmpty(t, facesOnPlane(mergeFaceted(t, slab), slab, 2, lifted), "the thin slab's top face")
}

// flushCircleLoft is a loft between two circles of radius 2 about the z axis, on z = 0 and z = 3.
func flushCircleLoft(t *testing.T, doc *Document) *Body {
	t.Helper()
	world := sketch.NewWorld()
	top, err := world.CreateOffsetPlane(world.XY(), 3)
	require.NoError(t, err)
	var sections [2]*sketch.Sketch
	var profiles [2]*sketch.Profile
	for i, plane := range []*sketch.Plane{world.XY(), top} {
		s, err := world.CreateSketch(plane)
		require.NoError(t, err)
		c := s.CreatePoint(0, 0)
		s.Fix(c)
		s.CreateCircle(c, 2)
		_, err = s.Solve(t.Context())
		require.NoError(t, err)
		sections[i], profiles[i] = s, s.Profiles()[0]
	}
	body, err := Loft(doc, sections[0], profiles[0], sections[1], profiles[1]).Do(t.Context())
	require.NoError(t, err)
	return body
}

func TestFlushContactChordedLoft(t *testing.T) {
	t.Parallel()
	doc := New()
	loft := flushCircleLoft(t, doc)
	// The box overlaps the loft's foot: its lower face shares the start cap's plane and facing, and its edge x = 0.5
	// crosses the cap 0.5 from its centre.
	box := internalBoxBodyAtZ(t, doc, -5, -5, 0.5, 5, 0, 1)
	joined, err := Union(box, loft).Do(t.Context())
	require.NoError(t, err)
	requireGapShift(t, joined)
}

// flushPlaneVertexBounds returns the bound of every result vertex on the plane coord(axis) = at.
func flushPlaneVertexBounds(fp facetedPayload, axis int, at float64) map[r3.Vec]float64 {
	out := map[r3.Vec]float64{}
	for _, tri := range fp.Tris {
		for _, v := range tri {
			p := fp.Verts[v]
			if [3]float64{p.X, p.Y, p.Z}[axis] == at {
				out[p] = fp.VertexBound[v]
			}
		}
	}
	return out
}

// flushToolOutlineBounds tessellates tool at the chord the Union with host requests and returns, for each outline
// edge of its cap on y = 0, the edge's endpoints and the bound of the cap facet that owns it: the facet whose edge the
// coplanar pass clips there.
func flushToolOutlineBounds(t *testing.T, host, tool *Body) [][3]r3.Vec {
	t.Helper()
	tol, diameter, err := booloperand.PairChord(t.Context(), &host.n, &tool.n)
	require.NoError(t, err)
	_, restating := booloperand.HeldFloor(&tool.n)
	chord := booloperand.ChordShare(t.Context(), &tool.n).EvaluationChord(tol, diameter, 1, restating, false)
	m, _, err := pairMeshes{}.TessellateOperand(t.Context(), &tool.n, chord)
	require.NoError(t, err)
	// The mesh pass reads each facet's bound as the Boolean prepares it, planar chording records included.
	bm, err := booleaneval.PrepBoolMesh(t.Context(), m, make([]int, len(m.Triangles)))
	require.NoError(t, err)
	var capFacets []int
	uses := map[[2]int]int{}
	for f, tri := range m.Triangles {
		if m.Vertices[tri[0]].Y != 0 || m.Vertices[tri[1]].Y != 0 || m.Vertices[tri[2]].Y != 0 {
			continue
		}
		capFacets = append(capFacets, f)
		for k := range 3 {
			a, b := tri[k], tri[(k+1)%3]
			uses[[2]int{min(a, b), max(a, b)}]++
		}
	}
	var out [][3]r3.Vec
	for _, f := range capFacets {
		tri, bound := m.Triangles[f], bm.FacetBound[f]
		for k := range 3 {
			a, b := tri[k], tri[(k+1)%3]
			if uses[[2]int{min(a, b), max(a, b)}] == 1 {
				out = append(out, [3]r3.Vec{m.Vertices[a], m.Vertices[b], {X: bound}})
			}
		}
	}
	require.NotEmpty(t, out)
	return out
}

// onSegment reports whether p lies on the segment from a to b, to within rounding.
func onSegment(p, a, b r3.Vec) bool {
	d, q := b.Sub(a), p.Sub(a)
	l := d.Len()
	return d.Cross(q).Len() <= 1e-12*l && q.Dot(d) >= -1e-12*l && q.Dot(d) <= l*l*(1+1e-12)
}

func TestFlushContactClipBounds(t *testing.T) {
	t.Parallel()
	doc := New()
	p := flushPlacements[0]
	x, y, z := r3.Vec{X: 1}, r3.Vec{Y: 1}, r3.Vec{Z: 1}
	operands := func() (*Body, *Body) {
		host := flushPolygonPrism(t, doc, p, r3.Vec{}, x, y, z.Scale(-1), [][2]float64{{-10, 0}, {10, 0}, {10, 16}, {-10, 16}}, 10)
		return host, flushArcPrism(t, doc, p, r3.Vec{}, x, z, y, chordRoundTongue(), 4)
	}
	// The Union retires its operands, so an identical pair is read for the tool's cap bounds.
	readHost, readTool := operands()
	outline := flushToolOutlineBounds(t, readHost, readTool)
	host, tool := operands()
	joined, err := Union(host, tool).Do(t.Context())
	require.NoError(t, err)
	fp := mergeFaceted(t, joined)
	// Every point on the flush plane inside the host's face that lies on the tool cap's outline was placed by the
	// coplanar pass clipping that outline, and carries at least the bound of the cap facet that owns the edge.
	positive := 0
	for at, beta := range flushPlaneVertexBounds(fp, 1, 0) {
		if at.Z > 0 {
			continue
		}
		want := 0.0
		for _, e := range outline {
			if onSegment(at, e[0], e[1]) {
				want = max(want, e[2].X)
			}
		}
		if want > 0 && at.Z < 0 {
			positive++
		}
		require.GreaterOrEqual(t, beta, want, "vertex %v on the flush plane", at)
	}
	require.NotZero(t, positive, "a placed point inside the host's face lies on a cap edge with a positive bound")
	// An exact pair records nothing: the polygon tongue's flush plane holds only rounding bounds.
	exact := mergeFaceted(t, flushTongueUnion(t, flushPlacements[0], 0))
	for at, beta := range flushPlaneVertexBounds(exact, 1, 0) {
		require.Less(t, beta, 1e-12, "vertex %v of the exact pair", at)
	}
}

// TestFlushContactRoundedCapLevelRefused: a square tool on the frame through z = 1e9, extruded just under 2, holds its
// end cap at the rounded level 1e9 + 2, in the plane of the blank's upper cap, while the level its record denotes lies
// 2^-52 below. The cap is no carrier face, so the coplanar pair stays refused under ExactOnly
// (agent-docs/flush-contact-design.md §4.1).
func TestFlushContactRoundedCapLevelRefused(t *testing.T) {
	t.Parallel()
	frame, err := r3.NewFrame(r3.Vec{Z: 1e9}, r3.Vec{X: 1}, r3.Vec{Y: 1})
	require.NoError(t, err)
	world := sketch.NewWorld()
	lower, err := world.CreatePlaneFromFrame(frame)
	require.NoError(t, err)
	upper, err := world.CreateOffsetPlane(lower, 2)
	require.NoError(t, err)
	square := func(plane *sketch.Plane, half float64) (*sketch.Sketch, *sketch.Profile) {
		s, err := world.CreateSketch(plane)
		require.NoError(t, err)
		rect := s.CreateRectangle(-half, -half, half, half)
		s.Fix(rect.A)
		_, err = s.Solve(t.Context())
		require.NoError(t, err)
		return s, s.Profiles()[0]
	}
	doc := New(ExactOnly())
	s0, p0 := square(lower, 5)
	s1, p1 := square(upper, 5)
	blank, err := Loft(doc, s0, p0, s1, p1).Do(t.Context())
	require.NoError(t, err)
	ts, tp := square(lower, 1)
	tool, err := Extrude(doc, ts, tp, Distance{D: units.Millimeters(math.Nextafter(2, 0)), Dir: Along}).Do(t.Context())
	require.NoError(t, err)
	_, err = Cut(blank, tool).Do(t.Context())
	require.ErrorIs(t, err, ErrUnsupported)
}
