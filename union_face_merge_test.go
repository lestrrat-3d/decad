package decad

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func mergeTranslated(t *testing.T, b *Body, x, y, z float64) *Body {
	t.Helper()
	tr, err := r3.Translation(r3.Vec{X: x, Y: y, Z: z})
	require.NoError(t, err)
	moved, err := Transform(t.Context(), b, tr)
	require.NoError(t, err)
	return moved
}

// mergeFlushPair is the Union of a 10×10×10 box and a copy of it moved to (0.5, 0, 10): both y walls are flush.
func mergeFlushPair(t *testing.T) *Body {
	t.Helper()
	doc := New()
	a := internalBoxBody(t, doc, 0, 0, 10, 10, 10)
	b := mergeTranslated(t, internalBoxBody(t, doc, 0, 0, 10, 10, 10), 0.5, 0, 10)
	joined, err := Union(a, b).Do(t.Context())
	require.NoError(t, err)
	return joined
}

// mergeFaceted returns the faceted payload of a result and fails when it is not a mesh-path result.
func mergeFaceted(t *testing.T, b *Body) facetedPayload {
	t.Helper()
	fp, ok := b.n.Payload.(facetedPayload)
	require.True(t, ok, "the result takes the mesh path")
	return fp
}

// mergedGroupsOn returns the groups holding a facet whose vertices all lie on the plane coord = at.
func mergedGroupsOn(fp facetedPayload, axis int, at float64) map[int]struct{} {
	out := map[int]struct{}{}
	for i, tri := range fp.Tris {
		on := true
		for _, v := range tri {
			p := fp.Verts[v]
			on = on && [3]float64{p.X, p.Y, p.Z}[axis] == at
		}
		if on {
			out[fp.Src[i]] = struct{}{}
		}
	}
	return out
}

func requireVolumeNear(t *testing.T, b *Body, want float64) {
	t.Helper()
	v, err := b.Volume()
	require.NoError(t, err)
	require.LessOrEqual(t, math.Abs(v.Value.Base()-want), v.Bound.Base())
}

func TestUnionFaceMergeFlushBoxes(t *testing.T) {
	t.Parallel()
	joined := mergeFlushPair(t)
	fp := mergeFaceted(t, joined)
	require.Len(t, joined.Faces(), 10)
	requireVolumeNear(t, joined, 2000)
	for _, y := range []float64{0, 10} {
		groups := mergedGroupsOn(fp, 1, y)
		require.Len(t, groups, 1, "y = %v is one source group", y)
		for g := range groups {
			src := fp.Groups[g].Source
			require.NotNil(t, src)
			require.Len(t, src.Members, 2)
			require.Equal(t, 0, src.Members[0].Operand)
			require.Equal(t, 1, src.Members[1].Operand)
		}
	}
}

func mergeDovetail(t *testing.T) *Body {
	t.Helper()
	doc := New()
	host := internalOffsetBox(t, doc, -10, 0, 10, 16, 0, Distance{D: units.Millimeters(10), Dir: Against})

	world := sketch.NewWorld()
	frame, err := r3.NewFrame(r3.Vec{}, r3.Vec{X: 1}, r3.Vec{Z: 1})
	require.NoError(t, err)
	plane, err := world.CreatePlaneFromFrame(frame)
	require.NoError(t, err)
	s, err := world.CreateSketch(plane)
	require.NoError(t, err)
	pts := [][2]float64{{-5, -1}, {5, -1}, {5, 0}, {6, 6}, {-6, 6}, {-5, 0}}
	verts := make([]*sketch.Point, len(pts))
	for i, p := range pts {
		verts[i] = s.CreatePoint(p[0], p[1])
		s.Fix(verts[i])
	}
	for i := range verts {
		s.CreateLine(verts[i], verts[(i+1)%len(verts)])
	}
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	require.Len(t, s.Profiles(), 1)
	tool, err := Extrude(doc, s, s.Profiles()[0], Distance{D: units.Millimeters(16), Dir: Against}).Do(t.Context())
	require.NoError(t, err)
	joined, err := Union(host, tool).Do(t.Context())
	require.NoError(t, err)
	return joined
}

func TestUnionFaceMergeDovetailEnds(t *testing.T) {
	t.Parallel()
	joined := mergeDovetail(t)
	fp := mergeFaceted(t, joined)
	require.Len(t, joined.Faces(), 10)
	requireVolumeNear(t, joined, 4256)
	for _, y := range []float64{0, 16} {
		groups := mergedGroupsOn(fp, 1, y)
		require.Len(t, groups, 1, "y = %v is one source group", y)
		for g := range groups {
			require.Len(t, fp.Groups[g].Source.Members, 2)
			ops := []int{fp.Groups[g].Source.Members[0].Operand, fp.Groups[g].Source.Members[1].Operand}
			require.ElementsMatch(t, []int{0, 1}, ops)
			roles := map[string]bool{}
			for _, o := range fp.Groups[g].Origins {
				roles[o.Role] = true
			}
			require.True(t, roles["capStart"] || roles["capEnd"], "the tool's cap role")
			require.True(t, roles["side(0,0)"] || roles["side(0,1)"] || roles["side(0,2)"] || roles["side(0,3)"], "the host's side role")
		}
	}
	// Each end face has one loop, which is outer.
	ends := 0
	for _, f := range joined.Faces() {
		for _, y := range []float64{0, 16} {
			if faceOnPlane(fp, joined, f, 1, y) {
				ends++
				require.Len(t, f.Loops(), 1)
				require.True(t, f.Loops()[0].IsOuter())
			}
		}
	}
	require.Equal(t, 2, ends)
}

// faceOnPlane reports whether every facet of f lies on the plane coord = at.
func faceOnPlane(fp facetedPayload, b *Body, f *Face, axis int, at float64) bool {
	idx := -1
	for i, g := range b.Faces() {
		if g == f {
			idx = i
		}
	}
	found := false
	for t, fi := range fp.FaceOf {
		if fi != idx {
			continue
		}
		found = true
		for _, v := range fp.Tris[t] {
			p := fp.Verts[v]
			if [3]float64{p.X, p.Y, p.Z}[axis] != at {
				return false
			}
		}
	}
	return found
}

// faceSummary lists each face's loop count and origin roles in face order.
func faceSummary(b *Body) []string {
	var out []string
	for _, f := range b.Faces() {
		s := string(rune('0' + len(f.Loops())))
		for _, o := range f.Origins() {
			s += "|" + o.Role
		}
		out = append(out, s)
	}
	return out
}

func TestUnionFaceMergeSurvivesPlacement(t *testing.T) {
	t.Parallel()
	want := faceSummary(mergeFlushPair(t))
	require.Len(t, want, 10)

	moved := mergeTranslated(t, mergeFlushPair(t), 3, -2, 5)
	require.Equal(t, want, faceSummary(moved))

	copied, err := Copy(t.Context(), mergeFlushPair(t))
	require.NoError(t, err)
	require.Equal(t, want, faceSummary(copied))
}

func TestUnionFaceMergeKeepsRoundedFacesApart(t *testing.T) {
	t.Parallel()
	doc := New()
	a := internalBoxBody(t, doc, 0, 0, 10, 10, 10)
	b := internalBoxBody(t, doc, 0, 0, 10, 10, 10)
	for _, deg := range []float64{20, -20} {
		turn, err := r3.Rotation(r3.Vec{X: 1, Y: 1, Z: 1}, units.Degrees(deg))
		require.NoError(t, err)
		b, err = Transform(t.Context(), b, turn)
		require.NoError(t, err)
	}
	b = mergeTranslated(t, b, 0.5, 0, 10)
	joined, err := Union(a, b).Do(t.Context())
	require.NoError(t, err)
	fp := mergeFaceted(t, joined)
	shifted := false
	for _, d := range joined.OperationDiagnostics() {
		shifted = shifted || d.Code == OpDiagGapShiftApplied
	}
	require.True(t, shifted, "the rounded operand takes the gap shift")
	require.Len(t, joined.Faces(), 12)
	for _, g := range fp.Groups {
		var members []*provenance.SourcePatch
		if g.Source != nil {
			members = g.Source.Members
		}
		require.Empty(t, members, "no group is merged")
	}
}

// flushPlacement is a signed axis permutation: the images of +x, +y and +z. It places a flush fixture exactly, with
// no rounding, so the fixture's planes stay exact in every placement.
type flushPlacement struct {
	name    string
	x, y, z r3.Vec
}

func (p flushPlacement) apply(v r3.Vec) r3.Vec {
	return p.x.Scale(v.X).Add(p.y.Scale(v.Y)).Add(p.z.Scale(v.Z))
}

// toward is the extrude direction that runs along local dir from a sketch whose local normal is u × v: a mirroring
// placement reverses the frame's normal.
func (p flushPlacement) toward(u, v, dir r3.Vec) Direction {
	n := p.apply(u).Cross(p.apply(v))
	if n.Dot(p.apply(dir)) > 0 {
		return Along
	}
	return Against
}

// axis is the world axis index of the image of local +y, the normal of the flush plane.
func (p flushPlacement) axis() int {
	switch {
	case p.y.X != 0:
		return 0
	case p.y.Y != 0:
		return 1
	}
	return 2
}

var flushPlacements = []flushPlacement{
	{"identity", r3.Vec{X: 1}, r3.Vec{Y: 1}, r3.Vec{Z: 1}},
	{"quarter turn about z", r3.Vec{Y: 1}, r3.Vec{X: -1}, r3.Vec{Z: 1}},
	{"quarter turn about x", r3.Vec{X: 1}, r3.Vec{Z: 1}, r3.Vec{Y: -1}},
	{"mirrored in x", r3.Vec{X: -1}, r3.Vec{Y: 1}, r3.Vec{Z: 1}},
	{"mirrored in y", r3.Vec{X: 1}, r3.Vec{Y: -1}, r3.Vec{Z: 1}},
}

// flushPolygonPrism extrudes the closed polygon pts, drawn in the local plane spanned by u and v through origin, by
// length along local dir. Every vertex is fixed.
func flushPolygonPrism(t *testing.T, doc *Document, p flushPlacement, origin, u, v, dir r3.Vec, pts [][2]float64, length float64) *Body {
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
	for i := range verts {
		s.CreateLine(verts[i], verts[(i+1)%len(verts)])
	}
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	require.Len(t, s.Profiles(), 1)
	body, err := Extrude(doc, s, s.Profiles()[0], Distance{D: units.Millimeters(length), Dir: p.toward(u, v, dir)}).Do(t.Context())
	require.NoError(t, err)
	return body
}

// chordRoundTongue is a strip x in [-1, 1] rising to z = 6 from feet at z = -2, with an 8-chord quarter round of
// radius 0.7 at each root. The host's edge at z = 0 crosses both rounds, so the crossing points are no floats.
func chordRoundTongue() [][2]float64 {
	pts := [][2]float64{{-2.4, -2}, {2.4, -2}, {2.4, -0.5}}
	for i := range 9 {
		a := (270 - 90*float64(i)/8) * math.Pi / 180
		pts = append(pts, [2]float64{1.7 + 0.7*math.Cos(a), 0.2 + 0.7*math.Sin(a)})
	}
	pts = append(pts, [2]float64{1, 6}, [2]float64{-1, 6})
	for i := range 9 {
		a := (0 - 90*float64(i)/8) * math.Pi / 180
		pts = append(pts, [2]float64{-1.7 + 0.7*math.Cos(a), 0.2 + 0.7*math.Sin(a)})
	}
	return append(pts, [2]float64{-2.4, -0.5})
}

// areaAbove is the area of the polygon pts clipped to the half plane z >= 0.
func areaAbove(pts [][2]float64) float64 {
	var clipped [][2]float64
	for i, a := range pts {
		b := pts[(i+1)%len(pts)]
		if a[1] >= 0 {
			clipped = append(clipped, a)
		}
		if (a[1] < 0) != (b[1] < 0) {
			s := a[1] / (a[1] - b[1])
			clipped = append(clipped, [2]float64{a[0] + s*(b[0]-a[0]), 0})
		}
	}
	area := 0.0
	for i, a := range clipped {
		b := clipped[(i+1)%len(clipped)]
		area += a[0]*b[1] - b[0]*a[1]
	}
	return math.Abs(area) / 2
}

// flushTongueUnion is the Union of the host box [-10, 10] × [0, 16] × [-10, 0] and the chord-round tongue drawn on
// the plane y = offset and extruded 4 toward +y, under placement p. At offset 0 the tongue's start cap is flush with
// the host's y = 0 face and overlaps it below z = 0.
func flushTongueUnion(t *testing.T, p flushPlacement, offset float64) *Body {
	t.Helper()
	doc := New()
	x, y, z := r3.Vec{X: 1}, r3.Vec{Y: 1}, r3.Vec{Z: 1}
	host := flushPolygonPrism(t, doc, p, r3.Vec{}, x, y, z.Scale(-1), [][2]float64{{-10, 0}, {10, 0}, {10, 16}, {-10, 16}}, 10)
	tongue := flushPolygonPrism(t, doc, p, r3.Vec{Y: offset}, x, z, y, chordRoundTongue(), 4)
	joined, err := Union(host, tongue).Do(t.Context())
	require.NoError(t, err)
	return joined
}

// facesOnPlane returns the indices of the faces whose every facet lies on the plane coord = at.
func facesOnPlane(fp facetedPayload, b *Body, axis int, at float64) []int {
	var out []int
	for i, f := range b.Faces() {
		if faceOnPlane(fp, b, f, axis, at) {
			out = append(out, i)
		}
	}
	return out
}

func requireNoGapShift(t *testing.T, b *Body) {
	t.Helper()
	for _, d := range b.OperationDiagnostics() {
		require.NotEqual(t, OpDiagGapShiftApplied, d.Code)
	}
}

func TestUnionFaceMergeChordRoundCap(t *testing.T) {
	t.Parallel()
	for _, p := range flushPlacements {
		t.Run(p.name, func(t *testing.T) {
			t.Parallel()
			joined := flushTongueUnion(t, p, 0)
			fp := mergeFaceted(t, joined)
			requireNoGapShift(t, joined)
			requireVolumeNear(t, joined, 3200+4*areaAbove(chordRoundTongue()))
			for _, f := range joined.Faces() {
				_, faceted := f.Surface().(Faceted)
				require.False(t, faceted, "every face keeps its analytic tag")
			}
			// The host face and the tongue's cap become one face, which holds rounded crossing points of the host's
			// edge with the chords: the merge reads its plane from a facet whose corners are exact.
			on := facesOnPlane(fp, joined, p.axis(), 0)
			require.Len(t, on, 1, "one face on the flush plane")
			face := joined.Faces()[on[0]]
			require.Len(t, face.Loops(), 1)
			require.True(t, face.Loops()[0].IsOuter())
			bounded := false
			for t2, fi := range fp.FaceOf {
				if fi != on[0] {
					continue
				}
				src := fp.Groups[fp.Src[t2]].Source
				require.NotNil(t, src)
				require.Len(t, src.Members, 2)
				for _, v := range fp.Tris[t2] {
					bounded = bounded || fp.VertexBound[v] > 0
				}
			}
			require.True(t, bounded, "the merged face holds a vertex with a positive bound")
			// 6 host faces with the flush face merged, the tongue's far cap, its top, its two sides and the two chords
			// of each round that stand above the host.
			require.Len(t, joined.Faces(), 14)
		})
	}
}

func TestUnionFaceMergeChordRoundCapSurvivesPlacement(t *testing.T) {
	t.Parallel()
	want := faceSummary(flushTongueUnion(t, flushPlacements[0], 0))
	require.Len(t, want, 14)
	moved := mergeTranslated(t, flushTongueUnion(t, flushPlacements[0], 0), 3, -2, 5)
	require.Equal(t, want, faceSummary(moved))
	mirror, err := r3.NewFrame(r3.Vec{}, r3.Vec{Y: 1}, r3.Vec{Z: 1})
	require.NoError(t, err)
	reflect, err := r3.Reflection(mirror)
	require.NoError(t, err)
	mirrored, err := Transform(t.Context(), flushTongueUnion(t, flushPlacements[0], 0), reflect)
	require.NoError(t, err)
	require.Len(t, mirrored.Faces(), 14)
	fp := mergeFaceted(t, mirrored)
	require.Len(t, facesOnPlane(fp, mirrored, 1, 0), 1)
}

func TestUnionFaceMergeKeepsOffsetCapApart(t *testing.T) {
	t.Parallel()
	for _, offset := range []float64{math.Ldexp(1, -40), 1e-3} {
		joined := flushTongueUnion(t, flushPlacements[0], offset)
		fp := mergeFaceted(t, joined)
		requireNoGapShift(t, joined)
		// The tongue starts inside the host, so only its part above z = 0 adds volume.
		requireVolumeNear(t, joined, 3200+4*areaAbove(chordRoundTongue()))
		host := facesOnPlane(fp, joined, 1, 0)
		capFaces := facesOnPlane(fp, joined, 1, offset)
		require.Len(t, host, 1, "offset %g: the host face", offset)
		require.Len(t, capFaces, 1, "offset %g: the tongue's cap", offset)
		for _, g := range fp.Groups {
			if g.Source != nil {
				require.Empty(t, g.Source.Members, "offset %g: no group is merged", offset)
			}
		}
	}
}
