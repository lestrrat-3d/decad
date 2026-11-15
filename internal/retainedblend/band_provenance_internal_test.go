package retainedblend

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// planeTagFace is a retained plane face whose tag passes through origin with the
// frame's axes u and v.
func planeTagFace(t *testing.T, origin, u, v r3.Vec) *topo.Face {
	t.Helper()
	f, err := r3.NewFrame(origin, u, v)
	require.NoError(t, err)
	return &topo.Face{Surface: surfacegeom.Plane{Frame: f}, RetainedTag: true}
}

// TestSameTagPlane reads two plane tags exactly (retained-edge-blend §4.3): the
// normals are parallel and one origin lies on the other plane, on the tags'
// floats as rationals. A tag one float step off, a tilted tag, a tag that states
// a departure and a face that is no retained plane never match.
func TestSameTagPlane(t *testing.T) {
	t.Parallel()

	x, y, z := r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0), r3.NewVec(0, 0, 1)
	g := planeTagFace(t, r3.NewVec(0, 13.5, 0), x, z)

	t.Run("the same plane at another origin matches", func(t *testing.T) {
		t.Parallel()
		require.True(t, sameTagPlane(planeTagFace(t, r3.NewVec(7, 13.5, -4), x, z), g))
		require.True(t, sameTagPlane(g, g))
	})
	t.Run("a flipped face on the plane matches", func(t *testing.T) {
		t.Parallel()
		require.True(t, sameTagPlane(planeTagFace(t, r3.NewVec(7, 13.5, -4), z, x), g))
	})
	t.Run("a tag one float step off does not match", func(t *testing.T) {
		t.Parallel()
		for _, off := range []float64{math.Nextafter(13.5, 14), math.Nextafter(13.5, 13)} {
			require.False(t, sameTagPlane(planeTagFace(t, r3.NewVec(0, off, 0), x, z), g))
		}
	})
	t.Run("a parallel plane or a perpendicular one does not match", func(t *testing.T) {
		t.Parallel()
		require.False(t, sameTagPlane(planeTagFace(t, r3.NewVec(0, 1.5, 0), x, z), g))
		require.False(t, sameTagPlane(planeTagFace(t, r3.NewVec(0, 13.5, 0), y, z), g))
	})
	t.Run("a tag that states a departure does not match", func(t *testing.T) {
		t.Parallel()
		for _, mod := range []func(f *topo.Face){
			func(f *topo.Face) { f.TagDeparture = 1e-9 },
			func(f *topo.Face) { f.NormalBound = 1e-9 },
			func(f *topo.Face) { f.HasAxialDelta, f.AxialDelta = true, 1e-9 },
			func(f *topo.Face) { f.RetainedTag = false },
		} {
			f := planeTagFace(t, r3.NewVec(7, 13.5, -4), x, z)
			mod(f)
			require.False(t, sameTagPlane(f, g))
		}
	})
	t.Run("a zero axial delta matches and g's own delta is not read", func(t *testing.T) {
		t.Parallel()
		f := planeTagFace(t, r3.NewVec(7, 13.5, -4), x, z)
		f.HasAxialDelta = true
		d := planeTagFace(t, r3.NewVec(0, 13.5, 0), x, z)
		d.HasAxialDelta, d.AxialDelta = true, 1e-9
		require.True(t, sameTagPlane(f, d))
		require.False(t, sameTagPlane(d, f), "the face that grants provenance states no delta")
	})
	t.Run("a cylinder is no plane", func(t *testing.T) {
		t.Parallel()
		c := &topo.Face{Surface: surfacegeom.Cylinder{Axis: z, Radius: units.Millimeters(3)}, RetainedTag: true}
		require.False(t, sameTagPlane(c, g))
		require.False(t, sameTagPlane(g, c))
		require.False(t, sameTagPlane(nil, g))
	})
}

// TestOnTagPlane admits the boundary vertices of faces whose tag is g's and no
// vertex inside a face's fan.
func TestOnTagPlane(t *testing.T) {
	t.Parallel()

	x, z := r3.NewVec(1, 0, 0), r3.NewVec(0, 0, 1)
	g := planeTagFace(t, r3.NewVec(0, 5, 0), x, z)
	same := planeTagFace(t, r3.NewVec(20, 5, 0), x, z)
	other := planeTagFace(t, r3.NewVec(0, 6, 0), x, z)
	// same is a fan around vertex 4: vertices 0..3 are its boundary. other is a
	// triangle on vertices 5..7.
	fp := facetedrecord.Payload{
		Tris:   [][3]int{{0, 1, 4}, {1, 2, 4}, {2, 3, 4}, {3, 0, 4}, {5, 6, 7}},
		FaceOf: []int{1, 1, 1, 1, 2},
	}
	w := &work{fp: fp, faceIx: map[*topo.Face]int{g: 0, same: 1, other: 2}}
	require.Equal(t, map[int]bool{0: true, 1: true, 2: true, 3: true}, w.onTagPlane(g))
	require.Equal(t, map[int]bool{5: true, 6: true, 7: true}, w.onTagPlane(other))
	require.Same(t, same, w.faceAt(1))
	require.Nil(t, w.faceAt(9))
	require.Empty(t, (&work{fp: fp}).onTagPlane(g), "no face map, no provenance")
}

// outArea is the total area of the facets of w.out.
func (w *work) outArea() float64 {
	area := 0.0
	for _, tri := range w.out {
		a, c, d := w.verts[tri[0]], w.verts[tri[1]], w.verts[tri[2]]
		area += c.Sub(a).Cross(d.Sub(a)).Z / 2
	}
	return area
}

// TestCutBandAlongEndPlane cuts a facet whose edge AB runs along the end plane
// y = 0 between two vertices that lie on it by provenance. The facet's corner A
// lies outside x ≥ 0 and B outside x ≤ 1, and the band is the rest of the
// facet inside both.
func TestCutBandAlongEndPlane(t *testing.T) {
	t.Parallel()

	a, b, c := r3.NewVec(-0.5, 0, 0), r3.NewVec(1.5, 0, 0), r3.NewVec(0.5, 3, 0)

	t.Run("cuts on the edge lie on the plane, the corner and the run are recorded", func(t *testing.T) {
		t.Parallel()
		w, band, f := flatBand(a, b, c)
		band.ends[0].on = map[int]bool{0: true, 1: true}
		cuts, err := w.cutBand(f, band)
		require.NoError(t, err)

		// Two cuts of the edge, at x = 0 and the band's corner at x = 1.
		require.Len(t, w.verts, 7, "two cuts on the edge and two on the other edges")
		ids := w.seam[[2]int{0, 1}]
		require.Len(t, ids, 2)
		require.Equal(t, cuts.corner[0], ids[1])
		require.InDelta(t, 1, w.verts[cuts.corner[0]].X, 1e-12)
		require.Equal(t, 0.0, w.verts[ids[0]].Y)
		require.InDelta(t, 0, w.verts[ids[0]].X, 1e-12)
		require.Empty(t, w.cuts[[2]int{0, 1}], "the corner is not a cut of the wall's row")

		// The run along the plane joins the two cuts: the corner has one neighbour.
		require.Equal(t, []int{ids[0]}, cuts.runs[0][ids[1]])
		require.Equal(t, []int{ids[1]}, cuts.runs[0][ids[0]])

		// The facet keeps its two corner triangles: 3 − 2.25.
		require.InDelta(t, 0.75, w.outArea(), 1e-12)
		require.Equal(t, map[int]bool{2: true}, cuts.removed)
	})

	t.Run("a held neighbour facet is split at the cuts", func(t *testing.T) {
		t.Parallel()
		w, band, f := flatBand(a, b, c)
		band.ends[0].on = map[int]bool{0: true, 1: true}
		_, err := w.cutBand(f, band)
		require.NoError(t, err)
		// A facet across the edge, below the plane, keeps the edge AB whole.
		w.verts = append(w.verts, r3.NewVec(0.5, -3, 0))
		w.beta = append(w.beta, 0.001)
		below := [3]int{1, 0, len(w.verts) - 1}
		facets, src := w.splitSeams([][3]int{below}, []int{7})
		require.Len(t, facets, 3, "the edge holds two cuts")
		require.Equal(t, []int{7, 7, 7}, src)
		area := 0.0
		for _, tri := range facets {
			p, q, r := w.verts[tri[0]], w.verts[tri[1]], w.verts[tri[2]]
			area += q.Sub(p).Cross(r.Sub(p)).Z / 2
			require.Contains(t, tri, len(w.verts)-1, "each piece fans from the opposite corner")
		}
		require.InDelta(t, 3, area, 1e-12, "the pieces keep the facet's area and winding")
	})

	t.Run("one vertex on the plane by provenance does not make the cut a corner", func(t *testing.T) {
		t.Parallel()
		// B lies on the plane too, but no face's tag says so: it is read by sign,
		// at no charge, and the cuts along AB are no band corner and no run.
		w, band, f := flatBand(a, b, c)
		band.ends[0].on = map[int]bool{0: true}
		cuts, err := w.cutBand(f, band)
		require.NoError(t, err)
		require.Equal(t, -1, cuts.corner[0])
		require.Empty(t, w.seam)
		_, err = cuts.endRun(0, 1)
		require.ErrorIs(t, err, decaderr.ErrUnsupported)
	})

	t.Run("the angle-main band reads no plane along both points", func(t *testing.T) {
		t.Parallel()
		w, band, _ := flatBand(a, b, c)
		band.ends[0].on = map[int]bool{0: true, 1: true}
		require.Equal(t, bandEnd0, band.alongBoth(w.heldPt(0), w.heldPt(1), bandOther))
		band.main = nil
		require.Equal(t, -1, band.alongBoth(w.heldPt(0), w.heldPt(1), bandOther))
	})
}
