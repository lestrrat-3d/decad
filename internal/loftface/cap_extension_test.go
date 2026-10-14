package loftface_test

import (
	"math"
	"math/big"
	"reflect"
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/loftface"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

const (
	faceBottom = iota
	faceCap
	faceFront
	faceBack
	faceLeft
	faceRight
)

// cubeMesh is the unit cube shifted by dx along X, wound outward. Its top
// face (faceCap) is the end cap of the tests; its four side faces are the
// walls the cap meets.
func cubeMesh(dx float64, base int) loftface.HeldMesh {
	mesh := loftface.HeldMesh{}
	for i := range 8 {
		mesh.Vertices = append(mesh.Vertices, r3.NewVec(dx+float64(i&1), float64(i>>1&1), float64(i>>2&1)))
		mesh.VertexBound = append(mesh.VertexBound, 1e-3)
	}
	for _, f := range []struct {
		face int
		tris [][3]int
	}{
		{faceBottom, [][3]int{{0, 2, 1}, {1, 2, 3}}},
		{faceCap, [][3]int{{4, 5, 6}, {5, 7, 6}}},
		{faceFront, [][3]int{{0, 1, 5}, {0, 5, 4}}},
		{faceBack, [][3]int{{2, 7, 3}, {2, 6, 7}}},
		{faceLeft, [][3]int{{0, 4, 6}, {0, 6, 2}}},
		{faceRight, [][3]int{{1, 3, 7}, {1, 7, 5}}},
	} {
		for _, tri := range f.tris {
			mesh.Triangles = append(mesh.Triangles, [3]int{tri[0] + base, tri[1] + base, tri[2] + base})
			mesh.Face = append(mesh.Face, f.face)
		}
	}
	return mesh
}

// tiltedPlane has the normal (0.1, 0, 1), so the extension vertices are not
// exact floats and every one of them carries a positive rounding gap. The
// loft lies below the plane, so side is -1 and the prism points along +N.
func tiltedPlane() loftface.TangentPlane {
	return loftface.TangentPlane{N: loftface.Vec(r3.NewVec(0.1, 0, 1)), D: rat(1, 1), Side: -1}
}

// cubeLink makes the cube's cap ring (4 -> 5 -> 7 -> 6) the station ring of
// four stations 0..3, with the given per-chord bounds. Chord j joins station
// j and j+1.
func cubeLink(chord [4]float64) loftface.StationLink {
	station := make([]int, 8)
	for i := range station {
		station[i] = -1
	}
	station[4], station[5], station[7], station[6] = 0, 1, 2, 3
	return loftface.StationLink{Station: station, ChordBound: chord[:]}
}

// chordOfWall returns the chord (0..3) of an extension wall triangle from
// its two original ring vertices.
func chordOfWall(tri [3]int, link loftface.StationLink) int {
	var stations []int
	for _, v := range tri {
		if v < len(link.Station) && link.Station[v] >= 0 {
			stations = append(stations, link.Station[v])
		}
	}
	lo, hi := min(stations[0], stations[1]), max(stations[0], stations[1])
	if hi-lo == 3 {
		return hi
	}
	return lo
}

func TestCapExtensionBuildsGluedProxy(t *testing.T) {
	held := cubeMesh(0, 0)
	// A bottom vertex with a large bound makes the front and left wall faces
	// larger than any bound the extension walls need.
	held.VertexBound[0] = 0.5
	plane := tiltedPlane()
	h := rat(1, 1)
	link := cubeLink([4]float64{0.05, 0.02, 0.03, 0.04})
	ext, err := loftface.CapExtension(t.Context(), held, faceCap, plane, h, link)
	require.NoError(t, err)

	// Four ring vertices gained one extension vertex each; four wall quads
	// of two triangles each joined the mesh.
	require.Len(t, ext.Vertices, 12)
	require.Len(t, ext.Triangles, len(held.Triangles)+8)
	require.Len(t, ext.Face, len(ext.Triangles))
	require.Len(t, ext.VertexBound, len(ext.Vertices))

	// No cap triangle stays at the old position; every cap triangle uses
	// extension vertices only.
	for i, tri := range ext.Triangles {
		if ext.Face[i] != faceCap {
			continue
		}
		for _, v := range tri {
			require.GreaterOrEqual(t, v, 8, "a cap triangle still uses an original vertex")
		}
	}

	// Each extension vertex is the exact translate rounded to nearest, and
	// its bound exceeds the ring vertex bound by the gap's 3D radius.
	normUp := proofbound.RatSqrtUp(loftface.VDot(plane.N, plane.N))
	disp := loftface.VScale(plane.N, new(big.Rat).Quo(h, new(big.Rat).SetFloat64(normUp)))
	for v := 4; v < 8; v++ {
		var found bool
		for e := 8; e < 12; e++ {
			exact := loftface.VAdd(loftface.Vec(held.Vertices[v]), disp)
			gap := new(big.Rat)
			for k := range 3 {
				d := new(big.Rat).Abs(new(big.Rat).Sub(loftface.Vec(ext.Vertices[e])[k], exact[k]))
				if d.Cmp(gap) > 0 {
					gap = d
				}
			}
			if gap.Cmp(new(big.Rat).SetFloat64(1e-9)) > 0 {
				continue
			}
			found = true
			require.Positive(t, gap.Sign(), "the tilted direction makes every extension vertex inexact")
			radius := proofbound.Radius3D(proofbound.RatFloatUp(gap))
			require.GreaterOrEqual(t, ext.VertexBound[e], held.VertexBound[v]+radius,
				"the extension vertex bound charges its rounding gap")
		}
		require.True(t, found, "ring vertex %d has an extension vertex", v)
	}

	// Each extension wall triangle's corner maximum covers its chord's bound.
	walls := ext.Triangles[len(held.Triangles):]
	for i, tri := range walls {
		corner := 0.0
		for _, v := range tri {
			corner = max(corner, ext.VertexBound[v])
		}
		// The two triangles of one wall quad stand for the same chord.
		chord := link.ChordBound[chordOfWall(walls[i-i%2], link)]
		require.Contains(t, []int{faceFront, faceBack, faceLeft, faceRight}, ext.Face[len(held.Triangles)+i])
		require.GreaterOrEqual(t, corner, chord, "wall triangle %d covers chord bound %g", i, chord)
		require.Greater(t, corner, chord, "the wall bound also charges the extension rounding gap")
	}

	// A face bound is the maximum facet bound over the face's final facets, so
	// the front wall (which also holds the bound-0.5 vertex) is at least 0.5.
	for _, face := range []int{faceCap, faceFront, faceBack, faceLeft, faceRight} {
		want := 0.0
		for i, tri := range ext.Triangles {
			if ext.Face[i] != face {
				continue
			}
			for _, v := range tri {
				want = max(want, ext.VertexBound[v])
			}
		}
		require.Equal(t, want, ext.FaceBound[face], "face %d bound is the recomputed maximum", face)
	}
	require.GreaterOrEqual(t, ext.FaceBound[faceFront], 0.5)
	require.GreaterOrEqual(t, ext.FaceBound[faceLeft], 0.5)
	require.NotContains(t, ext.FaceBound, faceBottom)
	require.Positive(t, ext.CoordCharge)

	// The extension carries no volume or area charge.
	for _, name := range []string{"VolumeCharge", "AreaCharge"} {
		_, found := reflect.TypeOf(ext).FieldByName(name)
		require.False(t, found, "Extension has no %s field", name)
	}

	// The proxy is the cube with a sheared prism on top. The prism's height is
	// the vertical part of h*N/|N|, so the volume is 1 + 1/sqrt(1.01).
	vol := 0.0
	for _, tri := range ext.Triangles {
		a, b, c := ext.Vertices[tri[0]], ext.Vertices[tri[1]], ext.Vertices[tri[2]]
		vol += a.Dot(b.Cross(c)) / 6
	}
	require.InDelta(t, 1+1/math.Sqrt(1.01), vol, 1e-9)

	// The ring must be the station ring: a rotated station index, a ring
	// vertex that is no station, and a short chord list all refuse.
	for name, bad := range map[string]loftface.StationLink{
		"rotated station": func() loftface.StationLink {
			l := cubeLink([4]float64{0.05, 0.02, 0.03, 0.04})
			l.Station[5] = 2
			return l
		}(),
		"no station": func() loftface.StationLink {
			l := cubeLink([4]float64{0.05, 0.02, 0.03, 0.04})
			l.Station[6] = -1
			return l
		}(),
		"short chords": {Station: cubeLink([4]float64{}).Station, ChordBound: []float64{0.1, 0.1, 0.1}},
	} {
		_, err := loftface.CapExtension(t.Context(), held, faceCap, plane, h, bad)
		require.ErrorIs(t, err, decaderr.ErrUnsupported, name)
	}
}

// circleOctagon is an octagonal prism whose cap ring is sampled from a circle
// of radius 2, with the mesh and the station link for its cap ring. The held
// chord departs from the true circle by the sagitta, which only the chord
// bound can supply. The returned sagitta is rounded up.
func circleOctagon() (mesh loftface.HeldMesh, link loftface.StationLink, sagitta float64, stride int) {
	const n, radius = 8, 2.0
	for _, z := range []float64{0, 1} {
		for k := range n {
			angle := 2 * math.Pi * float64(k) / n
			mesh.Vertices = append(mesh.Vertices, r3.NewVec(radius*math.Cos(angle), radius*math.Sin(angle), z))
		}
		mesh.Vertices = append(mesh.Vertices, r3.NewVec(0, 0, z))
	}
	stride = n + 1
	for range mesh.Vertices {
		mesh.VertexBound = append(mesh.VertexBound, 1e-9)
	}
	add := func(face int, a, b, c int) {
		mesh.Triangles = append(mesh.Triangles, [3]int{a, b, c})
		mesh.Face = append(mesh.Face, face)
	}
	for k := range n {
		l0, l1 := k, (k+1)%n
		u0, u1 := stride+k, stride+(k+1)%n
		add(faceBottom, n, l1, l0)
		add(faceCap, stride+n, u0, u1)
		add(faceFront+k, l0, l1, u1)
		add(faceFront+k, l0, u1, u0)
	}
	// The top ring runs counter-clockwise, so ring edge k joins stations k and k+1.
	link = loftface.StationLink{Station: make([]int, len(mesh.Vertices)), ChordBound: make([]float64, n)}
	for i := range link.Station {
		link.Station[i] = -1
	}
	sagitta = proofbound.AbsSumUpper(radius*(1-math.Cos(math.Pi/n)), 1e-15)
	for k := range n {
		link.Station[stride+k] = k
		link.ChordBound[k] = sagitta
	}
	return mesh, link, sagitta, stride
}

// TestCapExtensionCurvedChordBound lifts the circle octagon straight up. The
// translation is exact, so each wall bound is the supplied rounded-up sagitta
// plus no gap, and never less.
func TestCapExtensionCurvedChordBound(t *testing.T) {
	const n = 8
	mesh, link, sagitta, _ := circleOctagon()
	require.Greater(t, sagitta, 0.1522)
	plane := loftface.TangentPlane{N: loftface.Vec(r3.NewVec(0, 0, 1)), D: rat(1, 1), Side: -1}
	ext, err := loftface.CapExtension(t.Context(), mesh, faceCap, plane, rat(1, 1), link)
	require.NoError(t, err)
	require.Zero(t, ext.CoordCharge, "a vertical unit translation of these coordinates is exact")

	wallTris := 0
	for i := len(mesh.Triangles); i < len(ext.Triangles); i++ {
		corner := 0.0
		for _, v := range ext.Triangles[i] {
			corner = max(corner, ext.VertexBound[v])
		}
		require.GreaterOrEqual(t, corner, sagitta, "extension wall %d covers the supplied circle sagitta", i)
		wallTris++
	}
	require.Equal(t, 2*n, wallTris)
}

// TestCapExtensionChordBoundWithTranslationGap tilts the prism so that every
// extension vertex carries a positive rounding gap. Each wall bound is the
// chord bound plus the larger gap radius of its two extension vertices.
func TestCapExtensionChordBoundWithTranslationGap(t *testing.T) {
	const n = 8
	mesh, link, sagitta, stride := circleOctagon()
	plane := tiltedPlane() // the loft lies below it, so the prism points along +N
	h := rat(1, 1)
	ext, err := loftface.CapExtension(t.Context(), mesh, faceCap, plane, h, link)
	require.NoError(t, err)
	require.Positive(t, ext.CoordCharge)

	normUp := proofbound.RatSqrtUp(loftface.VDot(plane.N, plane.N))
	// radius is the 3D radius of the rounding gap of extension vertex e over
	// the ring vertex v.
	radius := func(v, e int) float64 {
		exact := loftface.VAdd(loftface.Vec(mesh.Vertices[v]), loftface.VScale(plane.N,
			new(big.Rat).Quo(h, new(big.Rat).SetFloat64(normUp))))
		worst := new(big.Rat)
		for k := range 3 {
			gap := new(big.Rat).Abs(new(big.Rat).Sub(loftface.Vec(ext.Vertices[e])[k], exact[k]))
			if gap.Cmp(worst) > 0 {
				worst = gap
			}
		}
		return proofbound.Radius3D(proofbound.RatFloatUp(worst))
	}
	walls := ext.Triangles[len(mesh.Triangles):]
	require.Len(t, walls, 2*n)
	for k := range n {
		// A wall quad is {a, b, e_b} then {a, e_b, e_a}; the ring starts anywhere.
		a, b := walls[2*k][0], walls[2*k][1]
		require.Equal(t, (a-stride+1)%n, b-stride, "ring edge a -> b is a station chord")
		eb, ea := walls[2*k][2], walls[2*k+1][2]
		ra, rb := radius(a, ea), radius(b, eb)
		require.Positive(t, max(ra, rb), "the tilted translation leaves a gap")
		want := proofbound.AbsSumUpper(max(1e-9, sagitta), max(ra, rb))
		require.Greater(t, want, sagitta)
		for _, tri := range walls[2*k : 2*k+2] {
			corner := 0.0
			for _, v := range tri {
				corner = max(corner, ext.VertexBound[v])
			}
			require.GreaterOrEqual(t, corner, want, "wall %d covers the chord bound and the translation gap", k)
		}
	}
}

func TestCapExtensionRefusesBrokenRing(t *testing.T) {
	plane := tiltedPlane()
	h := rat(1, 1)

	// Two disjoint caps under one face id give two boundary cycles.
	one, two := cubeMesh(0, 0), cubeMesh(3, 8)
	twoCycles := loftface.HeldMesh{
		Vertices:    append(append([]r3.Vec(nil), one.Vertices...), two.Vertices...),
		Triangles:   append(append([][3]int(nil), one.Triangles...), two.Triangles...),
		Face:        append(append([]int(nil), one.Face...), two.Face...),
		VertexBound: append(append([]float64(nil), one.VertexBound...), two.VertexBound...),
	}
	_, err := loftface.CapExtension(t.Context(), twoCycles, faceCap, plane, h, cubeLink([4]float64{0.1, 0.1, 0.1, 0.1}))
	require.ErrorIs(t, err, decaderr.ErrUnsupported)
	require.ErrorContains(t, err, "more than one cycle")

	// A boundary edge with two non-cap neighbours.
	doubled := cubeMesh(0, 0)
	doubled.Triangles = append(doubled.Triangles, doubled.Triangles[4])
	doubled.Face = append(doubled.Face, doubled.Face[4])
	_, err = loftface.CapExtension(t.Context(), doubled, faceCap, plane, h, cubeLink([4]float64{0.1, 0.1, 0.1, 0.1}))
	require.ErrorIs(t, err, decaderr.ErrUnsupported)

	// A cap with no cap triangle, and a mesh whose cap is the whole boundary.
	_, err = loftface.CapExtension(t.Context(), one, 99, plane, h, cubeLink([4]float64{0.1, 0.1, 0.1, 0.1}))
	require.ErrorIs(t, err, decaderr.ErrUnsupported)

	// A wall facet that does not close the cube leaves a boundary edge with
	// no wall neighbour.
	open := cubeMesh(0, 0)
	open.Triangles = open.Triangles[:len(open.Triangles)-2]
	open.Face = open.Face[:len(open.Face)-2]
	_, err = loftface.CapExtension(t.Context(), open, faceCap, plane, h, cubeLink([4]float64{0.1, 0.1, 0.1, 0.1}))
	require.ErrorIs(t, err, decaderr.ErrUnsupported)

	// A direction that makes the proxy self-intersect fails the audit: the
	// prism points down into the cube.
	inward := tiltedPlane()
	inward.Side = 1
	_, err = loftface.CapExtension(t.Context(), one, faceCap, inward, h, cubeLink([4]float64{0.1, 0.1, 0.1, 0.1}))
	require.ErrorIs(t, err, decaderr.ErrUnsupported)
}
