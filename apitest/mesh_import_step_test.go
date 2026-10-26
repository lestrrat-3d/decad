package apitest_test

import (
	"bytes"
	"context"
	"fmt"
	"math"
	"strings"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/decadtest"
	"github.com/lestrrat-3d/decad/export"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file is agent-docs/step-import-design.md's public-surface tests for
// ReadSTEP. Most files come from decad's own STEP writer, so a body written and
// read back gives its own volume and bounds; the rest are written by hand.

func writeSTEP(t *testing.T, body *decad.Body) []byte {
	t.Helper()
	var out bytes.Buffer
	require.NoError(t, export.STEP(t.Context(), &out, body, units.Millimeters(0.1),
		export.WithSTEPName("part"), export.WithSTEPAuthor("apitest"), export.WithSTEPOrganization("decad")))
	return out.Bytes()
}

func readSTEPBytes(t *testing.T, data []byte, opts ...decad.MeshImportOption) (*decad.MeshBody, error) {
	t.Helper()
	return decad.ReadSTEP(t.Context(), bytes.NewReader(data), opts...)
}

func requireMeshVolume(t *testing.T, m *decad.MeshBody, want float64) {
	t.Helper()
	require.True(t, m.Report().Closed, "the mesh closes")
	volume, err := m.Volume()
	require.NoError(t, err)
	require.InDelta(t, want, volume.Value.Base(), 1e-6)
}

func TestReadSTEPRoundTripsABlock(t *testing.T) {
	t.Parallel()
	block := decadtest.NewBlock(t, decad.New(), 0, 0, 20, 30, units.Millimeters(10))
	m, err := readSTEPBytes(t, writeSTEP(t, block))
	require.NoError(t, err)
	requireMeshVolume(t, m, 6000)
	require.True(t, m.Bounds().Max.Equal(r3.NewVec(20, 30, 10), 1e-9))
	require.True(t, m.Bounds().Min.Equal(r3.NewVec(0, 0, 0), 1e-9))
	require.Equal(t, 12, m.Report().Triangles, "six faces of two triangles")
}

func TestReadSTEPRoundTripsAConcavePrism(t *testing.T) {
	t.Parallel()
	_, lshape := polygonPrism(t, [][2]float64{{0, 0}, {20, 0}, {20, 10}, {10, 10}, {10, 20}, {0, 20}})
	m, err := readSTEPBytes(t, writeSTEP(t, lshape))
	require.NoError(t, err)
	// The L is 20 x 10 plus 10 x 10 and 10 mm tall.
	requireMeshVolume(t, m, 10*(200+100))
}

func TestReadSTEPRoundTripsACavity(t *testing.T) {
	t.Parallel()
	closed, err := decad.Shell(t.Context(), decadtest.NewBlock(t, decad.New(), 0, 0, 20, 20, units.Millimeters(20)), nil,
		units.Millimeters(3), decad.WithNoOpenings())
	require.NoError(t, err)
	data := writeSTEP(t, closed)
	require.Contains(t, string(data), "BREP_WITH_VOIDS")
	m, err := readSTEPBytes(t, data)
	require.NoError(t, err)
	require.Equal(t, 2, m.Report().Shells, "the outer skin and the cavity")
	// 20³ less the 14³ cavity: the cavity's shell winds into it, so it takes volume away.
	requireMeshVolume(t, m, 8000-14*14*14)
}

func TestReadSTEPHonoursAnOrientedVoidShellsFlag(t *testing.T) {
	t.Parallel()
	closed, err := decad.Shell(t.Context(), decadtest.NewBlock(t, decad.New(), 0, 0, 20, 20, units.Millimeters(20)), nil,
		units.Millimeters(3), decad.WithNoOpenings())
	require.NoError(t, err)
	data := writeSTEP(t, closed)
	flipped := string(data)
	require.Contains(t, flipped, ",.T.)", "the writer's void is oriented as written")
	// Reversing the flag turns the cavity's faces over, so the cavity reads as
	// material added rather than taken away.
	i := strings.Index(flipped, "ORIENTED_CLOSED_SHELL")
	j := strings.Index(flipped[i:], ".T.")
	flipped = flipped[:i+j] + ".F." + flipped[i+j+3:]
	m, err := readSTEPBytes(t, []byte(flipped))
	require.NoError(t, err)
	requireMeshVolume(t, m, 8000+14*14*14)
}

func TestReadSTEPRoundTripsAFacetedCone(t *testing.T) {
	t.Parallel()
	// A cone's STEP is one planar face per mesh triangle, so it reads whole.
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	origin, apex, top := s.CreatePoint(0, 0), s.CreatePoint(10, 0), s.CreatePoint(0, 5)
	s.Fix(origin)
	s.CreateLine(origin, apex)
	s.CreateLine(apex, top)
	s.CreateLine(top, origin)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	axis := decad.SketchLine{Start: decad.Point2{U: 0, V: 0}, End: decad.Point2{U: 1, V: 0}}
	cone, err := decad.Revolve(t.Context(), decad.New(), s, s.Profiles()[0], axis, decad.FullRevolution{})
	require.NoError(t, err)
	var out bytes.Buffer
	require.NoError(t, export.STEP(t.Context(), &out, cone, units.Millimeters(0.5),
		export.WithSTEPName("cone"), export.WithSTEPAuthor("apitest"), export.WithSTEPOrganization("decad")))
	m, err := readSTEPBytes(t, out.Bytes())
	require.NoError(t, err)
	require.True(t, m.Report().Closed)
	// The file holds the cone's tessellation, one planar face per triangle, so the read
	// mesh has the volume of that tessellation, a little under the true cone's.
	volume, err := m.Volume()
	require.NoError(t, err)
	mesh, err := cone.Tessellate(t.Context(), units.Millimeters(0.5), decad.WithVerification(decad.VerifyBoundary))
	require.NoError(t, err)
	held, err := decad.NewMeshBody(t.Context(), mesh.Vertices(), mesh.Triangles())
	require.NoError(t, err)
	heldVolume, err := held.Volume()
	require.NoError(t, err)
	require.InDelta(t, heldVolume.Value.Base(), volume.Value.Base(), 1e-9)
	exact := math.Pi * 25 * 10 / 3
	require.Less(t, volume.Value.Base(), exact)
	require.Greater(t, volume.Value.Base(), 0.85*exact)
	require.InDelta(t, 10, m.Bounds().Max.X, 1e-9)
}

func TestReadSTEPRefusesACurvedBody(t *testing.T) {
	t.Parallel()
	// A drilled plate's STEP carries the bore as a cylinder, which is not read.
	doc := decad.New()
	plate := decadtest.NewBlock(t, doc, 0, 0, 20, 20, units.Millimeters(10))
	drilled, err := decad.Hole(t.Context(), plate, decad.HoleSpec{
		Face:     capFace(true, plate),
		At:       r3.Vec{X: 10, Y: 10, Z: 10},
		Diameter: units.Millimeters(6),
		Depth:    decad.ThroughHole{},
	})
	require.NoError(t, err)
	_, err = readSTEPBytes(t, writeSTEP(t, drilled))
	require.ErrorIs(t, err, decad.ErrUnsupported)
	require.Contains(t, err.Error(), "CYLINDRICAL_SURFACE")
}

func TestReadSTEPHandWrittenPlateWithAHoleAndInches(t *testing.T) {
	t.Parallel()
	// A 2 x 2 inch square face with a 1 x 1 inch square hole, extruded 1 inch, as one
	// solid made of ten faces. Units are inches by conversion.
	src := plateWithSquareHoleFile("")
	m, err := readSTEPBytes(t, []byte(src))
	require.NoError(t, err)
	require.True(t, m.Report().Closed)
	inch := 25.4
	requireMeshVolume(t, m, (4-1)*inch*inch*inch)
	require.InDelta(t, 2*inch, m.Bounds().Max.X, 1e-9)
}

func TestReadSTEPRefusals(t *testing.T) {
	t.Parallel()
	good := writeSTEP(t, decadtest.NewBlock(t, decad.New(), 0, 0, 4, 4, units.Millimeters(2)))
	cases := map[string]struct {
		data []byte
		want error
	}{
		"text that is not STEP":    {[]byte("hello"), decad.ErrMalformedMesh},
		"an unclosed data section": {[]byte("ISO-10303-21;DATA;#1=A();"), decad.ErrMalformedMesh},
		"no solid":                 {[]byte("ISO-10303-21;DATA;#1=CARTESIAN_POINT('',(0.,0.,0.));ENDSEC;"), decad.ErrMalformedMesh},
		"a dangling reference":     {[]byte(strings.Replace(string(good), "CLOSED_SHELL('',(", "CLOSED_SHELL('',(#99999,", 1)), decad.ErrMalformedMesh},
	}
	for name, tc := range cases {
		_, err := readSTEPBytes(t, tc.data)
		require.ErrorIs(t, err, tc.want, name)
	}
	_, err := readSTEPBytes(t, good, decad.WithMaxMeshTriangles(5))
	require.ErrorIs(t, err, decad.ErrMeshTooLarge)
	_, err = readSTEPBytes(t, good, decad.WithMaxMeshBytes(100))
	require.ErrorIs(t, err, decad.ErrMeshTooLarge)
	// A cylinder face is not read.
	_, err = readSTEPBytes(t, []byte(plateWithSquareHoleFile("CYLINDRICAL_SURFACE")))
	require.ErrorIs(t, err, decad.ErrUnsupported)
	var nilContext context.Context
	_, err = decad.ReadSTEP(nilContext, bytes.NewReader(good))
	require.ErrorIs(t, err, decad.ErrDegenerate)
	_, err = decad.ReadSTEP(t.Context(), nil)
	require.ErrorIs(t, err, decad.ErrDegenerate)
	canceled, cancel := context.WithCancel(t.Context())
	cancel()
	_, err = decad.ReadSTEP(canceled, bytes.NewReader(good))
	require.ErrorIs(t, err, context.Canceled)
}

// stepBuilder writes a STEP file one instance at a time.
type stepBuilder struct {
	lines []string
	edges map[[2][3]float64]int
}

func (b *stepBuilder) add(format string, args ...any) int {
	id := len(b.lines) + 1
	b.lines = append(b.lines, fmt.Sprintf("#%d=%s;", id, fmt.Sprintf(format, args...)))
	return id
}

func (b *stepBuilder) point(p [3]float64) (point, vertex int) {
	point = b.add("CARTESIAN_POINT('',(%v,%v,%v))", p[0], p[1], p[2])
	return point, b.add("VERTEX_POINT('',#%d)", point)
}

// orientedEdge returns an ORIENTED_EDGE from a to b, creating the straight edge
// the first time either direction is asked for and sharing it after.
func (b *stepBuilder) orientedEdge(a, c [3]float64) int {
	if b.edges == nil {
		b.edges = map[[2][3]float64]int{}
	}
	if e, ok := b.edges[[2][3]float64{c, a}]; ok {
		return b.add("ORIENTED_EDGE('',*,*,#%d,.F.)", e)
	}
	pa, va := b.point(a)
	_, vc := b.point(c)
	dir := b.add("DIRECTION('',(%v,%v,%v))", c[0]-a[0], c[1]-a[1], c[2]-a[2])
	vec := b.add("VECTOR('',#%d,1.)", dir)
	line := b.add("LINE('',#%d,#%d)", pa, vec)
	e := b.add("EDGE_CURVE('',#%d,#%d,#%d,.T.)", va, vc, line)
	b.edges[[2][3]float64{a, c}] = e
	return b.add("ORIENTED_EDGE('',*,*,#%d,.T.)", e)
}

// face adds an ADVANCED_FACE on the plane with the given normal through the
// first ring's first point; the first ring is the outer bound.
func (b *stepBuilder) face(surface string, normal [3]float64, rings ...[][3]float64) int {
	var bounds []string
	for i, ring := range rings {
		var oriented []string
		for k := range ring {
			oriented = append(oriented, fmt.Sprintf("#%d", b.orientedEdge(ring[k], ring[(k+1)%len(ring)])))
		}
		loop := b.add("EDGE_LOOP('',(%s))", strings.Join(oriented, ","))
		kind := "FACE_BOUND"
		if i == 0 {
			kind = "FACE_OUTER_BOUND"
		}
		bounds = append(bounds, fmt.Sprintf("#%d", b.add("%s('',#%d,.T.)", kind, loop)))
	}
	origin := b.add("CARTESIAN_POINT('',(%v,%v,%v))", rings[0][0][0], rings[0][0][1], rings[0][0][2])
	axis := b.add("DIRECTION('',(%v,%v,%v))", normal[0], normal[1], normal[2])
	ref := b.add("DIRECTION('',(0.,0.,1.))")
	placement := b.add("AXIS2_PLACEMENT_3D('',#%d,#%d,#%d)", origin, axis, ref)
	var geometry string
	if surface == "" {
		geometry = fmt.Sprintf("#%d", b.add("PLANE('',#%d)", placement))
	} else {
		geometry = fmt.Sprintf("#%d", b.add("%s('',#%d,5.)", surface, placement))
	}
	return b.add("ADVANCED_FACE('',(%s),%s,.T.)", strings.Join(bounds, ","), geometry)
}

// plateWithSquareHole is a 2 x 2 x 1 inch plate with a 1 x 1 inch square hole
// through it, in inches, as a MANIFOLD_SOLID_BREP of ten planar faces. firstSurface,
// when not empty, names a surface type for the top face in place of PLANE.
func plateWithSquareHoleFile(firstSurface string) string {
	b := &stepBuilder{}
	millimetre := b.add("( LENGTH_UNIT() NAMED_UNIT(*) SI_UNIT(.MILLI.,.METRE.) )")
	inch := b.add("MEASURE_WITH_UNIT(LENGTH_MEASURE(25.4),#%d)", millimetre)
	inchUnit := b.add("( CONVERSION_BASED_UNIT('INCH',#%d) LENGTH_UNIT() NAMED_UNIT(*) )", inch)
	b.add("( GEOMETRIC_REPRESENTATION_CONTEXT(3) GLOBAL_UNIT_ASSIGNED_CONTEXT((#%d)) REPRESENTATION_CONTEXT('','') )", inchUnit)
	z := func(pts [][2]float64, height float64) [][3]float64 {
		out := make([][3]float64, len(pts))
		for i, p := range pts {
			out[i] = [3]float64{p[0], p[1], height}
		}
		return out
	}
	rev := func(r [][3]float64) [][3]float64 {
		out := make([][3]float64, len(r))
		for i := range r {
			out[len(r)-1-i] = r[i]
		}
		return out
	}
	outer := [][2]float64{{0, 0}, {2, 0}, {2, 2}, {0, 2}}
	hole := [][2]float64{{0.5, 0.5}, {0.5, 1.5}, {1.5, 1.5}, {1.5, 0.5}} // clockwise from +z
	var faces []int
	faces = append(faces, b.face(firstSurface, [3]float64{0, 0, 1}, z(outer, 1), z(hole, 1)))
	faces = append(faces, b.face("", [3]float64{0, 0, -1}, rev(z(outer, 0)), rev(z(hole, 0))))
	side := func(ring [][2]float64) {
		for i := range ring {
			a, c := ring[i], ring[(i+1)%len(ring)]
			dx, dy := c[0]-a[0], c[1]-a[1]
			n := math.Hypot(dx, dy)
			faces = append(faces, b.face("", [3]float64{dy / n, -dx / n, 0},
				[][3]float64{{a[0], a[1], 0}, {c[0], c[1], 0}, {c[0], c[1], 1}, {a[0], a[1], 1}}))
		}
	}
	side(outer)
	side(hole)
	var refs []string
	for _, f := range faces {
		refs = append(refs, fmt.Sprintf("#%d", f))
	}
	shell := b.add("CLOSED_SHELL('',(%s))", strings.Join(refs, ","))
	b.add("MANIFOLD_SOLID_BREP('plate',#%d)", shell)
	return "ISO-10303-21;\nHEADER;\nFILE_DESCRIPTION(('hand written'),'2;1');\nENDSEC;\nDATA;\n" +
		strings.Join(b.lines, "\n") + "\nENDSEC;\nEND-ISO-10303-21;\n"
}
