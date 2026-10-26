package apitest_test

import (
	"fmt"
	"math"
	"strings"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/stretchr/testify/require"
)

// This file is agent-docs/step-import-design.md §10's public-surface tests for
// spherical faces, on hand-written domes: a sphere of radius 5 about the origin cut by
// the plane z = h, the cap above it as one SPHERICAL_SURFACE face with one circular loop
// and the disc below it as a plane.

// domeFile is that solid in millimetres. sphereRadius is the radius written for the sphere
// (5 for a true one); reverse writes the cap's loop backward with its bound flagged
// reversed, which is the same ring. insideOut writes both faces with a reversed sense and
// their loops walked the other way, so every normal points into the solid.
func domeFile(h, sphereRadius float64, reverse bool) string {
	return dome(h, sphereRadius, reverse, false)
}

func dome(h, sphereRadius float64, reverse, insideOut bool) string {
	sense := ".T."
	if insideOut {
		sense = ".F."
	}
	b := &stepBuilder{}
	shared := map[float64]int{}
	ring := math.Sqrt(25 - h*h)
	origin := b.add("CARTESIAN_POINT('',(0.,0.,%v))", h)
	down := b.add("DIRECTION('',(0.,0.,-1.))")
	ref := b.add("DIRECTION('',(1.,0.,0.))")
	basePlacement := b.add("AXIS2_PLACEMENT_3D('',#%d,#%d,#%d)", origin, down, ref)
	basePlane := b.add("PLANE('',#%d)", basePlacement)
	baseLoop := b.circleLoop(h, ring, insideOut, shared)
	baseBound := b.add("FACE_OUTER_BOUND('',#%d,.T.)", baseLoop)
	base := b.add("ADVANCED_FACE('',(#%d),#%d,%s)", baseBound, basePlane, sense)

	centre := b.add("CARTESIAN_POINT('',(0.,0.,0.))")
	up := b.add("DIRECTION('',(0.,0.,1.))")
	spherePlacement := b.add("AXIS2_PLACEMENT_3D('',#%d,#%d,#%d)", centre, up, ref)
	sphere := b.add("SPHERICAL_SURFACE('',#%d,%v)", spherePlacement, sphereRadius)
	capLoop := b.circleLoop(h, ring, !reverse != insideOut, shared)
	flag := ".T."
	if reverse {
		flag = ".F."
	}
	capBound := b.add("FACE_OUTER_BOUND('',#%d,%s)", capLoop, flag)
	capFace := b.add("ADVANCED_FACE('',(#%d),#%d,%s)", capBound, sphere, sense)
	shell := b.add("CLOSED_SHELL('',(#%d,#%d))", base, capFace)
	b.add("MANIFOLD_SOLID_BREP('dome',#%d)", shell)
	return "ISO-10303-21;\nHEADER;\nENDSEC;\nDATA;\n" + strings.Join(b.lines, "\n") + "\nENDSEC;\nEND-ISO-10303-21;\n"
}

// capVolume is the volume of the part of a sphere of radius 5 above z = h, π·c²·(3R − c)/3 for
// the cap's height c = R − h.
func capVolume(h float64) float64 {
	c := 5 - h
	return math.Pi * c * c * (15 - c) / 3
}

func TestReadSTEPReadsASphericalCap(t *testing.T) {
	t.Parallel()
	// A hemisphere, a smaller cap and a cap larger than a hemisphere, so the base lies at
	// the centre, above it and below it.
	for _, h := range []float64{0, 3, -2} {
		t.Run(fmt.Sprintf("h=%v", h), func(t *testing.T) {
			t.Parallel()
			m, err := readSTEPBytes(t, []byte(domeFile(h, 5, false)))
			require.NoError(t, err)
			require.True(t, m.Report().Closed, "the cap meets the disc along the same chords")
			require.False(t, m.Report().Inverted, "wound outward")
			volume, err := m.Volume()
			require.NoError(t, err)
			// Inscribed facets enclose a little less than the sphere, and the pole's
			// height and the base are exact.
			require.Less(t, volume.Value.Base(), capVolume(h))
			require.Greater(t, volume.Value.Base(), 0.98*capVolume(h))
			require.InDelta(t, 5, m.Bounds().Max.Z, 1e-9, "the pole is a vertex")
			require.InDelta(t, h, m.Bounds().Min.Z, 1e-9, "the base is flat at z = h")
		})
	}
}

func TestReadSTEPReadsASphericalCapWalkedBackward(t *testing.T) {
	t.Parallel()
	forward, err := readSTEPBytes(t, []byte(domeFile(3, 5, false)))
	require.NoError(t, err)
	backward, err := readSTEPBytes(t, []byte(domeFile(3, 5, true)))
	require.NoError(t, err)
	vf, err := forward.Volume()
	require.NoError(t, err)
	vb, err := backward.Volume()
	require.NoError(t, err)
	require.InDelta(t, vf.Value.Base(), vb.Value.Base(), 1e-9, "a loop walked the other way with a reversed bound is the same ring")
	require.True(t, backward.Report().Closed)
}

func TestReadSTEPRefusesSphericalFacesItDoesNotRead(t *testing.T) {
	t.Parallel()
	// A top face with a hole has two bounds: a zone, not a cap.
	_, err := readSTEPBytes(t, []byte(plateWithSquareHoleFile("SPHERICAL_SURFACE")))
	require.ErrorIs(t, err, decad.ErrUnsupported)
	require.Contains(t, err.Error(), "sphere face")
	// A boundary that does not lie on the sphere it names is not a model.
	_, err = readSTEPBytes(t, []byte(domeFile(3, 4, false)))
	require.Error(t, err)
	require.Contains(t, err.Error(), "off its sphere")
}

func TestReadSTEPReadsAnInsideOutSphericalCap(t *testing.T) {
	t.Parallel()
	// A face whose sense is reversed has its normal toward the sphere's centre, and its loop
	// runs counter-clockwise about that normal: the same cap, read with every normal turned in.
	m, err := readSTEPBytes(t, []byte(dome(3, 5, false, true)))
	require.NoError(t, err)
	require.True(t, m.Report().Closed)
	require.True(t, m.Report().Inverted, "every normal points into the solid")
	outer, err := readSTEPBytes(t, []byte(domeFile(3, 5, false)))
	require.NoError(t, err)
	require.Equal(t, outer.Report().Triangles, m.Report().Triangles, "the same cap, in the other sense")
	require.True(t, m.Bounds().Max.Equal(outer.Bounds().Max, 1e-9))
}
