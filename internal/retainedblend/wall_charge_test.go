package retainedblend_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/retainedblend"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

const (
	holeSegments = 96
	holeRadius   = 3.0
	plateHeight  = 5.0
	plateHalf    = 10.0
)

// holedPlate is a 20 x 20 x 5 plate with a round through hole of radius 3 at the origin, held as a faceted payload
// of 96 segments. The hole wall is group 6, a retained reversed cylinder that states wallDelta when hasDelta is set.
func holedPlate(t *testing.T, wallDelta float64, hasDelta bool) (*topo.Body, facetedrecord.Payload) {
	t.Helper()
	n := holeSegments
	var fp facetedrecord.Payload
	add := func(p r3.Vec, bound float64) int {
		fp.Verts = append(fp.Verts, p)
		fp.VertexBound = append(fp.VertexBound, bound)
		return len(fp.Verts) - 1
	}
	holeBound := holeRadius * (1 - math.Cos(math.Pi/float64(n))) * 1.01
	var holeLow, holeHigh, sqLow, sqHigh [holeSegments]int
	for k := range n {
		theta := 2 * math.Pi * float64(k) / float64(n)
		c, s := math.Cos(theta), math.Sin(theta)
		sx, sy := square(k, n, c, s)
		for _, z := range []float64{0, plateHeight} {
			h := add(r3.NewVec(holeRadius*c, holeRadius*s, z), holeBound)
			q := add(r3.NewVec(sx, sy, z), 0)
			if z == 0 {
				holeLow[k], sqLow[k] = h, q
			} else {
				holeHigh[k], sqHigh[k] = h, q
			}
		}
	}
	tri := func(g, a, b, c int) {
		fp.Tris = append(fp.Tris, [3]int{a, b, c})
		fp.Src = append(fp.Src, g)
	}
	const top, bottom, wall = 0, 1, 6
	for k := range n {
		j := (k + 1) % n
		// Top and bottom annulus quads between hole ring k..j and square ring k..j.
		tri(top, holeHigh[k], sqHigh[k], sqHigh[j])
		tri(top, holeHigh[k], sqHigh[j], holeHigh[j])
		tri(bottom, holeLow[k], sqLow[j], sqLow[k])
		tri(bottom, holeLow[k], holeLow[j], sqLow[j])
		// The hole wall faces the axis.
		tri(wall, holeLow[k], holeHigh[j], holeLow[j])
		tri(wall, holeLow[k], holeHigh[k], holeHigh[j])
		// The outer side the segment lies on.
		side := 2 + ((k+n/8)%n)/(n/4)
		tri(side, sqLow[k], sqLow[j], sqHigh[j])
		tri(side, sqLow[k], sqHigh[j], sqHigh[k])
	}
	fp.Groups = []facetedrecord.Group{
		axialPlane(t, "top", r3.NewVec(0, 0, plateHeight), r3.NewVec(1, 0, plateHeight), r3.NewVec(0, 1, plateHeight)),
		axialPlane(t, "bottom", r3.NewVec(0, 0, 0), r3.NewVec(0, 1, 0), r3.NewVec(1, 0, 0)),
		axialPlane(t, "right", r3.NewVec(plateHalf, 0, 0), r3.NewVec(plateHalf, 1, 0), r3.NewVec(plateHalf, 0, 1)),
		axialPlane(t, "back", r3.NewVec(0, plateHalf, 0), r3.NewVec(0, plateHalf, 1), r3.NewVec(1, plateHalf, 0)),
		axialPlane(t, "left", r3.NewVec(-plateHalf, 0, 0), r3.NewVec(-plateHalf, 0, 1), r3.NewVec(-plateHalf, 1, 0)),
		axialPlane(t, "front", r3.NewVec(0, -plateHalf, 0), r3.NewVec(1, -plateHalf, 0), r3.NewVec(0, -plateHalf, 1)),
	}
	hole := axialPlane(t, "hole", r3.Vec{}, r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	hole.Planar = false
	hole.Surface = surfacegeom.Cylinder{Origin: r3.Vec{}, Axis: r3.NewVec(0, 0, 1), Radius: units.Millimeters(holeRadius)}
	hole.Reversed = true
	hole.HasAxialDelta, hole.AxialDelta = hasDelta, wallDelta
	fp.Groups = append(fp.Groups, hole)
	fp.DPair = r3.NewVec(2*plateHalf, 2*plateHalf, plateHeight).Len()
	b, err := facetedrecord.Build(t.Context(), axialEnv(), fp)
	require.NoError(t, err)
	built := payloadOf(t, b)
	require.InDelta(t, 2000-45*math.Pi, meshVolume(built), 0.5)
	return b, built
}

// square is the point of the square ring at the angle of ring index k: the ray of cos and sin onto the square,
// with the dominant coordinate exactly +-10 and the four 45 degree points exactly (+-10, +-10).
func square(k, n int, c, s float64) (float64, float64) {
	if k%(n/4) == n/8 {
		return math.Copysign(plateHalf, c), math.Copysign(plateHalf, s)
	}
	if math.Abs(c) > math.Abs(s) {
		return math.Copysign(plateHalf, c), plateHalf * s / math.Abs(c)
	}
	return plateHalf * c / math.Abs(s), math.Copysign(plateHalf, s)
}

// topRimEdge is the edge of the top plane and the hole wall, whatever curve it carries.
func topRimEdge(t *testing.T, b *topo.Body) *topo.Edge {
	t.Helper()
	for _, e := range topobuild.Edges(b) {
		p := e.Start.Position
		if math.Abs(p.Z-plateHeight) > 1e-9 || math.Abs(math.Hypot(p.X, p.Y)-holeRadius) > 1e-9 || len(e.Faces) != 2 {
			continue
		}
		planes, cyls := 0, 0
		for _, f := range e.Faces {
			switch f.Surface.(type) {
			case surfacegeom.Plane:
				planes++
			case surfacegeom.Cylinder:
				cyls++
			}
		}
		if planes == 1 && cyls == 1 {
			return e
		}
	}
	require.FailNow(t, "no top rim edge")
	return nil
}

// requireChargedBlend checks the shared outcome of a fillet or chamfer on a charged wall against its uncharged twin.
func requireChargedBlend(t *testing.T, exact, out *topo.Body, delta float64) {
	t.Helper()
	ef, of := payloadOf(t, exact), payloadOf(t, out)
	requireSameShape(t, ef, of, 1e-9)
	blend := blendGroup(of)
	require.False(t, blend.Retained, "a curved blend of a charged wall is not published")
	require.Nil(t, blend.Surface)
	wall := of.Groups[6]
	require.False(t, wall.Retained)
	require.Nil(t, wall.Surface)
	require.False(t, wall.HasAxialDelta)

	last := len(of.Groups) - 1
	strip := 0
	for i, tri := range of.Tris {
		if of.Src[i] != last {
			continue
		}
		for _, v := range tri {
			strip++
			require.GreaterOrEqual(t, of.VertexBound[v]-ef.VertexBound[v], 0.99*4*delta/1e-3, "strip vertex %d", v)
		}
	}
	require.Positive(t, strip)
	for i, b := range ef.VertexBound {
		require.GreaterOrEqual(t, of.VertexBound[i], b, "vertex %d", i)
	}
	require.GreaterOrEqual(t, out.Volume.Bound.Base(), exact.Volume.Bound.Base())
	// The result's edges are rebuilt from the untagged wall, so none beside it keeps a retained curve.
	beside := 0
	for _, e := range topobuild.Edges(out) {
		for _, f := range e.Faces {
			if f.Surface != nil && f.Surface.Kind() == surfacegeom.KindFaceted && math.Abs(math.Hypot(e.Start.Position.X,
				e.Start.Position.Y)-holeRadius) < 1e-6 {
				require.False(t, e.RetainedCurve, "edge at %v beside the untagged wall", e.Start.Position)
				beside++
				break
			}
		}
	}
	require.Positive(t, beside)
}

func TestFilletChargesCylinderWall(t *testing.T) {
	t.Parallel()
	fillet := func(t *testing.T, wallDelta float64, hasDelta bool) (*topo.Body, bool, error) {
		t.Helper()
		b, built := holedPlate(t, wallDelta, hasDelta)
		return retainedblend.Fillet(t.Context(), axialEnv(), b, built, []*topo.Edge{topRimEdge(t, b)}, 0.5, 0)
	}

	for name, delta := range map[string]float64{"a rounding-sized": 1e-14, "a stated": 1e-6} {
		t.Run(name+" displacement is charged and the tags are not published", func(t *testing.T) {
			t.Parallel()
			exact, recognized, err := fillet(t, 0, false)
			require.True(t, recognized)
			require.NoError(t, err)
			require.NotNil(t, exact)
			ep := payloadOf(t, exact)
			require.True(t, blendGroup(ep).Retained)
			require.IsType(t, surfacegeom.Torus{}, blendGroup(ep).Surface)
			require.True(t, ep.Groups[6].Retained)
			require.IsType(t, surfacegeom.Cylinder{}, ep.Groups[6].Surface)

			out, recognized, err := fillet(t, delta, true)
			require.True(t, recognized)
			require.NoError(t, err)
			require.NotNil(t, out)
			requireChargedBlend(t, exact, out, delta)
			require.GreaterOrEqual(t, out.Area.Bound.Base(), exact.Area.Bound.Base()+0.99*22.44*0.5*delta)
		})
	}

	for name, wallDelta := range map[string]float64{"negative": -1, "infinite": math.Inf(1), "NaN": math.NaN()} {
		t.Run("a "+name+" displacement names no wall", func(t *testing.T) {
			t.Parallel()
			out, recognized, err := fillet(t, wallDelta, true)
			require.False(t, recognized && err == nil, "a displacement no bound can cover was admitted")
			require.Nil(t, out)
		})
	}
}

func TestChamferChargesCylinderWall(t *testing.T) {
	t.Parallel()
	const delta = 1e-14
	chamfer := func(t *testing.T, wallDelta float64, hasDelta bool) *topo.Body {
		t.Helper()
		b, built := holedPlate(t, wallDelta, hasDelta)
		out, recognized, err := retainedblend.Chamfer(t.Context(), axialEnv(), b, built,
			[]*topo.Edge{topRimEdge(t, b)}, 0.5, 0, nil)
		require.True(t, recognized)
		require.NoError(t, err)
		require.NotNil(t, out)
		return out
	}

	exact := chamfer(t, 0, false)
	ep := payloadOf(t, exact)
	require.True(t, blendGroup(ep).Retained)
	require.IsType(t, surfacegeom.Cone{}, blendGroup(ep).Surface)
	require.True(t, ep.Groups[6].Retained)

	out := chamfer(t, delta, true)
	requireChargedBlend(t, exact, out, delta)
	require.GreaterOrEqual(t, out.Area.Bound.Base(), exact.Area.Bound.Base()+0.99*22.44*0.5*delta)
}
