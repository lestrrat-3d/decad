package retainedblend

import (
	"math"
	"testing"

	clr "github.com/lestrrat-3d/decad/internal/clearance"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func chargedCylinder(radius, delta float64) *topo.Face {
	return &topo.Face{
		Surface: surfacegeom.Cylinder{Origin: r3.NewVec(3, 3, 0), Axis: r3.NewVec(0, 0, 1),
			Radius: units.Millimeters(radius)},
		RetainedTag: true, HasAxialDelta: true, AxialDelta: delta,
	}
}

func chargedPlane(t *testing.T, o, u, w r3.Vec, delta float64) *topo.Face {
	t.Helper()
	f, err := r3.NewFrame(o, u, w)
	require.NoError(t, err)
	face := &topo.Face{Surface: surfacegeom.Plane{Frame: f}, RetainedTag: true}
	if delta != 0 {
		face.HasAxialDelta, face.AxialDelta = true, delta
	}
	return face
}

func TestChargedTagReaders(t *testing.T) {
	t.Parallel()

	t.Run("a charged cylinder is no record to the plane rule but a wall to the rim rule", func(t *testing.T) {
		t.Parallel()
		f := chargedCylinder(3, 1e-14)
		require.False(t, tagIsRecord(f))
		require.False(t, TagIsRecord(f))
		require.True(t, wallRecord(f))
		require.True(t, chargedCurve(f))
	})

	t.Run("an uncharged cylinder is both", func(t *testing.T) {
		t.Parallel()
		f := chargedCylinder(3, 0)
		require.True(t, tagIsRecord(f))
		require.True(t, wallRecord(f))
		require.False(t, chargedCurve(f))
		f.HasAxialDelta = false
		require.True(t, tagIsRecord(f))
		require.True(t, wallRecord(f))
	})

	t.Run("a plane with a displacement stays a plane record and no wall", func(t *testing.T) {
		t.Parallel()
		f := chargedPlane(t, r3.Vec{}, r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0), 1e-14)
		require.True(t, tagIsRecord(f))
		require.False(t, wallRecord(f))
		require.False(t, chargedCurve(f))
	})

	t.Run("a displacement no bound can cover is no wall", func(t *testing.T) {
		t.Parallel()
		for _, delta := range []float64{-1, math.NaN(), math.Inf(1)} {
			f := chargedCylinder(3, delta)
			require.False(t, wallRecord(f), "delta %v", delta)
			require.False(t, tagIsRecord(f), "delta %v", delta)
		}
	})

	t.Run("a rim reads a charged wall and refuses a charged arc", func(t *testing.T) {
		t.Parallel()
		plane := chargedPlane(t, r3.NewVec(0, 0, 5), r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0), 0)
		edge := &topo.Edge{Curve: surfacegeom.Circle3{}, RetainedCurve: true, Faces: []*topo.Face{plane, chargedCylinder(3, 1e-14)}}
		require.True(t, planeAndWall(edge))
		edge.Faces[1].TagDeparture = 1e-3
		require.False(t, planeAndWall(edge))
		edge.Faces[1].TagDeparture = 0
		edge.Faces[1].AxialDelta = -1
		require.False(t, planeAndWall(edge))
	})

	t.Run("a cylinder line and a cylinder pair refuse a charged cylinder", func(t *testing.T) {
		t.Parallel()
		plane := chargedPlane(t, r3.NewVec(0, 0, 5), r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0), 0)
		a, b := &topo.Vertex{Position: r3.Vec{}}, &topo.Vertex{Position: r3.NewVec(0, 0, 1)}
		line := &topo.Edge{Curve: surfacegeom.Line3{}, RetainedCurve: true, Start: a, End: b,
			Faces: []*topo.Face{plane, chargedCylinder(3, 0)}}
		require.True(t, looksLikeCylLine(line), "the uncharged twin reads")
		line.Faces[1] = chargedCylinder(3, 1e-14)
		require.False(t, looksLikeCylLine(line))

		pair := &topo.Edge{Curve: surfacegeom.Line3{}, RetainedCurve: true, Start: a, End: b,
			Faces: []*topo.Face{chargedCylinder(3, 0), chargedCylinder(4, 0)}}
		require.True(t, looksLikeCylCyl(pair), "the uncharged twin reads")
		pair.Faces[0] = chargedCylinder(3, 1e-14)
		require.False(t, looksLikeCylCyl(pair))
	})

	t.Run("an unread edge names a charged curved tag as a departure", func(t *testing.T) {
		t.Parallel()
		require.True(t, tagFaceOf(chargedCylinder(3, 1e-14)).departs)
		require.False(t, tagFaceOf(chargedCylinder(3, 0)).departs)
	})
}

func TestChargedArcAndAreaCharge(t *testing.T) {
	t.Parallel()

	t.Run("refuseChargedArc", func(t *testing.T) {
		t.Parallel()
		require.NoError(t, refuseChargedArc(rim{}))
		err := refuseChargedArc(rim{wallDelta: 1e-14})
		require.ErrorIs(t, err, decaderr.ErrUnsupported)
		require.ErrorContains(t, err, "an arc on a wall whose tag states a displacement is not built")
	})

	t.Run("rimAreaCharge", func(t *testing.T) {
		t.Parallel()
		spec := blendSpec{fillet: true, r: 0.5}
		require.Zero(t, rimAreaCharge(rim{}, spec))
		require.GreaterOrEqual(t, rimAreaCharge(rim{wallDelta: 1e-14}, spec), 22.44*0.5*1e-14)
		chamfer := blendSpec{dA: 0.5, dB: 2}
		require.GreaterOrEqual(t, rimAreaCharge(rim{wallDelta: 1e-14}, chamfer), 22.44*2*1e-14)
	})

	t.Run("cylCharge", func(t *testing.T) {
		t.Parallel()
		require.Zero(t, cylCharge(0))
		require.GreaterOrEqual(t, cylCharge(1e-14), planeAmp*1e-14)
	})
}

// chargedJunction is the top-front line and the top corner arc of a rounded box, with a corner cylinder of the given
// radius that states a displacement.
func chargedJunction(t *testing.T, radius, delta float64) (line, arc *topo.Edge, v *topo.Vertex) {
	t.Helper()
	top := chargedPlane(t, r3.NewVec(0, 0, 20), r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0), 0)
	front := chargedPlane(t, r3.Vec{}, r3.NewVec(1, 0, 0), r3.NewVec(0, 0, 1), 0)
	cyl := chargedCylinder(radius, delta)
	v = &topo.Vertex{Position: r3.NewVec(3, 0, 20), Bound: units.Millimeters(1e-9)}
	far := &topo.Vertex{Position: r3.NewVec(37, 0, 20)}
	side := &topo.Vertex{Position: r3.NewVec(0, 3, 20)}
	line = &topo.Edge{Curve: surfacegeom.Line3{}, RetainedCurve: true, Start: far, End: v,
		Faces: []*topo.Face{top, front}}
	arc = &topo.Edge{Curve: surfacegeom.Arc3{Center: r3.NewVec(3, 3, 20), Axis: r3.NewVec(0, 0, 1),
		Radius: units.Millimeters(3)}, RetainedCurve: true, Start: side, End: v, Faces: []*topo.Face{top, cyl}}
	return line, arc, v
}

func TestRefuseChargedJunctions(t *testing.T) {
	t.Parallel()
	selection := func(edges ...*topo.Edge) map[*topo.Edge]struct{} {
		m := map[*topo.Edge]struct{}{}
		for _, e := range edges {
			m[e] = struct{}{}
		}
		return m
	}

	t.Run("the strict readers never read a charged wall", func(t *testing.T) {
		t.Parallel()
		line, arc, v := chargedJunction(t, 3, 1e-14)
		require.True(t, Chainable(line))
		require.False(t, Chainable(arc))
		require.True(t, chainableMode(arc, true))
		require.NotEqual(t, clr.DegYes, Tangent(line, arc, v))
		_, state, _ := tangentStateMode(line, arc, v, true)
		require.Equal(t, clr.DegYes, state, "the charged reading proves the junction")
	})

	t.Run("an uncharged twin is chainable", func(t *testing.T) {
		t.Parallel()
		line, arc, v := chargedJunction(t, 3, 0)
		require.True(t, Chainable(arc))
		require.Equal(t, clr.DegYes, Tangent(line, arc, v))
		all := []*topo.Edge{line, arc}
		require.NoError(t, refuseChargedJunctions(all, []*topo.Edge{arc}, selection(arc)))
	})

	t.Run("the arc alone is refused with its continuation", func(t *testing.T) {
		t.Parallel()
		line, arc, _ := chargedJunction(t, 3, 1e-14)
		err := refuseChargedJunctions([]*topo.Edge{line, arc}, []*topo.Edge{arc}, selection(arc))
		require.ErrorIs(t, err, decaderr.ErrUnsupported)
		require.ErrorContains(t, err, "whose continuation is not selected")
		require.ErrorContains(t, err, "(read on tags that state a displacement)")
	})

	t.Run("the arc and the line are refused as a chain", func(t *testing.T) {
		t.Parallel()
		line, arc, _ := chargedJunction(t, 3, 1e-14)
		err := refuseChargedJunctions([]*topo.Edge{line, arc}, []*topo.Edge{arc, line}, selection(arc, line))
		require.ErrorIs(t, err, decaderr.ErrUnsupported)
		require.ErrorContains(t, err, "a tangent chain whose walls state a displacement is not built")
	})

	t.Run("a near tangent charged junction is refused as undecided", func(t *testing.T) {
		t.Parallel()
		line, arc, _ := chargedJunction(t, 3*(1+1e-12), 1e-14)
		err := refuseChargedJunctions([]*topo.Edge{line, arc}, []*topo.Edge{arc}, selection(arc))
		require.ErrorIs(t, err, decaderr.ErrUnsupported)
		require.ErrorContains(t, err, "is nearly tangent and the tags cannot decide it")
		require.ErrorContains(t, err, "(read on tags that state a displacement)")
	})

	t.Run("a cylinder far from tangent falls through", func(t *testing.T) {
		t.Parallel()
		line, arc, _ := chargedJunction(t, 4, 1e-14)
		require.NoError(t, refuseChargedJunctions([]*topo.Edge{line, arc}, []*topo.Edge{arc}, selection(arc)))
	})

	t.Run("an unusable displacement falls through to the strict refusal", func(t *testing.T) {
		t.Parallel()
		line, arc, _ := chargedJunction(t, 3, math.NaN())
		require.False(t, chainableMode(arc, true))
		require.NoError(t, refuseChargedJunctions([]*topo.Edge{line, arc}, []*topo.Edge{arc}, selection(arc)))
	})
}
