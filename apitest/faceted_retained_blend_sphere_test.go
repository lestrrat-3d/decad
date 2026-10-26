package apitest_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/decadtest"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// The fixtures of agent-docs/retained-edge-blend-design.md §4.5 (R6): a
// Revolve ball (a half-disk meridian about the x axis) cut by a box that a
// Transform placed at z >= ballCut, which keeps the pair on the mesh path. The
// cut leaves the spherical cap below z = ballCut, height R + ballCut, whose rim
// is a closed circle between the retained plane and the retained sphere. The
// two meet at 60°.

const (
	ballR, ballCut = 2.0, -1.0
)

// revolvedBall revolves the half-disk of radius r centred at (cu, cv) about
// the x axis.
func revolvedBall(t *testing.T, sc *retentionScene, cu, cv, r float64) *decad.Body {
	t.Helper()
	s, err := sc.w.CreateSketch(sc.w.XY())
	require.NoError(t, err)
	o := s.CreatePoint(cu-r, cv)
	s.Fix(o)
	end := s.CreatePoint(cu+r, cv)
	c := s.CreatePoint(cu, cv)
	s.CreateLine(o, end)
	s.CreateArc(c, end, o)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	body, err := decad.Revolve(t.Context(), sc.doc, s, s.Profiles()[0], uAxis, decad.FullRevolution{})
	require.NoError(t, err)
	return body
}

// ballCap is the box z in [h, h + 20] that the Cut removes. Its 200 mm width
// sets the mesh Boolean's chord tolerance coarse enough for the ball's mesh to
// stay inside the facet-pair audit's budget.
func ballCap(t *testing.T, sc *retentionScene, h float64) *decad.Body {
	t.Helper()
	s, err := sc.w.CreateSketch(sc.w.XY())
	require.NoError(t, err)
	r := s.CreateRectangle(-100, -100, 100, 100)
	s.Fix(r.A)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	box, err := decad.Extrude(t.Context(), sc.doc, s, s.Profiles()[0],
		decad.Distance{D: units.Millimeters(20), Dir: decad.Along})
	require.NoError(t, err)
	move, err := r3.Translation(r3.Vec{Z: h})
	require.NoError(t, err)
	placed, err := decad.Transform(t.Context(), box, move)
	require.NoError(t, err)
	return placed
}

// flatBall is the ball of radius ballR about (cu, cv, 0) with all of it above
// z = ballCut removed.
func flatBall(t *testing.T, cu, cv float64) *decad.Body {
	t.Helper()
	sc := newRetentionScene()
	cut, err := decad.Cut(t.Context(), revolvedBall(t, sc, cu, cv, ballR), ballCap(t, sc, ballCut))
	require.NoError(t, err)
	return cut
}

// discRim selects the one circular edge of b.
func discRim(t *testing.T, b *decad.Body) decad.EdgeSelector {
	t.Helper()
	for _, e := range b.Edges() {
		if _, ok := e.Curve().(decad.Circle3); ok {
			return decad.Edges(decad.EndpointAt(e.Start().Position().Value)).Exactly(1)
		}
	}
	require.Fail(t, "no circular rim")
	return nil
}

// ballZone is the volume of the ball between a cut plane and the parallel
// plane zB inside it, z measured from the cut plane along its outward normal:
// π∫(R² − (z + h)²) dz over [zB, 0], with the ball's centre at z = −h. The top
// cut of flatBall has h = ballCut.
func ballZone(h, zB float64) float64 {
	return math.Pi * (-h*h*h/3 - ballR*ballR*zB + (zB+h)*(zB+h)*(zB+h)/3)
}

// filletCut is what a fillet of radius r removes at the rim of a cut plane
// whose outward normal sees the ball's centre at z = −h, and the blend's
// numbers, with z measured from the cut plane. The ball of radius r touches
// the cut plane from below, so its centre sits at z = −r, and touches
// the sphere from inside, R − r from its centre at z = −h:
// ρc = √((R − r)² − (h − r)²). The tangent point on the sphere lies on the line
// from the sphere's centre through the ball's. Under the blend the solid's
// radius at height z is ρc + √(r² − (z + r)²), so the removed volume is the
// zone less the solid of revolution under the torus.
func filletCut(h, r float64) (removed, rhoC, zB, torusArea float64) {
	R := ballR
	rhoC = math.Sqrt((R-r)*(R-r) - (h-r)*(h-r))
	zB = -h + (h-r)*R/(R-r)
	prim := func(x float64) float64 { return (x*math.Sqrt(r*r-x*x) + r*r*math.Asin(x/r)) / 2 }
	x0, x1 := zB+r, r
	under := math.Pi * ((rhoC*rhoC+r*r)*(x1-x0) - (x1*x1*x1-x0*x0*x0)/3 + 2*rhoC*(prim(x1)-prim(x0)))
	removed = ballZone(h, zB) - under
	// The arc turns from straight up at the plane to the direction (ρc, h − r)
	// at the sphere; ρ = ρc + r·sin α along it.
	alpha := math.Atan2(rhoC, h-r)
	torusArea = 2 * math.Pi * r * (rhoC*alpha + r*(1-math.Cos(alpha)))
	return removed, rhoC, zB, torusArea
}

// chamferCut is what a chamfer removes whose setback runs dA in along the
// plane from the rim and dB down the sphere's meridian: the bevel is the
// frustum between the two setback circles.
func chamferCut(dA, dB float64) (removed, rhoA, rhoB, zB float64) {
	h, R := ballCut, ballR
	rim := math.Sqrt(R*R - h*h)
	theta := math.Atan2(h, rim) - dB/R
	rhoA, rhoB, zB = rim-dA, R*math.Cos(theta), -h+R*math.Sin(theta)
	frustum := math.Pi * -zB * (rhoA*rhoA + rhoA*rhoB + rhoB*rhoB) / 3
	return ballZone(h, zB) - frustum, rhoA, rhoB, zB
}

func TestFacetedRetainedBlendSphereRim(t *testing.T) {
	t.Parallel()
	rim := math.Sqrt(ballR*ballR - ballCut*ballCut)
	// The cap of height c = R + ballCut: volume πc²(3R − c)/3, and area the
	// disc πρ0² plus the sphere's zone 2πR·c (Archimedes).
	c := ballR + ballCut
	capVolume, capArea := math.Pi*c*c*(3*ballR-c)/3, math.Pi*rim*rim+2*math.Pi*ballR*c

	t.Run("fillet", func(t *testing.T) {
		t.Parallel()
		const r = 0.4
		cut := flatBall(t, 0, 0)
		before, err := cut.Volume()
		require.NoError(t, err)
		blended, err := decad.Fillet(t.Context(), cut, discRim(t, cut), units.Millimeters(r))
		require.NoError(t, err)
		after, err := blended.Volume()
		require.NoError(t, err)
		removed, rhoC, zB, torusArea := filletCut(ballCut, r)
		requireVolumeChange(t, before, after, removed)
		decadtest.MeasuresVolume(t, blended, units.CubicMillimeters(capVolume-removed))
		// The torus replaces the annulus out from ρc and the sphere's zone of
		// height |zB| (Archimedes: 2πR·|zB|).
		decadtest.MeasuresArea(t, blended,
			units.SquareMillimeters(capArea+torusArea-math.Pi*(rim*rim-rhoC*rhoC)+2*math.Pi*ballR*zB))

		kinds := map[decad.SurfaceKind]int{}
		for _, f := range blended.Faces() {
			kinds[f.Surface().Kind()]++
			tor, ok := f.Surface().(decad.Torus)
			if !ok {
				continue
			}
			require.InDelta(t, rhoC, tor.Major.Base(), 1e-12)
			require.InDelta(t, r, tor.Minor.Base(), 1e-12)
			require.InDelta(t, ballCut-r, tor.Center.Z, 1e-12)
			require.InDelta(t, 1, math.Abs(tor.Axis.Z), 1e-12)
			n, err := f.NormalAt(firstVertex(t, f))
			require.NoError(t, err)
			require.Less(t, n.Bound.Base(), 1.0)
		}
		require.Equal(t, map[decad.SurfaceKind]int{decad.KindPlane: 1, decad.KindSphere: 1, decad.KindTorus: 1}, kinds)
		mesh, err := blended.Tessellate(t.Context(), units.Millimeters(0.1), decad.WithVerification(decad.VerifyAll))
		require.NoError(t, err)
		require.True(t, mesh.BoundaryVerified())
		require.True(t, mesh.VolumeVerified())
	})

	t.Run("chamfer", func(t *testing.T) {
		t.Parallel()
		const d = 0.5
		cut := flatBall(t, 0, 0)
		before, err := cut.Volume()
		require.NoError(t, err)
		blended, err := decad.Chamfer(t.Context(), cut, discRim(t, cut), units.Millimeters(d))
		require.NoError(t, err)
		after, err := blended.Volume()
		require.NoError(t, err)
		removed, rhoA, rhoB, zB := chamferCut(d, d)
		requireVolumeChange(t, before, after, removed)
		decadtest.MeasuresVolume(t, blended, units.CubicMillimeters(capVolume-removed))
		// The frustum's side replaces the annulus out from ρA and the sphere's
		// zone of height |zB|.
		decadtest.MeasuresArea(t, blended, units.SquareMillimeters(capArea+
			math.Pi*(rhoA+rhoB)*math.Hypot(rhoB-rhoA, zB)-math.Pi*(rim*rim-rhoA*rhoA)+2*math.Pi*ballR*zB))

		kinds := map[decad.SurfaceKind]int{}
		for _, f := range blended.Faces() {
			kinds[f.Surface().Kind()]++
			c, ok := f.Surface().(decad.Cone)
			if !ok {
				continue
			}
			// The bevel widens from the plane's setback circle down to the
			// sphere's.
			require.InDelta(t, math.Atan((rhoB-rhoA)/-zB), c.HalfAngle.Base(), 1e-12)
			require.InDelta(t, rhoA, c.Radius.Base(), 1e-12)
			require.InDelta(t, ballCut, c.Origin.Z, 1e-12)
		}
		require.Equal(t, map[decad.SurfaceKind]int{decad.KindPlane: 1, decad.KindSphere: 1, decad.KindCone: 1}, kinds)
		mesh, err := blended.Tessellate(t.Context(), units.Millimeters(0.1), decad.WithVerification(decad.VerifyAll))
		require.NoError(t, err)
		require.True(t, mesh.BoundaryVerified())
		require.True(t, mesh.VolumeVerified())
	})

	t.Run("two-distance chamfer", func(t *testing.T) {
		t.Parallel()
		// The plane, the reference, takes 0.3 mm and the sphere 0.6 mm.
		const dA, dB = 0.3, 0.6
		cut := flatBall(t, 0, 0)
		before, err := cut.Volume()
		require.NoError(t, err)
		blended, err := decad.Chamfer(t.Context(), cut, discRim(t, cut), units.Millimeters(dA),
			decad.WithAsymmetricChamfer(decad.Faces(decad.Facing(r3.NewVec(0, 0, 1))), units.Millimeters(dB)))
		require.NoError(t, err)
		after, err := blended.Volume()
		require.NoError(t, err)
		removed, rhoA, rhoB, zB := chamferCut(dA, dB)
		requireVolumeChange(t, before, after, removed)
		decadtest.MeasuresVolume(t, blended, units.CubicMillimeters(capVolume-removed))
		// Here the bevel narrows from the plane's setback circle down to the
		// sphere's, so the cone starts at the sphere's circle.
		require.Less(t, rhoB, rhoA)
		cones := 0
		for _, f := range blended.Faces() {
			if c, ok := f.Surface().(decad.Cone); ok {
				cones++
				require.InDelta(t, math.Atan((rhoA-rhoB)/-zB), c.HalfAngle.Base(), 1e-12)
				require.InDelta(t, rhoB, c.Radius.Base(), 1e-12)
				require.InDelta(t, ballCut+zB, c.Origin.Z, 1e-12)
			}
		}
		require.Equal(t, 1, cones)
	})

	t.Run("both rims of a slice", func(t *testing.T) {
		t.Parallel()
		// A second placed Cut removes z <= low: the slice keeps the sphere's
		// zone between z = low and z = ballCut, and the call blends one rim,
		// then finds the other again on the partly blended body. The bottom
		// plane's outward normal is -z, so it sees the centre at −low.
		const r, low = 0.2, -1.5
		sc := newRetentionScene()
		top, err := decad.Cut(t.Context(), revolvedBall(t, sc, 0, 0, ballR), ballCap(t, sc, ballCut))
		require.NoError(t, err)
		slice, err := decad.Cut(t.Context(), top, ballCap(t, sc, low-20))
		require.NoError(t, err)
		before, err := slice.Volume()
		require.NoError(t, err)
		blended, err := decad.Fillet(t.Context(), slice, decad.Edges(decad.Circular()).Exactly(2), units.Millimeters(r))
		require.NoError(t, err)
		after, err := blended.Volume()
		require.NoError(t, err)
		removedTop, _, _, _ := filletCut(ballCut, r)
		removedBottom, _, _, _ := filletCut(-low, r)
		requireVolumeChange(t, before, after, removedTop+removedBottom)
		zone := func(z float64) float64 { return ballR*ballR*z - z*z*z/3 }
		decadtest.MeasuresVolume(t, blended,
			units.CubicMillimeters(math.Pi*(zone(ballCut)-zone(low))-removedTop-removedBottom))
		tori := 0
		for _, f := range blended.Faces() {
			if _, ok := f.Surface().(decad.Torus); ok {
				tori++
			}
		}
		require.Equal(t, 2, tori)
		mesh, err := blended.Tessellate(t.Context(), units.Millimeters(0.1), decad.WithVerification(decad.VerifyAll))
		require.NoError(t, err)
		require.True(t, mesh.VolumeVerified())
	})

	t.Run("a fillet too large for the disc", func(t *testing.T) {
		t.Parallel()
		// The ball must clear the plane and stay inside the sphere:
		// R − r > r − h, so r < (R + h)/2 = 0.5.
		cut := flatBall(t, 0, 0)
		_, err := decad.Fillet(t.Context(), cut, discRim(t, cut), units.Millimeters(0.6))
		require.ErrorIs(t, err, decad.ErrDegenerate)
		require.ErrorContains(t, err, "no ball of that radius touches both faces")
	})

	t.Run("a chamfer past the pole", func(t *testing.T) {
		t.Parallel()
		cut := flatBall(t, 0, 0)
		// The arc from the rim down the meridian to the pole is
		// (π/2 + atan2(h, ρ0))·R; the plane takes 0.2 mm and the sphere 0.1 mm
		// past the pole.
		pole := ballR * (math.Pi/2 + math.Atan2(ballCut, rim))
		_, err := decad.Chamfer(t.Context(), cut, discRim(t, cut), units.Millimeters(0.2),
			decad.WithAsymmetricChamfer(decad.Faces(decad.Facing(r3.NewVec(0, 0, 1))), units.Millimeters(pole+0.1)))
		require.ErrorIs(t, err, decad.ErrDegenerate)
		require.ErrorContains(t, err, "the blend reaches past the sphere's pole")
	})

	t.Run("a plane setback past the rim's axis", func(t *testing.T) {
		t.Parallel()
		cut := flatBall(t, 0, 0)
		_, err := decad.Chamfer(t.Context(), cut, discRim(t, cut), units.Millimeters(rim+0.1),
			decad.WithAsymmetricChamfer(decad.Faces(decad.Facing(r3.NewVec(0, 0, 1))), units.Millimeters(0.2)))
		require.ErrorIs(t, err, decad.ErrDegenerate)
		require.ErrorContains(t, err, "the blend reaches the rim's axis")
	})

	t.Run("a rotated revolve refuses", func(t *testing.T) {
		t.Parallel()
		// A rotated placement makes the record's axis frame a held float, not
		// an exact frame.
		sc := newRetentionScene()
		turned, err := decad.Transform(t.Context(), revolvedBall(t, sc, 0, 0, ballR),
			rotation(t, r3.Vec{}, r3.NewVec(0, 0, 1), 0.3))
		require.NoError(t, err)
		cut, err := decad.Cut(t.Context(), turned, ballCap(t, sc, ballCut))
		require.NoError(t, err)
		_, err = decad.Fillet(t.Context(), cut, discRim(t, cut), units.Millimeters(0.3))
		require.ErrorIs(t, err, decad.ErrUnsupported)
		require.ErrorContains(t, err, "the sphere's revolve record is not exact")
	})

	t.Run("a snapped centre refuses", func(t *testing.T) {
		t.Parallel()
		// The half-disk's centre sits 5e-10 off the axis: the chord snaps onto
		// it and the tag's centre is on the axis, but the record's is not.
		cut := flatBall(t, 0, 5e-10)
		_, err := decad.Fillet(t.Context(), cut, discRim(t, cut), units.Millimeters(0.3))
		require.ErrorIs(t, err, decad.ErrUnsupported)
		require.ErrorContains(t, err, "the sphere's tag centre is not exactly its recorded centre")
	})

	t.Run("a rounded radius refuses", func(t *testing.T) {
		t.Parallel()
		// About x = 0.1 the recorded radius 2.1 − 0.1 is no float, so the record
		// holds an interval and the tag's 2 is not provably it.
		cut := flatBall(t, 0.1, 0)
		_, err := decad.Chamfer(t.Context(), cut, discRim(t, cut), units.Millimeters(0.3))
		require.ErrorIs(t, err, decad.ErrUnsupported)
		require.ErrorContains(t, err, "the sphere's tag radius is not exactly its recorded radius")
	})
}
