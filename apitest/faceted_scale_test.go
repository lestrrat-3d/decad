package apitest_test

import (
	"fmt"
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// requireScaledFacetedFaces checks that a scaled loft keeps every source role,
// keeps its two caps as planes and reads every side as Faceted.
func requireScaledFacetedFaces(t *testing.T, body *decad.Body, sides int) map[string]decad.Plane {
	t.Helper()
	caps := make(map[string]decad.Plane)
	faceted := 0
	for _, face := range body.Faces() {
		require.Len(t, face.Origins(), 1)
		role := face.Origins()[0].Role
		switch surface := face.Surface().(type) {
		case decad.Plane:
			caps[role] = surface
		case decad.Faceted:
			faceted++
			require.Contains(t, role, "side(0,")
		default:
			t.Fatalf("face %s reads %v after the scale", role, face.Surface().Kind())
		}
	}
	require.Equal(t, sides, faceted)
	require.Len(t, caps, 2)
	require.Contains(t, caps, roleCapStart)
	require.Contains(t, caps, roleCapEnd)
	return caps
}

// requireVerifiedScaledMesh restates the held mesh at chord mm, which must be
// at least the scaled body's mesh bound.
func requireVerifiedScaledMesh(t *testing.T, doc *decad.Document, body *decad.Body, chord float64) *decad.Mesh {
	t.Helper()
	mesh, err := body.Tessellate(t.Context(), units.Millimeters(chord), decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
	report, err := doc.Verify(t.Context())
	require.NoError(t, err)
	require.NotEqual(t, decad.Unsound, report.Status)
	return mesh
}

func TestScaledAtUnmarkedFacetedSolid(t *testing.T) {
	t.Parallel()

	t.Run("FourRectangleLoftSections", func(t *testing.T) {
		t.Parallel()
		sections := fourRectangleSections(t, [4]float64{2, 3, 4, 5},
			[4]float64{3, 4, 5, 7}, [4]float64{0, 4, 8, 12})
		doc := decad.New()
		body, err := decad.LoftSections(t.Context(), doc, sections[:]...)
		require.NoError(t, err)
		// The smooth walls read NURBS before the scale and Faceted after it.
		nurbs := 0
		for _, face := range body.Faces() {
			if face.Surface().Kind() == decad.KindNURBS {
				nurbs++
			}
		}
		require.Equal(t, 4, nurbs)
		anchor := r3.NewVec(1, -2, 3)
		scaled, err := decad.ScaledAt(t.Context(), body, anchor, units.Scalar(10))
		require.NoError(t, err)
		require.Equal(t, []*decad.Body{scaled}, doc.Bodies())

		// Cubics through the four stations are w=2+3t and h=3+4t-4.5t²+4.5t³
		// over a 12 mm rise, so the source holds 48·∫(w·h)dt = 819.6 mm³.
		// The image about any anchor holds 10³ times that. The held source
		// misses 819.6 by about a quarter of its bound, so a bound grown by
		// 10 or 10² would not cover the image's 10³-scaled miss.
		volume, err := scaled.Volume()
		require.NoError(t, err)
		require.False(t, math.IsInf(volume.Bound.Base(), 0) || math.IsNaN(volume.Bound.Base()))
		require.LessOrEqual(t, math.Abs(volume.Value.Base()-1000*819.6), volume.Bound.Base())

		caps := requireScaledFacetedFaces(t, scaled, 4)
		// The source caps lie at z=0 and z=12; anchor.z + 10(z − anchor.z)
		// puts them at z=−27 and z=93.
		require.InDelta(t, -27, caps[roleCapStart].Frame.Origin().Z, 1e-12)
		require.InDelta(t, 93, caps[roleCapEnd].Frame.Origin().Z, 1e-12)
		require.InDelta(t, 1, math.Abs(caps[roleCapEnd].Frame.N().Z), 1e-12)

		mesh := requireVerifiedScaledMesh(t, doc, scaled, 1)
		// The source holds its middle station corner at (3.5, 4.4375, 6).
		// The image anchor + 10(p − anchor) is exactly (26, 62.375, 33).
		middleCorner := false
		for _, vertex := range mesh.Vertices() {
			middleCorner = middleCorner || vertex == r3.NewVec(26, 62.375, 33)
		}
		require.True(t, middleCorner, "the held mesh must carry the scaled middle corner")
	})

	t.Run("VaryingCircleLoftSectionsAfterPlacementAndRepeatedScale", func(t *testing.T) {
		t.Parallel()
		sections := threeArcCircleSections(t, [3]float64{8, 10, 12}, [3]float64{0, 5, 10})
		doc := decad.New()
		body, err := decad.LoftSections(t.Context(), doc, sections[:]...)
		require.NoError(t, err)
		move, err := r3.Translation(r3.NewVec(4, 0, -2))
		require.NoError(t, err)
		placed, err := decad.Transform(t.Context(), body, move)
		require.NoError(t, err)
		first, err := decad.ScaledAt(t.Context(), placed, r3.NewVec(0, 1, 0), units.Scalar(3))
		require.NoError(t, err)
		scaled, err := decad.ScaledAt(t.Context(), first, r3.NewVec(-2, 0, 5), units.Scalar(0.5))
		require.NoError(t, err)
		require.Equal(t, []*decad.Body{scaled}, doc.Bodies())

		// A linearly changing radius from 8 to 12 over 10 mm holds
		// (3040/3)π mm³. Scales by 3 and then 0.5 multiply it by 3.375.
		volume, err := scaled.Volume()
		require.NoError(t, err)
		piLo, ok := new(big.Rat).SetString("3.14159265358979323846")
		require.True(t, ok)
		piHi, ok := new(big.Rat).SetString("3.14159265358979323847")
		require.True(t, ok)
		coefficient := new(big.Rat).Mul(big.NewRat(3040, 3), big.NewRat(27, 8))
		reading := new(big.Rat).SetFloat64(volume.Value.Base())
		bound := new(big.Rat).SetFloat64(volume.Bound.Base())
		require.LessOrEqual(t, new(big.Rat).Sub(reading, bound).Cmp(new(big.Rat).Mul(coefficient, piLo)), 0)
		require.GreaterOrEqual(t, new(big.Rat).Add(reading, bound).Cmp(new(big.Rat).Mul(coefficient, piHi)), 0)

		requireScaledFacetedFaces(t, scaled, 4)
		requireVerifiedScaledMesh(t, doc, scaled, 0.1)
	})

	t.Run("RetainedTangentChainFillet", func(t *testing.T) {
		t.Parallel()
		drilled := crossDrilled(t)
		filleted, err := decad.Fillet(t.Context(), drilled, topFront(), units.Millimeters(1), decad.WithTangentChain())
		require.NoError(t, err)
		scaled, err := decad.ScaledAt(t.Context(), filleted, r3.NewVec(1, 2, 3), units.Scalar(2))
		require.NoError(t, err)
		require.Len(t, scaled.Document().Bodies(), 1)

		// The 40×20×20 box loses four r=3 vertical corners, the Ø8 bore along
		// y through its 20 mm depth, and the r=1 fillet of its top outline. The
		// fillet removes the corner area 1−π/4 swept along the outline at its
		// centroid, which sits (10−3π)/(3(4−π)) from the corner: the 96 mm of
		// straight run plus the four r=3 arcs at radius 3 minus that offset.
		// The image by 2 holds 2³ times that.
		offset := (10 - 3*math.Pi) / (3 * (4 - math.Pi))
		fillet := (1 - math.Pi/4) * (96 + 2*math.Pi*(3-offset))
		source := 40.0*20*20 - 4*20*9*(1-math.Pi/4) - math.Pi*16*20 - fillet
		volume, err := scaled.Volume()
		require.NoError(t, err)
		require.LessOrEqual(t, math.Abs(volume.Value.Base()-8*source), volume.Bound.Base())

		// Every retained cylinder and torus keeps its kind at twice its radii:
		// the bore (4), the four box corners (3), the four outline fillets (1)
		// and the four corner tori (major 3−1, tube 1).
		cylinders := map[float64]int{}
		tori := 0
		for _, face := range scaled.Faces() {
			switch surface := face.Surface().(type) {
			case decad.Cylinder:
				cylinders[surface.Radius.Base()]++
			case decad.Torus:
				require.InDelta(t, 4, surface.Major.Base(), 1e-12)
				require.InDelta(t, 2, surface.Minor.Base(), 1e-12)
				tori++
			}
		}
		require.Equal(t, map[float64]int{8: 1, 6: 4, 2: 4}, cylinders)
		require.Equal(t, 4, tori)
		requireVerifiedScaledMesh(t, scaled.Document(), scaled, 0.1)
	})

	t.Run("PointLoftPyramid", func(t *testing.T) {
		t.Parallel()
		s, profile := pointLoftSquare(t, sketch.NewWorld(), 6)
		doc := decad.New()
		body, err := decad.LoftFromPoint(t.Context(), doc, r3.Vec{}, s, profile)
		require.NoError(t, err)
		scaled, err := decad.ScaledAt(t.Context(), body, r3.Vec{}, units.Scalar(3))
		require.NoError(t, err)
		require.Equal(t, []*decad.Body{scaled}, doc.Bodies())

		// A 4×4 base 6 mm from the apex holds 16·6/3 = 32 mm³; the image
		// by 3 holds 27 times that.
		volume, err := scaled.Volume()
		require.NoError(t, err)
		require.LessOrEqual(t, math.Abs(volume.Value.Base()-27*32), volume.Bound.Base())

		apex := false
		for _, v := range scaled.Vertices() {
			apex = apex || v.Position().Value == r3.Vec{}
		}
		require.True(t, apex, "the apex is the anchor and stays put")
		roles := make(map[string]bool)
		for _, face := range scaled.Faces() {
			for _, origin := range face.Origins() {
				roles[origin.Role] = true
			}
		}
		require.True(t, roles[roleCapEnd], "missing the far cap role")
		for i := range 4 {
			role := fmt.Sprintf("side(0,%d)", i)
			require.True(t, roles[role], "missing source role %s", role)
		}
		requireVerifiedScaledMesh(t, doc, scaled, 0.1)
	})
}
