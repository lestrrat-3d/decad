package apitest_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"

	"github.com/lestrrat-3d/decad"
)

// The numbers below are the blank, cones and one tooth of a straight bevel
// gear member (module 4, 32 teeth meshing with 16, 90 degree shaft). The involute
// base circle lies inside the root circle, so Sketch trims each flank at the
// root circle and the tooth root is a fragment of a Circle.
var (
	bevelBlank = [][2]float64{{32, 0}, {48, 0}, {48, 56},
		{36.47213595499959, 61.76393202250021}, {29.542430123549664, 50.028784938225165}, {32, 48.8}}
	bevelToeApex, bevelHeelApex = 129.59999999999997, 159.99999999999997
	bevelConeAngle              = 1.1071487177940904
	bevelRootRadius             = 137.65835055998653
	bevelFlank                  = [][2]float64{{-134.38653173247118, -4.9553190064669028}, {-135.29065544982657, -4.9186846732736802}, {-136.19683200656632, -4.8234350222981286}, {-137.10422558521304, -4.6892870949145484}, {-138.0124734766982, -4.5231199435221106}, {-138.92132258000498, -4.3288328521962331}, {-139.83056811149615, -4.1090077362395432}, {-140.74003162727976, -3.8655053864492861}, {-141.64955084904307, -3.5997416080636357}, {-142.55897421189957, -3.3128353009844367}, {-143.46815764510174, -3.0056959860756556}, {-144.37696253425341, -2.6790792902035374}, {-145.28525436192405, -2.3336240338416192}, {-146.19290176287313, -1.9698780751709302}, {-147.09977584549117, -1.588316935043675}}
)

func bevelDimensioned(s *sketch.Sketch, x, y float64) *sketch.Point {
	p := s.CreatePoint(x, y)
	s.AddConstraint(sketch.NewHorizontalDistance(s.Origin(), p, x), sketch.NewVerticalDistance(s.Origin(), p, y))
	return p
}

// bevelRevolved revolves a closed polygon given as (axial station, radius)
// about the world X axis.
func bevelRevolved(t *testing.T, doc *decad.Document, outline [][2]float64) *decad.Body {
	t.Helper()
	world := sketch.NewWorld()
	s, err := world.CreateSketch(world.XY())
	require.NoError(t, err)
	points := make([]*sketch.Point, len(outline))
	for i, xy := range outline {
		points[i] = bevelDimensioned(s, xy[0], xy[1])
	}
	for i := range points {
		s.CreateLine(points[i], points[(i+1)%len(points)])
	}
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	require.Len(t, s.Profiles(), 1)
	body, err := decad.Revolve(t.Context(), doc, s, s.Profiles()[0],
		decad.SketchLine{Start: decad.Point2{}, End: decad.Point2{U: 1}}, decad.FullRevolution{})
	require.NoError(t, err)
	return body
}

func bevelEndCone(t *testing.T, doc *decad.Document, apex float64) *decad.Body {
	t.Helper()
	far := -0.1 * 71.554175279993274
	return bevelRevolved(t, doc, [][2]float64{{far, 0}, {far, (apex - far) / math.Tan(bevelConeAngle)}, {apex, 0}})
}

// bevelTooth lofts one tooth from the common apex and trims it at the toe and
// heel cones.
func bevelTooth(t *testing.T, doc *decad.Document) *decad.Body {
	t.Helper()
	world := sketch.NewWorld()
	frame, err := r3.NewFrame(r3.NewVec(bevelHeelApex, 0, 0),
		r3.NewVec(math.Sin(bevelConeAngle), -math.Cos(bevelConeAngle), 0), r3.NewVec(0, 0, 1))
	require.NoError(t, err)
	plane, err := world.CreatePlaneFromFrame(frame)
	require.NoError(t, err)
	s, err := world.CreateSketch(plane)
	require.NoError(t, err)
	n := len(bevelFlank)
	left, right := make([]*sketch.Point, n), make([]*sketch.Point, n)
	for i, xy := range bevelFlank {
		if i == n-1 {
			// The tip arc's radius relation fixes the radial coordinate.
			left[i] = s.CreatePoint(xy[0], xy[1])
			s.AddConstraint(sketch.NewVerticalDistance(s.Origin(), left[i], xy[1]))
		} else {
			left[i] = bevelDimensioned(s, xy[0], xy[1])
		}
		right[i] = bevelDimensioned(s, xy[0], -xy[1])
	}
	_, err = s.CreateFitSpline(right...)
	require.NoError(t, err)
	s.CreateArc(s.Origin(), right[n-1], left[n-1])
	_, err = s.CreateFitSpline(left...)
	require.NoError(t, err)
	root := s.CreateCircle(s.Origin(), bevelRootRadius)
	s.AddConstraint(sketch.NewRadius(root, bevelRootRadius))
	_, err = s.Solve(t.Context())
	require.NoError(t, err)

	var profile *sketch.Profile
	for _, candidate := range s.Profiles() {
		if !candidate.Valid || len(candidate.Outer) != 4 {
			continue
		}
		trimmed := 0
		for _, edge := range candidate.Outer {
			if _, ok := edge.Entity.(*sketch.FitSpline); ok && edge.Partial && edge.TExact {
				trimmed++
			}
		}
		if trimmed == 2 {
			profile = candidate
		}
	}
	require.NotNil(t, profile, "no tooth region with two exactly trimmed flanks")
	tooth, err := decad.LoftFromPoint(t.Context(), doc, r3.Vec{}, s, profile)
	require.NoError(t, err)
	toeKept, err := decad.Cut(t.Context(), tooth, bevelEndCone(t, doc, bevelToeApex))
	require.NoError(t, err)
	trimmed, err := decad.Intersect(t.Context(), toeKept, bevelEndCone(t, doc, bevelHeelApex))
	require.NoError(t, err)
	return trimmed
}

// TestUnionBevelBlankAndEmbeddedRootTeeth joins a blank with three copies of
// a tooth whose root is a Circle fragment between two partial fit-spline
// flanks. The Union must verify, and the three teeth must add three times the
// gain of one tooth.
func TestUnionBevelBlankAndEmbeddedRootTeeth(t *testing.T) {
	if testing.Short() {
		t.Skip("joins three sectors")
	}
	const teeth = 3
	doc := decad.New()
	blank := bevelRevolved(t, doc, bevelBlank)
	tooth := bevelTooth(t, doc)
	pattern := make([]*decad.Body, teeth)
	pattern[0] = tooth
	for i := 1; i < teeth; i++ {
		turn, err := r3.Rotation(r3.Vec{X: 1}, units.Radians(2*math.Pi*float64(i)/teeth))
		require.NoError(t, err)
		pattern[i], err = decad.Copy(t.Context(), tooth, decad.WithCopyPlacement(turn))
		require.NoError(t, err)
	}
	// One tooth on an independent blank gives the per-tooth gain.
	singleDoc := decad.New()
	single, err := decad.Union(t.Context(), bevelRevolved(t, singleDoc, bevelBlank), bevelTooth(t, singleDoc))
	require.NoError(t, err)
	blankVolume, err := blank.Volume()
	require.NoError(t, err)
	singleVolume, err := single.Volume()
	require.NoError(t, err)

	joined, err := decad.Union(t.Context(), blank, pattern...)
	require.NoError(t, err)
	mesh, err := joined.Tessellate(t.Context(), units.Millimeters(0.1),
		decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())

	volume, err := joined.Volume()
	require.NoError(t, err)
	gain := singleVolume.Value.Base() - blankVolume.Value.Base()
	require.Greater(t, gain, 0.0)
	slack := volume.Bound.Base() + blankVolume.Bound.Base() +
		teeth*(singleVolume.Bound.Base()+blankVolume.Bound.Base())
	require.InDelta(t, teeth*gain, volume.Value.Base()-blankVolume.Value.Base(), slack)
}
