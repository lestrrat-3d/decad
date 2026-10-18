package decad

import (
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestLoftContactTwoLineCapsRealTrapezoid524(t *testing.T) {
	world := sketch.NewWorld()
	upper, err := world.CreateOffsetPlane(world.XY(), 4)
	require.NoError(t, err)
	var sections [2]*sketch.Sketch
	var profiles [2]*sketch.Profile
	for cap, corners := range [2][4][2]float64{
		{{8, 4}, {12, 4}, {13, 6}, {8, 6}},
		{{8, 4}, {13, 4}, {14, 6}, {8, 6}},
	} {
		plane := world.XY()
		if cap == 1 {
			plane = upper
		}
		s, createErr := world.CreateSketch(plane)
		require.NoError(t, createErr)
		var points [4]*sketch.Point
		for i, xy := range corners {
			points[i] = s.CreatePoint(xy[0], xy[1])
			s.Fix(points[i])
		}
		for i := range points {
			s.CreateLine(points[i], points[(i+1)%len(points)])
		}
		_, solveErr := s.Solve(t.Context())
		require.NoError(t, solveErr)
		found := s.Profiles()
		require.Equal(t, 1, len(found))
		require.True(t, found[0].Valid)
		sections[cap], profiles[cap] = s, found[0]
	}

	doc := New()
	prism := internalBoxBody(t, doc, 0, 0, 10, 10, 5)
	loft, err := Loft(t.Context(), doc, sections[0], profiles[0], sections[1], profiles[1])
	require.NoError(t, err)
	operation := proofbound.NewWorkBudget(t.Context())
	boundary, admitted, err := certifyLoftOutputBoundary(
		t.Context(), prism, loft, operation)
	require.NoError(t, err)
	require.True(t, admitted)
	require.Greater(t, len(boundary.shell.vertices), 0)
	require.Greater(t, len(boundary.shell.triangles), 0)

	source := boundary.source
	require.Greater(t, source.sourceVisits, uint64(0))
	require.Less(t, source.sourceVisits, uint64(4096))
	require.Equal(t, [3]string{"0", "4", "5"}, [3]string{
		source.capLevels[0].RatString(), source.capLevels[1].RatString(),
		source.capLevels[2].RatString(),
	})
	require.Equal(t, [2][3]int{{2, 1, 1}, {0, 1, 0}}, source.capOwners)
	require.Equal(t, [2][2]sectionrecord.Point2{
		{{U: 10, V: 4}, {U: 10, V: 6}},
		{{U: 10, V: 4}, {U: 10, V: 6}},
	}, source.junction)
	for cap, prepared := range source.prepared {
		require.NotNil(t, prepared.scene)
		require.Equal(t, 8, len(prepared.tags))
		require.Equal(t, 8, len(prepared.scene.Entities()))
		require.Equal(t, 3, len(prepared.raw))
		junctions := map[[2]float64]struct{}{}
		for _, cell := range prepared.raw {
			require.True(t, prepared.scene == cell.Sketch())
			require.False(t, cell.IsStale())
			require.True(t, cell.Valid)
			require.Zero(t, len(cell.Holes))
			for i, left := range cell.Outer {
				right := cell.Outer[(i+1)%len(cell.Outer)]
				a, okA := prepared.tags[left.Entity]
				b, okB := prepared.tags[right.Entity]
				if !okA || !okB || a.IsB == b.IsB {
					continue
				}
				point, certified := prepared.scene.CertifyLineLineJunction(left, right)
				if certified {
					junctions[point] = struct{}{}
				}
			}
		}
		require.Equal(t, 2, len(junctions), "cap %d junctions", cap)
		_, hasLower := junctions[[2]float64{10, 4}]
		_, hasUpper := junctions[[2]float64{10, 6}]
		require.True(t, hasLower, "cap %d lower junction", cap)
		require.True(t, hasUpper, "cap %d upper junction", cap)
	}

	pp := prism.n.Payload.(prismPayload)
	lp := loft.n.Payload.(loftPayload)
	proof, admitted, err := loftVolumeCompose(t.Context(), operation, pp, lp, boundary)
	require.NoError(t, err)
	require.True(t, admitted)
	require.Zero(t, proof.sourceVolume.Cmp(big.NewRat(524, 1)))
	require.False(t, math.IsNaN(proof.symDiffUpper))
	require.False(t, math.IsInf(proof.symDiffUpper, 0))
	require.GreaterOrEqual(t, proof.symDiffUpper, float64(0))
	require.Less(t, proof.symDiffUpper, float64(524))
	require.Greater(t, boundary.workVisits, uint64(0))
	require.Greater(t, proof.totalOutputVisits, boundary.workVisits)
	require.LessOrEqual(t, proof.totalOutputVisits, uint64(65_536))
	t.Logf("source visits=%d output visits=%d facets=%d", source.sourceVisits,
		proof.totalOutputVisits, len(boundary.shell.triangles))
}

func TestLoftContactTwoLineCapsNarrowTranslated524(t *testing.T) {
	world := sketch.NewWorld()
	epsilon := math.Ldexp(1, -52)
	originY := 1 + epsilon
	frame, err := r3.NewFrame(r3.Vec{Y: originY}, r3.Vec{X: 1}, r3.Vec{Y: 1})
	require.NoError(t, err)
	base, err := world.CreatePlaneFromFrame(frame)
	require.NoError(t, err)
	upper, err := world.CreateOffsetPlane(base, 1)
	require.NoError(t, err)
	var sections [2]*sketch.Sketch
	var profiles [2]*sketch.Profile
	for cap, corners := range [2][4][2]float64{
		{{119, 0}, {163 - 896*epsilon, 0}, {163 - 896*epsilon, 1}, {119, 1}},
		{{119, 0}, {165 - 1024*epsilon, 0}, {165 - 1024*epsilon, 1}, {119, 1}},
	} {
		plane := base
		if cap == 1 {
			plane = upper
		}
		s, createErr := world.CreateSketch(plane)
		require.NoError(t, createErr)
		var points [4]*sketch.Point
		for i, xy := range corners {
			points[i] = s.CreatePoint(xy[0], xy[1])
			s.Fix(points[i])
		}
		for i := range points {
			s.CreateLine(points[i], points[(i+1)%len(points)])
		}
		_, solveErr := s.Solve(t.Context())
		require.NoError(t, solveErr)
		found := s.Profiles()
		require.Equal(t, 1, len(found))
		require.True(t, found[0].Valid)
		sections[cap], profiles[cap] = s, found[0]
	}
	doc := New()
	prismSketch, err := world.CreateSketch(base)
	require.NoError(t, err)
	rect := prismSketch.CreateRectangle(0, -epsilon, 120, 1+epsilon)
	prismSketch.Fix(rect.A)
	_, err = prismSketch.Solve(t.Context())
	require.NoError(t, err)
	prismProfiles := prismSketch.Profiles()
	require.Equal(t, 1, len(prismProfiles))
	require.True(t, prismProfiles[0].Valid)
	prism, err := Extrude(t.Context(), doc, prismSketch, prismProfiles[0],
		Distance{D: units.Millimeters(4), Dir: Along})
	require.NoError(t, err)
	loft, err := Loft(t.Context(), doc, sections[0], profiles[0], sections[1], profiles[1])
	require.NoError(t, err)
	pp, lp := prism.n.Payload.(prismPayload), loft.n.Payload.(loftPayload)
	require.True(t, lp.AuthenticatedReconstruction)
	require.NotNil(t, lp.SourceIdentity)
	require.Equal(t, pp.Frame, lp.Frame0)
	wantWorldY := new(big.Rat).Add(big.NewRat(2, 1), big.NewRat(1, 1<<52))
	for _, tc := range []struct {
		name          string
		first, second *Body
	}{{loftOperandPrismFirst, prism, loft}, {loftOperandLoftFirst, loft, prism}} {
		t.Run(tc.name, func(t *testing.T) {
			operation := proofbound.NewWorkBudget(t.Context())
			boundary, admitted, boundaryErr := certifyLoftOutputBoundary(
				t.Context(), tc.first, tc.second, operation)
			require.NoError(t, boundaryErr, "real boundary")
			require.True(t, admitted, "real boundary")
			require.Less(t, boundary.source.sourceVisits, uint64(4096))
			t.Logf("source visits=%d boundary output visits=%d",
				boundary.source.sourceVisits, boundary.workVisits)
			proof, admitted, composeErr := loftVolumeCompose(t.Context(), operation, pp, lp, boundary)
			require.NoError(t, composeErr, "real composer")
			require.True(t, admitted, "real composer")
			require.Zero(t, proof.sourceVolume.Cmp(big.NewRat(524, 1)))
			displaced := false
			for _, vertex := range proof.boundary.shell.vertices {
				if vertex.point.u.Cmp(big.NewRat(120, 1)) != 0 ||
					vertex.point.v.Cmp(big.NewRat(1, 1)) != 0 {
					continue
				}
				require.Equal(t, wantWorldY.RatString(), vertex.world[1].RatString())
				require.Equal(t, float64(2), vertex.held.Y)
				require.Positive(t, vertex.bound)
				displaced = true
			}
			require.True(t, displaced)
			require.False(t, math.IsNaN(proof.symDiffUpper))
			require.False(t, math.IsInf(proof.symDiffUpper, 0))
			require.Greater(t, proof.symDiffUpper, float64(0))
			require.Less(t, proof.symDiffUpper, float64(524))
			require.LessOrEqual(t, proof.totalOutputVisits, uint64(loftVolumeOutputSteps))
			t.Logf("source visits=%d output visits=%d facets=%d bound=%g",
				boundary.source.sourceVisits, proof.totalOutputVisits,
				len(boundary.shell.triangles), proof.symDiffUpper)
		})
	}
}
