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

// A tube of outer radius 10 and inner radius 7, 40 mm tall, takes twisted
// rectangular tools (half sides 4 and 1.5, 6 mm long, twisting 60 degrees) set
// 8.5 mm from its axis, so each tool crosses the wall obliquely. A tool's cut
// leaves a rim bound on the wall facets it crosses, and each wall facet runs
// the whole tube height.
const (
	spreadTubeOuter  = 10.0
	spreadTubeInner  = 7.0
	spreadTubeHeight = 40.0
	spreadToolHalfX  = 4.0
	spreadToolHalfY  = 1.5
	spreadToolSpan   = 6.0
	spreadToolTwist  = 60.0
	spreadToolOffset = 8.5
)

func spreadTool(t *testing.T, doc *decad.Document, z float64) *decad.Body {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	r := s.CreateRectangle(-spreadToolHalfX, -spreadToolHalfY, spreadToolHalfX, spreadToolHalfY)
	s.Fix(r.A)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	path, err := decad.NewPath(r3.Vec{}, decad.LineTo{End: r3.NewVec(0, 0, spreadToolSpan)})
	require.NoError(t, err)
	tool, err := decad.Sweep(t.Context(), doc, s, s.Profiles()[0], path,
		decad.WithSweepTwist(units.Degrees(spreadToolTwist)))
	require.NoError(t, err)
	move, err := r3.Translation(r3.NewVec(spreadToolOffset, 0, z))
	require.NoError(t, err)
	placed, err := decad.Transform(t.Context(), tool, move)
	require.NoError(t, err)
	return placed
}

// spreadOverlap integrates, independently of the evaluator, the volume the
// tool at height z shares with the tube wall, on a midpoint grid.
func spreadOverlap(z float64) float64 {
	const slices, nu, nv = 120, 640, 240
	du, dv := 2*spreadToolHalfX/nu, 2*spreadToolHalfY/nv
	dz := spreadToolSpan / slices
	total := 0.0
	for k := range slices {
		a := spreadToolTwist * math.Pi / 180 * (float64(k) + 0.5) / slices
		sin, cos := math.Sincos(a)
		count := 0
		for i := range nu {
			u := -spreadToolHalfX + (float64(i)+0.5)*du
			for j := range nv {
				v := -spreadToolHalfY + (float64(j)+0.5)*dv
				x, y := spreadToolOffset+u*cos-v*sin, u*sin+v*cos
				if r2 := x*x + y*y; r2 >= spreadTubeInner*spreadTubeInner && r2 <= spreadTubeOuter*spreadTubeOuter {
					count++
				}
			}
		}
		total += float64(count) * du * dv * dz
	}
	_ = z
	return total
}

func spreadCut(t *testing.T, doc *decad.Document, body *decad.Body, z float64) (*decad.Body, error) {
	t.Helper()
	return decad.Cut(t.Context(), body, spreadTool(t, doc, z))
}

func spreadBound(t *testing.T, body *decad.Body) float64 {
	t.Helper()
	mesh, err := body.Tessellate(t.Context(), units.Millimeters(0.3), decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
	return mesh.Bound().Base()
}

// A second cut far along the tube reads most of the wall facets the first cut
// left with their own bound, not the first cut's rim bound, so its rim stays
// near the first cut's figure. A far piece is split off only where the whole
// piece is proven to stand clear of the tool, so a few long facets beside the
// chain stay whole and the second cut's bound is not quite the first's: it stays
// under 1.7 times the first cut's, where the document without WithRimSplit
// reaches over 2 times. The volume still encloses the independent integral.
func TestChainedCutsFarApartKeepOneRimBound(t *testing.T) {
	t.Parallel()
	doc := decad.New(decad.WithChainTolerance(units.Millimeters(0.2)), decad.WithRimSplit())
	tube := twistBoreTube(t, doc, spreadTubeOuter, spreadTubeInner, spreadTubeHeight)
	tubeVolume, err := tube.Volume()
	require.NoError(t, err)

	first, err := spreadCut(t, doc, tube, 8)
	require.NoError(t, err)
	boundFirst := spreadBound(t, first)
	second, err := spreadCut(t, doc, first, 28)
	require.NoError(t, err)
	boundSecond := spreadBound(t, second)
	require.LessOrEqual(t, boundSecond, 1.7*boundFirst, "first %v second %v", boundFirst, boundSecond)

	volume, err := second.Volume()
	require.NoError(t, err)
	want := tubeVolume.Value.Base() - 2*spreadOverlap(0)
	require.LessOrEqual(t, math.Abs(volume.Value.Base()-want),
		volume.Bound.Base()+tubeVolume.Bound.Base()+0.005*2*spreadOverlap(0), "volume %v want %v", volume, want)
}

// A second cut over the first cut's rim still reads the rim's bound and
// amplifies it: only the far part of a facet is read at its own bound.
func TestChainedCutsOverTheRimStillReadItsBound(t *testing.T) {
	t.Parallel()
	doc := decad.New(decad.WithChainTolerance(units.Millimeters(0.5)), decad.WithRimSplit())
	tube := twistBoreTube(t, doc, spreadTubeOuter, spreadTubeInner, spreadTubeHeight)
	first, err := spreadCut(t, doc, tube, 8)
	require.NoError(t, err)
	boundFirst := spreadBound(t, first)
	second, err := spreadCut(t, doc, first, 9)
	require.NoError(t, err)
	require.Greater(t, spreadBound(t, second), 1.2*boundFirst)
}

// A document that does not set WithRimSplit keeps its rims in one piece: the
// second cut reads the first cut's rim bound along the whole wall and
// amplifies it, as it always has.
func TestChainedCutsWithoutRimSplitKeepTheSpread(t *testing.T) {
	t.Parallel()
	doc := decad.New(decad.WithChainTolerance(units.Millimeters(0.2)))
	tube := twistBoreTube(t, doc, spreadTubeOuter, spreadTubeInner, spreadTubeHeight)
	first, err := spreadCut(t, doc, tube, 8)
	require.NoError(t, err)
	boundFirst := spreadBound(t, first)
	second, err := spreadCut(t, doc, first, 28)
	require.NoError(t, err)
	require.Greater(t, spreadBound(t, second), 1.5*boundFirst)
}
