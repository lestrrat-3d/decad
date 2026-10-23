package decad

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/stretchr/testify/require"

	"github.com/lestrrat-3d/decad/internal/facetedrecord"
)

// TestPointLoftSourceStaysCurrentOnATiltedPlane builds a point loft on planes
// whose axes are not exact in binary and requires its recorded source to read as
// current. NewFrame is not idempotent, so a source frame recorded by one NewFrame
// differs in the last bit from the sketch plane's own frame.
func TestPointLoftSourceStaysCurrentOnATiltedPlane(t *testing.T) {
	// The cone angles of an 8-tooth pair at shaft angles of 60 and 90 degrees, and an
	// 11-tooth by 7-tooth pair at 75 degrees.
	for _, shaft := range []float64{math.Pi / 3, math.Pi / 2, 5 * math.Pi / 12} {
		a := math.Atan2(math.Sin(shaft)*4, 4+math.Cos(shaft)*4)
		deg := a * 180 / math.Pi
		u := r3.NewVec(math.Sin(a), -math.Cos(a), 0)
		frame, err := r3.NewFrame(r3.NewVec(8*math.Cos(a), 0, 0), u, r3.NewVec(0, 0, 1))
		require.NoError(t, err)

		world := sketch.NewWorld()
		plane, err := world.CreatePlaneFromFrame(frame)
		require.NoError(t, err)
		s, err := world.CreateSketch(plane)
		require.NoError(t, err)
		points := []*sketch.Point{
			s.CreatePoint(-1, -1), s.CreatePoint(-1.5, 0), s.CreatePoint(-1, 1),
			s.CreatePoint(1, 1), s.CreatePoint(1, -1),
		}
		for _, point := range points {
			s.Fix(point)
		}
		_, err = s.CreateFitSpline(points[0], points[1], points[2])
		require.NoError(t, err)
		for i := 2; i < len(points); i++ {
			s.CreateLine(points[i], points[(i+1)%len(points)])
		}
		_, err = s.Solve(t.Context())
		require.NoError(t, err)

		doc := New()
		body, err := LoftFromPoint(t.Context(), doc, r3.Vec{}, s, s.Profiles()[0])
		require.NoError(t, err, "%g deg", deg)
		fp, ok := body.n.Payload.(facetedPayload)
		require.True(t, ok)
		require.NotNil(t, fp.PointSection)
		require.True(t, facetedrecord.PointSourceCurrent(fp.PointSection), "%g deg", deg)
	}
}
