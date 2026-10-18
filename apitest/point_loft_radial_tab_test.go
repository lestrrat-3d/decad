package apitest_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/stretchr/testify/require"
)

// pointLoftRadialTab is a rounded tab with fitted sides. Its point-Loft fan
// gives Split, LoftFaces, and similarity tests one real Sketch source.
func pointLoftRadialTab(t *testing.T, doc *decad.Document) *decad.Body {
	t.Helper()
	angle := math.Pi / 4
	frame, err := r3.NewFrame(r3.NewVec(8, 0, 0),
		r3.NewVec(math.Sin(angle), -math.Cos(angle), 0), r3.NewVec(0, 0, 1))
	require.NoError(t, err)
	world := sketch.NewWorld()
	plane, err := world.CreatePlaneFromFrame(frame)
	require.NoError(t, err)
	s, err := world.CreateSketch(plane)
	require.NoError(t, err)
	fixed := func(x, y float64) *sketch.Point {
		p := s.CreatePoint(x, y)
		s.Fix(p)
		return p
	}
	center := fixed(0, 0)
	rightRoot, leftRoot := fixed(-4.3, -0.35), fixed(-4.3, 0.35)
	rightStart, leftStart := fixed(-4.6, -0.45), fixed(-4.6, 0.45)
	rightMid, leftMid := fixed(-5.4, -0.451), fixed(-5.4, 0.451)
	rightEnd, leftEnd := fixed(-6.3, -0.45), fixed(-6.3, 0.45)
	s.CreateLine(rightRoot, rightStart)
	_, err = s.CreateFitSpline(rightStart, rightMid, rightEnd)
	require.NoError(t, err)
	s.CreateArc(center, leftEnd, rightEnd)
	_, err = s.CreateFitSpline(leftEnd, leftMid, leftStart)
	require.NoError(t, err)
	s.CreateLine(leftStart, leftRoot)
	s.CreateArc(center, leftRoot, rightRoot)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	require.Len(t, s.Profiles(), 1)
	body, err := decad.LoftFromPoint(t.Context(), doc, r3.Vec{}, s, s.Profiles()[0])
	require.NoError(t, err)
	return body
}
