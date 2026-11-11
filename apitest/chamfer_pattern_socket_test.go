package apitest_test

import (
	"math"
	"strings"
	"testing"

	"github.com/lestrrat-3d/decad"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

const (
	socketRadius = 4.0
	socketDepth  = 1.0
)

// socketTool is a through tool at (plateOrbit, 0) from z=-1 to z=4: a round section of socketRadius, or, when d is set,
// a D section whose flat lies socketDepth inside the rim on the +X side.
func socketTool(t *testing.T, doc *decad.Document, world *sketch.World, d bool) *decad.Body {
	t.Helper()
	plane, err := world.CreateOffsetPlane(world.XY(), -1)
	require.NoError(t, err)
	s, err := world.CreateSketch(plane)
	require.NoError(t, err)
	c := s.CreatePoint(plateOrbit, 0)
	s.Fix(c)
	if !d {
		s.CreateCircle(c, socketRadius)
	} else {
		flat := socketRadius - socketDepth
		half := math.Sqrt(socketRadius*socketRadius - flat*flat)
		upper := s.CreatePoint(plateOrbit+flat, half)
		lower := s.CreatePoint(plateOrbit+flat, -half)
		s.Fix(upper)
		s.Fix(lower)
		s.CreateArc(c, upper, lower)
		s.CreateLine(lower, upper)
	}
	require.NoError(t, solveSketch(t, s))
	require.Len(t, s.Profiles(), 1)
	b, err := decad.Extrude(t.Context(), doc, s, s.Profiles()[0], decad.Distance{D: units.Millimeters(5), Dir: decad.Along})
	require.NoError(t, err)
	return b
}

// socketArea is the section area of socketTool.
func socketArea(d bool) float64 {
	if !d {
		return math.Pi * socketRadius * socketRadius
	}
	flat := socketRadius - socketDepth
	half := math.Sqrt(socketRadius*socketRadius - flat*flat)
	segment := socketRadius*socketRadius*math.Acos(flat/socketRadius) - flat*half
	return math.Pi*socketRadius*socketRadius - segment
}

// heightsLoopEdges selects every edge of the loops that pass keep on the planar faces at the heights zs, matching edges
// by start point and kind as planeLoopEdges does, all in one query.
func heightsLoopEdges(t *testing.T, b *decad.Body, keep func(*decad.Loop, []*decad.Edge) bool, zs ...float64) (*decad.EdgeQuery, int) {
	t.Helper()
	var q *decad.EdgeQuery
	count := 0
	for _, f := range b.Faces() {
		pl, ok := f.Surface().(decad.Plane)
		if !ok || math.Abs(math.Abs(pl.Frame.N().Z)-1) > 1e-9 {
			continue
		}
		atHeight := false
		for _, z := range zs {
			atHeight = atHeight || math.Abs(pl.Frame.Origin().Z-z) <= 1e-6
		}
		if !atHeight {
			continue
		}
		for _, loop := range f.Loops() {
			edges := loop.Edges()
			if !keep(loop, edges) {
				continue
			}
			for _, e := range edges {
				start := e.Start().Position().Value
				var pred []decad.EdgePredicate
				switch e.Curve().(type) {
				case decad.Circle3, decad.Arc3:
					pred = []decad.EdgePredicate{decad.EndpointAt(start), decad.Circular()}
				case decad.Line3:
					pred = []decad.EdgePredicate{decad.EndpointAt(start), decad.ParallelTo(e.End().Position().Value.Sub(start))}
				default:
					t.Fatalf("unexpected curve %T", e.Curve())
				}
				if q == nil {
					q = decad.Edges(pred...)
				} else {
					q = q.Or(pred...)
				}
				count++
			}
		}
	}
	require.NotNil(t, q)
	return q, count
}

// TestChamferPlateAfterPatternSocketCut chamfers both outlines of a plate that three pattern copies of one through
// socket were cut from, the copies joined first or cut one at a time. The rotated copies carry the rounding of their
// rotation as a section displacement, and so does the plate they are cut from; the plate's own outline keeps exact
// carriers, so its loops chamfer, while a loop a rotated socket left keeps the refusal.
func TestChamferPlateAfterPatternSocketCut(t *testing.T) {
	t.Parallel()
	for _, tc := range []struct {
		name    string
		d, join bool
	}{
		{"D joined", true, true},
		{"D one at a time", true, false},
		{"round joined", false, true},
		{"round one at a time", false, false},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			doc, world := decad.New(), sketch.NewWorld()
			plate := roundedTrianglePlate(t, doc, world, plateOrbit, plateRadius)
			socket := socketTool(t, doc, world, tc.d)
			copies, err := decad.PatternCopies(t.Context(), socket, decad.CircularPattern{
				Center: r3.NewVec(0, 0, 0), Axis: r3.NewVec(0, 0, 1), Count: 3,
			})
			require.NoError(t, err)
			require.Len(t, copies, 2)
			carrier := plate
			if tc.join {
				tool, err := decad.Union(t.Context(), socket, copies...)
				require.NoError(t, err)
				carrier, err = decad.Cut(t.Context(), plate, tool)
				require.NoError(t, err)
			} else {
				for _, tool := range append([]*decad.Body{socket}, copies...) {
					carrier, err = decad.Cut(t.Context(), carrier, tool)
					require.NoError(t, err)
				}
			}
			require.True(t, carrier.IsSolid())
			plateVolume, err := plate.Volume()
			require.NoError(t, err)
			cutVolume, err := carrier.Volume()
			require.NoError(t, err)
			require.InEpsilon(t, plateVolume.Value.Base()-3*3*socketArea(tc.d), cutVolume.Value.Base(), 1e-9)

			// A loop that a rotated socket left on the cap is displaced, so its chamfer still refuses.
			rotated := func(l *decad.Loop, edges []*decad.Edge) bool {
				return !l.IsOuter() && edges[0].Start().Position().Value.X < 0
			}
			q, n := heightsLoopEdges(t, carrier, rotated, 0)
			require.Positive(t, n)
			_, err = decad.Chamfer(t.Context(), carrier, q.Exactly(n), units.Millimeters(0.5))
			require.ErrorIs(t, err, decad.ErrUnsupported)

			// Both outlines in one chamfer: six edges on each cap.
			outer := func(l *decad.Loop, _ []*decad.Edge) bool { return l.IsOuter() }
			q, n = heightsLoopEdges(t, carrier, outer, 0, 3)
			require.Equal(t, 12, n)
			faces, edges := len(carrier.Faces()), len(carrier.Edges())
			out, err := decad.Chamfer(t.Context(), carrier, q.Exactly(n), units.Millimeters(0.5))
			require.NoError(t, err)
			requireChamferRemoved(t, carrier, out, 2*roundedPlateOutline())

			// Each chamfered edge becomes one chamfer face, a plane on a line and a cone on an arc, which adds two
			// edges: one on the cap and one on the wall.
			planes, cones := 0, 0
			for _, f := range out.Faces() {
				origins := f.Origins()
				if len(origins) == 0 || !strings.HasPrefix(origins[0].Role, "chamfer(") {
					continue
				}
				switch f.Surface().(type) {
				case decad.Plane:
					planes++
				case decad.Cone:
					cones++
				default:
					t.Fatalf("chamfer face %s on %T", origins[0].Role, f.Surface())
				}
			}
			require.Equal(t, 6, planes)
			require.Equal(t, 6, cones)
			require.Len(t, out.Faces(), faces+12)
			require.Len(t, out.Edges(), edges+24)
		})
	}
}
