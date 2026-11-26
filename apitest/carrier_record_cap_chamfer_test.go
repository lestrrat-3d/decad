package apitest_test

import (
	"fmt"
	"math"
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/decadtest"
)

// This file is agent-docs/prism-boolean-design.md Part K §10's PR-2 public-API tests:
// the wheel-like part of carrier_record_test.go (two windows, a tooth and two
// rotated copies, the six roots filleted, the D bore cut), then a Chamfer of
// every edge of both windows' cap loops on both caps. The windows are exact
// loops of a receiver whose outer loop carries the teeth's displacement, so
// the chamfer builds a displaced cap-loop payload with analytic faces.

const surveyWall = "wall"

const (
	capChamferSize    = 0.4
	capChamferWindows = 2
	capChamferTeeth   = 3
)

// requireAnalyticCurves asserts every edge of body is a line, arc or circle,
// whatever the payload class.
func requireAnalyticCurves(t *testing.T, body *decad.Body) {
	t.Helper()
	for _, e := range body.Edges() {
		switch e.Curve().(type) {
		case decad.Line3, decad.Arc3, decad.Circle3:
		default:
			t.Fatalf("edge curve %T is not an analytic line, arc or circle", e.Curve())
		}
	}
}

// windowRimQuery names every edge of every window loop (inner loops of more
// than four edges, which leaves the two-edge D bore alone) on both cap faces.
func windowRimQuery(t *testing.T, body *decad.Body, z0, z1 float64) *decad.EdgeQuery {
	t.Helper()
	var query *decad.EdgeQuery
	count := 0
	for _, f := range body.Faces() {
		plane, ok := f.Surface().(decad.Plane)
		if !ok {
			continue
		}
		origin, n := plane.Frame.Origin(), plane.Frame.N()
		if math.Abs(math.Abs(n.Z)-1) > 1e-9 || (math.Abs(origin.Z-z0) > 1e-9 && math.Abs(origin.Z-z1) > 1e-9) {
			continue
		}
		for _, loop := range f.Loops() {
			edges := loop.Edges()
			if loop.IsOuter() || len(edges) <= 4 {
				continue
			}
			for _, e := range edges {
				start := e.Start().Position().Value
				var preds []decad.EdgePredicate
				switch e.Curve().(type) {
				case decad.Arc3, decad.Circle3:
					preds = []decad.EdgePredicate{decad.EndpointAt(start), decad.Circular()}
				case decad.Line3:
					preds = []decad.EdgePredicate{decad.EndpointAt(start), decad.ParallelTo(e.End().Position().Value.Sub(start))}
				default:
					t.Fatalf("cannot select a %T edge", e.Curve())
				}
				if query == nil {
					query = decad.Edges(preds...)
				} else {
					query = query.Or(preds...)
				}
				count++
			}
		}
	}
	require.Positive(t, count)
	return query.Exactly(count)
}

// wheelFilletedAndBored is the part both tests chamfer: the displaced
// receiver is built from the disc, tooth and rotated copies.
func wheelFilletedAndBored(t *testing.T) *decad.Body {
	t.Helper()
	wheel := wheelUnion(t, capChamferWindows, capChamferTeeth)
	filleted, err := decad.Fillet(t.Context(), wheel, rootEdges(capChamferTeeth), units.Millimeters(wheelRoot))
	require.NoError(t, err)
	doc := filleted.Document()
	w := sketch.NewWorld()
	plane, err := w.CreateOffsetPlane(w.XY(), -0.75)
	require.NoError(t, err)
	s, err := w.CreateSketch(plane)
	require.NoError(t, err)
	wheelD(s)
	tool := wheelExtrude(t, doc, s, 0, wheelHeight+1.5)
	bored, err := decad.Cut(filleted, tool).Do(t.Context())
	require.NoError(t, err)
	requireAnalyticEdges(t, bored)
	return bored
}

func TestDisplacedCapChamferWheelWindows(t *testing.T) {
	t.Parallel()
	bored := wheelFilletedAndBored(t)
	query := windowRimQuery(t, bored, 0, wheelHeight)
	chamfered, err := decad.Chamfer(t.Context(), bored, query, units.Millimeters(capChamferSize))
	require.NoError(t, err)
	require.Equal(t, "decad.displacedCapBlendPayload", decadtest.PayloadClass(chamfered))
	requireAnalyticCurves(t, chamfered)
	require.True(t, chamfered.IsSolid())

	drawn := wheelInPlace(t, capChamferWindows, capChamferTeeth, true, true)
	drawnChamfered, err := decad.Chamfer(t.Context(), drawn, windowRimQuery(t, drawn, 0, wheelHeight), units.Millimeters(capChamferSize))
	require.NoError(t, err)
	require.Equal(t, "decad.capBlendPayload", decadtest.PayloadClass(drawnChamfered))

	got, err := chamfered.Volume()
	require.NoError(t, err)
	want, err := drawnChamfered.Volume()
	require.NoError(t, err)
	require.InDelta(t, want.Value.Base(), got.Value.Base(), got.Bound.Base()+want.Bound.Base())
	require.Greater(t, got.Bound.Base(), want.Bound.Base(), "the outer loop's displacement is charged")
	before, err := bored.Volume()
	require.NoError(t, err)
	require.Less(t, got.Value.Base(), before.Value.Base(), "a chamfer removes material")

	mesh, err := chamfered.Tessellate(t.Context(), units.Millimeters(0.05), decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
}

// chamferedWheel is the displaced cap-loop chamfer both reader tests read.
func chamferedWheel(t *testing.T) *decad.Body {
	t.Helper()
	bored := wheelFilletedAndBored(t)
	chamfered, err := decad.Chamfer(t.Context(), bored, windowRimQuery(t, bored, 0, wheelHeight), units.Millimeters(capChamferSize))
	require.NoError(t, err)
	require.Equal(t, "decad.displacedCapBlendPayload", decadtest.PayloadClass(chamfered))
	return chamfered
}

// TestDisplacedCapChamferMeshCutByBoxComposes cuts a box through one tooth: the
// result is a mesh Boolean whose displaced walls state their loop bound, and it
// verifies.
func TestDisplacedCapChamferMeshCutByBoxComposes(t *testing.T) {
	t.Parallel()
	chamfered := chamferedWheel(t)
	w := sketch.NewWorld()
	plane, err := w.CreateOffsetPlane(w.XY(), -1)
	require.NoError(t, err)
	s, err := w.CreateSketch(plane)
	require.NoError(t, err)
	fixedRect(s, 12, -1, 20, 1)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	box, err := decad.Extrude(t.Context(), chamfered.Document(), s, s.Profiles()[0], decad.Distance{D: units.Millimeters(6), Dir: decad.Along})
	require.NoError(t, err)
	cut, err := decad.Cut(chamfered, box).Do(t.Context())
	require.NoError(t, err)
	before, err := chamfered.Volume()
	require.NoError(t, err)
	after, err := cut.Volume()
	require.NoError(t, err)
	require.Less(t, after.Value.Base(), before.Value.Base())
	mesh, err := cut.Tessellate(t.Context(), units.Millimeters(0.05), decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
}

// TestDisplacedCapChamferReaders pins each reader that does not know a
// displaced cap-loop chamfer to its answer for a payload class it does not
// know (agent-docs/prism-boolean-design.md Part K §8.8).
func TestDisplacedCapChamferReaders(t *testing.T) {
	t.Parallel()
	chamfered := chamferedWheel(t)
	doc := chamfered.Document()

	t.Run("a later modify op refuses with SX10", func(t *testing.T) {
		_, err := decad.Chamfer(t.Context(), chamfered, decad.Edges(decad.ParallelTo(r3.NewVec(0, 0, 1))), units.Millimeters(0.1))
		require.ErrorIs(t, err, decad.ErrUnsupported)
		require.Contains(t, err.Error(), "does not yet compose another modify op onto a cap-loop chamfer result")
		_, err = decad.Fillet(t.Context(), chamfered, decad.Edges(decad.ParallelTo(r3.NewVec(0, 0, 1))), units.Millimeters(0.1))
		require.ErrorIs(t, err, decad.ErrUnsupported)
		require.Contains(t, err.Error(), "does not yet compose another modify op onto a cap-loop chamfer result")
		_, err = decad.Shell(t.Context(), chamfered, decad.Faces(decad.FaceCreatedBy(decad.CapStart(chamfered))), units.Millimeters(0.5))
		require.ErrorIs(t, err, decad.ErrUnsupported)
		require.Contains(t, err.Error(), "does not yet compose another modify op onto a cap-loop chamfer result")
	})
	t.Run("a through-all stop with the body live refuses", func(t *testing.T) {
		w := sketch.NewWorld()
		plane, err := w.CreateOffsetPlane(w.XY(), -1)
		require.NoError(t, err)
		s, err := w.CreateSketch(plane)
		require.NoError(t, err)
		fixedRect(s, -7, 3.2, -6, 4.2)
		_, err = s.Solve(t.Context())
		require.NoError(t, err)
		_, err = decad.Extrude(t.Context(), doc, s, s.Profiles()[0], decad.ThroughAll{Dir: decad.Along})
		require.ErrorIs(t, err, decad.ErrUnsupported)
		require.Contains(t, err.Error(), "through-all stop cannot read the extent")
	})
	t.Run("the surveys are staged", func(t *testing.T) {
		for name, opt := range map[string]decad.VerifyOption{
			surveyWall:       decad.WithMinWallThickness(units.Millimeters(1)),
			"pull":           decad.WithPullDirection(r3.NewVec(0, 0, 1)),
			"concave-radius": decad.WithConcaveRadius(),
		} {
			report, err := doc.Verify(t.Context(), opt)
			require.NoError(t, err)
			reading, err := report.ForBody(chamfered)
			require.NoError(t, err)
			require.Equal(t, decad.Suspect, reading.Status, name)
			staged, found := findDiagnostic(reading.Diagnostics, decad.DiagUnsupportedSurveyPayload)
			require.True(t, found, name)
			require.Equal(t, "displacedCapBlendPayload "+name+" survey support is not implemented; use an analytic body or wait for wider "+
				"displacedCapBlendPayload survey support", staged.Message)
		}
	})
	t.Run("the clearance kernel builds no model and the tolerance gate reads witnesses", func(t *testing.T) {
		// A pin standing in a window: its box lies inside the wheel's, so only a
		// model of the wheel could decide the pair.
		w := sketch.NewWorld()
		s, err := w.CreateSketch(zeroPlane(t, w))
		require.NoError(t, err)
		fixedRect(s, -7, 3.2, -6, 4.2)
		_, err = s.Solve(t.Context())
		require.NoError(t, err)
		pin, err := decad.Extrude(t.Context(), doc, s, s.Profiles()[0], decad.Distance{D: units.Millimeters(2), Dir: decad.Along})
		require.NoError(t, err)
		report, err := doc.Verify(t.Context(), decad.WithClearances())
		require.NoError(t, err)
		require.Equal(t, decad.Suspect, report.Status)
		_, found := findDiagnostic(report.Diagnostics, decad.DiagUndecidedClearance)
		require.True(t, found, fmt.Sprint(report.Diagnostics))
		// The tolerance gate takes a gate diameter from the witness prisms, so
		// the body is not failed closed for lack of one.
		gated, err := doc.Verify(t.Context(), decad.WithTolerance(units.Scalar(1e-6)))
		require.NoError(t, err)
		for _, b := range []*decad.Body{chamfered, pin} {
			reading, err := gated.ForBody(b)
			require.NoError(t, err)
			require.Equal(t, decad.Sound, reading.Status)
		}
	})
}

func zeroPlane(t *testing.T, w *sketch.World) *sketch.Plane {
	t.Helper()
	plane, err := w.CreateOffsetPlane(w.XY(), 0)
	require.NoError(t, err)
	return plane
}
