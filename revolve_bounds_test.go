package decad_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/decadtest"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file proves the revolve angular-denotation rule PR 1 adopts
// (docs/evaluator-design.md §6): every reading built on a held sweep angle
// must contain the angle the recorded AngularExtent denotes, not merely the
// float the resolver rounded to. Every assertion below is a RELATION — a
// containment, a ratio, or a zero/nonzero split — never a bound literal,
// since the bound differs between amd64 and arm64 through FMA.

// TestRevolveBoundsEnclosesDenotedExtreme is design §11 test 9: the box a
// degree-stated quarter turn publishes must contain the exact quarter-turn
// minimum (cos(pi/2) = 0) once it is grown by its own proven bound, and that
// bound must be nonzero — the charge is not free. A wide (120deg/180deg)
// sweep's box stays tight through the interior-critical-angle arm this PR
// also adds to sweepExtremeBounds. A partial-sweep cap vertex carries the
// same charge and nothing else, so it is exactly zero for a radian-stated
// sweep and nonzero for a degree-stated one; its cap face's own normal
// carries the charge on top of the frame's own baseline rounding, so it
// stays nonzero-but-tiny for a radian-stated sweep and grows (still tiny)
// for a degree-stated one.
func TestRevolveBoundsEnclosesDenotedExtreme(t *testing.T) {
	t.Parallel()

	t.Run("quarter turn box encloses the exact minimum", func(t *testing.T) {
		s, p := annularSketch(t)
		doc := decad.New()
		body, err := doc.Revolve(s, p, uAxis, decad.AngleExtent{A: units.Degrees(90), Dir: decad.Along})
		require.NoError(t, err)

		box, err := body.Bounds()
		require.NoError(t, err)
		require.Positive(t, box.Bound.Base(), `the angular displacement charge is not free`)

		// The quarter turn's true Y minimum is cos(pi/2) = 0 exactly; the
		// held float misses it by ~3e-16, and the box must contain 0 once
		// grown by its own bound.
		decadtest.Encloses(t, "quarter-turn box", box, r3.NewVec(box.Min.X, 0, box.Min.Z))
	})

	t.Run("wide sweeps keep a tight box", func(t *testing.T) {
		for _, deg := range []float64{120, 180} {
			t.Run("", func(t *testing.T) {
				s, p := annularSketch(t)
				doc := decad.New()
				body, err := doc.Revolve(s, p, uAxis, decad.AngleExtent{A: units.Degrees(deg), Dir: decad.Along})
				require.NoError(t, err)

				box, err := body.Bounds()
				require.NoError(t, err)
				diameter := box.Max.Sub(box.Min).Len()
				decadtest.HasBoundAtMost(t, "wide-sweep box", box.Bound, units.Millimeters(1e-9*diameter))
			})
		}
	})

	t.Run("partial cap vertex and normal carry the angular charge", func(t *testing.T) {
		s, p := annularSketch(t)

		degDoc := decad.New()
		degBody, err := degDoc.Revolve(s, p, uAxis, decad.AngleExtent{A: units.Degrees(90), Dir: decad.Along})
		require.NoError(t, err)

		radDoc := decad.New()
		radBody, err := radDoc.Revolve(s, p, uAxis, decad.AngleExtent{A: units.Radians(math.Pi / 2), Dir: decad.Along})
		require.NoError(t, err)

		degCap := faceByRole(t, degBody, roleCapEnd)
		radCap := faceByRole(t, radBody, roleCapEnd)

		degEdges := degCap.Loops()[0].Edges()
		radEdges := radCap.Loops()[0].Edges()
		require.NotEmpty(t, degEdges)
		require.NotEmpty(t, radEdges)

		degVertex := degEdges[0].Start()
		radVertex := radEdges[0].Start()
		require.Positive(t, degVertex.Position().Bound.Base(),
			`a degree-stated sweep's cap vertex carries the angular displacement`)
		require.Zero(t, radVertex.Position().Bound.Base(),
			`a radian-stated sweep denotes its own held angle exactly, so its cap vertex is exact`)

		degPlane, ok := degCap.Surface().(decad.Plane)
		require.True(t, ok)
		radPlane, ok := radCap.Surface().(decad.Plane)
		require.True(t, ok)

		degNormal, err := degCap.NormalAt(degPlane.Frame.Origin())
		require.NoError(t, err)
		radNormal, err := radCap.NormalAt(radPlane.Frame.Origin())
		require.NoError(t, err)
		require.Positive(t, degNormal.Bound.Base(),
			`a degree-stated sweep's cap normal carries the angular displacement`)
		// The frame's own cross-product rounding (planeNormalAllow,
		// normal_bound.go) contributes a baseline ulp-level bound even at a
		// radian-stated angle, so the radian case is not claimed exactly
		// zero here — only that adopting the denotation rule keeps it at
		// that same tiny scale rather than widening it to the old envelope.
		decadtest.HasBoundAtMost(t, "radian-stated cap normal", radNormal.Bound, units.Scalar(1e-9))
	})
}
