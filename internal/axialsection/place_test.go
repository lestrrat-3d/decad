package axialsection_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"

	"github.com/lestrrat-3d/decad/internal/axialsection"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
)

func xyFrame(t *testing.T, z float64) r3.Frame {
	t.Helper()
	f, err := r3.NewFrame(r3.NewVec(0, 0, z), r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	return f
}

// sweep is an axial operand between z = 0 and z = 5 carrying a section and a placement.
func sweep(t *testing.T, section momentinput.Profile, x r3.Transform, loft bool) axialsection.Operand {
	t.Helper()
	return axialsection.Operand{
		Section: section, Frame0: xyFrame(t, 0), Frame1: xyFrame(t, 5), FromLoft: loft, Xform: x,
		Level0: proofarith.FloatRat(0), Level1: proofarith.FloatRat(5),
	}
}

func turnAbout(t *testing.T, axis r3.Vec, turn float64) r3.Transform {
	t.Helper()
	x, err := r3.RotationAround(r3.Vec{}, axis, units.Radians(2*math.Pi*turn))
	require.NoError(t, err)
	return x
}

func fitOf(t *testing.T, p momentinput.Profile) [][]sectionrecord.Point2 {
	t.Helper()
	var out [][]sectionrecord.Point2
	for _, seg := range p.Outer.Segments {
		if f, ok := seg.(sectionrecord.FitSplineSeg); ok {
			out = append(out, f.Fit)
		}
	}
	return out
}

// TestPlaceRecordsARotatedLobeAsTheLobeDrawnThere pins the rule on a shape-neutral
// section: a lobe placed by a turn about the sweep axis is re-recorded on the
// reference frame within a displacement bound that is positive and tiny, and each
// fit point is the rotated source point.
func TestPlaceRecordsARotatedLobeAsTheLobeDrawnThere(t *testing.T) {
	t.Parallel()
	const n = 17
	lobe, _ := lobeSection(t, 3, 0, 0.4*math.Pi/n, 6)
	ref := sweep(t, lobe, r3.Identity(), true)
	placed := sweep(t, lobe, turnAbout(t, r3.NewVec(0, 0, 1), 1.0/n), false)
	out, ok, err := axialsection.Place(t.Context(), []axialsection.Operand{ref, placed})
	require.NoError(t, err)
	require.True(t, ok)
	require.Equal(t, r3.Identity(), out[1].Xform)
	require.Positive(t, out[1].Delta)
	require.Less(t, out[1].Delta, 1e-12)
	require.Zero(t, out[0].Delta, "the reference operand is untouched")

	drawn, _ := lobeSection(t, 3, 2*math.Pi/n, 0.4*math.Pi/n, 6)
	want, got := fitOf(t, drawn), fitOf(t, out[1].Section)
	require.Len(t, got, len(want))
	for i := range want {
		require.Len(t, got[i], len(want[i]))
		for j := range want[i] {
			require.InDelta(t, want[i][j].U, got[i][j].U, 1e-12)
			require.InDelta(t, want[i][j].V, got[i][j].V, 1e-12)
		}
	}
}

func TestPlaceMissesWhatItCannotPlaceExactly(t *testing.T) {
	t.Parallel()
	lobe, _ := lobeSection(t, 3, 0, 0.1, 4)
	ref := sweep(t, lobe, r3.Identity(), true)
	lift, err := r3.Translation(r3.NewVec(0, 0, 1))
	require.NoError(t, err)
	cases := map[string]axialsection.Operand{
		"a tilted placement":      sweep(t, lobe, turnAbout(t, r3.NewVec(1, 0, 0), 0.01), false),
		"a lifted placement":      sweep(t, lobe, lift, false),
		"a placed reference only": sweep(t, lobe, turnAbout(t, r3.NewVec(0, 0, 1), 0.1), true),
	}
	for name, op := range cases {
		t.Run(name, func(t *testing.T) {
			ops := []axialsection.Operand{ref, op}
			if name == "a placed reference only" {
				ops = []axialsection.Operand{op, op}
			}
			_, ok, err := axialsection.Place(t.Context(), ops)
			require.NoError(t, err)
			require.False(t, ok)
		})
	}
}

func TestPlaceLeavesUnplacedOperandsAlone(t *testing.T) {
	t.Parallel()
	lobe, _ := lobeSection(t, 2.5, 0, 0.1, 4)
	ops := []axialsection.Operand{sweep(t, lobe, r3.Identity(), true), sweep(t, lobe, r3.Identity(), false)}
	out, ok, err := axialsection.Place(t.Context(), ops)
	require.NoError(t, err)
	require.True(t, ok)
	require.Equal(t, ops, out)
}

// TestPlaceGuardsFireOnTheirOwn isolates the checks that can fire alone: the
// tilted placement passes the height check because both its levels are zero
// (the plane and similarity checks refuse it), and the placement that keeps the
// plane but not the end height is refused by the height check alone.
func TestPlaceGuardsFireOnTheirOwn(t *testing.T) {
	t.Parallel()
	lobe, _ := lobeSection(t, 3, 0, 0.1, 4)
	flat := func(x r3.Transform, loft bool) axialsection.Operand {
		op := sweep(t, lobe, x, loft)
		op.Level1 = proofarith.FloatRat(0)
		return op
	}
	_, ok, err := axialsection.Place(t.Context(), []axialsection.Operand{
		flat(r3.Identity(), true), flat(turnAbout(t, r3.NewVec(1, 0, 0), 0.01), false)})
	require.NoError(t, err)
	require.False(t, ok, "a tilted plane is refused although its end heights match")

	taller := sweep(t, lobe, turnAbout(t, r3.NewVec(0, 0, 1), 0.1), false)
	taller.Level1 = proofarith.FloatRat(6)
	_, ok, err = axialsection.Place(t.Context(), []axialsection.Operand{sweep(t, lobe, r3.Identity(), true), taller})
	require.NoError(t, err)
	require.False(t, ok, "an end plane at another height is refused by the height check alone")
}

// TestPlaceOnAnObliqueFrame places a lobe by a 17th of a turn about the normal of a
// frame no axis aligns with. The frames are orthonormal only to rounding and the end
// frame is the start frame lifted by a float multiple of the normal, so the placement
// is charged for its off-plane distance, height gaps and lateral offset; it still
// places, and the charge is positive and tiny.
func TestPlaceOnAnObliqueFrame(t *testing.T) {
	t.Parallel()
	rot, err := r3.RotationAround(r3.Vec{}, r3.NewVec(1, 1, 1), units.Radians(math.Pi/6))
	require.NoError(t, err)
	f0, err := r3.NewFrame(r3.NewVec(3, -2, 5), rot.ApplyDir(r3.NewVec(1, 0, 0)), rot.ApplyDir(r3.NewVec(0, 1, 0)))
	require.NoError(t, err)
	f1, err := r3.NewFrame(f0.Origin().Add(f0.N().Scale(5)), f0.U(), f0.V())
	require.NoError(t, err)
	const n = 17
	lobe, _ := lobeSection(t, 3, 0, 0.4*math.Pi/n, 6)
	oblique := func(x r3.Transform, loft bool) axialsection.Operand {
		op := sweep(t, lobe, x, loft)
		op.Frame0, op.Frame1 = f0, f1
		op.Level1 = proofarith.FloatRat(5)
		return op
	}
	turn, err := r3.RotationAround(f0.Origin(), f0.N(), units.Radians(2*math.Pi/n))
	require.NoError(t, err)
	ref, placed := oblique(r3.Identity(), true), oblique(turn, false)
	out, ok, err := axialsection.Place(t.Context(), []axialsection.Operand{ref, placed})
	require.NoError(t, err)
	require.True(t, ok)
	require.Positive(t, out[1].Delta)
	require.Less(t, out[1].Delta, 1e-12)
	require.Equal(t, ref.Frame0, out[1].Frame0)
}
