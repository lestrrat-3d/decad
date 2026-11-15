package filletband_test

import (
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/brepgeom"
	"github.com/lestrrat-3d/decad/internal/capband"
	"github.com/lestrrat-3d/decad/internal/capblend"
	"github.com/lestrrat-3d/decad/internal/filletband"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/offset2d"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/survey2d"
	"github.com/stretchr/testify/require"
)

// slotLoop is a 30 × 10 slot whose right arc's centre sits shift to the
// right of (15, 0): the arc still passes through both lines' ends, but its
// tangents there turn by θ, sin θ = shift/√(25 + shift²), off the lines. The
// left arc meets both lines exactly tangent. Corner k sits at segment k's
// start: 0 and 3 are exact, 1 and 2 near tangent.
func slotLoop(shift float64) sectionrecord.LoopRecord {
	return loopOf(
		line(-15, -5, 15, -5),
		ccwArc(15+shift, 0, 15, -5, 15, 5),
		line(15, 5, -15, 5),
		ccwArc(-15, 0, -15, 5, -15, -5),
	)
}

// readLoop reads loop at fillet radius 1 through filletband.ReadLoop.
func readLoop(t *testing.T, loop sectionrecord.LoopRecord) (filletband.Loop, []survey2d.SideWalk, int, string) {
	t.Helper()
	budget := proofbound.NewWorkBudget(t.Context())
	cl, err := capblend.OneLoopCornerLoop(budget, loop, freeform.NewFreeformWork())
	require.NoError(t, err)
	read, _, _, bad, why, err := filletband.ReadLoop(budget, loop, cl.Walks, 1, 0, offset2d.ShellTol)
	require.NoError(t, err)
	return read, cl.Walks, bad, why
}

// TestReadLoopNearTangentJoins classifies the slot's corners at radius 1:
// the left arc's two exact joins stay LF5 with no reach, and the right arc's
// two joins, sin θ ≈ 2⁻³⁴, are LF5′ with capband.NearTangentCorner's ρ for
// two walks moving by the radius. Reading every LF5′ corner as LF5 (or
// dropping its reach) turns the reach rows red.
func TestReadLoopNearTangentJoins(t *testing.T) {
	t.Parallel()
	shift := 5 * math.Ldexp(1, -34)
	loop := slotLoop(shift)
	read, _, bad, why := readLoop(t, loop)
	require.Equal(t, -1, bad, why)
	require.Equal(t, []filletband.Corner{filletband.Tangent, filletband.NearTangent, filletband.NearTangent, filletband.Tangent}, read.Corners)
	require.Len(t, read.Rho, 4)
	for _, k := range []int{1, 2} {
		charge, reason, _ := capband.NearTangentCorner(loop.Segments[k-1], loop.Segments[k], 1, 1, 0)
		require.Empty(t, reason)
		require.Positive(t, read.Rho[k])
		require.Equal(t, charge.Rho, read.Rho[k], "corner %d", k)
	}
	require.Zero(t, read.Rho[0])
	require.Zero(t, read.Rho[3])
}

// TestReadLoopNearTangentRefusals pins SF1's near-tangent reasons: a join
// inside the offset's G1 dead zone that turns past sin²θ ≤ 2⁻⁶⁴ (the slot at
// sin θ ≈ 2⁻³¹), and a wall too short to survive its ends' reach (a 2⁻⁴⁰ rad
// arc between two near-tangent walls, filleted by 1 mm). Each names the
// corner and its reason. Removing nearTangentWalls admits the second row.
func TestReadLoopNearTangentRefusals(t *testing.T) {
	t.Parallel()
	t.Run("turn past the limit", func(t *testing.T) {
		t.Parallel()
		_, _, bad, why := readLoop(t, slotLoop(5*math.Ldexp(1, -31)))
		require.Equal(t, 1, bad)
		require.Contains(t, why, "exceeds the admitted sin²θ ≤ 2⁻⁶⁴")
	})
	t.Run("arc window within its ends' reach", func(t *testing.T) {
		t.Parallel()
		e40, e36 := math.Ldexp(1, -40), math.Ldexp(1, -36)
		loop := loopOf(
			line(2+e40, -e40-e36, 1+e40, 1),
			ccwArc(0, 0, 1+e40, 1, 1, 1+e40),
			line(1, 1+e40, -e40, 2+e40-e36),
			line(-e40, 2+e40-e36, -2, -2),
			line(-2, -2, 2+e40, -e40-e36),
		)
		_, _, bad, why := readLoop(t, loop)
		require.Equal(t, 1, bad)
		require.Contains(t, why, "a circular wall at a near-tangent join")
	})
}

// TestNearTangentAllowances pins §5.5's closed forms on one corner of reach
// ρ = 2⁻⁴⁰ at (3, −4), radius 2 within 2⁻²⁰: the volume r_hi·ρ·(r_hi + 3ρ),
// each in-plane moment that times |coordinate| + r_hi + 2ρ, the patch area
// π·r_hi·ρ, an extent widened by 2ρ times the direction's planar length, and
// MassOf widening an LF5′ corner's three-times volume by three times the
// volume where an LF5 corner adds nothing.
func TestNearTangentAllowances(t *testing.T) {
	t.Parallel()
	rho := math.Ldexp(1, -40)
	rIv, err := filletband.RadiusInterval(2, math.Ldexp(1, -20))
	require.NoError(t, err)
	d := new(big.Rat).SetFloat64(rho)
	rHi := rIv.Hi
	wantVol := new(big.Rat).Mul(new(big.Rat).Mul(rHi, d), new(big.Rat).Add(rHi, new(big.Rat).Mul(big.NewRat(3, 1), d)))
	vol, mu, mv, ok := filletband.NearTangentMassAllowance(3, -4, 0, rho, rIv)
	require.True(t, ok)
	require.Zero(t, vol.Cmp(wantVol))
	reach := new(big.Rat).Add(rHi, new(big.Rat).Mul(big.NewRat(2, 1), d))
	require.Zero(t, mu.Cmp(new(big.Rat).Mul(wantVol, new(big.Rat).Add(big.NewRat(3, 1), reach))))
	require.Zero(t, mv.Cmp(new(big.Rat).Mul(wantVol, new(big.Rat).Add(big.NewRat(4, 1), reach))))

	area, ok := filletband.NearTangentAreaAllowance(rho, rIv)
	require.True(t, ok)
	require.Zero(t, area.Cmp(new(big.Rat).Mul(new(big.Rat).Mul(proofbound.PiUpper, rHi), d)))

	old := proofbound.Interval(big.NewRat(1, 1), big.NewRat(2, 1))
	g := [3]*big.Rat{big.NewRat(3, 5), big.NewRat(4, 5), big.NewRat(7, 1)}
	widened, err := filletband.WidenNearTangentExtent([]float64{0, rho}, g, old)
	require.NoError(t, err)
	grow := new(big.Rat).Sub(old.Lo, widened.Lo)
	require.GreaterOrEqual(t, grow.Cmp(new(big.Rat).Mul(big.NewRat(2, 1), d)), 0, "the extent grows by at least 2ρ")
	require.Zero(t, grow.Cmp(new(big.Rat).Sub(widened.Hi, old.Hi)))
	same, err := filletband.WidenNearTangentExtent(nil, g, old)
	require.NoError(t, err)
	require.Equal(t, old, same)

	// The slot's two LF5′ corners at radius 1.
	shift := 5 * math.Ldexp(1, -34)
	read, walks, bad, why := readLoop(t, slotLoop(shift))
	require.Equal(t, -1, bad, why)
	pieces, err := read.Pieces()
	require.NoError(t, err)
	r1, err := filletband.RadiusInterval(1, 0)
	require.NoError(t, err)
	in := filletband.BandMassInput{Pieces: pieces, Walks: walks, Corners: read.Corners, Rho: read.Rho,
		Radius: r1, MaterialSign: 1, Sigma: -1, Embed: brepgeom.Embed{Axis: [3]int{0, 1, 2}, Sign: [3]float64{1, 1, 1}}}
	charged, err := filletband.MassOf(in)
	require.NoError(t, err)
	exact := in
	exact.Corners = []filletband.Corner{filletband.Tangent, filletband.Tangent, filletband.Tangent, filletband.Tangent}
	plain, err := filletband.MassOf(exact)
	require.NoError(t, err)
	want := new(big.Rat)
	for _, k := range []int{1, 2} {
		v, _, _, ok := filletband.NearTangentMassAllowance(walks[k].StartU, walks[k].StartV, 0, read.Rho[k], r1)
		require.True(t, ok)
		want.Add(want, new(big.Rat).Mul(big.NewRat(3, 1), v))
	}
	require.Zero(t, new(big.Rat).Sub(plain.Volume3.Lo, charged.Volume3.Lo).Cmp(want))
	require.Zero(t, new(big.Rat).Sub(charged.Volume3.Hi, plain.Volume3.Hi).Cmp(want))
}

// TestLoopContourDeltaChargesNearTangentReach reads the slot's cap contour
// bound at radius 1: the G1 contour proof's bound plus twice the larger LF5′
// reach, and the exact slot's bound alone.
func TestLoopContourDeltaChargesNearTangentReach(t *testing.T) {
	t.Parallel()
	shift := 5 * math.Ldexp(1, -34)
	for _, s := range []float64{0, shift} {
		loop := slotLoop(s)
		budget := proofbound.NewWorkBudget(t.Context())
		cl, err := capblend.OneLoopCornerLoop(budget, loop, freeform.NewFreeformWork())
		require.NoError(t, err)
		base, err := filletband.ContourDelta(budget, cl.Walks, 1, 0, offset2d.ShellTol)
		require.NoError(t, err)
		got, err := filletband.LoopContourDelta(t.Context(), loop, 1, 0, offset2d.ShellTol)
		require.NoError(t, err)
		read, _, bad, why := readLoop(t, loop)
		require.Equal(t, -1, bad, why)
		if s == 0 {
			require.Nil(t, read.Rho)
			require.Equal(t, base, got)
			continue
		}
		require.Equal(t, proofbound.AbsSumUpper(base, filletband.NearTangentReach(read.Rho)), got)
		require.Greater(t, got, base)
	}
}
