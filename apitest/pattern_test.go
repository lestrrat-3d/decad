package apitest_test

import (
	"context"
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// The pattern tests of docs/mirror-pattern-design.md §8 for PatternCopies.

var (
	patternX = r3.NewVec(1, 0, 0)
	patternZ = r3.NewVec(0, 0, 1)
)

// requireCentroidNear asserts c lies within its own bound of want, compared
// exactly: want is given as exact rationals (or the tightest big.Float the
// caller has), and the distance is squared before the comparison.
func requireCentroidNear(t *testing.T, c decad.VecMeasurement, want [3]*big.Float, slack float64, msg string) {
	t.Helper()
	bound, err := c.Bound.In(units.Millimeter)
	require.NoError(t, err)
	got := [3]float64{c.Value.X, c.Value.Y, c.Value.Z}
	sq := new(big.Float).SetPrec(512)
	for k := range got {
		d := new(big.Float).SetPrec(512).Sub(new(big.Float).SetPrec(512).SetFloat64(got[k]), want[k])
		sq.Add(sq, d.Mul(d, d))
	}
	allow := new(big.Float).SetPrec(512).SetFloat64(bound + slack)
	require.LessOrEqual(t, sq.Cmp(allow.Mul(allow, allow)), 0, "%s: centroid %v (bound %v) misses %v", msg, c.Value, bound, want)
}

func bigVec(x, y, z float64) [3]*big.Float {
	f := func(v float64) *big.Float { return new(big.Float).SetPrec(512).SetFloat64(v) }
	return [3]*big.Float{f(x), f(y), f(z)}
}

// TestPatternCopiesLinearExact is §8's exact linear pattern: a 5 mm peg
// patterned four times at 10 mm along +x yields three live copies beside the
// live receiver, each Exact 125 mm³ with its centroid at x = 2.5 + 10i.
func TestPatternCopiesLinearExact(t *testing.T) {
	t.Parallel()
	sc := newMirrorScene()
	peg := sc.box(t, sc.w.XY(), 0, 0, 5, 5, mirrorAlong(5))
	copies, err := peg.PatternCopies(t.Context(), decad.LinearPattern{Dir: patternX, Step: units.Millimeters(10), Count: 4})
	require.NoError(t, err)
	require.Len(t, copies, 3)
	require.Equal(t, append([]*decad.Body{peg}, copies...), sc.doc.Bodies(), "the receiver stays live, copies follow in order")
	for i, c := range copies {
		v, err := c.Volume()
		require.NoError(t, err)
		require.Equal(t, decad.Exact, v.Exactness)
		require.Equal(t, 125.0, volumeMM(t, v))
		centroid, err := c.Centroid()
		require.NoError(t, err)
		require.Equal(t, decad.Exact, centroid.Exactness)
		require.Equal(t, r3.NewVec(2.5+10*float64(i+1), 2.5, 2.5), centroid.Value)
		requireMirrorBox(t, c, r3.NewVec(10*float64(i+1), 0, 0), r3.NewVec(10*float64(i+1)+5, 5, 5))
	}
}

// TestPatternCopiesLinearInches is §8's charged linear pattern: a 0.3 in
// step converts to millimetres with rounding (1 in is the float 25.4 exactly,
// so it would not), so each instance's readings are not Exact, and its
// centroid still encloses the exact 2.5 + i·0.3·25.4 over the held floats
// (the conversion the step denotes, computed exactly).
func TestPatternCopiesLinearInches(t *testing.T) {
	t.Parallel()
	sc := newMirrorScene()
	peg := sc.box(t, sc.w.XY(), 0, 0, 5, 5, mirrorAlong(5))
	copies, err := peg.PatternCopies(t.Context(), decad.LinearPattern{Dir: patternX, Step: units.Inches(0.3), Count: 3})
	require.NoError(t, err)
	require.Len(t, copies, 2)
	inch := new(big.Rat).Quo(new(big.Rat).Mul(new(big.Rat).SetFloat64(0.3), new(big.Rat).SetFloat64(units.Inch.Factor())), new(big.Rat).SetFloat64(units.Millimeter.Factor()))
	for i, c := range copies {
		v, err := c.Volume()
		require.NoError(t, err)
		require.NotEqual(t, decad.Exact, v.Exactness, "the instance carries the conversion's rounding")
		centroid, err := c.Centroid()
		require.NoError(t, err)
		x := new(big.Rat).Add(big.NewRat(5, 2), new(big.Rat).Mul(big.NewRat(int64(i+1), 1), inch))
		want := bigVec(0, 2.5, 2.5)
		want[0] = new(big.Float).SetPrec(512).SetRat(x)
		requireCentroidNear(t, centroid, want, 0, "inch instance")
	}
}

// TestPatternCopiesCircular is §8's circular pattern: a Ø4 pin at (20, 0)
// patterned six times about the z axis lands instance i's centroid on
// (20 cos 60i°, 20 sin 60i°) within its bound, with √3 taken to 512 bits; a
// 4×2 block patterned four times lands on the exact quarter turns, Exact.
func TestPatternCopiesCircular(t *testing.T) {
	t.Parallel()
	t.Run("six pins", func(t *testing.T) {
		t.Parallel()
		sc := newMirrorScene()
		pin := sc.cylinder(t, sc.w.XY(), 20, 0, 2, mirrorAlong(5))
		copies, err := pin.PatternCopies(t.Context(), decad.CircularPattern{Axis: patternZ, Count: 6})
		require.NoError(t, err)
		require.Len(t, copies, 5)
		root3 := new(big.Float).SetPrec(512).Sqrt(new(big.Float).SetPrec(512).SetInt64(3))
		ten3 := new(big.Float).SetPrec(512).Mul(root3, new(big.Float).SetPrec(512).SetInt64(10))
		neg := func(f *big.Float) *big.Float { return new(big.Float).SetPrec(512).Neg(f) }
		f := func(v float64) *big.Float { return new(big.Float).SetPrec(512).SetFloat64(v) }
		wants := [][3]*big.Float{
			{f(10), ten3, f(2.5)}, {f(-10), ten3, f(2.5)}, {f(-20), f(0), f(2.5)},
			{f(-10), neg(ten3), f(2.5)}, {f(10), neg(ten3), f(2.5)},
		}
		for i, c := range copies {
			centroid, err := c.Centroid()
			require.NoError(t, err)
			requireCentroidNear(t, centroid, wants[i], 0, "pin instance")
			v, err := c.Volume()
			require.NoError(t, err)
			requirePiLinearEnclosed(t, v, 0, 20)
		}
	})
	t.Run("four blocks", func(t *testing.T) {
		t.Parallel()
		sc := newMirrorScene()
		block := sc.box(t, sc.w.XY(), 18, -1, 22, 1, mirrorAlong(5))
		copies, err := block.PatternCopies(t.Context(), decad.CircularPattern{Axis: patternZ, Count: 4})
		require.NoError(t, err)
		require.Len(t, copies, 3)
		for i, want := range []r3.Vec{{X: 0, Y: 20, Z: 2.5}, {X: -20, Y: 0, Z: 2.5}, {X: 0, Y: -20, Z: 2.5}} {
			centroid, err := copies[i].Centroid()
			require.NoError(t, err)
			require.Equal(t, decad.Exact, centroid.Exactness)
			require.Equal(t, want, centroid.Value)
			v, err := copies[i].Volume()
			require.NoError(t, err)
			require.Equal(t, decad.Exact, v.Exactness)
			require.Equal(t, 40.0, volumeMM(t, v))
		}
	})
}

// TestPatternCopiesRotationSense checks the frame-keeping arm's rotation
// sense against the exact world rotation: about +z, about −z, and on a
// mirrored receiver whose placement is a reflection, instance i of eight sits
// at the receiver's centroid turned by 45i° about the stated axis, with √2/2
// taken to 512 bits, within the instance's and the receiver's bounds.
//
// Shown to fail with the reflection's sense correction removed from
// circularMotion: the mirrored receiver's instances turn the other way.
func TestPatternCopiesRotationSense(t *testing.T) {
	t.Parallel()
	half2 := new(big.Float).SetPrec(512).Quo(new(big.Float).SetPrec(512).Sqrt(new(big.Float).SetPrec(512).SetInt64(2)), new(big.Float).SetPrec(512).SetInt64(2))
	f := func(v float64) *big.Float { return new(big.Float).SetPrec(512).SetFloat64(v) }
	neg := func(x *big.Float) *big.Float { return new(big.Float).SetPrec(512).Neg(x) }
	// cos and sin of 45k°, k = 0..7.
	cosK := []*big.Float{f(1), half2, f(0), neg(half2), f(-1), neg(half2), f(0), half2}
	sinK := []*big.Float{f(0), half2, f(1), half2, f(0), neg(half2), f(-1), neg(half2)}
	for _, tc := range []struct {
		Name   string
		Axis   r3.Vec
		Sense  int
		Mirror bool
	}{
		{Name: "about +z", Axis: patternZ, Sense: 1},
		{Name: "about -z", Axis: r3.NewVec(0, 0, -1), Sense: -1},
		{Name: "mirrored receiver", Axis: patternZ, Sense: 1, Mirror: true},
	} {
		t.Run(tc.Name, func(t *testing.T) {
			t.Parallel()
			sc := newMirrorScene()
			block := sc.box(t, sc.w.XY(), 10, 2, 16, 5, mirrorAlong(5))
			if tc.Mirror {
				var err error
				block, err = block.Mirrored(t.Context(), mirrorAtX30(t))
				require.NoError(t, err)
			}
			src, err := block.Centroid()
			require.NoError(t, err)
			sb, err := src.Bound.In(units.Millimeter)
			require.NoError(t, err)
			copies, err := block.PatternCopies(t.Context(), decad.CircularPattern{Axis: tc.Axis, Count: 8})
			require.NoError(t, err)
			for i, c := range copies {
				k := ((tc.Sense*(i+1))%8 + 8) % 8
				x := new(big.Float).SetPrec(512).Sub(
					new(big.Float).SetPrec(512).Mul(f(src.Value.X), cosK[k]),
					new(big.Float).SetPrec(512).Mul(f(src.Value.Y), sinK[k]))
				y := new(big.Float).SetPrec(512).Add(
					new(big.Float).SetPrec(512).Mul(f(src.Value.X), sinK[k]),
					new(big.Float).SetPrec(512).Mul(f(src.Value.Y), cosK[k]))
				got, err := c.Centroid()
				require.NoError(t, err)
				// A rotation is an isometry, so the receiver's own bound
				// carries over to its image unchanged.
				requireCentroidNear(t, got, [3]*big.Float{x, y, f(src.Value.Z)}, sb, "instance against the exact rotation")
			}
		})
	}
}

// TestPatternCopiesNonCoDirectional is §8's non-co-directional row: a peg
// patterned along its own sweep, or about an axis in its plane, instances
// through PlacedCopy, with each centroid where the motion puts it.
func TestPatternCopiesNonCoDirectional(t *testing.T) {
	t.Parallel()
	t.Run("along the sweep", func(t *testing.T) {
		t.Parallel()
		sc := newMirrorScene()
		peg := sc.box(t, sc.w.XY(), 0, 0, 5, 5, mirrorAlong(5))
		copies, err := peg.PatternCopies(t.Context(), decad.LinearPattern{Dir: patternZ, Step: units.Millimeters(10), Count: 3})
		require.NoError(t, err)
		require.Len(t, copies, 2)
		for i, c := range copies {
			centroid, err := c.Centroid()
			require.NoError(t, err)
			requireCentroidNear(t, centroid, bigVec(2.5, 2.5, 2.5+10*float64(i+1)), 0, "stacked peg")
		}
	})
	t.Run("about an in-plane axis", func(t *testing.T) {
		t.Parallel()
		sc := newMirrorScene()
		peg := sc.box(t, sc.w.XY(), 0, 0, 5, 5, mirrorAlong(5))
		copies, err := peg.PatternCopies(t.Context(), decad.CircularPattern{Axis: patternX, Count: 2})
		require.NoError(t, err)
		require.Len(t, copies, 1)
		centroid, err := copies[0].Centroid()
		require.NoError(t, err)
		// A PlacedCopy instance denotes its held placement: the receiver's
		// identity composed with RotationAround's transform, whose float
		// trig makes it a turn of nearly, not exactly, 180°. The reference
		// is that transform applied exactly to the receiver's Exact centroid.
		rot, err := r3.RotationAround(r3.Vec{}, patternX, units.Degrees(180))
		require.NoError(t, err)
		placed, err := r3.Identity().Then(rot)
		require.NoError(t, err)
		requireCentroidNear(t, centroid, exactApplyBig(placed, r3.NewVec(2.5, 2.5, 2.5)), 0, "half turn about x")
	})
}

// TestPatternCopiesStacked patterns a blind-cut plate: each instance is the
// stacked record moved in its plane, Exact 936 mm³, one lump, translated.
func TestPatternCopiesStacked(t *testing.T) {
	t.Parallel()
	sc := newMirrorScene()
	plate := sc.box(t, sc.w.XY(), 0, 0, 10, 10, mirrorAlong(10))
	top, err := sc.w.CreateOffsetPlane(sc.w.XY(), 6)
	require.NoError(t, err)
	pocket := sc.box(t, top, 3, 3, 7, 7, mirrorAlong(4))
	part, err := decad.Cut(t.Context(), plate, pocket)
	require.NoError(t, err)
	copies, err := part.PatternCopies(t.Context(), decad.LinearPattern{Dir: patternX, Step: units.Millimeters(20), Count: 3})
	require.NoError(t, err)
	for i, c := range copies {
		v, err := c.Volume()
		require.NoError(t, err)
		require.Equal(t, decad.Exact, v.Exactness)
		require.Equal(t, 936.0, volumeMM(t, v))
		require.Len(t, c.Lumps(), 1)
		requireManifold(t, c)
		x := 20 * float64(i+1)
		requireMirrorBox(t, c, r3.NewVec(x, 0, 0), r3.NewVec(x+10, 10, 10))
	}
}

// TestPatternCopiesGates covers the spec's own gates and PlacedCopy's: every
// refusal leaves the document unchanged.
func TestPatternCopiesGates(t *testing.T) {
	t.Parallel()
	sc := newMirrorScene()
	peg := sc.box(t, sc.w.XY(), 0, 0, 5, 5, mirrorAlong(5))
	canceled, cancel := context.WithCancel(t.Context())
	cancel()
	mm := units.Millimeters(10)
	testcases := []struct {
		Name string
		Spec decad.PatternSpec
		Ctx  context.Context //nolint:containedctx // each row's context is the input under test.
		Want error
	}{
		{Name: "nil spec", Spec: nil, Ctx: t.Context(), Want: decad.ErrDegenerate},
		{Name: "typed nil linear", Spec: (*decad.LinearPattern)(nil), Ctx: t.Context(), Want: decad.ErrDegenerate},
		{Name: "typed nil circular", Spec: (*decad.CircularPattern)(nil), Ctx: t.Context(), Want: decad.ErrDegenerate},
		{Name: "count one", Spec: decad.LinearPattern{Dir: patternX, Step: mm, Count: 1}, Ctx: t.Context(), Want: decad.ErrDegenerate},
		{Name: "circular count one", Spec: decad.CircularPattern{Axis: patternZ, Count: 1}, Ctx: t.Context(), Want: decad.ErrDegenerate},
		{Name: "zero direction", Spec: decad.LinearPattern{Step: mm, Count: 2}, Ctx: t.Context(), Want: decad.ErrDegenerate},
		{Name: "NaN direction", Spec: decad.LinearPattern{Dir: r3.NewVec(math.NaN(), 0, 0), Step: mm, Count: 2}, Ctx: t.Context(), Want: decad.ErrNotFinite},
		{Name: "zero axis", Spec: decad.CircularPattern{Count: 2}, Ctx: t.Context(), Want: decad.ErrDegenerate},
		{Name: "infinite centre", Spec: decad.CircularPattern{Center: r3.NewVec(math.Inf(1), 0, 0), Axis: patternZ, Count: 2}, Ctx: t.Context(), Want: decad.ErrNotFinite},
		{Name: "angle step", Spec: decad.LinearPattern{Dir: patternX, Step: units.Degrees(10), Count: 2}, Ctx: t.Context(), Want: decad.ErrUnitKind},
		{Name: "negative step", Spec: decad.LinearPattern{Dir: patternX, Step: units.Millimeters(-10), Count: 2}, Ctx: t.Context(), Want: decad.ErrNegativeMagnitude},
		{Name: "zero step", Spec: decad.LinearPattern{Dir: patternX, Step: units.Millimeters(0), Count: 2}, Ctx: t.Context(), Want: decad.ErrDegenerate},
		{Name: "nil context", Spec: decad.LinearPattern{Dir: patternX, Step: mm, Count: 2}, Ctx: nil, Want: decad.ErrDegenerate},
		{Name: "canceled context", Spec: decad.LinearPattern{Dir: patternX, Step: mm, Count: 3}, Ctx: canceled, Want: context.Canceled},
	}
	for _, tc := range testcases {
		t.Run(tc.Name, func(t *testing.T) {
			_, err := peg.PatternCopies(tc.Ctx, tc.Spec)
			require.ErrorIs(t, err, tc.Want)
			require.Equal(t, []*decad.Body{peg}, sc.doc.Bodies())
		})
	}
	// A pointer variant names the same pattern its value does.
	copies, err := peg.PatternCopies(t.Context(), &decad.LinearPattern{Dir: patternX, Step: mm, Count: 2})
	require.NoError(t, err)
	require.Len(t, copies, 1)
	require.NoError(t, sc.doc.Remove(peg))
	_, err = peg.PatternCopies(t.Context(), decad.LinearPattern{Dir: patternX, Step: mm, Count: 2})
	require.ErrorIs(t, err, decad.ErrRetiredBody)
}

// exactApplyBig applies t's held basis and translation to p in 512-bit
// arithmetic, which holds every product and sum of these floats exactly.
func exactApplyBig(t r3.Transform, p r3.Vec) [3]*big.Float {
	f := func(v float64) *big.Float { return new(big.Float).SetPrec(512).SetFloat64(v) }
	b := t.Basis()
	tr := t.Translation()
	var out [3]*big.Float
	for k, pick := range []func(r3.Vec) float64{
		func(v r3.Vec) float64 { return v.X }, func(v r3.Vec) float64 { return v.Y }, func(v r3.Vec) float64 { return v.Z },
	} {
		sum := f(pick(tr))
		sum.Add(sum, new(big.Float).SetPrec(512).Mul(f(pick(b.EX)), f(p.X)))
		sum.Add(sum, new(big.Float).SetPrec(512).Mul(f(pick(b.EY)), f(p.Y)))
		sum.Add(sum, new(big.Float).SetPrec(512).Mul(f(pick(b.EZ)), f(p.Z)))
		out[k] = sum
	}
	return out
}

// TestPatternCopiesUnionStack patterns a union-built stack, whose outer loop
// changes between its slabs: a 10 mm plate 5 mm thick with a 4 mm square boss
// 5 mm tall. An integer step moves the record exactly and keeps it Exact
// (580 mm³); a 0.3 in step would round the two outers apart by its own
// rounding, so it copies through PlacedCopy and lands where the step puts it.
//
// Shown to fail with stackedInterfaces re-deriving a changed-outer interface
// from exclusive holes: the moved stack's floor record is then missing and
// the stacked audit refuses the integer instance.
func TestPatternCopiesUnionStack(t *testing.T) {
	t.Parallel()
	for _, tc := range []struct {
		Name  string
		Step  units.Value
		Shift *big.Rat
		Exact bool
	}{
		{Name: "integer step", Step: units.Millimeters(20), Shift: big.NewRat(20, 1), Exact: true},
		// The PlacedCopy instance denotes its held placement: a translation by
		// the step's held millimetre float along the unit Dir.
		{Name: "inch step", Step: units.Inches(0.3), Shift: heldMillimeters(t, units.Inches(0.3))},
	} {
		t.Run(tc.Name, func(t *testing.T) {
			t.Parallel()
			doc := decad.New()
			plate := boxBody(t, doc, 0, 0, 10, 10, 5)
			boss := boxBodyAtZ(t, doc, 3, 3, 7, 7, 5, 5)
			part, err := decad.Union(t.Context(), plate, boss)
			require.NoError(t, err)
			require.False(t, anyFaceIsFaceted(part), "the premise: the union is a stacked prism")
			src, err := part.Centroid()
			require.NoError(t, err)
			sb, err := src.Bound.In(units.Millimeter)
			require.NoError(t, err)
			copies, err := part.PatternCopies(t.Context(), decad.LinearPattern{Dir: patternX, Step: tc.Step, Count: 2})
			require.NoError(t, err)
			require.Len(t, copies, 1)
			c := copies[0]
			v, err := c.Volume()
			require.NoError(t, err)
			if tc.Exact {
				require.Equal(t, decad.Exact, v.Exactness)
				require.Equal(t, 580.0, volumeMM(t, v))
			} else {
				require.LessOrEqual(t, math.Abs(volumeMM(t, v)-580), boundMM3(t, v))
			}
			require.Len(t, c.Lumps(), 1)
			requireManifold(t, c)
			got, err := c.Centroid()
			require.NoError(t, err)
			x := new(big.Float).SetPrec(512).SetRat(new(big.Rat).Add(new(big.Rat).SetFloat64(src.Value.X), tc.Shift))
			want := bigVec(0, src.Value.Y, src.Value.Z)
			want[0] = x
			requireCentroidNear(t, got, want, sb, "union stack instance")
		})
	}
}

// heldMillimeters is v's held millimetre float as an exact rational.
func heldMillimeters(t *testing.T, v units.Value) *big.Rat {
	t.Helper()
	mm, err := v.In(units.Millimeter)
	require.NoError(t, err)
	return new(big.Rat).SetFloat64(mm)
}
