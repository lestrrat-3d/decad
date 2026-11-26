package apitest_test

import (
	"errors"
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/decadtest"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// newSeatScene is a scene whose document takes no gap tolerance, so each Boolean answers from the exact path alone
// and never from a retry that moves an operand (agent-docs/gap-tolerance-design.md).
func newSeatScene() *tcScene {
	return &tcScene{doc: decad.New(decad.WithChainTolerance(units.Millimeters(0.01)), decad.ExactOnly()), w: sketch.NewWorld()}
}

// seatBox is the box [x0, x1] × [y0, y1] × [z0, z1].
func seatBox(t *testing.T, sc *tcScene, x0, y0, z0, x1, y1, z1 float64) *decad.Body {
	t.Helper()
	return sc.prism(t, r3.NewVec(0, 0, z0), tcX, tcY, []tcSeg{
		tcLine(x0, y0, x1, y0), tcLine(x1, y0, x1, y1), tcLine(x1, y1, x0, y1), tcLine(x0, y1, x0, y0),
	}, tcAlong(z1-z0))
}

// seatDisc is the cylinder of radius r about the z axis from z0 to z1, drawn as two arcs.
func seatDisc(t *testing.T, sc *tcScene, r, z0, z1 float64) *decad.Body {
	t.Helper()
	return sc.prism(t, r3.NewVec(0, 0, z0), tcX, tcY, []tcSeg{
		tcArc(0, 0, r, 0, -r, 0), tcArc(0, 0, -r, 0, r, 0),
	}, tcAlong(z1-z0))
}

// seatPlate is a 40 × 40 × 10 plate with a pocket [-5, 5]² from z = 6 up, and a bore of radius 2.8 through its floor.
func seatPlate(t *testing.T, sc *tcScene) *decad.Body {
	t.Helper()
	return tcCut(t, seatBox(t, sc, -20, -20, 0, 20, 20, 10), seatBox(t, sc, -5, -5, 6, 5, 5, 11), seatDisc(t, sc, 2.8, -1, 11))
}

func seatCopy(t *testing.T, b *decad.Body) *decad.Body {
	t.Helper()
	c, err := decad.Copy(t.Context(), b)
	require.NoError(t, err)
	return c
}

// requireSeatEmpty requires Intersect of a and b to prove them disjoint in both operand orders.
func requireSeatEmpty(t *testing.T, a, b *decad.Body) {
	t.Helper()
	for _, ops := range [][2]*decad.Body{{a, b}, {b, a}} {
		_, err := decad.Intersect(seatCopy(t, ops[0]), seatCopy(t, ops[1])).Do(t.Context())
		var be *decad.BooleanError
		require.True(t, errors.As(err, &be) && be.Code == decad.BooleanEmpty, "got %v", err)
	}
}

// TestIntersectSeat intersects hardware-like bodies resting flush on a pocket floor that a bore runs through
// (agent-docs/coplanar-contact-design.md §16). The floor's rim round the bore is chorded, so the floor is no exact
// face, but its true piece lies in its plane, and the bore's wall leaves that plane into the plate along a closed rim.
func TestIntersectSeat(t *testing.T) {
	t.Parallel()
	t.Run("a bored block on the floor", func(t *testing.T) {
		t.Parallel()
		sc := newSeatScene()
		plate := seatPlate(t, sc)
		block := tcCut(t, seatBox(t, sc, -4, -4, 6, 4, 4, 9), seatDisc(t, sc, 2.5, 5, 10))
		requireSeatEmpty(t, block, plate)
	})
	t.Run("a bored round nut on the floor", func(t *testing.T) {
		t.Parallel()
		// The nut's own wall is a cylinder that leaves the floor's plane into the nut along its rim.
		sc := newSeatScene()
		plate := seatPlate(t, sc)
		nut := tcCut(t, seatDisc(t, sc, 4.5, 6, 9.5), seatDisc(t, sc, 2.5, 5, 10))
		requireSeatEmpty(t, nut, plate)
	})
	t.Run("a block sunk 0.05 into the floor overlaps it", func(t *testing.T) {
		t.Parallel()
		sc := newSeatScene()
		plate := seatPlate(t, sc)
		block := tcCut(t, seatBox(t, sc, -4, -4, 5.95, 4, 4, 9), seatDisc(t, sc, 2.5, 5, 10))
		for _, ops := range [][2]*decad.Body{{block, plate}, {plate, block}} {
			out, err := decad.Intersect(seatCopy(t, ops[0]), seatCopy(t, ops[1])).Do(t.Context())
			require.NoError(t, err)
			v, err := out.Volume()
			require.NoError(t, err)
			// The 8 × 8 footprint less the floor's bore of radius 2.8, 0.05 deep.
			want := (64 - math.Pi*2.8*2.8) * 0.05
			require.InDelta(t, want, v.Value.Base(), v.Bound.Base()+1e-3)
		}
	})

	t.Run("a bored block flush with a solid plate's top", func(t *testing.T) {
		t.Parallel()
		// The block's top faces the same way as the plate's top: §16 admits no such pair, and the result is the block
		// less the plate's wider bore.
		sc := newTCScene()
		plate := tcCut(t, seatBox(t, sc, -20, -20, 0, 20, 20, 10), seatDisc(t, sc, 2.8, -1, 11))
		block := tcCut(t, seatBox(t, sc, -4, -4, 6, 4, 4, 10), seatDisc(t, sc, 2.5, 5, 11))
		for _, ops := range [][2]*decad.Body{{block, plate}, {plate, block}} {
			out, err := decad.Intersect(seatCopy(t, ops[0]), seatCopy(t, ops[1])).Do(t.Context())
			require.NoError(t, err)
			v, err := out.Volume()
			require.NoError(t, err)
			require.InDelta(t, (64-math.Pi*2.8*2.8)*4, v.Value.Base(), v.Bound.Base())
		}
	})

	// Rejected variants: each stays refused, where admitting it would answer without a proof.
	t.Run("a seated Union still refuses", func(t *testing.T) {
		t.Parallel()
		// The cover of a chorded rim decides the result's surface in a Union; §16 admits seats for Intersect only.
		sc := newSeatScene()
		plate := seatPlate(t, sc)
		block := tcCut(t, seatBox(t, sc, -4, -4, 6, 4, 4, 9), seatDisc(t, sc, 2.5, 5, 10))
		_, err := decad.Union(plate, block).Do(t.Context())
		require.ErrorIs(t, err, decad.ErrUnsupported)
	})
	t.Run("a turned seat whose planes state a departure refuses", func(t *testing.T) {
		t.Parallel()
		// Turning both bodies rounds their planes, which then only lie within a stated departure of their tags: the
		// true planes may be apart or overlap by that much, which no exact predicate decides.
		sc := newSeatScene()
		turn, err := r3.Rotation(r3.NewVec(1, 2, 3), units.Degrees(31))
		require.NoError(t, err)
		plate, err := decad.Transform(t.Context(), seatPlate(t, sc), turn)
		require.NoError(t, err)
		block, err := decad.Transform(t.Context(), tcCut(t, seatBox(t, sc, -4, -4, 6, 4, 4, 9), seatDisc(t, sc, 2.5, 5, 10)), turn)
		require.NoError(t, err)
		_, err = decad.Intersect(seatCopy(t, plate), seatCopy(t, block)).Do(t.Context())
		require.ErrorIs(t, err, decad.ErrUnsupported)
	})
}

// TestIntersectPlacedSeat intersects seated bodies after a placement (agent-docs/coplanar-contact-design.md §17). A
// turn about the bore's axis carries the floor's plane onto itself exactly, so the placed tags state a departure only
// for their rounded frames, and the seat still decides.
func TestIntersectPlacedSeat(t *testing.T) {
	t.Parallel()
	turned := func(t *testing.T, b *decad.Body, axis r3.Vec, deg float64) *decad.Body {
		t.Helper()
		turn, err := r3.Rotation(axis, units.Degrees(deg))
		require.NoError(t, err)
		out, err := decad.Transform(t.Context(), b, turn)
		require.NoError(t, err)
		return out
	}
	t.Run("a round nut turned about the bore's axis", func(t *testing.T) {
		t.Parallel()
		sc := newSeatScene()
		plate := turned(t, seatPlate(t, sc), r3.NewVec(0, 0, 1), 25)
		nut := turned(t, tcCut(t, seatDisc(t, sc, 4.5, 6, 9.5), seatDisc(t, sc, 2.5, 5, 10)), r3.NewVec(0, 0, 1), 70)
		requireSeatEmpty(t, nut, plate)
	})
	t.Run("mesh-built bodies turned about the bore's axis", func(t *testing.T) {
		t.Parallel()
		// A tilted hole away from the seat sends each Cut down the mesh path, so both bodies hold faceted records
		// whose placed groups carry the proved plane (Group.PlacedPlane).
		sc := newSeatScene()
		tilted := func(t *testing.T, cx, cy, r, z0 float64) *decad.Body {
			t.Helper()
			tool := sc.prism(t, r3.NewVec(0, 0, z0), tcX, tcY, []tcSeg{
				tcArc(cx, cy, cx+r, cy, cx-r, cy), tcArc(cx, cy, cx-r, cy, cx+r, cy),
			}, tcAlong(20))
			return turned(t, tool, r3.NewVec(1, 0, 0), 10)
		}
		plate := tcCut(t, seatPlate(t, sc), tilted(t, 14, 14, 1, -5))
		nut := tcCut(t, tcCut(t, seatDisc(t, sc, 4.5, 6, 9.5), seatDisc(t, sc, 2.5, 5, 10)), tilted(t, 3.5, 0, 0.4, 7.5))
		require.Equal(t, "decad.facetedPayload", decadtest.PayloadClass(plate))
		require.Equal(t, "decad.facetedPayload", decadtest.PayloadClass(nut))
		requireSeatEmpty(t, turned(t, nut, r3.NewVec(0, 0, 1), 70), turned(t, plate, r3.NewVec(0, 0, 1), 25))
	})
	// Rejected: a move too small to change a held float still moves the true plane. The plate below sits 10⁻³⁰ higher
	// than its held floor, so the block overlaps it by that much, and no exact reading of the held floats decides it.
	t.Run("a move below the floats' resolution refuses", func(t *testing.T) {
		t.Parallel()
		sc := newSeatScene()
		lift, err := r3.Translation(r3.NewVec(0, 0, 1e-30))
		require.NoError(t, err)
		plate, err := decad.Transform(t.Context(), seatPlate(t, sc), lift)
		require.NoError(t, err)
		block := tcCut(t, seatBox(t, sc, -4, -4, 6, 4, 4, 9), seatDisc(t, sc, 2.5, 5, 10))
		for _, ops := range [][2]*decad.Body{{block, plate}, {plate, block}} {
			_, err := decad.Intersect(seatCopy(t, ops[0]), seatCopy(t, ops[1])).Do(t.Context())
			require.ErrorIs(t, err, decad.ErrUnsupported)
		}
	})
	// Rejected: a turn that tilts the floor leaves its true plane where no held float lies, so no exact reading
	// decides the contact.
	t.Run("turned about a horizontal axis refuses", func(t *testing.T) {
		t.Parallel()
		sc := newSeatScene()
		plate := turned(t, seatPlate(t, sc), r3.NewVec(1, 0, 0), 31)
		block := turned(t, tcCut(t, seatBox(t, sc, -4, -4, 6, 4, 4, 9), seatDisc(t, sc, 2.5, 5, 10)), r3.NewVec(1, 0, 0), 31)
		_, err := decad.Intersect(seatCopy(t, block), seatCopy(t, plate)).Do(t.Context())
		require.ErrorIs(t, err, decad.ErrUnsupported)
	})
}

// TestIntersectSeatWithinGapTolerance intersects seats the exact path cannot decide, in a document with the default
// gap tolerance (agent-docs/gap-tolerance-design.md §5.2 items 7 and 8). The retry moves the second operand off the
// contact, and the answer must not depend on which operand comes first.
func TestIntersectSeatWithinGapTolerance(t *testing.T) {
	t.Parallel()
	turned := func(t *testing.T, b *decad.Body) *decad.Body {
		t.Helper()
		turn, err := r3.Rotation(r3.NewVec(1, 0, 0), units.Degrees(31))
		require.NoError(t, err)
		out, err := decad.Transform(t.Context(), b, turn)
		require.NoError(t, err)
		return out
	}
	t.Run("a turned bored block on the floor, both orders", func(t *testing.T) {
		t.Parallel()
		sc := newTCScene()
		plate := turned(t, seatPlate(t, sc))
		block := turned(t, tcCut(t, seatBox(t, sc, -4, -4, 6, 4, 4, 9), seatDisc(t, sc, 2.5, 5, 10)))
		requireSeatEmpty(t, block, plate)
	})
	t.Run("a turned round nut on the floor, both orders", func(t *testing.T) {
		t.Parallel()
		sc := newTCScene()
		plate := turned(t, seatPlate(t, sc))
		nut := turned(t, tcCut(t, seatDisc(t, sc, 4.5, 6, 9.5), seatDisc(t, sc, 2.5, 5, 10)))
		requireSeatEmpty(t, nut, plate)
	})
	// Rejected: a move within the tolerance never hides a real overlap, and the swapped retry never answers with a
	// body: a block sunk 0.05 into the floor, deeper than the 0.01 reach, overlaps it in both orders.
	t.Run("a turned block sunk 0.05 still overlaps", func(t *testing.T) {
		t.Parallel()
		sc := newTCScene()
		plate := turned(t, seatPlate(t, sc))
		block := turned(t, tcCut(t, seatBox(t, sc, -4, -4, 5.95, 4, 4, 9), seatDisc(t, sc, 2.5, 5, 10)))
		for _, ops := range [][2]*decad.Body{{block, plate}, {plate, block}} {
			out, err := decad.Intersect(seatCopy(t, ops[0]), seatCopy(t, ops[1])).Do(t.Context())
			require.NoError(t, err)
			v, err := out.Volume()
			require.NoError(t, err)
			want := (64 - math.Pi*2.8*2.8) * 0.05
			require.InDelta(t, want, v.Value.Base(), v.Bound.Base()+1e-3)
		}
	})
}
