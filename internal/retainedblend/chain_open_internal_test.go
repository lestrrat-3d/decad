package retainedblend

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func TestCornerTurn(t *testing.T) {
	t.Parallel()
	x, y, z := r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0), r3.NewVec(0, 0, 1)

	// A loop that enters a corner along the +y line, travelling toward -y,
	// and leaves along +x turns left about +z: the face is convex there.
	require.Equal(t, 1, cornerTurn(y, x, z), "a convex corner")
	require.Equal(t, -1, cornerTurn(x, y, z), "a reflex corner")
	require.Equal(t, -1, cornerTurn(y, x, z.Scale(-1)), "the other side of the face")
	require.Equal(t, 0, cornerTurn(x, x.Scale(-1), z), "a straight run is undecided")
	require.Equal(t, 0, cornerTurn(x, r3.NewVec(1, 1e-12, 0), z), "lines too near parallel are undecided")
}

func TestCheckEndSense(t *testing.T) {
	t.Parallel()
	// G's loop leaves the first corner toward the base's side and enters the
	// last one from it.
	require.NoError(t, checkEndSense(&chainEndCut{}, false), "the first end")
	require.NoError(t, checkEndSense(&chainEndCut{prevP: true}, true), "the last end")
	for _, c := range []struct {
		name  string
		prevP bool
		last  bool
	}{
		{"the first end entered from the base's side", true, false},
		{"the last end left toward the base's side", false, true},
	} {
		err := checkEndSense(&chainEndCut{prevP: c.prevP}, c.last)
		require.ErrorIs(t, err, decaderr.ErrUnsupported, c.name)
		require.ErrorContains(t, err, "an end face's loop runs its edge with the base the same way as the base's loop",
			c.name)
	}
}

func TestEndFaceTurn(t *testing.T) {
	t.Parallel()
	x, y, z := r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0), r3.NewVec(0, 0, 1)
	neg := func(v r3.Vec) r3.Vec { return v.Scale(-1) }
	cases := []struct {
		name string
		ec   chainEndCut
		want int
	}{
		// The tongue's x = 0 face, outward -x, at (0,0,20) and (0,20,20).
		{"the tongue's first end", chainEndCut{rP: y, rW: neg(z), g: plane{n: neg(x)}}, 1},
		{"the tongue's last end", chainEndCut{rP: neg(y), rW: neg(z), g: plane{n: neg(x)}, prevP: true}, 1},
		// The wall's x = 10 face, outward +x, at (10,10,10) and (10,30,10).
		{"the reflex foot's first end", chainEndCut{rP: y, rW: neg(z), g: plane{n: x}}, -1},
		{"the reflex foot's last end", chainEndCut{rP: neg(y), rW: neg(z), g: plane{n: x}, prevP: true}, -1},
	}
	for _, c := range cases {
		t.Run(c.name, func(t *testing.T) {
			t.Parallel()
			require.Equal(t, c.want, c.ec.endFaceTurn())
		})
	}
}

func TestCheckEndTurns(t *testing.T) {
	t.Parallel()
	x, y, z := r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0), r3.NewVec(0, 0, 1)
	neg := func(v r3.Vec) r3.Vec { return v.Scale(-1) }

	t.Run("a box corner's first end is convex in base and wall", func(t *testing.T) {
		t.Parallel()
		// The tongue's end at (0,0,20): the chain leaves along +x, G ∩ P runs
		// +y and G ∩ wall -z; the wall y = 0 faces -y.
		ec := &chainEndCut{rP: y, rW: neg(z), wallN: neg(y)}
		require.NoError(t, checkEndTurns(ec, x, z, true))
	})

	t.Run("a wall that wraps the corner is refused", func(t *testing.T) {
		t.Parallel()
		// The step's end at (10,0,10): G ∩ wall runs up the riser.
		ec := &chainEndCut{rP: y, rW: z, wallN: neg(y)}
		err := checkEndTurns(ec, x, z, true)
		require.ErrorIs(t, err, decaderr.ErrUnsupported)
		require.ErrorContains(t, err, "a chain end's base or wall is reflex at the corner")
	})

	t.Run("a base that wraps the corner is refused", func(t *testing.T) {
		t.Parallel()
		// The pocket mouth's last end at (10,10,10): the base loop enters
		// along the chain from +x; the wall y = 10 faces +y.
		ec := &chainEndCut{rP: y, rW: neg(z), wallN: y}
		err := checkEndTurns(ec, x, z, false)
		require.ErrorIs(t, err, decaderr.ErrUnsupported)
		require.ErrorContains(t, err, "a chain end's base or wall is reflex at the corner")
	})

	t.Run("an undecided turn is refused with its own text", func(t *testing.T) {
		t.Parallel()
		ec := &chainEndCut{rP: x, rW: neg(z), wallN: neg(y)}
		err := checkEndTurns(ec, x, z, true)
		require.ErrorIs(t, err, decaderr.ErrUnsupported)
		require.ErrorContains(t, err, "a chain end's base or wall turn at the corner is undecided")
	})

	t.Run("an arc end is read elsewhere", func(t *testing.T) {
		t.Parallel()
		ec := &chainEndCut{rP: y, rW: z, wallN: neg(y), arc: &arcEnd{}}
		require.NoError(t, checkEndTurns(ec, x, z, true))
	})
}

func TestChainEndCapArea(t *testing.T) {
	t.Parallel()

	const r, tP, tW = 1.0, 2.0, 0.5
	fillet := r * r * (1 - math.Pi/4)
	cases := []struct {
		name   string
		ec     chainEndCut
		fillet bool
		want   float64
	}{
		{"a convex end face gives up a fillet's cap", chainEndCut{}, true, fillet},
		{"a reflex end face takes a fillet's cap", chainEndCut{reflex: true}, true, -fillet},
		{"a convex end face gives up a chamfer's cap", chainEndCut{}, false, tP * tW / 2},
		{"a reflex end face takes a chamfer's cap", chainEndCut{reflex: true}, false, -tP * tW / 2},
		{"an arc end gives up its own corner", chainEndCut{arc: &arcEnd{cornerArea: 0.3}}, true, 0.3},
	}
	for _, c := range cases {
		t.Run(c.name, func(t *testing.T) {
			t.Parallel()
			require.InDelta(t, c.want, c.ec.capArea(c.fillet, r, tP, tW), 1e-15)
		})
	}
}
