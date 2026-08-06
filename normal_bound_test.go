package decad_test

import (
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file is Face.NormalAt's own exactness contract (topology.go,
// normal_bound.go). The reading is a DIRECTION, so it is Exact only where the
// float triple handed back really is the exact unit normal of the surface the
// face is tagged with — and two ordinary constructions leave it not being one:
// a rotated placement, whose cap frame normal is the rounded image of a
// rotation and so is no longer of unit length, and any cone at all, whose
// normal is assembled from a float cosine and sine that do not satisfy
// cos² + sin² = 1.

// normalFaceByRole finds the one face carrying the named feature role.
func normalFaceByRole(t *testing.T, b *decad.Body, role string) *decad.Face {
	t.Helper()
	for _, f := range b.Faces() {
		for _, origin := range f.Origins() {
			if origin.Role == role {
				return f
			}
		}
	}
	require.FailNow(t, `no face carries the role`, role)
	return nil
}

// unitDefect is the exact |1 − |n|| of a held direction, computed over
// big.Rat so no float step can hide it. A normal reading whose own bound is
// below this number is understating: no unit vector sits closer to n than
// that.
func unitDefect(n r3.Vec) *big.Rat {
	sum := new(big.Rat)
	for _, c := range []float64{n.X, n.Y, n.Z} {
		r := new(big.Rat).SetFloat64(c)
		sum.Add(sum, new(big.Rat).Mul(r, r))
	}
	// |1 − |n|| = |1 − |n|²| / (1 + |n|). For a near-unit vector the divisor
	// is just under two, so halving the squared defect UNDERSTATES the true
	// distance slightly — which is what this assertion wants: a lower bound
	// the published bound must still clear.
	defect := new(big.Rat).Sub(sum, big.NewRat(1, 1))
	defect.Abs(defect)
	return defect.Quo(defect, big.NewRat(2, 1))
}

func TestNormalAtExactnessIsProven(t *testing.T) {
	t.Run(`a rotated placement's cap normal is Approximate`, func(t *testing.T) {
		s, p := plateSketch(t)
		doc := decad.New()
		body, err := doc.Extrude(s, p, decad.Distance{D: units.Millimeters(8), Dir: decad.Along})
		require.NoError(t, err)
		rotation, err := r3.Rotation(r3.NewVec(1, 2, 3), units.Degrees(37))
		require.NoError(t, err)
		placed, err := body.Placed(rotation)
		require.NoError(t, err)

		for _, role := range []string{roleCapStart, roleCapEnd} {
			f := normalFaceByRole(t, placed, role)
			at := f.Surface().(decad.Plane).Frame.Origin()
			n, err := f.NormalAt(at)
			require.NoError(t, err)
			require.Equal(t, decad.Approximate, n.Exactness, `%s: a rounded rotation image is not an exact unit normal`, role)
			bound, err := n.Bound.In(units.One)
			require.NoError(t, err)
			require.Positive(t, bound, `%s: the bound must be positive`, role)

			// The bound is not merely nonzero: it covers the held vector's own
			// distance from the nearest unit vector, which is what disqualifies
			// it from being any surface's exact normal.
			defect, _ := unitDefect(n.Value).Float64()
			require.Positive(t, defect, `%s: the held normal is not of unit length`, role)
			require.GreaterOrEqual(t, bound, defect, `%s: the bound must cover the unit-length defect`, role)
		}
	})

	t.Run(`an unplaced cone normal is Approximate`, func(t *testing.T) {
		wall, cone := coneWall(t)
		require.True(t, cone.HalfAngle.Equal(units.Degrees(45), 1e-12), `the half angle is a whole number of degrees`)

		at := r3.NewVec(5, 5, 0)
		n, err := wall.NormalAt(at)
		require.NoError(t, err)
		// The cone opens toward +u from its apex at the origin, so the outward
		// normal leans away from the axis and back toward the apex.
		require.InDelta(t, -1/math.Sqrt(2), n.Value.X, 1e-9)
		require.InDelta(t, 1/math.Sqrt(2), n.Value.Y, 1e-9)
		require.Equal(t, decad.Approximate, n.Exactness, `a float cosine and sine do not build an exact unit normal`)
		bound, err := n.Bound.In(units.One)
		require.NoError(t, err)
		require.Positive(t, bound)

		defect, _ := unitDefect(n.Value).Float64()
		require.Positive(t, defect, `cos² + sin² off the float library is not one`)
		require.GreaterOrEqual(t, bound, defect, `the bound must cover the unit-length defect`)
		// The reading is still a usable direction: the overclaim was the
		// exactness, never the value.
		require.Less(t, bound, 1e-14, `the bound stays at rounding scale`)
	})

	t.Run(`a normal exact under the lift stays Exact`, func(t *testing.T) {
		s, p := plateSketch(t)
		doc := decad.New()
		body, err := doc.Extrude(s, p, decad.Distance{D: units.Millimeters(8), Dir: decad.Along})
		require.NoError(t, err)
		shift, err := r3.Translation(r3.NewVec(1000, -250, 80))
		require.NoError(t, err)
		placed, err := body.Placed(shift)
		require.NoError(t, err)

		// A translation moves no direction at all, and the plate's own cap
		// frame normal is an axis vector, so the lift reproduces it exactly.
		f := normalFaceByRole(t, placed, roleCapEnd)
		n, err := f.NormalAt(f.Surface().(decad.Plane).Frame.Origin())
		require.NoError(t, err)
		require.Equal(t, decad.Exact, n.Exactness)
		bound, err := n.Bound.In(units.One)
		require.NoError(t, err)
		require.Zero(t, bound)
		require.Equal(t, r3.NewVec(0, 0, 1), n.Value)

		// Every side wall of the same plate is axis aligned too, so the whole
		// unplaced body reads Exact rather than only its caps.
		unplacedExact := 0
		for _, face := range placed.Faces() {
			m, err := face.NormalAt(face.Surface().(decad.Plane).Frame.Origin())
			require.NoError(t, err)
			require.Equal(t, decad.Exact, m.Exactness)
			unplacedExact++
		}
		require.Equal(t, 6, unplacedExact)
	})
}

// coneWall revolves a 45° right triangle about the u axis, giving one Cone
// wall whose half angle is a whole number of degrees and whose body no
// placement ever touched.
func coneWall(t *testing.T) (*decad.Face, decad.Cone) {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	o := s.CreatePoint(0, 0)
	base := s.CreatePoint(10, 0)
	top := s.CreatePoint(10, 10)
	s.Fix(o)
	s.Fix(base)
	s.Fix(top)
	s.CreateLine(o, base)
	s.CreateLine(base, top)
	s.CreateLine(top, o)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)

	doc := decad.New()
	body, err := doc.Revolve(s, s.Profiles()[0], uAxis, decad.FullRevolution{})
	require.NoError(t, err)
	for _, f := range body.Faces() {
		if c, ok := f.Surface().(decad.Cone); ok {
			return f, c
		}
	}
	require.FailNow(t, `the revolve built no cone wall`)
	return nil, decad.Cone{}
}
