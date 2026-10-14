package loftcylinder

import (
	"context"
	"errors"
	"fmt"
	"math"
	"math/big"
	"math/rand/v2"
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// legacyRingWallClear is the body RingWallClear had before it read walls in
// place. It copies each RingWall by value and is the oracle for the
// differential tests below.
func legacyRingWallClear(ctx context.Context, a, b RingVec,
	box RingBox, bound2 *big.Rat, walls []RingWall,
	result *RingOccupancy) error {
	for wallIndex, wall := range walls {
		if !ringBoxesMeet(box, wall.Box) {
			continue
		}
		result.Candidates++
		if result.Candidates > 1000000 || result.Candidates%256 == 0 && ctx.Err() != nil {
			if err := ctx.Err(); err != nil {
				return err
			}
			return fmt.Errorf("%w: ring wall clearance work limit", decaderr.ErrUnsupported)
		}
		if legacyRingPlaneApart(a, b, wall, bound2) {
			continue
		}
		q := ringSegmentTriangle2(a, b, wall.Points)
		if q == nil || q.Cmp(bound2) <= 0 {
			gap2 := math.NaN()
			if q != nil {
				gap2, _ = q.Float64()
			}
			bound2, _ := bound2.Float64()
			return fmt.Errorf("%w: ring may meet true Loft wall %d (gap²≈%g, bound²≈%g)",
				decaderr.ErrUnsupported, wallIndex, gap2, bound2)
		}
		if result.MinimumClearance2 == nil || q.Cmp(result.MinimumClearance2) < 0 {
			result.MinimumClearance2 = q
		}
	}
	return nil
}

// legacyRingPlaneApart is the body ringPlaneApart had when it compared
// normalised big.Rat values.
func legacyRingPlaneApart(a, b RingVec, wall RingWall, bound2 *big.Rat) bool {
	if bound2 == nil || bound2.Sign() <= 0 || wall.Normal2 == nil ||
		wall.Normal2.Sign() == 0 || wall.Plane0 == nil {
		return false
	}
	da := proofarith.SubRat(new(big.Rat), RingDot(a, wall.Normal), wall.Plane0)
	db := proofarith.SubRat(new(big.Rat), RingDot(b, wall.Normal), wall.Plane0)
	if da.Sign() == 0 || da.Sign() != db.Sign() {
		return false
	}
	if da.Sign() < 0 {
		da.Neg(da)
		db.Neg(db)
	}
	if db.Cmp(da) < 0 {
		da = db
	}
	return proofarith.MulRat(new(big.Rat), da, da).Cmp(proofarith.MulRat(new(big.Rat), bound2, wall.Normal2)) > 0
}

func newTestRingWall(tri [3]r3.Vec, displacement float64) RingWall {
	wall := RingWall{
		Points: [3]RingVec{RingVertex(tri[0]), RingVertex(tri[1]), RingVertex(tri[2])},
		Box:    RingTriangleBox(tri, displacement),
	}
	wall.Normal = RingCross(RingSub(wall.Points[1], wall.Points[0]),
		RingSub(wall.Points[2], wall.Points[0]))
	wall.Normal2 = RingDot(wall.Normal, wall.Normal)
	wall.Plane0 = RingDot(wall.Points[0], wall.Normal)
	return wall
}

func cloneRingOccupancy(o RingOccupancy) RingOccupancy {
	if o.MinimumClearance2 != nil {
		o.MinimumClearance2 = new(big.Rat).Set(o.MinimumClearance2)
	}
	return o
}

// requireSameRingWallClear runs the legacy and current bodies on separate
// copies of start and requires equal errors, candidates and clearance.
func requireSameRingWallClear(t *testing.T, ctx context.Context, a, b RingVec,
	bound2 *big.Rat, walls []RingWall, start RingOccupancy) (RingOccupancy, error) {
	t.Helper()
	box := RingEdgeBox(a, b)
	oldRes, newRes := cloneRingOccupancy(start), cloneRingOccupancy(start)
	oldErr := legacyRingWallClear(ctx, a, b, box, bound2, walls, &oldRes)
	newErr := RingWallClear(ctx, a, b, box, bound2, walls, &newRes)
	if oldErr == nil {
		require.NoError(t, newErr)
	} else {
		require.Error(t, newErr)
		require.Equal(t, oldErr.Error(), newErr.Error())
		require.Equal(t, errors.Is(oldErr, decaderr.ErrUnsupported), errors.Is(newErr, decaderr.ErrUnsupported))
		require.Equal(t, errors.Is(oldErr, context.Canceled), errors.Is(newErr, context.Canceled))
	}
	require.Equal(t, oldRes.Candidates, newRes.Candidates)
	require.Equal(t, oldRes.Inner, newRes.Inner)
	require.Equal(t, oldRes.Outer, newRes.Outer)
	require.Equal(t, oldRes.Seam, newRes.Seam)
	if oldRes.MinimumClearance2 == nil {
		require.Nil(t, newRes.MinimumClearance2)
	} else {
		require.NotNil(t, newRes.MinimumClearance2)
		require.Zero(t, oldRes.MinimumClearance2.Cmp(newRes.MinimumClearance2))
	}
	return newRes, newErr
}

func TestRingWallClearNamesFirstRefusingWall(t *testing.T) {
	// The segment crosses z = 0 at the origin. Wall 7 is far away and
	// passes, wall 23 lies 0.1 from the crossing point and refuses, and
	// wall 30 refuses too but must not be reached.
	far := [3]r3.Vec{{X: 5, Y: 0, Z: 0}, {X: 6, Y: 0, Z: 0}, {X: 5, Y: 1, Z: 0}}
	near := [3]r3.Vec{{X: 0.1, Y: 0, Z: 0}, {X: 1, Y: 0, Z: 0}, {X: 0.1, Y: 1, Z: 0}}
	nearer := [3]r3.Vec{{X: 0.05, Y: 0, Z: 0}, {X: 1, Y: 0, Z: 0}, {X: 0.05, Y: 1, Z: 0}}
	elsewhere := [3]r3.Vec{{X: 40, Y: 40, Z: 40}, {X: 41, Y: 40, Z: 40}, {X: 40, Y: 41, Z: 40}}
	walls := make([]RingWall, 40)
	for i := range walls {
		walls[i] = newTestRingWall(elsewhere, 0.5)
	}
	walls[7] = newTestRingWall(far, 10)
	walls[23] = newTestRingWall(near, 0.5)
	walls[30] = newTestRingWall(nearer, 0.5)

	a := RingVertex(r3.Vec{X: 0, Y: 0, Z: 1})
	b := RingVertex(r3.Vec{X: 0, Y: 0, Z: -1})
	bound2 := big.NewRat(1, 4)
	res, err := requireSameRingWallClear(t, t.Context(), a, b, bound2, walls, RingOccupancy{})
	require.ErrorIs(t, err, decaderr.ErrUnsupported)
	require.ErrorContains(t, err, "true Loft wall 23 ")
	require.Equal(t, 2, res.Candidates)
	require.NotNil(t, res.MinimumClearance2)
	require.Zero(t, res.MinimumClearance2.Cmp(big.NewRat(25, 1)))

	// Without the refusing walls the whole list passes and wall 7's
	// clearance is the minimum.
	walls[23] = newTestRingWall(elsewhere, 0.5)
	walls[30] = newTestRingWall(elsewhere, 0.5)
	res, err = requireSameRingWallClear(t, t.Context(), a, b, bound2, walls, RingOccupancy{})
	require.NoError(t, err)
	require.Equal(t, 1, res.Candidates)
	require.Zero(t, res.MinimumClearance2.Cmp(big.NewRat(25, 1)))
}

func TestRingWallClearLastWallRefuses(t *testing.T) {
	near := [3]r3.Vec{{X: 0.2, Y: 0, Z: 0}, {X: 1, Y: 0, Z: 0}, {X: 0.2, Y: 1, Z: 0}}
	walls := []RingWall{newTestRingWall(near, 0.5), newTestRingWall(near, 0.5), newTestRingWall(near, 0.5)}
	// Only the last wall refuses: it touches the segment at the origin, and
	// the first two are 0.2 away and the bound is 0.1.
	touching := [3]r3.Vec{{X: 0, Y: 0, Z: 0}, {X: 1, Y: 0, Z: 0}, {X: 0, Y: 1, Z: 0}}
	walls[2] = newTestRingWall(touching, 0.5)
	a := RingVertex(r3.Vec{X: 0, Y: 0, Z: 1})
	b := RingVertex(r3.Vec{X: 0, Y: 0, Z: -1})
	res, err := requireSameRingWallClear(t, t.Context(), a, b, big.NewRat(1, 100), walls, RingOccupancy{})
	require.ErrorContains(t, err, "true Loft wall 2 ")
	require.Equal(t, 3, res.Candidates)
	require.Zero(t, res.MinimumClearance2.Cmp(ringSegmentTriangle2(a, b, walls[0].Points)))
}

func TestRingWallClearEmptyAndSingle(t *testing.T) {
	a := RingVertex(r3.Vec{X: 0, Y: 0, Z: 1})
	b := RingVertex(r3.Vec{X: 0, Y: 0, Z: -1})
	bound2 := big.NewRat(1, 4)

	res, err := requireSameRingWallClear(t, t.Context(), a, b, bound2, nil, RingOccupancy{})
	require.NoError(t, err)
	require.Zero(t, res.Candidates)
	require.Nil(t, res.MinimumClearance2)

	res, err = requireSameRingWallClear(t, t.Context(), a, b, bound2, []RingWall{}, RingOccupancy{})
	require.NoError(t, err)
	require.Zero(t, res.Candidates)

	far := [3]r3.Vec{{X: 5, Y: 0, Z: 0}, {X: 6, Y: 0, Z: 0}, {X: 5, Y: 1, Z: 0}}
	res, err = requireSameRingWallClear(t, t.Context(), a, b, bound2,
		[]RingWall{newTestRingWall(far, 10)}, RingOccupancy{})
	require.NoError(t, err)
	require.Equal(t, 1, res.Candidates)
	require.Zero(t, res.MinimumClearance2.Cmp(big.NewRat(25, 1)))

	// A wall that only touches the edge box at its boundary is a candidate.
	edgeTouch := newTestRingWall([3]r3.Vec{{X: 0, Y: 0, Z: 3}, {X: 1, Y: 0, Z: 3}, {X: 0, Y: 1, Z: 3}}, 0)
	edgeTouch.Box[2] = [2]float64{1, 2}
	res, _ = requireSameRingWallClear(t, t.Context(), a, b, bound2, []RingWall{edgeTouch}, RingOccupancy{})
	require.Equal(t, 1, res.Candidates)

	// A wall whose box just misses the edge box is skipped.
	edgeTouch.Box[2] = [2]float64{math.Nextafter(1, 2), 2}
	res, err = requireSameRingWallClear(t, t.Context(), a, b, bound2, []RingWall{edgeTouch}, RingOccupancy{})
	require.NoError(t, err)
	require.Zero(t, res.Candidates)
}

func TestRingWallClearDegenerateAndNilBound(t *testing.T) {
	a := RingVertex(r3.Vec{X: 0, Y: 0, Z: 1})
	b := RingVertex(r3.Vec{X: 0, Y: 0, Z: -1})
	collinear := newTestRingWall([3]r3.Vec{{X: 0, Y: 0, Z: 0}, {X: 1, Y: 0, Z: 0}, {X: 2, Y: 0, Z: 0}}, 1)
	require.Zero(t, collinear.Normal2.Sign())
	point := newTestRingWall([3]r3.Vec{{X: 0, Y: 0, Z: 0}, {X: 0, Y: 0, Z: 0}, {X: 0, Y: 0, Z: 0}}, 1)
	noPlane := newTestRingWall([3]r3.Vec{{X: 1, Y: 0, Z: 0}, {X: 2, Y: 0, Z: 0}, {X: 1, Y: 1, Z: 0}}, 1)
	noPlane.Plane0 = nil
	noNormal2 := newTestRingWall([3]r3.Vec{{X: 1, Y: 0, Z: 0}, {X: 2, Y: 0, Z: 0}, {X: 1, Y: 1, Z: 0}}, 1)
	noNormal2.Normal2 = nil
	ok := newTestRingWall([3]r3.Vec{{X: 5, Y: 0, Z: 0}, {X: 6, Y: 0, Z: 0}, {X: 5, Y: 1, Z: 0}}, 10)
	for name, walls := range map[string][]RingWall{
		"collinear first": {collinear, ok},
		"collinear last":  {ok, collinear},
		"point":           {ok, point, ok},
		"no plane":        {noPlane},
		"no normal2":      {noNormal2, ok},
	} {
		for _, bound2 := range []*big.Rat{big.NewRat(1, 4), big.NewRat(0, 1), big.NewRat(-1, 4)} {
			t.Run(name, func(t *testing.T) {
				_, _ = requireSameRingWallClear(t, t.Context(), a, b, bound2, walls, RingOccupancy{})
			})
		}
	}
	// A wall whose box misses the edge box never reaches the bound.
	res, err := requireSameRingWallClear(t, t.Context(), a, b, big.NewRat(1, 4),
		[]RingWall{newTestRingWall([3]r3.Vec{{X: 90, Y: 0, Z: 0}, {X: 91, Y: 0, Z: 0}, {X: 90, Y: 1, Z: 0}}, 0)},
		RingOccupancy{})
	require.NoError(t, err)
	require.Zero(t, res.Candidates)
}

func TestRingWallClearWorkLimitAndCancellation(t *testing.T) {
	a := RingVertex(r3.Vec{X: 0, Y: 0, Z: 1})
	b := RingVertex(r3.Vec{X: 0, Y: 0, Z: -1})
	far := newTestRingWall([3]r3.Vec{{X: 5, Y: 0, Z: 0}, {X: 6, Y: 0, Z: 0}, {X: 5, Y: 1, Z: 0}}, 10)
	walls := make([]RingWall, 600)
	for i := range walls {
		walls[i] = far
	}
	bound2 := big.NewRat(1, 4)

	_, err := requireSameRingWallClear(t, t.Context(), a, b, bound2, walls, RingOccupancy{Candidates: 999999})
	require.ErrorIs(t, err, decaderr.ErrUnsupported)
	require.ErrorContains(t, err, "work limit")

	ctx, cancel := context.WithCancel(t.Context())
	cancel()
	res, err := requireSameRingWallClear(t, ctx, a, b, bound2, walls, RingOccupancy{})
	require.ErrorIs(t, err, context.Canceled)
	require.Equal(t, 256, res.Candidates)

	// Below the poll period a cancelled context is not consulted.
	res, err = requireSameRingWallClear(t, ctx, a, b, bound2, walls[:200], RingOccupancy{})
	require.NoError(t, err)
	require.Equal(t, 200, res.Candidates)
}

func TestRingWallClearMatchesLegacyOnRandomRings(t *testing.T) {
	rng := rand.New(rand.NewPCG(7, 11))
	coord := func(span float64) float64 { return float64(rng.IntN(13)-6) / 4 * span }
	vec := func(span float64) r3.Vec { return r3.Vec{X: coord(span), Y: coord(span), Z: coord(span)} }
	refused, passed, minimums := 0, 0, 0
	for range 300 {
		walls := make([]RingWall, rng.IntN(70))
		for i := range walls {
			tri := [3]r3.Vec{vec(1), vec(1), vec(1)}
			if rng.IntN(8) == 0 {
				tri[2] = tri[1] // degenerate wall
			}
			walls[i] = newTestRingWall(tri, float64(rng.IntN(3))/4)
		}
		a, b := RingVertex(vec(1)), RingVertex(vec(1))
		bound2 := big.NewRat(int64(rng.IntN(4)), 16)
		res, err := requireSameRingWallClear(t, t.Context(), a, b, bound2, walls, RingOccupancy{})
		if err != nil {
			refused++
		} else {
			passed++
		}
		if res.MinimumClearance2 != nil {
			minimums++
		}
	}
	require.Positive(t, refused)
	require.Positive(t, passed)
	require.Positive(t, minimums)
}

// mutatingContext runs hook on every Err call, as a context that edits the
// caller's wall list while the poll runs.
//
//nolint:containedctx // test double that wraps a context to override Err
type mutatingContext struct {
	context.Context
	hook func()
}

func (c mutatingContext) Err() error {
	c.hook()
	return nil
}

func TestRingWallClearReadsWallBeforePoll(t *testing.T) {
	a := RingVertex(r3.Vec{X: 0, Y: 0, Z: 1})
	b := RingVertex(r3.Vec{X: 0, Y: 0, Z: -1})
	bound2 := big.NewRat(1, 4)
	far := newTestRingWall([3]r3.Vec{{X: 5, Y: 0, Z: 0}, {X: 6, Y: 0, Z: 0}, {X: 5, Y: 1, Z: 0}}, 10)
	touching := newTestRingWall([3]r3.Vec{{X: 0, Y: 0, Z: 0}, {X: 1, Y: 0, Z: 0}, {X: 0, Y: 1, Z: 0}}, 10)
	elsewhere := newTestRingWall([3]r3.Vec{{X: 40, Y: 40, Z: 40}, {X: 41, Y: 40, Z: 40}, {X: 40, Y: 41, Z: 40}}, 0.5)
	build := func() []RingWall {
		walls := make([]RingWall, 300)
		for i := range walls {
			walls[i] = far
		}
		walls[299] = elsewhere
		return walls
	}
	box := RingEdgeBox(a, b)
	// The 256th candidate is walls[255], where the poll fires first.
	for name, mutate := range map[string]func(walls []RingWall){
		"next candidate":    func(walls []RingWall) { walls[256].Points = touching.Points },
		"current wall":      func(walls []RingWall) { walls[255].Points = touching.Points },
		"earlier candidate": func(walls []RingWall) { walls[3].Points = touching.Points },
		"non candidate":     func(walls []RingWall) { walls[299].Points = touching.Points },
		"non candidate box": func(walls []RingWall) { walls[299].Box = far.Box },
		"next candidate box": func(walls []RingWall) {
			walls[256].Box = elsewhere.Box
		},
	} {
		t.Run(name, func(t *testing.T) {
			oldWalls, newWalls := build(), build()
			var oldRes, newRes RingOccupancy
			oldErr := legacyRingWallClear(mutatingContext{t.Context(), func() { mutate(oldWalls) }},
				a, b, box, bound2, oldWalls, &oldRes)
			newErr := RingWallClear(mutatingContext{t.Context(), func() { mutate(newWalls) }},
				a, b, box, bound2, newWalls, &newRes)
			if oldErr == nil {
				require.NoError(t, newErr)
			} else {
				require.EqualError(t, newErr, oldErr.Error())
			}
			require.Equal(t, oldRes.Candidates, newRes.Candidates)
			if oldRes.MinimumClearance2 == nil {
				require.Nil(t, newRes.MinimumClearance2)
			} else {
				require.NotNil(t, newRes.MinimumClearance2)
				require.Zero(t, oldRes.MinimumClearance2.Cmp(newRes.MinimumClearance2))
			}
		})
	}
}
