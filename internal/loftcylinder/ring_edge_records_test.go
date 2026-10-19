package loftcylinder

import (
	"context"
	"errors"
	"fmt"
	"math/big"
	"math/rand/v2"
	"strings"
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// legacyRingEdgesInside is the body RingEdgesInside had before it shared
// per-edge records and compared fractions without cancelling. It recomputes
// every edge from its points, and is the oracle for the tests below.
func legacyRingEdgesInside(ctx context.Context, edges [][2]proofarith.Xpt, teeth []RingToothLevel,
	blankRadius, boreRadius float64) (RingOccupancy, error) {
	var result RingOccupancy
	refuse := func(reason string) (RingOccupancy, error) {
		return result, fmt.Errorf("%w: %s", decaderr.ErrUnsupported, reason)
	}
	ringXY := func(p RingVec) XY { return XY{X: p[0], Y: p[1]} }
	ringRadius2 := func(p RingVec) *big.Rat { return SourceDot(ringXY(p), ringXY(p)) }
	r2 := proofarith.MulRat(new(big.Rat), proofarith.FloatRat(blankRadius), proofarith.FloatRat(blankRadius))
	bore2 := proofarith.MulRat(new(big.Rat), proofarith.FloatRat(boreRadius), proofarith.FloatRat(boreRadius))
	witness := false
	var levelBoxes [][2]float64
	var seams []ringSeamLevel
	for edgeIndex, edge := range edges {
		if err := ctx.Err(); err != nil {
			return result, err
		}
		a, b := RingPoint(edge[0]), RingPoint(edge[1])
		amin, bmin := ringXY(a), ringXY(b)
		min2 := sourceRadiusMin2(amin, bmin)
		if min2.Cmp(bore2) <= 0 {
			return refuse("ring may reach the centered bore")
		}
		ra, rb := ringRadius2(a), ringRadius2(b)
		if ra.Cmp(r2) < 0 || rb.Cmp(r2) < 0 {
			witness = true
		}
		max2 := ra
		if rb.Cmp(max2) > 0 {
			max2 = rb
		}
		if max2.Cmp(r2) < 0 {
			result.Inner++
			continue
		}
		if min2.Cmp(r2) > 0 {
			result.Outer++
			if levelBoxes == nil {
				levelBoxes = make([][2]float64, len(teeth))
				for i := range teeth {
					levelBoxes[i] = [2]float64{
						proofbound.RatFloatDown(teeth[i].Level), proofbound.RatFloatUp(teeth[i].Level),
					}
				}
			}
			xy := RingEdgeBox(a, b)
			for toothIndex, tooth := range teeth {
				mappedA, mappedB := a, b
				mappedA[2], mappedB[2] = tooth.Level, tooth.Level
				box := xy
				box[2] = levelBoxes[toothIndex]
				if err := legacyRingWallClear(ctx, mappedA, mappedB,
					box, tooth.Bound2,
					tooth.Exterior, &result); err != nil {
					return result, fmt.Errorf("ring edge %d tooth %d from %v to %v at proxy z=%s: %w",
						edgeIndex, toothIndex, edge[0].Vec(), edge[1].Vec(), tooth.Level.RatString(), err)
				}
			}
			continue
		}
		result.Seam++
		if seams == nil {
			seams = make([]ringSeamLevel, len(teeth))
		}
		sectorCount := 0
		for toothIndex, tooth := range teeth {
			mappedEdge := [2]proofarith.Xpt{
				RingMappedPoint(edge[0], tooth.Level),
				RingMappedPoint(edge[1], tooth.Level),
			}
			seam := &seams[toothIndex]
			if !seam.built {
				*seam = ringSeamLevelOf(tooth)
			}
			if seam.sectorOK && triangleInSector(seam.sector, tooth.Level, tooth.Level,
				[3]proofarith.Xpt{mappedEdge[0], mappedEdge[1], mappedEdge[1]}) {
				sectorCount++
			}
			for _, slice := range seam.connectors {
				if EdgesMeet(amin, bmin, slice[0], slice[1]) ||
					EdgesMeet(amin, bmin, slice[1], slice[2]) {
					return refuse("ring may touch a true connector wall")
				}
			}
			mappedA, mappedB := a, b
			mappedA[2], mappedB[2] = tooth.Level, tooth.Level
			if err := legacyRingWallClear(ctx, mappedA, mappedB,
				RingEdgeBox(mappedA, mappedB), tooth.Bound2,
				tooth.Fitted, &result); err != nil {
				return result, fmt.Errorf("ring seam edge %d tooth %d from %v to %v at proxy z=%s: %w",
					edgeIndex, toothIndex, edge[0].Vec(), edge[1].Vec(), tooth.Level.RatString(), err)
			}
		}
		if sectorCount != 1 {
			return refuse("ring circle crossing has no unique occupied tooth sector")
		}
	}
	if !witness {
		return refuse("ring has no strict blank-inside witness")
	}
	return result, nil
}

// requireSameOutcome requires equal errors, counts and clearances.
func requireSameOutcome(t *testing.T, want RingOccupancy, wantErr error, got RingOccupancy, gotErr error) {
	t.Helper()
	if wantErr == nil {
		require.NoError(t, gotErr)
	} else {
		require.Error(t, gotErr)
		require.Equal(t, wantErr.Error(), gotErr.Error())
		require.Equal(t, errors.Is(wantErr, decaderr.ErrUnsupported), errors.Is(gotErr, decaderr.ErrUnsupported))
		require.Equal(t, errors.Is(wantErr, context.Canceled), errors.Is(gotErr, context.Canceled))
	}
	require.Equal(t, want.Inner, got.Inner)
	require.Equal(t, want.Outer, got.Outer)
	require.Equal(t, want.Seam, got.Seam)
	require.Equal(t, want.Candidates, got.Candidates)
	if want.MinimumClearance2 == nil {
		require.Nil(t, got.MinimumClearance2)
		return
	}
	require.NotNil(t, got.MinimumClearance2)
	require.Zero(t, want.MinimumClearance2.Cmp(got.MinimumClearance2))
}

// adversarialRat draws a rational from the values that break comparisons:
// zero, small integers of both signs, dyadics of mixed scale, and odd
// denominators, some with a huge numerator or denominator.
func adversarialRat(rng *rand.Rand) *big.Rat {
	switch rng.IntN(7) {
	case 0:
		return new(big.Rat)
	case 1:
		return big.NewRat(int64(rng.IntN(9)-4), 1)
	case 2:
		return proofarith.FloatRat((rng.Float64()*2 - 1) * float64(int64(1)<<rng.IntN(30)))
	case 3:
		return proofarith.FloatRat((rng.Float64()*2 - 1) / float64(int64(1)<<rng.IntN(40)))
	case 4:
		return big.NewRat(int64(rng.IntN(2001)-1000), int64(2*rng.IntN(500)+1))
	case 5:
		num := new(big.Int).Lsh(big.NewInt(int64(rng.IntN(1<<20)+1)), uint(rng.IntN(150)))
		den := new(big.Int).Exp(big.NewInt(int64(2*rng.IntN(20)+3)), big.NewInt(int64(rng.IntN(30)+1)), nil)
		if rng.IntN(2) == 0 {
			num.Neg(num)
		}
		return new(big.Rat).SetFrac(num, den)
	default:
		den := new(big.Int).Lsh(big.NewInt(int64(rng.IntN(1<<20)+1)), uint(rng.IntN(150)))
		return new(big.Rat).SetFrac(big.NewInt(int64(rng.IntN(2001)-1000)), den)
	}
}

func TestSourceRadiusMin2LazyMatchesLegacy(t *testing.T) {
	rng := rand.New(rand.NewPCG(31, 37))
	compare := func(a, b XY) {
		t.Helper()
		want := sourceRadiusMin2(a, b)
		lazy := sourceRadiusMin2Lazy(a, b)
		require.Zero(t, want.Cmp(lazy.Rat()), "min2 of %v..%v", a, b)
		for _, bound := range []*big.Rat{
			want, new(big.Rat).Add(want, big.NewRat(1, 1<<30)), new(big.Rat).Sub(want, big.NewRat(1, 1<<30)),
			new(big.Rat), big.NewRat(-1, 3), adversarialRat(rng),
		} {
			require.Equal(t, want.Cmp(bound), proofarith.LazyCmp(lazy, proofarith.LazyOf(bound)),
				"min2 %v against %v", want, bound)
		}
	}
	xy := func(x, y *big.Rat) XY { return XY{x, y} }
	scale := func(p XY, k *big.Rat) XY {
		return XY{new(big.Rat).Mul(p.X, k), new(big.Rat).Mul(p.Y, k)}
	}
	for range 3000 {
		a, b := xy(adversarialRat(rng), adversarialRat(rng)), xy(adversarialRat(rng), adversarialRat(rng))
		compare(a, b)
		compare(a, a)
		compare(xy(new(big.Rat), new(big.Rat)), b)
		compare(a, xy(new(big.Rat), new(big.Rat)))
		// Collinear with the origin: b = a*(1-k) puts the closest point at
		// the end (k < 1), exactly at b (k = 1, the clamp boundary) or in
		// the interior at the origin (k > 1).
		for _, k := range []*big.Rat{
			big.NewRat(1, 2), big.NewRat(1, 1), big.NewRat(3, 2), big.NewRat(-1, 1), big.NewRat(2, 1), new(big.Rat),
		} {
			compare(a, scale(a, new(big.Rat).Sub(big.NewRat(1, 1), k)))
		}
		// A segment starting where the perpendicular foot is a itself.
		compare(a, xy(new(big.Rat).Add(a.X, a.Y.Neg(new(big.Rat).Set(a.Y))), new(big.Rat).Add(a.Y, a.X)))
	}
}

func TestRingPlaneApartMatchesLegacy(t *testing.T) {
	rng := rand.New(rand.NewPCG(41, 43))
	vec := func() RingVec { return RingVec{adversarialRat(rng), adversarialRat(rng), adversarialRat(rng)} }
	wallOf := func() RingWall {
		w := RingWall{Points: [3]RingVec{vec(), vec(), vec()}}
		switch rng.IntN(6) {
		case 0: // axis-aligned plane
			w.Points[1][2], w.Points[2][2] = w.Points[0][2], w.Points[0][2]
		case 1: // degenerate triangle
			w.Points[2] = w.Points[1]
		}
		w.Normal = RingCross(RingSub(w.Points[1], w.Points[0]), RingSub(w.Points[2], w.Points[0]))
		w.Normal2 = RingDot(w.Normal, w.Normal)
		w.Plane0 = RingDot(w.Points[0], w.Normal)
		switch rng.IntN(14) {
		case 0:
			w.Normal2 = nil
		case 1:
			w.Plane0 = nil
		case 2: // a zero squared normal that the normal itself does not show
			w.Normal2 = new(big.Rat)
		}
		return w
	}
	apart, notApart := 0, 0
	check := func(a, b RingVec, w RingWall, bound2 *big.Rat) {
		t.Helper()
		want := legacyRingPlaneApart(a, b, w, bound2)
		require.Equal(t, want, ringPlaneApart(ringSegmentPlaneOf(a, b, bound2), &w))
		if want {
			apart++
		} else {
			notApart++
		}
	}
	for range 4000 {
		w := wallOf()
		a, b := vec(), vec()
		if rng.IntN(3) == 0 {
			// Put both ends on one side of the plane at a chosen offset.
			shift := RingVec{adversarialRat(rng), adversarialRat(rng), adversarialRat(rng)}
			b = RingVec{
				new(big.Rat).Add(a[0], shift[0]), new(big.Rat).Add(a[1], shift[1]), new(big.Rat).Add(a[2], shift[2]),
			}
		}
		bounds := []*big.Rat{nil, new(big.Rat), big.NewRat(-1, 4), adversarialRat(rng), big.NewRat(1, 4)}
		if w.Normal2 != nil && w.Plane0 != nil && w.Normal2.Sign() != 0 {
			// The threshold itself: bound2 = min(|da|,|db|)^2 / |n|^2 is not
			// apart, and slightly below it is apart.
			da := new(big.Rat).Sub(RingDot(a, w.Normal), w.Plane0)
			db := new(big.Rat).Sub(RingDot(b, w.Normal), w.Plane0)
			if da.Sign() != 0 && da.Sign() == db.Sign() {
				da.Abs(da)
				db.Abs(db)
				if db.Cmp(da) < 0 {
					da = db
				}
				edge := new(big.Rat).Quo(new(big.Rat).Mul(da, da), w.Normal2)
				bounds = append(bounds, edge,
					new(big.Rat).Sub(edge, new(big.Rat).Quo(edge, big.NewRat(1<<20, 1))),
					new(big.Rat).Add(edge, new(big.Rat).Quo(edge, big.NewRat(1<<20, 1))))
			}
		}
		for _, bound2 := range bounds {
			check(a, b, w, bound2)
			check(b, a, w, bound2)
		}
	}
	require.Greater(t, apart, 500)
	require.Greater(t, notApart, 500)
}

// ringFixture is one random ring with teeth, over the circle of the seam
// level test so that seam edges meet real connectors.
type ringFixture struct {
	edges          [][2]proofarith.Xpt
	blank, bore    float64
	exterior, seam [][]RingWall
	bound2         []*big.Rat
}

func fixtureCircle() SourceCircle {
	xy := func(x, y, den int64) XY { return XY{big.NewRat(x, den), big.NewRat(y, den)} }
	up := Connector{a: [2]XY{xy(2, 0, 1), xy(5, 1, 3)}, d: [2]XY{xy(2, 0, 1), xy(5, 1, 3)},
		f: [2]XY{xy(7, 1, 1), xy(22, 5, 3)}}
	down := Connector{a: [2]XY{xy(2, 0, 1), xy(5, -1, 3)}, d: [2]XY{xy(2, 0, 1), xy(5, -1, 3)},
		f: [2]XY{xy(7, -1, 1), xy(22, -5, 3)}}
	return SourceCircle{Z0: big.NewRat(0, 1), Z1: big.NewRat(1, 1), SafeBottom: big.NewRat(0, 1),
		SafeTop: big.NewRat(1, 1), Radius: big.NewRat(7, 2), GapAxis: xy(1, 0, 1),
		Connector: [2]Connector{down, up}}
}

func newRingFixture(rng *rand.Rand, edgeCount, toothCount int) ringFixture {
	f := ringFixture{blank: 3.5, bore: 0.5}
	denominators := []int64{1, 2, 3, 4, 5, 6, 7, 8}
	point := func() proofarith.Xpt {
		w := denominators[rng.IntN(len(denominators))]
		scale := rng.IntN(3) + 1
		return proofarith.Xpt{
			X: big.NewInt(int64(rng.IntN(13)-6) * w * int64(scale) / 2),
			Y: big.NewInt(int64(rng.IntN(13)-6) * w * int64(scale) / 2),
			Z: big.NewInt(int64(rng.IntN(5)) * w / 2), W: big.NewInt(w),
		}
	}
	for range edgeCount {
		f.edges = append(f.edges, [2]proofarith.Xpt{point(), point()})
	}
	for range toothCount {
		var walls [2][]RingWall
		for k := range walls {
			for range rng.IntN(6) {
				tri := [3]r3.Vec{}
				for i := range tri {
					tri[i] = r3.Vec{
						X: float64(rng.IntN(25)-12) / 4, Y: float64(rng.IntN(25)-12) / 4, Z: float64(rng.IntN(9)-2) / 4,
					}
				}
				walls[k] = append(walls[k], newTestRingWall(tri, float64(rng.IntN(3))/8))
			}
		}
		f.exterior, f.seam = append(f.exterior, walls[0]), append(f.seam, walls[1])
		f.bound2 = append(f.bound2, big.NewRat(int64(rng.IntN(3)), 64))
	}
	return f
}

func (f ringFixture) teeth(rng *rand.Rand) []RingToothLevel {
	teeth := make([]RingToothLevel, len(f.exterior))
	for i := range teeth {
		teeth[i] = RingToothLevel{
			Circle: fixtureCircle(), Level: big.NewRat(int64(rng.IntN(7)+1), int64(rng.IntN(5)+8)),
			Bound2: f.bound2[i], Exterior: f.exterior[i], Fitted: f.seam[i],
		}
	}
	return teeth
}

// rewriteRingInPlace rewrites random X, Y and W integers of the fixture's
// edges through their existing pointers, and replaces random walls in place.
func rewriteRingInPlace(rng *rand.Rand, f ringFixture) {
	for _, edge := range f.edges {
		for _, p := range edge {
			if rng.IntN(3) != 0 {
				continue
			}
			w := int64(rng.IntN(8) + 1)
			p.X.SetInt64(int64(rng.IntN(13)-6) * w / 2)
			p.Y.SetInt64(int64(rng.IntN(13)-6) * w / 2)
			p.W.SetInt64(w)
		}
	}
	for _, list := range append(append([][]RingWall{}, f.exterior...), f.seam...) {
		for i := range list {
			if rng.IntN(3) != 0 {
				continue
			}
			tri := [3]r3.Vec{}
			for k := range tri {
				tri[k] = r3.Vec{
					X: float64(rng.IntN(25)-12) / 4, Y: float64(rng.IntN(25)-12) / 4, Z: float64(rng.IntN(9)-2) / 4,
				}
			}
			list[i] = newTestRingWall(tri, float64(rng.IntN(3))/8)
		}
	}
}

func TestRingEdgesInsideSharedMatchesLegacy(t *testing.T) {
	rng := rand.New(rand.NewPCG(53, 59))
	outcomes := map[string]int{}
	for range 600 {
		f := newRingFixture(rng, rng.IntN(4)+1, rng.IntN(3)+1)
		other := newRingFixture(rng, rng.IntN(4)+1, len(f.exterior))
		records := new(RingEdgeRecords)
		// The same records serve the direct check, then a reflected check at
		// other levels, then another ring, other radii, and the first again.
		steps := []struct {
			f           ringFixture
			blank, bore float64
		}{{f, f.blank, f.bore}, {f, f.blank, f.bore}, {other, f.blank, f.bore}, {f, 3.25, f.bore},
			{f, f.blank, 0.75}, {f, f.blank, f.bore}}
		for _, step := range steps {
			teeth := step.f.teeth(rng)
			want, wantErr := legacyRingEdgesInside(t.Context(), step.f.edges, teeth, step.blank, step.bore)
			got, gotErr := RingEdgesInsideShared(t.Context(), step.f.edges, teeth, step.blank, step.bore, records)
			requireSameOutcome(t, want, wantErr, got, gotErr)
			fresh, freshErr := RingEdgesInside(t.Context(), step.f.edges, teeth, step.blank, step.bore)
			requireSameOutcome(t, want, wantErr, fresh, freshErr)
			// Rewrite the ring and a wall in place before the next step.
			rewriteRingInPlace(rng, step.f)
			if wantErr == nil {
				outcomes["pass"]++
			}
			for _, phrase := range []string{"reach the centered bore", "no strict blank-inside witness",
				"no unique occupied tooth sector", "touch a true connector wall", "ring edge ", "ring seam edge "} {
				if wantErr != nil && strings.Contains(wantErr.Error(), phrase) {
					outcomes[phrase]++
				}
			}
			if want.Inner > 0 {
				outcomes["inner edge"]++
			}
			if want.Outer > 0 {
				outcomes["outer edge"]++
			}
			if want.Seam > 0 {
				outcomes["seam edge"]++
			}
			if want.MinimumClearance2 != nil {
				outcomes["clearance"]++
			}
		}
	}
	for _, key := range []string{"pass", "inner edge", "outer edge", "seam edge", "clearance",
		"reach the centered bore", "no strict blank-inside witness", "no unique occupied tooth sector",
		"touch a true connector wall", "ring edge ", "ring seam edge "} {
		require.Positive(t, outcomes[key], "%q never occurred in %v", key, outcomes)
	}
}

// TestRingEdgesInsideSharedEqualityBoundaries puts edge radii exactly on the
// bore and blank radii, where the strict and non-strict comparisons differ.
func TestRingEdgesInsideSharedEqualityBoundaries(t *testing.T) {
	pt := func(x, y, w int64) proofarith.Xpt {
		return proofarith.Xpt{X: big.NewInt(x), Y: big.NewInt(y), Z: big.NewInt(0), W: big.NewInt(w)}
	}
	// Radius 1/2 is the bore, 2 is the blank, 5/2 lies in between at 3-4-5.
	for name, edges := range map[string][][2]proofarith.Xpt{
		"edge touches bore":          {{pt(1, 0, 2), pt(1, 1, 1)}},
		"edge through bore":          {{pt(-1, 0, 2), pt(1, 1, 2)}},
		"edge ends on bore":          {{pt(2, 0, 4), pt(3, 4, 2)}},
		"edge ends on blank":         {{pt(2, 0, 1), pt(0, 1, 1)}, {pt(0, 1, 1), pt(-1, 0, 1)}},
		"vertex on blank only":       {{pt(2, 0, 1), pt(3, 0, 1)}},
		"inner on blank":             {{pt(0, 2, 1), pt(0, 1, 1)}},
		"both ends on blank":         {{pt(2, 0, 1), pt(0, 2, 1)}},
		"collinear through the bore": {{pt(1, 0, 1), pt(-1, 0, 1)}},
		"foot of perpendicular":      {{pt(3, 1, 1), pt(-3, 1, 1)}},
	} {
		for _, radii := range [][2]float64{{2, 0.5}, {2, 1}, {1, 0.5}, {2.5, 0.5}} {
			t.Run(name, func(t *testing.T) {
				teeth := []RingToothLevel{{Circle: fixtureCircle(), Level: big.NewRat(1, 2), Bound2: big.NewRat(1, 64)}}
				want, wantErr := legacyRingEdgesInside(t.Context(), edges, teeth, radii[0], radii[1])
				records := new(RingEdgeRecords)
				got, gotErr := RingEdgesInsideShared(t.Context(), edges, teeth, radii[0], radii[1], records)
				requireSameOutcome(t, want, wantErr, got, gotErr)
				got, gotErr = RingEdgesInsideShared(t.Context(), edges, teeth, radii[0], radii[1], records)
				requireSameOutcome(t, want, wantErr, got, gotErr)
			})
		}
	}
}

// TestRingEdgeRecordsRebuildOnOtherInputs shows that a record is reused only
// for the same edge points and radii.
func TestRingEdgeRecordsRebuildOnOtherInputs(t *testing.T) {
	pt := func(x, y int64) proofarith.Xpt {
		return proofarith.Xpt{X: big.NewInt(x), Y: big.NewInt(y), Z: big.NewInt(0), W: big.NewInt(1)}
	}
	edge := [2]proofarith.Xpt{pt(1, 0), pt(1, 1)}
	records := new(RingEdgeRecords)
	bore2, blank2 := big.NewRat(1, 4), big.NewRat(4, 1)
	first := *records.recordOf(0, edge, bore2, blank2)
	again := records.recordOf(0, edge, bore2, blank2)
	require.Same(t, first.amin.X, again.amin.X, "an equal request must reuse the record")
	require.Equal(t, ringEdgeInner, again.class)

	moved := records.recordOf(0, [2]proofarith.Xpt{pt(3, 0), pt(3, 1)}, bore2, blank2)
	require.Equal(t, ringEdgeOuter, moved.class)
	require.Zero(t, moved.amin.X.Cmp(big.NewRat(3, 1)))

	smaller := records.recordOf(0, [2]proofarith.Xpt{pt(3, 0), pt(3, 1)}, bore2, big.NewRat(25, 1))
	require.Equal(t, ringEdgeInner, smaller.class)

	wider := records.recordOf(0, [2]proofarith.Xpt{pt(3, 0), pt(3, 1)}, big.NewRat(16, 1), big.NewRat(25, 1))
	require.True(t, wider.reachesBore)

	second := records.recordOf(1, edge, bore2, blank2)
	require.Zero(t, second.amin.X.Cmp(big.NewRat(1, 1)))
	require.Len(t, records.edges, 2)
}

// TestRingEdgesInsideBoxIsOutwardRounded puts a wall box exactly on each
// outward-rounded face of an edge's box, on x, y and the tooth level, where
// rounding the other way misses the wall and changes Candidates.
func TestRingEdgesInsideBoxIsOutwardRounded(t *testing.T) {
	pt := func(x, y int64) proofarith.Xpt {
		return proofarith.Xpt{X: big.NewInt(x), Y: big.NewInt(y), Z: big.NewInt(0), W: big.NewInt(3)}
	}
	edge := [2]proofarith.Xpt{pt(10, 20), pt(20, 10)}
	level := big.NewRat(1, 3)
	base := RingBox{{-100, 100}, {-100, 100}, {-100, 100}}
	ends := [3][2]*big.Rat{
		{big.NewRat(10, 3), big.NewRat(20, 3)}, {big.NewRat(10, 3), big.NewRat(20, 3)}, {level, level},
	}
	var walls []RingWall
	for axis := range base {
		lo, hi := ends[axis][0], ends[axis][1]
		below, above := base, base
		below[axis][1] = proofbound.RatFloatDown(lo)
		above[axis][0] = proofbound.RatFloatUp(hi)
		for _, box := range []RingBox{below, above} {
			wall := newTestRingWall([3]r3.Vec{{X: 30, Y: 30, Z: 30}, {X: 31, Y: 30, Z: 30}, {X: 30, Y: 31, Z: 30}}, 0)
			wall.Box = box
			walls = append(walls, wall)
		}
	}
	for _, class := range []struct {
		name        string
		blank, bore float64
	}{{"outer", 1, 0.5}, {"seam", 7.2, 0.5}} {
		t.Run(class.name, func(t *testing.T) {
			teeth := []RingToothLevel{{Circle: fixtureCircle(), Level: level, Bound2: big.NewRat(1, 64),
				Exterior: walls, Fitted: walls}}
			want, wantErr := legacyRingEdgesInside(t.Context(), [][2]proofarith.Xpt{edge}, teeth, class.blank, class.bore)
			got, gotErr := RingEdgesInsideShared(t.Context(), [][2]proofarith.Xpt{edge}, teeth,
				class.blank, class.bore, new(RingEdgeRecords))
			requireSameOutcome(t, want, wantErr, got, gotErr)
			require.Equal(t, len(walls), want.Candidates)
		})
	}
}

// TestRingEdgesInsideSharedRewriteInPlace rewrites one integer of an edge
// through its existing pointer between two calls on one records value.
func TestRingEdgesInsideSharedRewriteInPlace(t *testing.T) {
	pt := func(x, y, w int64) proofarith.Xpt {
		return proofarith.Xpt{X: big.NewInt(x), Y: big.NewInt(y), Z: big.NewInt(0), W: big.NewInt(w)}
	}
	rewrites := map[string]func(edge [2]proofarith.Xpt){
		"first X":   func(e [2]proofarith.Xpt) { e[0].X.SetInt64(0) },
		"first Y":   func(e [2]proofarith.Xpt) { e[0].Y.SetInt64(1) },
		"second X":  func(e [2]proofarith.Xpt) { e[1].X.SetInt64(0) },
		"second Y":  func(e [2]proofarith.Xpt) { e[1].Y.SetInt64(7) },
		"first W":   func(e [2]proofarith.Xpt) { e[0].W.SetInt64(8) },
		"second W":  func(e [2]proofarith.Xpt) { e[1].W.SetInt64(8) },
		"first X+":  func(e [2]proofarith.Xpt) { e[0].X.SetInt64(40) },
		"second Y+": func(e [2]proofarith.Xpt) { e[1].Y.SetInt64(-40) },
	}
	for name, rewrite := range rewrites {
		t.Run(name, func(t *testing.T) {
			edges := [][2]proofarith.Xpt{{pt(1, 0, 1), pt(2, 0, 1)}}
			records := new(RingEdgeRecords)
			got, err := RingEdgesInsideShared(t.Context(), edges, nil, 3, 0.5, records)
			require.NoError(t, err)
			require.Equal(t, 1, got.Inner)
			rewrite(edges[0])
			want, wantErr := legacyRingEdgesInside(t.Context(), edges, nil, 3, 0.5)
			got, gotErr := RingEdgesInsideShared(t.Context(), edges, nil, 3, 0.5, records)
			requireSameOutcome(t, want, wantErr, got, gotErr)
		})
	}
	t.Run("witness", func(t *testing.T) {
		edges := [][2]proofarith.Xpt{{pt(1, 0, 1), pt(2, 0, 1)}}
		records := new(RingEdgeRecords)
		_, err := RingEdgesInsideShared(t.Context(), edges, nil, 3, 0.5, records)
		require.NoError(t, err)
		edges[0][0].X.SetInt64(0)
		_, err = RingEdgesInsideShared(t.Context(), edges, nil, 3, 0.5, records)
		require.ErrorContains(t, err, "ring may reach the centered bore")
	})
}

// TestRingEdgeRecordsRadiusRewrittenInPlace rewrites the squared radii a
// record was built with through the same pointers.
func TestRingEdgeRecordsRadiusRewrittenInPlace(t *testing.T) {
	pt := func(x, y int64) proofarith.Xpt {
		return proofarith.Xpt{X: big.NewInt(x), Y: big.NewInt(y), Z: big.NewInt(0), W: big.NewInt(1)}
	}
	edge := [2]proofarith.Xpt{pt(3, 0), pt(3, 1)}
	for name, rewrite := range map[string]func(bore2, blank2 *big.Rat){
		"bore":  func(bore2, _ *big.Rat) { bore2.SetInt64(16) },
		"blank": func(_, blank2 *big.Rat) { blank2.SetInt64(100) },
	} {
		t.Run(name, func(t *testing.T) {
			bore2, blank2 := big.NewRat(1, 4), big.NewRat(4, 1)
			records := new(RingEdgeRecords)
			records.recordOf(0, edge, bore2, blank2)
			rewrite(bore2, blank2)
			got := *records.recordOf(0, edge, bore2, blank2)
			want := *new(RingEdgeRecords).recordOf(0, edge, bore2, blank2)
			require.Equal(t, want.class, got.class)
			require.Equal(t, want.reachesBore, got.reachesBore)
			require.Equal(t, want.witness, got.witness)
		})
	}
}

// TestRingPlaneApartZeroPlaneDistanceGuard pins the da == 0 guard with a wall
// whose squared normal is inconsistent with its normal: without the guard the
// zero distances compare 0 > bound2 * (-1) and the plane test rejects.
func TestRingPlaneApartZeroPlaneDistanceGuard(t *testing.T) {
	zero := RingVec{new(big.Rat), new(big.Rat), new(big.Rat)}
	wall := RingWall{Normal: RingVec{new(big.Rat), new(big.Rat), new(big.Rat)},
		Normal2: big.NewRat(-1, 1), Plane0: new(big.Rat)}
	bound2 := big.NewRat(1, 1)
	require.False(t, legacyRingPlaneApart(zero, zero, wall, bound2))
	require.False(t, ringPlaneApart(ringSegmentPlaneOf(zero, zero, bound2), &wall))
}
