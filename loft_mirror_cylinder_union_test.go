package decad

import (
	"context"
	"errors"
	"fmt"
	"math"
	"slices"
	"sync/atomic"
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestLoftMirrorCylinderLevelsRequireExactMidpoint(t *testing.T) {
	t.Parallel()
	require.True(t, loftMirrorCylinderLevels(0, 1, 2))
	require.True(t, loftMirrorCylinderLevels(-3, -1, 1))
	require.False(t, loftMirrorCylinderLevels(0, math.Nextafter(1, 2), 2))
	require.False(t, loftMirrorCylinderLevels(0, 1, math.Nextafter(2, 3)))
	require.False(t, loftMirrorCylinderLevels(0, math.NaN(), 2))
}

func TestLoftMirrorCylinderCapsMustTileWithOppositeWinding(t *testing.T) {
	t.Parallel()
	lower := facetedPayload{
		tris: [][3]int{{0, 1, 2}, {0, 2, 3}}, src: []int{4, 4},
	}
	upper := facetedPayload{
		tris: [][3]int{{2, 1, 0}, {3, 2, 0}}, src: []int{4, 4},
	}
	require.True(t, loftMirrorCylinderCapsOppose(lower, upper, []int{0, 1, 2, 3}, 4, 4))
	upper.tris[1] = [3]int{0, 2, 3}
	require.False(t, loftMirrorCylinderCapsOppose(lower, upper, []int{0, 1, 2, 3}, 4, 4))
	upper.tris[1] = [3]int{3, 2, 0}
	require.False(t, loftMirrorCylinderCapsOppose(lower, upper, []int{0, 1, 2, 4}, 4, 4))
	upper.src[1] = 3
	require.False(t, loftMirrorCylinderCapsOppose(lower, upper, []int{0, 1, 2, 3}, 4, 4))
}

func TestLoftMirrorCylinderPairKeepsAuthenticatedSource(t *testing.T) {
	t.Parallel()
	source := &loftMirrorJoinRecord{}
	other := &loftMirrorJoinRecord{}
	identity := r3.Identity()
	motion, ok := loftMirrorCylinderPairMotion(facetedPayload{
		loftMirrorJoin: source, xform: identity,
	}, source)
	require.True(t, ok)
	require.Equal(t, identity, motion)
	_, ok = loftMirrorCylinderPairMotion(facetedPayload{
		loftMirrorJoin: other, xform: identity,
	}, source)
	require.False(t, ok)

	rotated, err := r3.Rotation(r3.Vec{Z: 1}, units.Radians(math.Pi/4))
	require.NoError(t, err)
	_, ok = loftMirrorCylinderPairMotion(facetedPayload{
		loftMirrorCopy: &loftMirrorPlacementRecord{source: source, motion: rotated},
		xform:          rotated,
	}, source)
	require.True(t, ok)
	_, ok = loftMirrorCylinderPairMotion(facetedPayload{
		loftMirrorCopy: &loftMirrorPlacementRecord{source: other, motion: rotated},
		xform:          rotated,
	}, source)
	require.False(t, ok)
	_, ok = loftMirrorCylinderPairMotion(facetedPayload{
		loftMirrorCopy: &loftMirrorPlacementRecord{source: source, motion: rotated},
		xform:          identity,
	}, source)
	require.False(t, ok)
	shifted, err := r3.Translation(r3.Vec{Z: 1})
	require.NoError(t, err)
	_, ok = loftMirrorCylinderPairMotion(facetedPayload{
		loftMirrorCopy: &loftMirrorPlacementRecord{source: source, motion: shifted},
		xform:          shifted,
	}, source)
	require.False(t, ok)
}

// teethFixture is the mirrored Union's operands as the chamfer apitest builds
// them: a centered cylinder blank and the lower tooth pair with two copies
// turned about Z.
type teethFixture struct {
	d     *Document
	blank *Body
	pairs []*Body
}

func newTeethFixture(t *testing.T) teethFixture {
	t.Helper()
	d := New()
	world := sketch.NewWorld()
	middleFrame, err := r3.NewFrame(r3.Vec{Z: 1}, r3.Vec{X: 1}, r3.Vec{Y: 1})
	require.NoError(t, err)
	middlePlane, err := world.CreatePlaneFromFrame(middleFrame)
	require.NoError(t, err)
	bottomSketch, bottomProfile := teethSection(t, world, world.XY(), 0)
	middleSketch, middleProfile := teethSection(t, world, middlePlane, 0.03)
	lower, err := d.Loft(t.Context(), bottomSketch, bottomProfile, middleSketch, middleProfile)
	require.NoError(t, err)
	upper, err := lower.MirroredCopy(t.Context(), MirrorFrame{Frame: middleFrame})
	require.NoError(t, err)
	pair, err := Union(t.Context(), lower, upper)
	require.NoError(t, err)
	pairs := []*Body{pair}
	for i := 1; i <= 2; i++ {
		turn, err := r3.Rotation(r3.Vec{Z: 1}, units.Degrees(float64(45*i)))
		require.NoError(t, err)
		turned, err := Copy(t.Context(), pair, WithCopyPlacement(turn))
		require.NoError(t, err)
		pairs = append(pairs, turned)
	}
	ws := sketch.NewWorld()
	plane, err := ws.CreateOffsetPlane(ws.XY(), 0)
	require.NoError(t, err)
	s, err := ws.CreateSketch(plane)
	require.NoError(t, err)
	center := s.CreatePoint(0, 0)
	s.Fix(center)
	s.CreateCircle(center, 4)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	blank, err := Extrude(t.Context(), d, s, s.Profiles()[0], Distance{D: units.Millimeters(2), Dir: Along})
	require.NoError(t, err)
	return teethFixture{d: d, blank: blank, pairs: pairs}
}

func teethSection(t *testing.T, world *sketch.World, plane *sketch.Plane, turn float64) (*sketch.Sketch, *sketch.Profile) {
	t.Helper()
	s, err := world.CreateSketch(plane)
	require.NoError(t, err)
	center := s.CreatePoint(0, 0)
	s.Fix(center)
	polar := func(radius, angle float64) *sketch.Point {
		point := s.CreatePoint(radius*math.Cos(angle+turn), radius*math.Sin(angle+turn))
		s.Fix(point)
		return point
	}
	rightRoot, leftRoot := polar(4, -0.12), polar(4, 0.12)
	right := make([]*sketch.Point, 3)
	left := make([]*sketch.Point, 3)
	for i := range right {
		radius := 5 + float64(i)
		angle := 0.09 - 0.02*float64(i)
		right[i] = polar(radius, -angle)
		left[i] = polar(radius, angle)
	}
	s.CreateLine(rightRoot, right[0])
	_, err = s.CreateFitSpline(right...)
	require.NoError(t, err)
	s.CreateArc(center, right[len(right)-1], left[len(left)-1])
	_, err = s.CreateFitSpline(left...)
	require.NoError(t, err)
	s.CreateLine(left[0], leftRoot)
	s.CreateArc(center, rightRoot, leftRoot)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	profiles := s.Profiles()
	require.Len(t, profiles, 1)
	return s, profiles[0]
}

// pairsOutcome is what one call of loftMirrorCylinderPairs answers.
type pairsOutcome struct {
	body    *Body
	matched bool
	err     error
}

func (o pairsOutcome) requireSame(t *testing.T, want pairsOutcome) {
	t.Helper()
	require.Equal(t, want.matched, o.matched)
	if want.err != nil {
		require.EqualError(t, o.err, want.err.Error())
		require.ErrorIs(t, o.err, errors.Unwrap(want.err))
	} else {
		require.NoError(t, o.err)
	}
	require.Equal(t, want.body == nil, o.body == nil)
	if want.body == nil {
		return
	}
	wantPayload, ok := want.body.payload.(facetedPayload)
	require.True(t, ok)
	gotPayload, ok := o.body.payload.(facetedPayload)
	require.True(t, ok)
	require.Equal(t, wantPayload.verts, gotPayload.verts)
	require.Equal(t, wantPayload.tris, gotPayload.tris)
	require.Equal(t, want.body.volume, o.body.volume)
	require.Equal(t, want.body.area, o.body.area)
	require.Equal(t, want.body.centroid, o.body.centroid)
	require.Equal(t, want.body.bounds, o.body.bounds)
	require.Equal(t, wantPayload.vertexBound, gotPayload.vertexBound)
	require.Equal(t, wantPayload.meshBound, gotPayload.meshBound)
}

func (f teethFixture) call(ctx context.Context, ref producerID, pairs []*Body, workers int) pairsOutcome {
	body, matched, err := loftMirrorCylinderPairs(ctx, f.d, ref, f.blank, pairs, nil, workers)
	return pairsOutcome{body: body, matched: matched, err: err}
}

// pollCancelContext reports cancellation from its n-th Err call on and stays
// cancelled. Its counter is atomic because placement goroutines poll it.
type pollCancelContext struct {
	context.Context //nolint:containedctx // deterministic cancellation wrapper used only within one test call.
	cancelAt        int64
	calls           atomic.Int64
}

func (c *pollCancelContext) Err() error {
	if c.calls.Add(1) >= c.cancelAt {
		return context.Canceled
	}
	return c.Context.Err()
}

// TestLoftMirrorCylinderPairsParallelMatchesSerial runs the real mirrored
// Union with one placement goroutine and with four, on the whole pair list and
// on lists where a pair is refused in the middle, at the end, or twice, and
// requires the same bodies, flags and error text.
func TestLoftMirrorCylinderPairsParallelMatchesSerial(t *testing.T) {
	t.Parallel()
	f := newTeethFixture(t)
	ref := f.d.nextProducerID()
	replaced := func(at ...int) []*Body {
		pairs := append([]*Body(nil), f.pairs...)
		for _, i := range at {
			pairs[i] = f.blank // a prism: the pair check refuses it without an error
		}
		return pairs
	}
	t.Run("all pairs", func(t *testing.T) {
		serial := f.call(t.Context(), ref, f.pairs, 1)
		require.NoError(t, serial.err)
		require.True(t, serial.matched)
		require.NotNil(t, serial.body)
		moved := 0
		for _, p := range f.pairs {
			if p.payload.(facetedPayload).xform != r3.Identity() {
				moved++
			}
		}
		require.Equal(t, 2, moved)
		f.call(t.Context(), ref, f.pairs, 4).requireSame(t, serial)
	})
	for name, pairs := range map[string][]*Body{
		"refused in the middle": replaced(1),
		"refused at the end":    replaced(2),
		"refused twice":         replaced(1, 2),
		"zero pairs":            nil,
	} {
		t.Run(name, func(t *testing.T) {
			serial := f.call(t.Context(), ref, pairs, 1)
			f.call(t.Context(), ref, pairs, 4).requireSame(t, serial)
		})
	}
	t.Run("no authenticated placement", func(t *testing.T) {
		// A copy whose placement record names another source ends the Union
		// with an error naming that pair, whichever teeth are placed beside it.
		pairs := f.pairs
		good := pairs[2].payload
		fp := good.(facetedPayload)
		fp.loftMirrorCopy = &loftMirrorPlacementRecord{source: &loftMirrorJoinRecord{}, motion: fp.loftMirrorCopy.motion}
		pairs[2].payload = fp
		t.Cleanup(func() { pairs[2].payload = good })
		serial := f.call(t.Context(), ref, pairs, 1)
		require.ErrorIs(t, serial.err, ErrUnsupported)
		require.ErrorContains(t, serial.err, "pair 3 has no authenticated axial placement")
		require.True(t, serial.matched)
		f.call(t.Context(), ref, pairs, 4).requireSame(t, serial)
	})
}

// TestLoftMirrorCylinderPlaceTeethCancelled cancels the context at polls spread
// over the real placements. A cancelled call returns the cancellation and no
// teeth, one cancelled after the last poll returns every tooth, and the
// parallel run polls about as often as the serial one. The poll count is not
// exact because the audit's own workers poll on a timer.
func TestLoftMirrorCylinderPlaceTeethCancelled(t *testing.T) {
	t.Parallel()
	f := newTeethFixture(t)
	ref := f.d.nextProducerID()
	record := f.pairs[0].payload.(facetedPayload).loftMirrorJoin
	lower := record.lower.payload.(loftPayload)
	checked := loftMirrorCylinderCheckPairs(f.d, f.pairs, record, lower.frame1.Origin().Z)
	run := func(cancelAt int64, workers int) (int64, []*Body, error) {
		ctx := &pollCancelContext{Context: t.Context(), cancelAt: cancelAt}
		teeth, err := loftMirrorCylinderPlaceTeeth(checked.stop, workers, func(i int) (*Body, error) {
			if checked.motions[i] == r3.Identity() {
				return record.lower, nil
			}
			return lower.placed(ctx, f.d, ref, checked.motions[i])
		})
		return ctx.calls.Load(), teeth, err
	}
	polls, teeth, err := run(math.MaxInt64, 1)
	require.NoError(t, err)
	require.Len(t, teeth, 3)
	require.Greater(t, polls, int64(100))
	parallelPolls, teeth, err := run(math.MaxInt64, 4)
	require.NoError(t, err)
	require.Len(t, teeth, 3)
	require.InDelta(t, polls, parallelPolls, float64(polls)/100)
	for _, n := range []int64{1, 2, polls / 3, polls * 2 / 3, polls * 9 / 10} {
		_, teeth, err := run(n, 4)
		require.ErrorIs(t, err, context.Canceled, "cancel at poll %d", n)
		require.Nil(t, teeth)
	}
	_, teeth, err = run(polls*12/10, 4)
	require.NoError(t, err)
	require.Len(t, teeth, 3)
}

// TestLoftMirrorCylinderPairsRefusalPlacesNothingPastIt compares the polls of
// a mirrored Union that refuses its last pair under four placement goroutines
// with those under one: the parallel call places no tooth the serial call would
// not reach.
func TestLoftMirrorCylinderPairsRefusalPlacesNothingPastIt(t *testing.T) {
	t.Parallel()
	f := newTeethFixture(t)
	ref := f.d.nextProducerID()
	pairs := append([]*Body(nil), f.pairs...)
	pairs[2] = f.blank
	count := func(workers int) (int64, pairsOutcome) {
		ctx := &pollCancelContext{Context: t.Context(), cancelAt: math.MaxInt64}
		out := f.call(ctx, ref, pairs, workers)
		return ctx.calls.Load(), out
	}
	serialPolls, want := count(1)
	parallelPolls, got := count(4)
	got.requireSame(t, want)
	require.False(t, got.matched)
	require.Greater(t, serialPolls, int64(100))
	require.InDelta(t, serialPolls, parallelPolls, float64(serialPolls)/100)
	// The refusal ends the call right after the placements, so a cancellation
	// in the last tenth of the polls lands in a placement. It must surface, not
	// the refusal.
	for _, workers := range []int{1, 4} {
		ctx := &pollCancelContext{Context: t.Context(), cancelAt: serialPolls * 9 / 10}
		out := f.call(ctx, ref, pairs, workers)
		require.ErrorIs(t, out.err, context.Canceled, "%d workers", workers)
		require.Nil(t, out.body)
	}
}

// TestLoftMirrorCylinderPlaceTeethMatchesSerialPlacement places the real lower
// tooth under every motion with one goroutine and with four, and with a
// cancelled context for the first, a middle, the last and two teeth.
func TestLoftMirrorCylinderPlaceTeethMatchesSerialPlacement(t *testing.T) {
	t.Parallel()
	f := newTeethFixture(t)
	ref := f.d.nextProducerID()
	record := f.pairs[0].payload.(facetedPayload).loftMirrorJoin
	lower := record.lower.payload.(loftPayload)
	checked := loftMirrorCylinderCheckPairs(f.d, f.pairs, record, lower.frame1.Origin().Z)
	require.Equal(t, len(f.pairs), checked.stop)
	live, stopLive := context.WithCancel(t.Context())
	defer stopLive()
	cancelled, cancel := context.WithCancel(t.Context())
	cancel()
	placeWith := func(failing ...int) func(i int) (*Body, error) {
		return func(i int) (*Body, error) {
			ctx := live
			if slices.Contains(failing, i) {
				ctx = cancelled
			}
			if checked.motions[i] == r3.Identity() {
				return record.lower, nil
			}
			return lower.placed(ctx, f.d, ref, checked.motions[i])
		}
	}
	serial, err := loftMirrorCylinderPlaceTeeth(checked.stop, 1, placeWith())
	require.NoError(t, err)
	parallel, err := loftMirrorCylinderPlaceTeeth(checked.stop, 4, placeWith())
	require.NoError(t, err)
	require.Len(t, parallel, len(serial))
	for i := range serial {
		want, got := serial[i].payload.(loftPayload), parallel[i].payload.(loftPayload)
		if checked.motions[i] == r3.Identity() {
			require.Same(t, serial[i], parallel[i])
			continue
		}
		require.NotEmpty(t, want.verts)
		require.Equal(t, want.verts, got.verts)
		require.Equal(t, want.tris, got.tris)
		require.Equal(t, want.proof, got.proof)
		require.Equal(t, serial[i].volume, parallel[i].volume)
		require.Equal(t, serial[i].bounds, parallel[i].bounds)
	}
	for _, failing := range [][]int{{1}, {2}, {1, 2}} {
		for _, workers := range []int{1, 4} {
			teeth, err := loftMirrorCylinderPlaceTeeth(checked.stop, workers, placeWith(failing...))
			require.ErrorIs(t, err, context.Canceled)
			require.Nil(t, teeth)
		}
	}
}

// TestLoftMirrorCylinderPlaceTeethFirstError drives the helper with placements
// that fail at chosen teeth: the lowest failing tooth's error comes back, every
// tooth below it ran, and a clean run keeps index order.
func TestLoftMirrorCylinderPlaceTeethFirstError(t *testing.T) {
	t.Parallel()
	for _, n := range []int{0, 1, 7} {
		for _, workers := range []int{1, 2, 4} {
			bodies := make([]*Body, n)
			for i := range bodies {
				bodies[i] = &Body{}
			}
			teeth, err := loftMirrorCylinderPlaceTeeth(n, workers, func(i int) (*Body, error) { return bodies[i], nil })
			require.NoError(t, err)
			require.Len(t, teeth, n)
			for i := range teeth {
				require.Same(t, bodies[i], teeth[i])
			}
		}
	}
	for _, failing := range [][]int{{0}, {2, 5}, {5, 2}, {6}, {1, 2, 3, 4, 5, 6}} {
		for _, workers := range []int{1, 2, 4} {
			ran := make([]atomic.Bool, 7)
			teeth, err := loftMirrorCylinderPlaceTeeth(7, workers, func(i int) (*Body, error) {
				ran[i].Store(true)
				if slices.Contains(failing, i) {
					return nil, fmt.Errorf("tooth %d", i)
				}
				return &Body{}, nil
			})
			lowest := slices.Min(failing)
			require.EqualError(t, err, fmt.Sprintf("tooth %d", lowest), "failing %v, %d workers", failing, workers)
			require.Nil(t, teeth)
			for i := range lowest {
				require.True(t, ran[i].Load(), "tooth %d below the first failure did not run", i)
			}
		}
	}
}
