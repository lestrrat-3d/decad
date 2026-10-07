package decad

import (
	"fmt"
	"math"
	"math/big"
	"slices"
	"testing"

	"github.com/lestrrat-3d/decad/internal/motionbound"

	"github.com/lestrrat-3d/decad/internal/proofbound"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file pins the pieces of docs/linkage-check-design.md §15 that sit
// below any public fixture's observability: the ideal pose over a range of
// joint values, the dependent travel term, the canonical chain, the
// zero-pose falsifier, and the scene's exact frame. Each test records the
// legs seen to fail when deleted.

// internalRocker is scene 7's crank-rocker without its bar bodies' shapes:
// ground pivots (0, 0) and (100, 0), crank 30, coupler 80, follower 70, the
// coupler closed onto the follower at the closed-form B above the ground line.
func internalRocker(t *testing.T) (*Linkage, *Link) {
	t.Helper()
	doc := New()
	d := math.Sqrt(100*100 + 30*30 - 2*100*30)
	t4 := math.Pi - math.Acos((70*70+d*d-80*80)/(2*70*d))
	b := [2]float64{100 + 70*math.Cos(t4), 70 * math.Sin(t4)}
	l := NewLinkage()
	z := r3.NewVec(0, 0, 1)
	crank, err := l.Ground().Revolute(r3.Vec{}, z, []*Body{internalBoxBodyAtZ(t, doc, 0, -4, 30, 4, 0, 8)})
	require.NoError(t, err)
	coupler, err := crank.Revolute(r3.NewVec(30, 0, 0), z, []*Body{internalBoxBodyAtZ(t, doc, 30, -4, 70, 4, 10, 8)})
	require.NoError(t, err)
	follower, err := l.Ground().Revolute(r3.NewVec(100, 0, 0), z, []*Body{internalBoxBodyAtZ(t, doc, 70, -4, 100, 4, 20, 8)})
	require.NoError(t, err)
	_, err = l.Close(coupler, follower, r3.NewVec(b[0], b[1], 0), z)
	require.NoError(t, err)
	return l, crank
}

// TestMotionFrameAtRange pins AtRange on a synthetic interval [θ − w, θ + w]:
// every entry of its rotation holds the rotation at both ends, and its sine
// entry is at least 2·w·cos θ wide less the point enclosure's own width.
//
// Leg seen to fail when deleted: widening the endpoint enclosure by the
// range's span — AtRange then evaluates one value alone, and the far end's
// rotation falls outside it.
func TestMotionFrameAtRange(t *testing.T) {
	t.Parallel()
	frame, ok := newMotionFrame(motionSpec{kind: motionbound.MotionRevolute, center: r3.NewVec(3, -2, 0), axis: r3.NewVec(0, 0, 1)})
	require.True(t, ok)
	theta, w := big.NewRat(7, 10), big.NewRat(1, 1000)
	lo := motionbound.MotionParam{Turn: new(big.Rat), Base: new(big.Rat).Sub(theta, w)}
	hi := motionbound.MotionParam{Turn: new(big.Rat), Base: new(big.Rat).Add(theta, w)}
	ranged := frame.AtRange(lo, hi)
	for _, end := range []motionbound.MotionParam{lo, hi} {
		point := frame.At(end)
		for i := range 3 {
			for j := range 3 {
				require.LessOrEqual(t, ranged.Rot[i][j].Lo.Cmp(point.Rot[i][j].Lo), 0)
				require.GreaterOrEqual(t, ranged.Rot[i][j].Hi.Cmp(point.Rot[i][j].Hi), 0)
			}
		}
	}
	sinWidth, _ := new(big.Rat).Sub(ranged.Rot[1][0].Hi, ranged.Rot[1][0].Lo).Float64()
	pointSin := frame.At(lo).Rot[1][0]
	pointWidth, _ := new(big.Rat).Sub(pointSin.Hi, pointSin.Lo).Float64()
	require.GreaterOrEqual(t, sinWidth, 2*1e-3*math.Cos(0.7)-pointWidth)
	require.Equal(t, frame.At(lo), frame.AtRange(lo, lo), `AtRange(p, p) is At(p)`)

	slide, ok := newMotionFrame(motionSpec{kind: motionbound.MotionPrismatic, dir: r3.NewVec(0, 2, 0)})
	require.True(t, ok)
	shift := slide.AtRange(lo, hi).Shift[1]
	require.Zero(t, shift.Lo.Cmp(lo.Base))
	require.Zero(t, shift.Hi.Cmp(hi.Base))
}

// TestDependentSpan pins §15.5's two-sided hull term on hand-made readings:
// a monotone joint travels the hull's width, and a joint that returns to its
// start travels it twice.
//
// Leg seen to fail when deleted: reading the term as the span of the two end
// readings — the turn-back case then reads 0.
func TestDependentSpan(t *testing.T) {
	t.Parallel()
	iv := func(lo, hi int64) proofbound.RatInterval {
		return proofbound.IntervalOwned(big.NewRat(lo, 100), big.NewRat(hi, 100))
	}
	monotone := dependentSpan(loopSpan{a: iv(10, 11), b: iv(50, 51), h: iv(10, 51)})
	require.Zero(t, monotone.Cmp(big.NewRat(42, 100)), `a_hi + b_hi − 2·h_lo = 0.11 + 0.51 − 0.20`)
	back := dependentSpan(loopSpan{a: iv(0, 1), b: iv(0, 1), h: iv(-30, 1)})
	require.Zero(t, back.Cmp(big.NewRat(62, 100)), `the dip to −0.30 and back`)
}

// TestLoopCanonicalChain pins the canonical chain of §15.3 on a four-level
// grid: for a drive that starts away from 0, one point ask at 5/8 asks the
// approach, the point at 0, the cell [0, 1/2], the point at 1/2, the cell
// [1/2, 5/8] and the point at 5/8, and nothing else.
func TestLoopCanonicalChain(t *testing.T) {
	t.Parallel()
	l, crank := internalRocker(t)
	spec, err := l.resolveDrive(Drive{{Link: crank, From: units.Degrees(10), To: units.Degrees(90)}})
	require.NoError(t, err)
	require.NoError(t, spec.prepare(t.Context()))
	ld := spec.loops[0]
	ld.mu.Lock()
	ask, err := ld.point(t.Context(), big.NewRat(5, 8))
	ld.mu.Unlock()
	require.NoError(t, err)
	require.NoError(t, ask.err)
	keys := make([]string, 0, len(ld.asks))
	for k := range ld.asks {
		keys = append(keys, k)
	}
	slices.Sort(keys)
	require.Equal(t, []string{"a", "c0,1/2", "c1/2,5/8", "p0", "p1/2", "p5/8"}, keys)
}

// TestLoopZeroPoseFalsifier: the document's pins are exact solutions of the
// scene at the driving value 0, so a pin outside E0's box disproves the
// enclosure. Moving one pin's recorded position by 1e-6 mm after the scene
// is built, so that E0 encloses a mechanism the record no longer describes,
// refuses the loop; the record as built is admitted.
//
// Leg seen to fail when deleted: the pin-in-box check in askZero — the moved
// record is then admitted.
func TestLoopZeroPoseFalsifier(t *testing.T) {
	t.Parallel()
	build := func(t *testing.T) *loopDrive {
		l, crank := internalRocker(t)
		spec, err := l.resolveDrive(Drive{{Link: crank, From: units.Degrees(0), To: units.Degrees(90)}})
		require.NoError(t, err)
		ld := spec.loops[0]
		require.NoError(t, ld.buildScene(t.Context(), spec))
		return ld
	}
	require.NoError(t, build(t).askZero(t.Context()))
	for n := range 4 {
		ld := build(t)
		require.Len(t, ld.scene.pins, 4, `O2, O4, A and B`)
		pin := &ld.scene.pins[n]
		pin.v = new(big.Rat).Add(pin.v, big.NewRat(1, 1000000))
		require.ErrorIs(t, ld.askZero(t.Context()), ErrUnsupported)
	}
}

// TestLoopSceneFrame: for each of the six closure axis senses and both scene
// signs, the scene's frame has U and V equal to unit coordinate axes bit for
// bit, its normal is the closure axis times the sign, and a pin's plane
// position is the caller's own coordinates bit for bit.
func TestLoopSceneFrame(t *testing.T) {
	t.Parallel()
	pin := r3.NewVec(1.1, -2.3, 3.7)
	coords := [3]float64{pin.X, pin.Y, pin.Z}
	for axis := range 3 {
		for _, sense := range []int{1, -1} {
			for _, sigma := range []int{1, -1} {
				lp := &LinkageLoop{axis: axis, sense: sense}
				frame, err := lp.sceneFrame(sigma)
				require.NoError(t, err)
				u, v := frame.U(), frame.V()
				for _, e := range []r3.Vec{u, v} {
					ones := 0
					for _, c := range []float64{e.X, e.Y, e.Z} {
						require.True(t, c == 0 || c == 1 || c == -1)
						if c != 0 {
							ones++
						}
					}
					require.Equal(t, 1, ones)
				}
				require.Equal(t, unitAxis(axis, sense*sigma), u.Cross(v))
				local := frame.ToLocal(pin)
				iu, iv := (axis+1)%3, (axis+2)%3
				require.Equal(t, coords[iu], local.X)
				wantV := coords[iv]
				if sense*sigma < 0 {
					wantV = -wantV
				}
				require.Equal(t, wantV, local.Y)
			}
		}
	}
}

// TestLoopIntervalGate: an interval whose cell sketch refused is undecided
// even where the points at both of its ends were enclosed. The refusal is put
// in place of the certified cell [0, 1/2] after the point at 1/2 was
// continued from it, which no real chain produces; the gate then names
// sketch's cause, and a cell certified is read into the dependents' spans.
//
// Leg seen to fail when deleted: intervalGate's refusal — the refused cell
// then passes as certified.
func TestLoopIntervalGate(t *testing.T) {
	t.Parallel()
	l, crank := internalRocker(t)
	spec, err := l.resolveDrive(Drive{{Link: crank, From: units.Degrees(0), To: units.Degrees(90)}})
	require.NoError(t, err)
	require.NoError(t, spec.prepare(t.Context()))
	ld := spec.loops[0]
	ld.reach = []*big.Rat{big.NewRat(10, 1), big.NewRat(10, 1)}
	half, end := big.NewRat(1, 2), big.NewRat(3, 4)
	dr := &linkageDriver{run: &motionRun{ctx: t.Context()}, spec: spec}
	require.Empty(t, dr.intervalGate(&motionPose{f: half}, &motionPose{f: end}), `a certified cell passes`)
	require.Len(t, ld.spans[loopSpanKey(half, end)], 2)
	ld.asks["c0,1/2"] = &loopAsk{err: fmt.Errorf(`%w: injected`, sketch.ErrNotCertified)}
	require.Contains(t, dr.intervalGate(&motionPose{f: new(big.Rat)}, &motionPose{f: half}), `injected`)
}
