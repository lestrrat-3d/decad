package decad

import (
	"fmt"
	"math"
	"math/big"
	"slices"
	"strings"
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
	return internalRockerUnder(t, r3.Identity(), r3.NewVec(0, 0, 1))
}

// internalRockerUnder is internalRocker with every pin carried by tilt and
// every joint about axis; the bodies stay where internalRocker has them,
// which no scene reads.
func internalRockerUnder(t *testing.T, tilt r3.Transform, axis r3.Vec) (*Linkage, *Link) {
	t.Helper()
	doc := New()
	d := math.Sqrt(100*100 + 30*30 - 2*100*30)
	t4 := math.Pi - math.Acos((70*70+d*d-80*80)/(2*70*d))
	b := [2]float64{100 + 70*math.Cos(t4), 70 * math.Sin(t4)}
	pin := func(x, y float64) r3.Vec { return tilt.Apply(r3.NewVec(x, y, 0)) }
	l := NewLinkage()
	crank, err := l.Ground().Revolute(pin(0, 0), axis, []*Body{internalBoxBodyAtZ(t, doc, 0, -4, 30, 4, 0, 8)})
	require.NoError(t, err)
	coupler, err := crank.Revolute(pin(30, 0), axis, []*Body{internalBoxBodyAtZ(t, doc, 30, -4, 70, 4, 10, 8)})
	require.NoError(t, err)
	follower, err := l.Ground().Revolute(pin(100, 0), axis, []*Body{internalBoxBodyAtZ(t, doc, 70, -4, 100, 4, 20, 8)})
	require.NoError(t, err)
	_, err = l.Close(coupler, follower, pin(b[0], b[1]), axis)
	require.NoError(t, err)
	return l, crank
}

// internalTilt is the rotation taking X to (1, −1, 0)/√2 and Z to
// (1, 1, 1)/√3.
func internalTilt(t *testing.T) r3.Transform {
	t.Helper()
	frame, err := r3.NewFrame(r3.Vec{}, r3.NewVec(1, -1, 0), r3.NewVec(1, 1, -2))
	require.NoError(t, err)
	tilt, err := r3.FromFrame(frame)
	require.NoError(t, err)
	return tilt
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
	frame, ok := motionbound.NewMotionFrame(motionbound.Spec{Kind: motionbound.MotionRevolute, Center: r3.NewVec(3, -2, 0), Axis: r3.NewVec(0, 0, 1)})
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

	slide, ok := motionbound.NewMotionFrame(motionbound.Spec{Kind: motionbound.MotionPrismatic, Dir: r3.NewVec(0, 2, 0)})
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
	ask, err := ld.point(t.Context(), ld.subs[0], big.NewRat(5, 8))
	ld.mu.Unlock()
	require.NoError(t, err)
	require.NoError(t, ask.err)
	keys := make([]string, 0, len(ld.asks))
	for k := range ld.asks {
		keys = append(keys, k)
	}
	slices.Sort(keys)
	require.Equal(t, []string{"0:a", "0:c0,1/2", "0:c1/2,5/8", "0:p0", "0:p1/2", "0:p5/8"}, keys)
}

// TestLoopZeroPoseFalsifier: the document's pins are exact solutions of the
// scene at the driving value 0, so a pin whose exact plane position is not
// proven inside E0's box disproves the enclosure. Moving one pin's recorded
// position by 1e-6 mm after the scene is built, so that E0 encloses a
// mechanism the record no longer describes, refuses the loop; the record as
// built is admitted. On the tilted loop every pin's position is an enclosure:
// the two fixed pins are stated as boxes, which E0 reports as their own, and
// the two free pins' enclosures sit inside E0's boxes.
//
// Leg seen to fail when deleted: the pin-in-box check in askZero — the moved
// record is then admitted, on either loop.
func TestLoopZeroPoseFalsifier(t *testing.T) {
	t.Parallel()
	t.Run("about Z", func(t *testing.T) {
		t.Parallel()
		requireFalsifier(t, func(t *testing.T) (*Linkage, *Link) { return internalRocker(t) })
	})
	t.Run("about (1, 1, 1)", func(t *testing.T) {
		t.Parallel()
		requireFalsifier(t, func(t *testing.T) (*Linkage, *Link) {
			return internalRockerUnder(t, internalTilt(t), r3.NewVec(1, 1, 1))
		})
	})
	t.Run("an anchored rail on a turning crank", func(t *testing.T) {
		t.Parallel()
		requireFalsifierPins(t, func(t *testing.T) (*Linkage, *Link) {
			l, links := internalRockingBlock(t)
			return l, links[2]
		}, units.Degrees(30), 6, `O2, O4, the pin, the frame point C, the anchor A and B`)
	})
}

// internalRockingBlock is the public tests' rocking block off the pivot: a
// block sliding along (2, 1) on a crank about the origin, its pin at
// (54, 32) on a rocker of length 40 about (30, 0). Its links: the crank, the
// block, the rocker.
func internalRockingBlock(t *testing.T) (*Linkage, []*Link) {
	t.Helper()
	z := r3.NewVec(0, 0, 1)
	doc := New()
	l := NewLinkage()
	crank, err := l.Ground().Revolute(r3.Vec{}, z, []*Body{internalBoxBodyAtZ(t, doc, 0, -4, 30, 4, 0, 8)})
	require.NoError(t, err)
	block, err := crank.Prismatic(r3.NewVec(2, 1, 0), []*Body{internalBoxBodyAtZ(t, doc, 50, 28, 58, 36, 10, 8)})
	require.NoError(t, err)
	rocker, err := l.Ground().Revolute(r3.NewVec(30, 0, 0), z, []*Body{internalBoxBodyAtZ(t, doc, 30, -4, 54, 4, 20, 8)})
	require.NoError(t, err)
	_, err = l.Close(block, rocker, r3.NewVec(54, 32, 0), z)
	require.NoError(t, err)
	return l, []*Link{crank, block, rocker}
}

// TestLoopAnchorPassed pins docs/linkage-check-design.md §15.2's anchored
// reading on the elliptic trammel — a bar of 50 from (40, 0) on a slide along
// X to (0, 30) on a slide along Y — with the Y slide's anchor moved to 1 mm
// behind its rider. Driving the X slide 0 → 8 mm takes the Y slide down by
// 16 mm, past the anchor near 0.73 mm of X travel: the report is not Sound,
// one of its diagnostics names the anchor, and every evaluated pose still
// reads the Y slide at its closed form √(2500 − (40 + q)²) − 30.
//
// Leg seen to fail when deleted: the anchor's positivity check (the chain
// reads the folded distance past the anchor, and poses there read the Y
// slide's displacement with the wrong sign).
func TestLoopAnchorPassed(t *testing.T) {
	t.Parallel()
	z := r3.NewVec(0, 0, 1)
	doc := New()
	l := NewLinkage()
	sx, err := l.Ground().Prismatic(r3.NewVec(1, 0, 0), []*Body{internalBoxBodyAtZ(t, doc, 38, -2, 42, 2, 0, 8)})
	require.NoError(t, err)
	bar, err := sx.Revolute(r3.NewVec(40, 0, 0), z, []*Body{internalBoxBodyAtZ(t, doc, 18, 13, 22, 17, 10, 8)})
	require.NoError(t, err)
	sy, err := l.Ground().Prismatic(r3.NewVec(0, 1, 0), []*Body{internalBoxBodyAtZ(t, doc, -2, 28, 2, 32, 20, 8)})
	require.NoError(t, err)
	lp, err := l.Close(bar, sy, r3.NewVec(0, 30, 0), z)
	require.NoError(t, err)
	require.Contains(t, lp.kappa, sy)
	lp.kappa[sy] = big.NewRat(1, 1)
	report, err := doc.VerifyLinkage(t.Context(), l, Drive{{Link: sx, From: units.Millimeters(0), To: units.Millimeters(8)}})
	require.NoError(t, err)
	require.NotEqual(t, Sound, report.Status)
	named := false
	for _, d := range report.Diagnostics {
		named = named || strings.Contains(d.Message, "anchor")
	}
	require.True(t, named, `a diagnostic names the anchor`)
	require.NotEmpty(t, report.Poses)
	for _, p := range report.Poses {
		x := 40 + 8*p.Pose.At.Mag()
		got, err := p.Pose.Values[2].In(units.Millimeter)
		require.NoError(t, err)
		require.InDelta(t, math.Sqrt(2500-x*x)-30, got, 1e-9, "the Y slide at s = %v", p.Pose.At.Mag())
	}
}

func requireFalsifier(t *testing.T, rocker func(t *testing.T) (*Linkage, *Link)) {
	t.Helper()
	requireFalsifierPins(t, rocker, units.Degrees(90), 4, `O2, O4, A and B`)
}

// requireFalsifierPins builds the scene of the drive 0 → to of the link
// build returns and asserts the zero-pose falsifier admits it as built and
// refuses each of its pins moved by 1e-6 mm.
func requireFalsifierPins(t *testing.T, build func(t *testing.T) (*Linkage, *Link), to units.Value, pins int, which string) {
	t.Helper()
	scene := func(t *testing.T) *loopScene {
		l, link := build(t)
		spec, err := l.resolveDrive(Drive{{Link: link, From: units.New(0, to.Unit()), To: to}})
		require.NoError(t, err)
		sc, err := spec.loops[0].buildScene(t.Context(), spec, 0)
		require.NoError(t, err)
		return sc
	}
	require.NoError(t, scene(t).askZero(t.Context()))
	for n := range pins {
		sc := scene(t)
		require.Len(t, sc.pins, pins, which)
		pin := &sc.pins[n]
		shift := big.NewRat(1, 1000000)
		pin.v = proofbound.IntervalOwned(new(big.Rat).Add(pin.v.Lo, shift), new(big.Rat).Add(pin.v.Hi, shift))
		require.ErrorIs(t, sc.askZero(t.Context()), ErrUnsupported)
	}
}

// TestLoopSceneFrame: for each of the six closure axis senses and both scene
// sides, the scene's float frame has U and V equal to unit coordinate axes
// bit for bit, U × V is the closure axis, flipped on the mirrored side, and a
// pin's plane position is the caller's own coordinates bit for bit, its exact
// enclosure that one value. A loop with a slide takes U along the slide's own
// sense — for each of the four slide directions perpendicular to the closure
// axis — and the half-turned side negates U and V under the same normal.
func TestLoopSceneFrame(t *testing.T) {
	t.Parallel()
	pin := r3.NewVec(1.1, -2.3, 3.7)
	coords := [3]float64{pin.X, pin.Y, pin.Z}
	requireUnitAxes := func(t *testing.T, frame r3.Frame) {
		t.Helper()
		for _, e := range []r3.Vec{frame.U(), frame.V()} {
			ones := 0
			for _, c := range []float64{e.X, e.Y, e.Z} {
				require.True(t, c == 0 || c == 1 || c == -1)
				if c != 0 {
					ones++
				}
			}
			require.Equal(t, 1, ones)
		}
	}
	// requireExact asserts the plane's exact enclosure of the pin is the one
	// float the frame placed it at.
	requireExact := func(t *testing.T, plane loopPlane) {
		t.Helper()
		local := plane.frame.ToLocal(pin)
		x, y := plane.coords(pin)
		for _, c := range []struct {
			iv proofbound.RatInterval
			f  float64
		}{{x, local.X}, {y, local.Y}} {
			require.Zero(t, c.iv.Lo.Cmp(c.iv.Hi), `a coordinate-axis loop's plane position is one exact value`)
			require.Zero(t, c.iv.Lo.Cmp(new(big.Rat).SetFloat64(c.f)))
		}
	}
	// along is the exact coordinate of p along the unit coordinate axis e.
	along := func(p, e r3.Vec) float64 {
		for i, c := range []float64{e.X, e.Y, e.Z} {
			if c != 0 {
				return c * [3]float64{p.X, p.Y, p.Z}[i]
			}
		}
		return 0
	}
	for axis := range 3 {
		for _, sense := range []int{1, -1} {
			for _, mirror := range []bool{false, true} {
				normal := sense
				if mirror {
					normal = -sense
				}
				lp := &LinkageLoop{normal: ratVecExact(unitAxis(axis, sense)), coord: axis}
				plane, err := lp.sceneFrame(mirror, false)
				require.NoError(t, err)
				requireUnitAxes(t, plane.frame)
				require.Equal(t, unitAxis(axis, normal), plane.frame.U().Cross(plane.frame.V()))
				local := plane.frame.ToLocal(pin)
				iu, iv := (axis+1)%3, (axis+2)%3
				require.Equal(t, coords[iu], local.X)
				wantV := coords[iv]
				if normal < 0 {
					wantV = -wantV
				}
				require.Equal(t, wantV, local.Y)
				requireExact(t, plane)

				for _, slideAxis := range []int{(axis + 1) % 3, (axis + 2) % 3} {
					for _, dir := range []int{1, -1} {
						for _, halfTurn := range []bool{false, true} {
							slide := &Link{joint: PrismaticJoint{Dir: unitAxis(slideAxis, dir)}}
							lp := &LinkageLoop{normal: ratVecExact(unitAxis(axis, sense)), coord: axis, slide: slide}
							plane, err := lp.sceneFrame(mirror, halfTurn)
							require.NoError(t, err)
							requireUnitAxes(t, plane.frame)
							require.Equal(t, unitAxis(axis, normal), plane.frame.U().Cross(plane.frame.V()))
							wantU := unitAxis(slideAxis, dir)
							if halfTurn {
								wantU = wantU.Scale(-1)
							}
							require.Equal(t, wantU, plane.frame.U())
							local := plane.frame.ToLocal(pin)
							require.Equal(t, along(pin, plane.frame.U()), local.X)
							require.Equal(t, along(pin, plane.frame.V()), local.Y)
							requireExact(t, plane)
						}
					}
				}
			}
		}
	}
}

// TestLoopSceneFrameTilted: on a loop about a tilted axis the plane's exact
// axes u and v are perpendicular, u × v points along the side's normal, u is
// the slide's own direction where there is a slide (negated on the
// half-turned side), and a pin's exact plane coordinates are enclosed:
// x² + y² over the enclosures holds the pin's exact squared distance from the
// axis through the origin, and each enclosure is below 1e-12 mm wide and
// within 1e-12 of the float frame's position.
//
// Leg seen to fail when deleted: dividing by |u| and |v| in coords — the
// enclosures then hold p·u and p·v, too long by |u| and |v|.
func TestLoopSceneFrameTilted(t *testing.T) {
	t.Parallel()
	pin := r3.NewVec(17.3, -42.1, 8.25)
	for _, axis := range []r3.Vec{r3.NewVec(1, 1, 1), r3.NewVec(-0.3, 2, 0.7)} {
		for _, slide := range []*Link{nil, {joint: PrismaticJoint{Dir: r3.NewVec(axis.Y, -axis.X, 0)}}} {
			for _, mirror := range []bool{false, true} {
				for _, halfTurn := range []bool{false, true} {
					lp := &LinkageLoop{normal: ratVecExact(axis), coord: -1, slide: slide}
					plane, err := lp.sceneFrame(mirror, halfTurn)
					require.NoError(t, err)
					require.Zero(t, ratDot(plane.u, plane.v).Sign(), `u ⟂ v`)
					n := lp.normal
					if mirror {
						n = ratNeg(n)
					}
					uv := ratCross(plane.u, plane.v)
					require.True(t, ratZero(ratCross(uv, n)), `u × v ∥ n`)
					require.Positive(t, ratDot(uv, n).Sign(), `u × v along the side's normal`)
					if slide != nil {
						j, _ := slide.joint.(PrismaticJoint)
						want := 1
						if halfTurn {
							want = -1
						}
						require.True(t, ratZero(ratCross(plane.u, ratVecExact(j.Dir))))
						require.Equal(t, want, ratDot(plane.u, ratVecExact(j.Dir)).Sign())
					}
					x, y := plane.coords(pin)
					sq := func(iv proofbound.RatInterval) (*big.Rat, *big.Rat) {
						lo, hi := new(big.Rat).Mul(iv.Lo, iv.Lo), new(big.Rat).Mul(iv.Hi, iv.Hi)
						if lo.Cmp(hi) > 0 {
							lo, hi = hi, lo
						}
						if iv.Lo.Sign() <= 0 && iv.Hi.Sign() >= 0 {
							lo = new(big.Rat)
						}
						return lo, hi
					}
					xlo, xhi := sq(x)
					ylo, yhi := sq(y)
					exact := lp.planeSq(pin, r3.Vec{})
					require.LessOrEqual(t, new(big.Rat).Add(xlo, ylo).Cmp(exact), 0)
					require.GreaterOrEqual(t, new(big.Rat).Add(xhi, yhi).Cmp(exact), 0)
					local := plane.frame.ToLocal(pin)
					for _, c := range []struct {
						iv proofbound.RatInterval
						f  float64
					}{{x, local.X}, {y, local.Y}} {
						width, _ := new(big.Rat).Sub(c.iv.Hi, c.iv.Lo).Float64()
						require.Less(t, width, 1e-12)
						mid, _ := new(big.Rat).Add(c.iv.Lo, c.iv.Hi).Float64()
						require.InDelta(t, mid/2, c.f, 1e-12)
					}
				}
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
	require.Len(t, ld.spans[loopSpanKey(half, end)], 1, `one piece`)
	require.Len(t, ld.spans[loopSpanKey(half, end)][0], 2, `two dependents`)
	ld.asks["0:c0,1/2"] = &loopAsk{err: fmt.Errorf(`%w: injected`, sketch.ErrNotCertified)}
	require.Contains(t, dr.intervalGate(&motionPose{f: new(big.Rat)}, &motionPose{f: half}), `injected`)
}

// TestLoopMixedCrossing: scene 7's crank driven from −30°, a whole-turn
// fraction, to 1 rad crosses 0 at the irrational s₀ = (π/6)/(1 + π/6). The
// segment is cut into three sub-segments that tile it: up to a rational cut
// below s₀ on the mirrored side, the straddle, and on from a rational cut
// above s₀ on the scene's own side, the cuts within 1e-60 of each other. A
// pose inside the straddle holds both neighbours' values at their cuts and
// stands within 1e-12 of the zero pose, and an interval holding the straddle
// is read in three pieces that tile it.
//
// Leg seen to fail when deleted: the straddle sub-segment — the stretch
// between the cuts then lies on no sub-segment, and the interval's pieces
// leave a gap.
func TestLoopMixedCrossing(t *testing.T) {
	t.Parallel()
	l, crank := internalRocker(t)
	spec, err := l.resolveDrive(Drive{{Link: crank, From: units.Degrees(-30), To: units.Radians(1)}})
	require.NoError(t, err)
	require.NoError(t, spec.prepare(t.Context()))
	ld := spec.loops[0]
	ld.reach = []*big.Rat{big.NewRat(10, 1), big.NewRat(10, 1)}
	require.Len(t, ld.subs, 3)
	below, straddle, above := ld.subs[0], ld.subs[1], ld.subs[2]
	require.True(t, straddle.Straddle)
	require.Equal(t, 1, below.Side, `the crank below s₀ turns negative: the mirrored side`)
	require.Equal(t, 0, above.Side)
	require.Zero(t, below.Hi.Cmp(straddle.Lo))
	require.Zero(t, straddle.Hi.Cmp(above.Lo))
	require.Zero(t, below.Near.Cmp(below.Hi), `the chain below starts at its cut`)
	require.Zero(t, above.Near.Cmp(above.Lo))
	width, _ := new(big.Rat).Sub(straddle.Hi, straddle.Lo).Float64()
	require.Positive(t, width)
	require.Less(t, width, 1e-60)
	s0 := (math.Pi / 6) / (1 + math.Pi/6)
	lo, _ := straddle.Lo.Float64()
	require.InDelta(t, s0, lo, 1e-15)

	mid := new(big.Rat).Add(straddle.Lo, straddle.Hi)
	mid.Quo(mid, big.NewRat(2, 1))
	values, err := ld.pointValues(t.Context(), mid)
	require.NoError(t, err)
	for _, cut := range []*big.Rat{straddle.Lo, straddle.Hi} {
		at, err := ld.pointValues(t.Context(), cut)
		require.NoError(t, err)
		for j := range ld.deps {
			inside := valueInterval(values[j][0], values[j][1])
			end := valueInterval(at[j][0], at[j][1])
			require.LessOrEqual(t, inside.Lo.Cmp(end.Lo), 0)
			require.GreaterOrEqual(t, inside.Hi.Cmp(end.Hi), 0)
			m, _ := magnitude(inside).Float64()
			require.Less(t, m, 1e-12)
		}
	}

	a, b := big.NewRat(1, 4), big.NewRat(1, 2)
	pieces := ld.subs.Pieces(a, b)
	require.Len(t, pieces, 3)
	require.Zero(t, pieces[0].Lo.Cmp(a))
	require.Zero(t, pieces[2].Hi.Cmp(b))
	for n := 1; n < len(pieces); n++ {
		require.Zero(t, pieces[n-1].Hi.Cmp(pieces[n].Lo), `the pieces tile the interval`)
	}
	require.True(t, pieces[1].Sub.Straddle)
	spans, err := ld.intervalSpans(t.Context(), a, b)
	require.NoError(t, err)
	require.Len(t, spans, 3)
}

// TestLoopFoldStatement pins the pieces of a fold's statement and a long
// ask's budget (docs/linkage-check-design.md §15.3, §15.6) that no public
// fixture reaches exactly: the fold's interval printed with both ends rounded
// outward, so the printed interval holds the exact one, and the piece budget
// scaled by the whole turns an angular range spans.
//
// Legs seen to fail when deleted: rounding up an end that is not a whole
// number of decimals (1/3's upper end then prints as its lower); the slide's
// exemption (a slide's range then takes a turn-scaled budget).
func TestLoopFoldStatement(t *testing.T) {
	t.Parallel()
	t.Run("the printed ends are rounded outward", func(t *testing.T) {
		t.Parallel()
		third := big.NewRat(1, 3)
		require.Equal(t, "0.333333333333", decimalDown(third))
		require.Equal(t, "0.333333333334", decimalUp(third))
		minus := new(big.Rat).Neg(third)
		require.Equal(t, "-0.333333333334", decimalDown(minus))
		require.Equal(t, "-0.333333333333", decimalUp(minus))
		exact := big.NewRat(3, 4)
		require.Equal(t, "0.750000000000", decimalDown(exact), `an end of fewer decimals prints as itself`)
		require.Equal(t, "0.750000000000", decimalUp(exact))
	})
	t.Run("the piece budget grows with the turns spanned", func(t *testing.T) {
		t.Parallel()
		angle, slide := &loopScene{}, &loopScene{slideDriver: true}
		_, ok := angle.pieceBudget(1, 1+2*math.Pi)
		require.False(t, ok, `one turn keeps sketch's default`)
		budget, ok := angle.pieceBudget(0, 2.5*2*math.Pi)
		require.True(t, ok)
		require.Equal(t, 3*loopPiecesPerTurn, budget, `two and a half turns round up to three`)
		_, ok = slide.pieceBudget(0, 100)
		require.False(t, ok, `a slide's range is a length, not turns`)
	})
}
