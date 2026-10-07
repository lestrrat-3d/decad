package decad

import (
	"context"
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/freeform"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// This file is the class-B Cut of docs/general-boolean-design.md §3: a target
// prism X and a tool prism Y whose sweeps are exactly perpendicular, built
// into a brepPayload (§4). Every reading is taken in X's frame: d is the
// in-plane axis Y sweeps along, e the other in-plane axis, w X's own normal.
//
// This increment builds the through-nesting reach (§3 B.3): Y passes through
// X along d, and over Y's section X is exactly a slab between two of its own
// straight walls. X ∩ Y is then Y's section swept between those walls, so
// the two walls become planar faces carrying Y's section as a hole (each
// decided by a perpendicular-face scene) and Y's walls become the hole's
// walls. Every other pair misses silently and takes the mesh path.

// classBPair is an admitted pair stated in X's frame: Y's axes as signed X
// axes, and Y's record re-expressed exactly into the frame g (Y's axes at X's
// origin).
type classBPair struct {
	x, y prismPayload // y is re-expressed: frame g, coordinates shifted exactly
	g    r3.Frame
	// axis/sign map Y's local axis i (U, V, N) onto X's axis axis[i] with
	// sign[i]; axis[2] is d.
	axis [3]int
	sign [3]float64
}

// d and e are X's axis indices for the axis Y sweeps along and for X's other
// in-plane axis; w is always axis 2.
func (cp classBPair) d() int { return cp.axis[2] }
func (cp classBPair) e() int { return 1 - cp.axis[2] }

// xLocalOfY places a point of Y's re-expressed section at Y level z in X's
// local coordinates; the map is a signed permutation, so it is exact.
func (cp classBPair) xLocalOfY(u, v, z float64) [3]float64 {
	var out [3]float64
	for i, c := range [3]float64{u, v, z} {
		out[cp.axis[i]] = cp.sign[i]*c + 0
	}
	return out
}

// yLocalOfX is xLocalOfY's inverse.
func (cp classBPair) yLocalOfX(x [3]float64) [3]float64 {
	var out [3]float64
	for i := range out {
		out[i] = cp.sign[i]*x[cp.axis[i]] + 0
	}
	return out
}

// tryClassBCut attempts the class-B Cut. ok=false (err nil) is a silent miss:
// the caller takes the mesh path unchanged. A non-nil err is a refusal past
// the point of no return (prism-boolean §3.4) or a cancellation.
func tryClassBCut(ctx context.Context, target, tool *Body) (brepPayload, bool, error) {
	cp, ok, err := admitClassBPair(ctx, target, tool)
	if err != nil || !ok {
		return brepPayload{}, false, err
	}
	reach, ok, err := classBThroughReach(ctx, cp)
	if err != nil || !ok {
		return brepPayload{}, false, err
	}
	return buildClassBThroughCut(ctx, cp, reach)
}

// admitClassBPair runs §3's entry gate B1–B8 in order, plus the two conditions
// the brep record itself needs: one shared placement, and Y's axes carried
// bit for bit as signed X axes at X's origin (§4.1). Every miss is silent.
func admitClassBPair(ctx context.Context, target, tool *Body) (classBPair, bool, error) {
	x, okX := target.payload.(prismPayload)
	y, okY := tool.payload.(prismPayload)
	// B1, this increment: both operands are prisms; a stacked or brep
	// operand waits on §11 PR 7.
	if !okX || !okY || x.surfaceResult || y.surfaceResult {
		return classBPair{}, false, nil
	}
	if err := ctx.Err(); err != nil {
		return classBPair{}, false, err
	}
	// B5: no section displacement to amplify.
	if x.sectionDelta != 0 || y.sectionDelta != 0 {
		return classBPair{}, false, nil
	}
	if x.xform != y.xform {
		return classBPair{}, false, nil
	}
	// B3: the stored normals are exactly perpendicular.
	if x.frame.N().Dot(y.frame.N()) != 0 {
		return classBPair{}, false, nil
	}
	// B4: every axis dot is exactly 0 or ±1.
	xAxes := [3]r3.Vec{x.frame.U(), x.frame.V(), x.frame.N()}
	yAxes := [3]r3.Vec{y.frame.U(), y.frame.V(), y.frame.N()}
	for _, a := range xAxes {
		for _, b := range yAxes {
			if dot := a.Dot(b); dot != 0 && dot != 1 && dot != -1 {
				return classBPair{}, false, nil
			}
		}
	}
	cp := classBPair{x: x}
	used := [3]bool{}
	for i, b := range yAxes {
		found := false
		for j, a := range xAxes {
			switch {
			case b == a:
				cp.axis[i], cp.sign[i], found = j, 1, true
			case b == a.Scale(-1):
				cp.axis[i], cp.sign[i], found = j, -1, true
			}
			if found {
				break
			}
		}
		if !found || used[cp.axis[i]] {
			return classBPair{}, false, nil
		}
		used[cp.axis[i]] = true
	}
	if cp.axis[2] == 2 {
		return classBPair{}, false, nil
	}
	g, err := r3.NewFrame(x.frame.Origin(), yAxes[0], yAxes[1])
	if err != nil {
		return classBPair{}, false, fmt.Errorf(`%w: the tool's axes at the target's origin make no frame: %s`, ErrDegenerate, err)
	}
	if g.U() != yAxes[0] || g.V() != yAxes[1] || g.N() != yAxes[2] {
		return classBPair{}, false, nil
	}
	cp.g = g
	// B2: lines, circles and arcs only, each over its natural range, so
	// every recorded point is the point the record states.
	for _, p := range []ProfileRecord{x.profile, y.profile} {
		if !classBNaturalRecord(p) {
			return classBPair{}, false, nil
		}
	}
	// G6: the tool is hole-free.
	if len(y.profile.Holes) != 0 {
		return classBPair{}, false, nil
	}
	shifted, ok := classBShiftedTool(x, y, g)
	if !ok {
		return classBPair{}, false, nil
	}
	cp.y = shifted
	// B6: every segment is parallel or perpendicular to the other operand's
	// normal, read in its own frame.
	if !classBAxisAligned(x.profile) || !classBAxisAligned(cp.y.profile) {
		return classBPair{}, false, nil
	}
	ok, err = classBCurvedApart(ctx, cp)
	if err != nil || !ok {
		return classBPair{}, false, err
	}
	ok, err = classBNoCoplanarFaces(ctx, cp)
	if err != nil || !ok {
		return classBPair{}, false, err
	}
	return cp, true, nil
}

// classBNaturalRecord reports whether every segment is a line, circle or arc
// recorded over its natural range: a line or an arc over [0, 1] either way,
// a circle over one whole turn.
func classBNaturalRecord(p ProfileRecord) bool {
	for _, loop := range append([]LoopRecord{p.Outer}, p.Holes...) {
		for _, seg := range loop.Segments {
			switch s := seg.(type) {
			case LineSeg:
				if s.TStart != 0 || s.TEnd != 1 {
					return false
				}
			case ArcSeg:
				if !(s.TStart == 0 && s.TEnd == 1) && !(s.TStart == 1 && s.TEnd == 0) {
					return false
				}
			case CircleSeg:
				if !(s.TStart == 0 && s.TEnd == 1) && !(s.TStart == 1 && s.TEnd == 0) {
					return false
				}
			default:
				return false
			}
		}
	}
	return true
}

// classBShiftedTool re-expresses Y's record into g, Y's axes at X's origin:
// every coordinate moves by the exact rational dot of the origin difference
// with its axis. The shift is admitted only where every shifted coordinate
// and level is itself a float, so the re-expression rounds nothing: a datum
// plane and its CreateOffsetPlane at a float distance meet that, and a pair
// whose sums would round misses.
func classBShiftedTool(x, y prismPayload, g r3.Frame) (prismPayload, bool) {
	ox, oy := x.frame.Origin(), y.frame.Origin()
	diff := [3]*big.Rat{
		new(big.Rat).Sub(proofarith.FloatRat(oy.X), proofarith.FloatRat(ox.X)),
		new(big.Rat).Sub(proofarith.FloatRat(oy.Y), proofarith.FloatRat(ox.Y)),
		new(big.Rat).Sub(proofarith.FloatRat(oy.Z), proofarith.FloatRat(ox.Z)),
	}
	var shift [3]*big.Rat
	for i, axis := range [3]r3.Vec{g.U(), g.V(), g.N()} {
		shift[i] = proofbound.RatAdd(
			proofbound.RatMul(diff[0], proofarith.FloatRat(axis.X)),
			proofbound.RatMul(diff[1], proofarith.FloatRat(axis.Y)),
			proofbound.RatMul(diff[2], proofarith.FloatRat(axis.Z)),
		)
	}
	ok := true
	move := func(value float64, i int) float64 {
		exact := new(big.Rat).Add(proofarith.FloatRat(value), shift[i])
		held, _ := exact.Float64()
		if proofarith.RationalFloatError(exact, held) != 0 {
			ok = false
		}
		return held
	}
	point := func(p Point2) Point2 { return Point2{U: move(p.U, 0), V: move(p.V, 1)} }
	loops := append([]LoopRecord{y.profile.Outer}, y.profile.Holes...)
	moved := make([]LoopRecord, len(loops))
	for li, loop := range loops {
		for _, seg := range loop.Segments {
			switch s := seg.(type) {
			case LineSeg:
				s.Start, s.End = point(s.Start), point(s.End)
				seg = s
			case ArcSeg:
				s.Center, s.Start, s.End = point(s.Center), point(s.Start), point(s.End)
				seg = s
			case CircleSeg:
				s.Center = point(s.Center)
				seg = s
			}
			moved[li].Segments = append(moved[li].Segments, seg)
		}
	}
	out := y
	out.frame = g
	out.profile = ProfileRecord{Outer: moved[0], Holes: moved[1:]}
	out.z0, out.z1 = move(y.z0, 2), move(y.z1, 2)
	out.walks = nil
	return out, ok
}

// classBAxisAligned is B6 for one record. The other operand's normal lies in
// this record's plane on one of its own axes (B4), so a line runs along or
// across it exactly when one of its two coordinate differences is zero.
// Circles and arcs have no direction and pass.
func classBAxisAligned(p ProfileRecord) bool {
	for _, loop := range append([]LoopRecord{p.Outer}, p.Holes...) {
		for _, seg := range loop.Segments {
			line, ok := seg.(LineSeg)
			if !ok {
				continue
			}
			du, dv := line.End.U-line.Start.U, line.End.V-line.Start.V
			if du != 0 && dv != 0 {
				return false
			}
		}
	}
	return true
}

// classBBox is a segment's outward box in its own record's plane
// coordinates, as exact rationals.
type classBBox struct{ lo, hi [2]*big.Rat }

// classBSegmentBox bounds one natural-range segment in its own plane: a line
// by its endpoints, a circle or an arc by its whole circle, with an arc's
// computed radius rounded outward by its proven bound.
func classBSegmentBox(seg CurveSegment) (classBBox, error) {
	rat := proofarith.FloatRat
	switch s := seg.(type) {
	case LineSeg:
		b := classBBox{}
		for i, pair := range [2][2]float64{{s.Start.U, s.End.U}, {s.Start.V, s.End.V}} {
			lo, hi := pair[0], pair[1]
			if lo > hi {
				lo, hi = hi, lo
			}
			b.lo[i], b.hi[i] = rat(lo), rat(hi)
		}
		return b, nil
	default:
		w, err := walkOf(seg, freeform.NewFreeformWork())
		if err != nil {
			return classBBox{}, err
		}
		r := rat(proofbound.UpRound(proofbound.AbsSumUpper(w.Radius, w.RadiusBound)))
		if r == nil {
			return classBBox{}, fmt.Errorf(`%w: a circular segment states no finite radius`, ErrNotFinite)
		}
		c := [2]*big.Rat{rat(w.CU), rat(w.CV)}
		b := classBBox{}
		for i := range c {
			b.lo[i] = new(big.Rat).Sub(c[i], r)
			b.hi[i] = new(big.Rat).Add(c[i], r)
		}
		return b, nil
	}
}

// classBApart reports whether two closed intervals are separated by an exact
// comparison.
func classBApart(alo, ahi, blo, bhi *big.Rat) bool {
	return ahi.Cmp(blo) < 0 || bhi.Cmp(alo) < 0
}

// classBCurvedApart is B7: every curved wall of X and every curved wall of Y
// have outward boxes separated along some axis by an exact comparison.
func classBCurvedApart(ctx context.Context, cp classBPair) (bool, error) {
	type box3 struct{ lo, hi [3]*big.Rat }
	collect := func(p prismPayload, axis [3]int, sign [3]float64) ([]box3, error) {
		var out []box3
		lz, hz := proofarith.FloatRat(min(p.z0, p.z1)), proofarith.FloatRat(max(p.z0, p.z1))
		for _, loop := range append([]LoopRecord{p.profile.Outer}, p.profile.Holes...) {
			for _, seg := range loop.Segments {
				if err := ctx.Err(); err != nil {
					return nil, err
				}
				if _, line := seg.(LineSeg); line {
					continue
				}
				b, err := classBSegmentBox(seg)
				if err != nil {
					return nil, err
				}
				// Each local axis lands on one X axis; a negative sign swaps
				// that axis's two ends.
				lo := [3]*big.Rat{b.lo[0], b.lo[1], lz}
				hi := [3]*big.Rat{b.hi[0], b.hi[1], hz}
				var placed box3
				for i := range 3 {
					k := axis[i]
					if sign[i] > 0 {
						placed.lo[k], placed.hi[k] = lo[i], hi[i]
						continue
					}
					placed.lo[k], placed.hi[k] = new(big.Rat).Neg(hi[i]), new(big.Rat).Neg(lo[i])
				}
				out = append(out, placed)
			}
		}
		return out, nil
	}
	xs, err := collect(cp.x, [3]int{0, 1, 2}, [3]float64{1, 1, 1})
	if err != nil {
		return false, err
	}
	ys, err := collect(cp.y, cp.axis, cp.sign)
	if err != nil {
		return false, err
	}
	for _, a := range xs {
		for _, b := range ys {
			apart := false
			for k := range 3 {
				if classBApart(a.lo[k], a.hi[k], b.lo[k], b.hi[k]) {
					apart = true
					break
				}
			}
			if !apart {
				return false, nil
			}
		}
	}
	return true, nil
}

// classBPlane is one planar face's carrier in X's local coordinates: the plane
// x[axis] == level.
type classBPlane struct {
	axis  int
	level *big.Rat
}

// classBPlanes lists every planar face of one operand: its two caps and each
// line wall. A line wall runs along one in-plane axis, so its plane holds the
// other in-plane coordinate fixed. axis and sign place the operand's local
// axes on X's.
func classBPlanes(p prismPayload, axis [3]int, sign [3]float64) []classBPlane {
	plane := func(i int, value float64) classBPlane {
		return classBPlane{axis: axis[i], level: proofarith.FloatRat(sign[i]*value + 0)}
	}
	out := []classBPlane{plane(2, p.z0), plane(2, p.z1)}
	for _, loop := range append([]LoopRecord{p.profile.Outer}, p.profile.Holes...) {
		for _, seg := range loop.Segments {
			line, ok := seg.(LineSeg)
			if !ok {
				continue
			}
			if line.Start.U == line.End.U {
				out = append(out, plane(0, line.Start.U))
				continue
			}
			out = append(out, plane(1, line.Start.V))
		}
	}
	return out
}

// classBNoCoplanarFaces is B8: no planar face of one operand lies in the
// plane of a planar face of the other, compared exactly.
func classBNoCoplanarFaces(ctx context.Context, cp classBPair) (bool, error) {
	xs := classBPlanes(cp.x, [3]int{0, 1, 2}, [3]float64{1, 1, 1})
	ys := classBPlanes(cp.y, cp.axis, cp.sign)
	for _, a := range xs {
		if err := ctx.Err(); err != nil {
			return false, err
		}
		for _, b := range ys {
			if a.axis == b.axis && a.level.Cmp(b.level) == 0 {
				return false, nil
			}
		}
	}
	return true, nil
}

// classBThrough is the through-nesting reach of an admitted pair: the two X
// line walls, at d = l0 and d = l1, that bound X over Y's section, by
// (loop, segment) index.
type classBThrough struct {
	l0, l1     float64
	seg0, seg1 [2]int
}

// classBThroughReach decides, by exact comparisons of recorded coordinates,
// whether the pair is in the through-nesting reach:
//
//   - Y's section box, in (e, w), lies strictly inside X's sweep interval,
//     with each end's level displacement as margin;
//   - exactly two X segments meet the closed strip of e that box spans, both
//     lines along e at d = l0 < l1, each running strictly past the strip at
//     both ends, with X's material on the +d side of the first and the −d
//     side of the second, so X over the strip is the slab l0 ≤ d ≤ l1;
//   - Y's sweep interval runs strictly past both walls, with its own level
//     displacements as margin.
//
// Then X ∩ Y is Y's section swept over [l0, l1]. A pair outside it misses.
func classBThroughReach(ctx context.Context, cp classBPair) (classBThrough, bool, error) {
	rat := proofarith.FloatRat
	e, d := cp.e(), cp.d()
	var eLo, eHi, wLo, wHi *big.Rat
	for _, seg := range cp.y.profile.Outer.Segments {
		if err := ctx.Err(); err != nil {
			return classBThrough{}, false, err
		}
		b, err := classBSegmentBox(seg)
		if err != nil {
			return classBThrough{}, false, err
		}
		for _, c := range [][2]*big.Rat{{b.lo[0], b.lo[1]}, {b.hi[0], b.hi[1]}} {
			x := classBRatLocal(cp, c[0], c[1])
			if eLo == nil || x[e].Cmp(eLo) < 0 {
				eLo = x[e]
			}
			if eHi == nil || x[e].Cmp(eHi) > 0 {
				eHi = x[e]
			}
			if wLo == nil || x[2].Cmp(wLo) < 0 {
				wLo = x[2]
			}
			if wHi == nil || x[2].Cmp(wHi) > 0 {
				wHi = x[2]
			}
		}
	}
	a0 := new(big.Rat).Add(rat(cp.x.z0), rat(cp.x.z0Delta))
	a1 := new(big.Rat).Sub(rat(cp.x.z1), rat(cp.x.z1Delta))
	if a0.Cmp(wLo) >= 0 || wHi.Cmp(a1) >= 0 {
		return classBThrough{}, false, nil
	}
	type wall struct {
		at       [2]int
		level    float64
		material int
	}
	var walls []wall
	for li, loop := range append([]LoopRecord{cp.x.profile.Outer}, cp.x.profile.Holes...) {
		for si, seg := range loop.Segments {
			if err := ctx.Err(); err != nil {
				return classBThrough{}, false, err
			}
			b, err := classBSegmentBox(seg)
			if err != nil {
				return classBThrough{}, false, err
			}
			if b.hi[e].Cmp(eLo) < 0 || eHi.Cmp(b.lo[e]) < 0 {
				continue
			}
			line, ok := seg.(LineSeg)
			start, end := [2]float64{line.Start.U, line.Start.V}, [2]float64{line.End.U, line.End.V}
			if !ok || start[d] != end[d] || b.lo[e].Cmp(eLo) >= 0 || eHi.Cmp(b.hi[e]) >= 0 {
				return classBThrough{}, false, nil
			}
			// Material lies left of the walk: the left normal of a walk along
			// e has d component −t_e when d is u, and +t_e when d is v.
			te := end[e] - start[e]
			left := te
			if d == 0 {
				left = -te
			}
			material := 1
			if left < 0 {
				material = -1
			}
			walls = append(walls, wall{at: [2]int{li, si}, level: start[d], material: material})
		}
	}
	if len(walls) != 2 {
		return classBThrough{}, false, nil
	}
	if walls[1].level < walls[0].level {
		walls[0], walls[1] = walls[1], walls[0]
	}
	if !(walls[0].level < walls[1].level) || walls[0].material != 1 || walls[1].material != -1 {
		return classBThrough{}, false, nil
	}
	lowY := cp.xLocalOfY(0, 0, cp.y.z0)[d]
	highY := cp.xLocalOfY(0, 0, cp.y.z1)[d]
	lowDelta, highDelta := cp.y.z0Delta, cp.y.z1Delta
	if lowY > highY {
		lowY, highY = highY, lowY
		lowDelta, highDelta = highDelta, lowDelta
	}
	b0 := new(big.Rat).Add(rat(lowY), rat(lowDelta))
	b1 := new(big.Rat).Sub(rat(highY), rat(highDelta))
	if b0.Cmp(rat(walls[0].level)) >= 0 || rat(walls[1].level).Cmp(b1) >= 0 {
		return classBThrough{}, false, nil
	}
	return classBThrough{l0: walls[0].level, l1: walls[1].level, seg0: walls[0].at, seg1: walls[1].at}, true, nil
}

// classBRatLocal places a Y-plane point, held as exact rationals, in X's local
// axes through the signed permutation.
func classBRatLocal(cp classBPair, u, v *big.Rat) [3]*big.Rat {
	out := [3]*big.Rat{new(big.Rat), new(big.Rat), new(big.Rat)}
	for i, c := range [2]*big.Rat{u, v} {
		x := new(big.Rat).Set(c)
		if cp.sign[i] < 0 {
			x.Neg(x)
		}
		out[cp.axis[i]] = x
	}
	return out
}

// buildClassBThroughCut assembles the result (§4): X's face view less its two
// slab walls, those walls as planar faces carrying Y's section as a hole, and
// Y's walls reversed, swept between them. Each planar face's region is the
// perpendicular-face scene's answer (§5): prism-boolean's clean-nesting match
// of the wall's rectangle against Y's section, both in g. A scene that does
// not return that match leaves the pair outside this increment's reach and
// misses silently (prism-boolean §3.4); every refusal after it is an error.
func buildClassBThroughCut(ctx context.Context, cp classBPair, reach classBThrough) (brepPayload, bool, error) {
	view, err := brepOfPrism(cp.x)
	if err != nil {
		return brepPayload{}, false, err
	}
	e, d := cp.e(), cp.d()
	type slab struct {
		level float64
		at    [2]int
		// outward is the side X's material does not lie on: −d at l0, +d
		// at l1, read against g's normal, which lands on d with sign
		// cp.sign[2].
		outward bool
	}
	slabs := []slab{
		{level: reach.l0, at: reach.seg0, outward: cp.sign[2] < 0},
		{level: reach.l1, at: reach.seg1, outward: cp.sign[2] > 0},
	}
	loops := append([]LoopRecord{cp.x.profile.Outer}, cp.x.profile.Holes...)
	var planar []brepFace
	skip := map[int]struct{}{}
	for _, s := range slabs {
		line, ok := loops[s.at[0]].Segments[s.at[1]].(LineSeg)
		if !ok {
			return brepPayload{}, false, fmt.Errorf(`%w: a slab wall of the through-nesting reach is not a line`, ErrDegenerate)
		}
		ends := [2]float64{line.Start.U, line.Start.V}
		far := [2]float64{line.End.U, line.End.V}
		eLo, eHi := min(ends[e], far[e]), max(ends[e], far[e])
		var corners []Point2
		for _, c := range [][2]float64{{eLo, cp.x.z0}, {eHi, cp.x.z0}, {eHi, cp.x.z1}, {eLo, cp.x.z1}} {
			var x [3]float64
			x[d], x[e], x[2] = s.level, c[0], c[1]
			y := cp.yLocalOfX(x)
			corners = append(corners, Point2{U: y[0], V: y[1]})
		}
		rect := classBRectLoop(corners)
		region, matched, err := classBPerpendicularRegion(ctx, cp, rect)
		if err != nil || !matched {
			return brepPayload{}, false, err
		}
		var at [3]float64
		at[d] = s.level
		level := cp.yLocalOfX(at)[2]
		// The face's outer edges lie on X's caps, so they move in its plane
		// by the caps' own level displacements.
		planar = append(planar, brepFace{frame: cp.g, region: &region, outward: s.outward,
			z0: level, z1: level, delta: max(cp.x.z0Delta, cp.x.z1Delta)})
		index := 0
		for li := range s.at[0] {
			index += len(loops[li].Segments)
		}
		skip[index+s.at[1]] = struct{}{}
	}
	// Point of no return (prism-boolean §3.4).
	hole, err := reverseLoopRecordContext(ctx, cp.y.profile.Outer)
	if err != nil {
		return brepPayload{}, false, err
	}
	z0, z1 := planar[0].z0, planar[1].z0
	if z0 > z1 {
		z0, z1 = z1, z0
	}
	out := brepPayload{xform: cp.x.xform}
	for fi, f := range view.faces {
		if _, drop := skip[fi]; drop {
			continue
		}
		out.faces = append(out.faces, f)
	}
	out.faces = append(out.faces, planar...)
	for _, seg := range hole.Segments {
		out.faces = append(out.faces, brepFace{frame: cp.g, wall: seg, z0: z0, z1: z1})
	}
	out.assignRoles()
	return out, true, nil
}

// classBRectLoop is the counter-clockwise loop through four corners given in
// either winding.
func classBRectLoop(c []Point2) LoopRecord {
	area := 0.0
	for i := range c {
		j := (i + 1) % len(c)
		area += c[i].U*c[j].V - c[j].U*c[i].V
	}
	if area < 0 {
		c[1], c[3] = c[3], c[1]
	}
	var loop LoopRecord
	for i := range c {
		loop.Segments = append(loop.Segments, LineSeg{Start: c[i], End: c[(i+1)%len(c)], TStart: 0, TEnd: 1})
	}
	return loop
}

// classBPerpendicularRegion is the perpendicular-face scene (§5) of one slab
// wall: the wall's rectangle and Y's section, both in g, resolved by
// prism-boolean's clean-nesting Cut match and authenticated through
// RecordProfile. matched is false when the scene returned no such match.
func classBPerpendicularRegion(ctx context.Context, cp classBPair, rect LoopRecord) (ProfileRecord, bool, error) {
	budget := proofbound.NewWorkBudget(ctx)
	target := prismPayload{profile: ProfileRecord{Outer: rect}, frame: cp.g, xform: cp.x.xform, z0: 0, z1: 1}
	tool := prismPayload{profile: cp.y.profile, frame: cp.g, xform: cp.x.xform, z0: 0, z1: 1}
	segments, within, err := prismSceneWithinWorkCap(budget, target, tool)
	if err != nil {
		return ProfileRecord{}, false, err
	}
	if !within {
		return ProfileRecord{}, false, fmt.Errorf(`%w: the class-B face scene charges at least %d arranger segments against this evaluator's cap of %d`,
			ErrUnsupported, segments, prismMaxArrangementSegments)
	}
	reexpress, err := newPrismReexpression(target, tool)
	if err != nil {
		return ProfileRecord{}, false, err
	}
	s, match, sceneDelta, resolved, err := resolvePrismCut(ctx, budget, target, tool, reexpress)
	if err != nil || !resolved || sceneDelta.a != 0 || sceneDelta.b != 0 {
		return ProfileRecord{}, false, err
	}
	profile, err := prismRecordProfileContext(ctx, s, match)
	if err != nil {
		return ProfileRecord{}, false, err
	}
	return profile, true, nil
}
