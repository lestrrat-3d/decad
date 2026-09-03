package decad

import (
	"context"
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/r3"
)

// This file is docs/tessellation-design.md §13's increment T7
// (docs/tessellation-reach-design.md §7): the cap-loop chamfer tessellator. It
// meshes a capBlendPayload's trimmed side walls, its chamfer band patches and
// its two cap faces, and publishes §2's per-face displacement bound and area
// slack for them.
//
// It is EXPORT-ONLY. No occupied-volume homotopy from the held facets to the
// exact offset family has been proven for the ruled-to-cone step, so the mesh
// carries symDiffOK false and the mesh boolean refuses a cap-blend operand
// (boolean.go's requireVolumeProvingPayload and operandSymDiff), exactly as a
// revolve mesh does until its own §11 proof lands. tess §2 permits that for an
// export-only increment; substituting bound × held area for the missing proof
// is forbidden outright (§11).
//
// One structural fact shapes the whole file, and it is the answer to
// docs/modify-reach-design.md §12 Table DX row DX3's "a strip whose two sides
// disagree on sample density is not watertight": each wall walk is chorded
// ONCE, at a single count, and that one count is shared three ways — by the
// trimmed side wall's own two rings, by the band patch ruled off the ring at
// the chamfered end, and by the cap contour arc the band ends on. Nothing is
// snapped, welded or dropped to make the strips meet; a count that cannot be
// chosen refuses instead.

// capBlendLoopMesh is one recorded loop's complete chording: the walks the
// build resolved, the offset joins the cap contour is trimmed at, the ONE chord
// count per walk every level of that loop reads, and the plane-local samples
// and mesh vertices that count produced.
//
// A loop chamfered on either cap carries a cap contour ring as well as its side
// ring. The contour does not depend on WHICH cap the loop is chamfered on — the
// in-plane offset is the same either way (capblend.go's mixedOffsetProfile) —
// so the samples are resolved once and lifted to whichever cap level(s) the
// selection named.
type capBlendLoopMesh struct {
	li    int
	loop  LoopRecord
	walks []sideWalk
	// joins is the per-corner offset join capOffsetJoins resolves, nil for an
	// unchamfered loop and for the one cornerless closed circle (whole).
	joins          []cornerJoin
	whole          bool
	onStart, onEnd bool
	chamfered      bool
	zLo, zHi       boundedScalar
	count          []int     // the shared chord count of walk i
	sideSag        []float64 // walk i's own side-directrix sagitta at that count
	capSag         []float64 // walk i's cap-directrix sagitta at that count
	capRadius      []float64
	capTh0, capTh1 []float64
	arcCount       []int // a reflex corner's connector-arc chord count, 0 elsewhere
	arcSag         []float64
	arcTh0, arcTh1 []float64
	locusGap       []float64 // corner i's ruling-versus-locus gap, both caps alike
	sidePts        []Point2
	sideBound      []walkEndBound
	sideStart      []int // walk i's first side sample
	capPts         []Point2
	capBound       []walkEndBound
	capWallStart   []int // walk i's first cap sample
	capArcStart    []int // corner i's first connector-arc sample, −1 when not reflex
	sideLo, sideHi []int // mesh vertices of each side sample at zLo and zHi
	capLoV, capHiV []int // mesh vertices of each cap sample at z0 and z1
}

// tessellateCapBlend meshes a cap-loop chamfer result
// (docs/tessellation-reach-design.md §7).
func tessellateCapBlend(ctx context.Context, b *Body, cbp capBlendPayload, chord float64) (*Mesh, error) {
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	byRole := map[string]*Face{}
	for _, f := range b.Faces() {
		for _, o := range f.Origins() {
			byRole[o.Role] = f
		}
	}
	faceOfRole := func(role string) (*Face, error) {
		f, ok := byRole[role]
		if !ok {
			return nil, fmt.Errorf(`%w: the body carries no face for role %q`, ErrDegenerate, role)
		}
		return f, nil
	}
	geomOfRole := map[string]capPatchGeom{}
	for _, p := range cbp.patches {
		geomOfRole[p.role] = p.geom
	}
	capStart, err := faceOfRole(roleCapStart)
	if err != nil {
		return nil, err
	}
	capEnd, err := faceOfRole(roleCapEnd)
	if err != nil {
		return nil, err
	}

	pl := cbp.prismLike(0, 0)
	work := newFreeformWork()
	budget := newWorkBudget(ctx)
	loops := cbp.loops()
	lms := make([]capBlendLoopMesh, len(loops))
	for li, loop := range loops {
		lm, err := chordCapBlendLoop(ctx, budget, cbp, li, loop, chord, work)
		if err != nil {
			return nil, err
		}
		lms[li] = lm
	}

	// Vertices. Every sample of a loop owns one vertex per level the loop
	// reaches: the side ring at zLo and zHi, and the cap contour ring at
	// whichever cap(s) the selection named. store is
	// docs/tessellation-reach-design.md §3's deltaStore, one entry per vertex.
	var mesh Mesh
	var store []float64
	addVertex := func(p Point2, z, plane float64) int {
		held := pl.point(p.U, p.V, z)
		mesh.vertices = append(mesh.vertices, held)
		store = append(store, absSumUpper(plane, exactPrismPointRound(pl, p.U, p.V, z, held)))
		return len(mesh.vertices) - 1
	}
	for li := range lms {
		lm := &lms[li]
		lm.sideLo = make([]int, len(lm.sidePts))
		lm.sideHi = make([]int, len(lm.sidePts))
		for j, p := range lm.sidePts {
			if err := budget.step(); err != nil {
				return nil, err
			}
			plane := walkEndBoundAllow(lm.sideBound[j])
			lm.sideLo[j] = addVertex(p, lm.zLo.value, plane)
			lm.sideHi[j] = addVertex(p, lm.zHi.value, plane)
		}
		if len(lm.capPts) == 0 {
			continue
		}
		lm.capLoV = make([]int, len(lm.capPts))
		lm.capHiV = make([]int, len(lm.capPts))
		for j, p := range lm.capPts {
			if err := budget.step(); err != nil {
				return nil, err
			}
			plane := walkEndBoundAllow(lm.capBound[j])
			if lm.onStart {
				lm.capLoV[j] = addVertex(p, cbp.z0, plane)
			}
			if lm.onEnd {
				lm.capHiV[j] = addVertex(p, cbp.z1, plane)
			}
		}
	}
	if _, err := requireDerivableStore(store); err != nil {
		return nil, err
	}

	// faceExtra is every displacement a face carries BESIDE its own vertices'
	// store term, which composeCapBlendBounds adds per face at the end.
	faceExtra := map[*Face]float64{}
	bump := func(f *Face, v float64) {
		if v > faceExtra[f] {
			faceExtra[f] = v
		}
	}

	// Trimmed side walls: the prism's own cells between the loop's two rings.
	for li := range lms {
		lm := &lms[li]
		axial := math.Max(lm.zLo.bound, lm.zHi.bound)
		height := math.Abs(lm.zHi.value - lm.zLo.value)
		n := len(lm.walks)
		for i, w := range lm.walks {
			if err := budget.step(); err != nil {
				return nil, err
			}
			face, err := faceOfRole(fmt.Sprintf("side(%d,%d)", lm.li, w.segs[0]))
			if err != nil {
				return nil, err
			}
			bump(face, absSumUpper(lm.sideSag[i], axial))
			mesh.areaSlack = absSumUpper(mesh.areaSlack, walkWallSlack(w.segmentWalk, lm.count[i], height))
			for k := range lm.count[i] {
				g0 := lm.sideStart[i] + k
				g1 := lm.sideStart[i] + k + 1
				if k == lm.count[i]-1 {
					g1 = lm.sideStart[(i+1)%n]
				}
				mesh.addTriangle([3]int{lm.sideLo[g0], lm.sideLo[g1], lm.sideHi[g1]}, face)
				mesh.addTriangle([3]int{lm.sideLo[g0], lm.sideHi[g1], lm.sideHi[g0]}, face)
			}
		}
	}

	// Chamfer bands: the patch cells between the side ring at the band's own
	// side level and the cap contour ring at its cap level.
	for li := range lms {
		lm := &lms[li]
		if lm.onStart {
			if err := emitCapBand(budget, &mesh, cbp, lm, true, faceOfRole, geomOfRole, bump); err != nil {
				return nil, err
			}
		}
		if lm.onEnd {
			if err := emitCapBand(budget, &mesh, cbp, lm, false, faceOfRole, geomOfRole, bump); err != nil {
				return nil, err
			}
		}
	}

	// Cap faces. Each is bounded, per loop, by that loop's contour ring where it
	// is chamfered on this cap and by its original ring where it is not — the
	// mixed profile evalCapBlendContext builds the same two faces from.
	if err := emitCapBlendCap(ctx, &mesh, cbp, lms, true, capStart, bump); err != nil {
		return nil, err
	}
	if err := emitCapBlendCap(ctx, &mesh, cbp, lms, false, capEnd, bump); err != nil {
		return nil, err
	}

	// A reflected placement flips handedness, turning every counter-clockwise
	// winding clockwise; reversing them restores outward orientation.
	if pl.reflected() {
		for i := range mesh.triangles {
			mesh.triangles[i][1], mesh.triangles[i][2] = mesh.triangles[i][2], mesh.triangles[i][1]
		}
	}

	if err := requireClosedMesh(&mesh); err != nil {
		return nil, fmt.Errorf(`%w: this cap-loop chamfer's cells do not close into a watertight boundary`, ErrUnsupported)
	}
	if err := requireVertexLinks(ctx, &mesh); err != nil {
		return nil, err
	}
	if err := requireCapBlendFacetAreas(&mesh); err != nil {
		return nil, err
	}
	if meshOrientationSign(mesh.vertices, mesh.triangles, pl.point(0, 0, cbp.z0)) <= 0 {
		return nil, fmt.Errorf(`%w: this cap-loop chamfer's assembled cells do not enclose a positive volume`, ErrUnsupported)
	}

	if err := composeCapBlendBounds(&mesh, faceExtra, store); err != nil {
		return nil, err
	}
	mesh.areaSlack = absSumUpper(mesh.areaSlack, meshStoreAreaAllow(&mesh, store))
	if isNonFinite(mesh.areaSlack) {
		return nil, fmt.Errorf(`%w: this cap-loop chamfer mesh states no finite area slack`, ErrUnsupported)
	}
	// The occupied-volume proof for the held-facet → bilinear → ruled → cone
	// chain is not built (docs/tessellation-reach-design.md §9), so this mesh
	// serves export alone and every boolean refuses it at operandSymDiff.
	mesh.symDiffOK = false
	return &mesh, nil
}

// chordCapBlendLoop resolves ONE loop's walks the way buildCapBand does and
// chords each of them ONCE, at the single count every level of that loop then
// reads (docs/tessellation-reach-design.md §7).
//
// For a circular wall of a chamfered loop the count is the larger of what the
// wall's own arc needs and what its cap-level offset arc needs, so the side
// ring, the band patch and the cap contour all carry the same number of
// stations and the band's strips meet along shared vertices rather than along
// two independently sampled polylines. A straight wall needs one sample; a
// reflex corner's connector arc is a directrix of its own and chords with its
// own count.
func chordCapBlendLoop(ctx context.Context, budget *workBudget, cbp capBlendPayload, li int, loop LoopRecord, chord float64, work *freeformWork) (capBlendLoopMesh, error) {
	if err := ctx.Err(); err != nil {
		return capBlendLoopMesh{}, err
	}
	cl, err := oneLoopCornerLoop(budget, loop, work)
	if err != nil {
		return capBlendLoopMesh{}, err
	}
	walks := cl.walks
	n := len(walks)
	lm := capBlendLoopMesh{
		li:      li,
		loop:    loop,
		walks:   walks,
		whole:   n == 1 && walks[0].closed,
		onStart: cbp.startLoops[li],
		onEnd:   cbp.endLoops[li],
	}
	lm.chamfered = lm.onStart || lm.onEnd

	lm.zLo = measuredScalar(cbp.z0, cbp.z0Delta)
	lm.zHi = measuredScalar(cbp.z1, cbp.z1Delta)
	setback := measuredScalar(cbp.d, cbp.dDelta)
	if lm.onStart {
		lm.zLo = boundedAdd(lm.zLo, setback)
	}
	if lm.onEnd {
		lm.zHi = boundedSub(lm.zHi, setback)
	}

	if lm.chamfered && !lm.whole {
		lm.joins, err = capOffsetJoins(budget, cl, cbp.d)
		if err != nil {
			return capBlendLoopMesh{}, err
		}
	}

	lm.count = make([]int, n)
	lm.sideSag = make([]float64, n)
	lm.capSag = make([]float64, n)
	lm.capRadius = make([]float64, n)
	lm.capTh0 = make([]float64, n)
	lm.capTh1 = make([]float64, n)
	for i, w := range walks {
		if err := budget.step(); err != nil {
			return capBlendLoopMesh{}, err
		}
		if !w.isCircular() {
			if !w.isLine() {
				return capBlendLoopMesh{}, fmt.Errorf(`%w: chording a cap-loop chamfer does not support walk kind %d`, ErrUnsupported, w.kind)
			}
			lm.count[i] = 1
			continue
		}
		// A cap-blend wall walk CAN be a whole closed curve — lm.whole is that
		// case — so it takes chordWalkMin's own minimum rather than a fixed
		// one: three chords for a whole circle, which is what keeps its
		// polygon bounded, and one otherwise.
		nSide, _, err := chordCount(w.segmentWalk, chord, chordWalkMin(w.segmentWalk))
		if err != nil {
			return capBlendLoopMesh{}, err
		}
		count := nSide
		if lm.chamfered {
			r, err := capBandRadius(w, cbp.d)
			if err != nil {
				return capBlendLoopMesh{}, err
			}
			lm.capRadius[i] = r
			lm.capTh0[i], lm.capTh1[i] = w.th0, w.th1
			if !lm.whole {
				start, end := capWallFoot(lm.joins, i, n)
				lm.capTh0[i], lm.capTh1[i], _ = capWallSweep(w.cU, w.cV, start, end, w.th1-w.th0)
			}
			capWalk := segmentWalk{kind: walkCircular, radius: r, th0: lm.capTh0[i], th1: lm.capTh1[i], closed: w.closed}
			// capWalk carries the wall walk's own closed bit, so its minimum
			// is the wall's: a whole closed wall's cap contour is a whole
			// closed circle too. Both counts are read at their own minimum
			// before max() shares one of them across the side ring, the band
			// patch and the cap contour.
			nCap, _, err := chordCount(capWalk, chord, chordWalkMin(capWalk))
			if err != nil {
				return capBlendLoopMesh{}, err
			}
			count = max(nSide, nCap)
		}
		lm.count[i] = count
		lm.sideSag[i] = chordSagitta(w.radius, math.Abs(w.th1-w.th0), count)
		if lm.chamfered {
			lm.capSag[i] = chordSagitta(lm.capRadius[i], math.Abs(lm.capTh1[i]-lm.capTh0[i]), count)
		}
	}

	lm.arcCount = make([]int, n)
	lm.arcSag = make([]float64, n)
	lm.arcTh0 = make([]float64, n)
	lm.arcTh1 = make([]float64, n)
	lm.locusGap = make([]float64, n)
	for i := range n {
		if lm.joins == nil {
			break
		}
		if err := budget.step(); err != nil {
			return capBlendLoopMesh{}, err
		}
		j := lm.joins[i]
		if !j.arc {
			gap, err := capBlendCornerLocusGap(budget, cbp, walks, i, j)
			if err != nil {
				return capBlendLoopMesh{}, err
			}
			lm.locusGap[i] = gap
			continue
		}
		// The inward offset's reflex connector walks CLOCKWISE from pA to pB,
		// exactly as buildCapBand records it, so th1 is normalized BELOW th0 and
		// the swept angle is the corner's own reflex turn rather than its
		// complement — which would chord a curve the offset never emitted.
		th0 := math.Atan2(j.pA.V-j.vV, j.pA.U-j.vU)
		th1 := math.Atan2(j.pB.V-j.vV, j.pB.U-j.vU)
		for th1 > th0 {
			th1 -= 2 * math.Pi
		}
		// The connector's minimum is ONE, stated rather than inherited: this
		// arc runs between two distinct offset feet of two distinct walks, so
		// it is never a whole closed curve and never reaches the three-chord
		// minimum a closed walk needs. Its sweep is the corner's own reflex
		// turn, which is strictly less than a full turn by construction.
		connector := segmentWalk{kind: walkCircular, radius: cbp.d, th0: th0, th1: th1}
		cnt, _, err := chordCount(connector, chord, 1)
		if err != nil {
			return capBlendLoopMesh{}, err
		}
		lm.arcCount[i] = cnt
		lm.arcTh0[i], lm.arcTh1[i] = th0, th1
		lm.arcSag[i] = chordSagitta(cbp.d, th0-th1, cnt)
	}

	if err := emitCapBlendSamples(budget, cbp, &lm); err != nil {
		return capBlendLoopMesh{}, err
	}
	return lm, nil
}

// emitCapBlendSamples fills one loop's side and cap sample arrays at the counts
// chordCapBlendLoop chose. A junction is emitted exactly once — every piece
// contributes its own START and leaves its end to the piece that follows — so
// the two rings close by construction rather than by comparison.
func emitCapBlendSamples(budget *workBudget, cbp capBlendPayload, lm *capBlendLoopMesh) error {
	n := len(lm.walks)
	lm.sideStart = make([]int, n)
	for i, w := range lm.walks {
		lm.sideStart[i] = len(lm.sidePts)
		if !w.isCircular() {
			lm.sidePts = append(lm.sidePts, Point2{U: w.startU, V: w.startV})
			lm.sideBound = append(lm.sideBound, w.startBound)
			continue
		}
		seg := lm.loop.Segments[w.segs[0]]
		count := lm.count[i]
		dth := (w.th1 - w.th0) / float64(count)
		for k := range count {
			if err := budget.step(); err != nil {
				return err
			}
			p := Point2{U: w.startU, V: w.startV}
			bound := w.startBound
			if k > 0 {
				th := w.th0 + float64(k)*dth
				p = Point2{U: w.cU + w.radius*math.Cos(th), V: w.cV + w.radius*math.Sin(th)}
				bound = chordStationBound(seg, k, count, p.U, p.V)
			}
			lm.sidePts = append(lm.sidePts, p)
			lm.sideBound = append(lm.sideBound, bound)
		}
	}
	if !lm.chamfered {
		return nil
	}

	lm.capWallStart = make([]int, n)
	lm.capArcStart = make([]int, n)
	for i := range n {
		lm.capArcStart[i] = -1
	}
	addCap := func(p Point2, bound walkEndBound) {
		lm.capPts = append(lm.capPts, p)
		lm.capBound = append(lm.capBound, bound)
	}
	if lm.whole {
		w := lm.walks[0]
		lm.capWallStart[0] = 0
		count := lm.count[0]
		dth := (lm.capTh1[0] - lm.capTh0[0]) / float64(count)
		for k := range count {
			if err := budget.step(); err != nil {
				return err
			}
			th := lm.capTh0[0] + float64(k)*dth
			p := Point2{U: w.cU + lm.capRadius[0]*math.Cos(th), V: w.cV + lm.capRadius[0]*math.Sin(th)}
			addCap(p, capStationBound(w.cU, w.cV, lm.capRadius[0], th, p.U, p.V))
		}
		return nil
	}

	for i, w := range lm.walks {
		lm.capWallStart[i] = len(lm.capPts)
		start, _ := capWallFoot(lm.joins, i, n)
		if !w.isCircular() {
			// A straight wall's cap directrix is the offset SEGMENT between two
			// corner feet, so it holds one station and chords nothing. The foot
			// itself is the point the offset denotes, within the band's own
			// contour displacement, which is charged as its own term.
			addCap(start, walkEndBound{})
		} else {
			count := lm.count[i]
			dth := (lm.capTh1[i] - lm.capTh0[i]) / float64(count)
			for k := range count {
				if err := budget.step(); err != nil {
					return err
				}
				th := lm.capTh0[i] + float64(k)*dth
				p := Point2{U: w.cU + lm.capRadius[i]*math.Cos(th), V: w.cV + lm.capRadius[i]*math.Sin(th)}
				if k == 0 {
					// Station 0 is the corner foot VERBATIM, so the wall patch and
					// the piece before it close on one vertex. Its own gap from the
					// station it stands for is measured, never assumed zero.
					p = start
				}
				addCap(p, capStationBound(w.cU, w.cV, lm.capRadius[i], th, p.U, p.V))
			}
		}
		ni := (i + 1) % n
		j := lm.joins[ni]
		if !j.arc {
			continue
		}
		lm.capArcStart[ni] = len(lm.capPts)
		count := lm.arcCount[ni]
		dth := (lm.arcTh1[ni] - lm.arcTh0[ni]) / float64(count)
		for k := range count {
			if err := budget.step(); err != nil {
				return err
			}
			th := lm.arcTh0[ni] + float64(k)*dth
			p := Point2{U: j.vU + cbp.d*math.Cos(th), V: j.vV + cbp.d*math.Sin(th)}
			if k == 0 {
				p = j.pA
			}
			addCap(p, capStationBound(j.vU, j.vV, cbp.d, th, p.U, p.V))
		}
	}
	return nil
}

// capStationBound is the certified plane-local gap between a cap-contour sample
// the build HOLDS and the point that sample's own station denotes on the held
// offset circle: centre plus radius times the sine and cosine of one exact
// float angle, each enclosed through normal_bound.go's radSinCosInterval.
//
// It is chordStationBound's cap-level twin, and it answers a different question
// only because the curve is different: a cap contour is a curve this evaluator
// COMPUTED, not one the record states, so the station's denoted point is the
// point on the held offset circle and the gap from THAT circle to the one the
// offset denotes is the band's own contour displacement, charged as its own
// term beside this one and never folded into it.
//
// An angle, radius or centre this arithmetic cannot enclose answers +Inf on
// both components — the underivable bound the tessellation refuses on
// (docs/tessellation-design.md §12) — never a zero.
func capStationBound(cU, cV, radius, theta, heldU, heldV float64) walkEndBound {
	underivable := walkEndBound{u: math.Inf(1), v: math.Inf(1)}
	rt, rr := floatRat(theta), floatRat(radius)
	ru, rv := floatRat(cU), floatRat(cV)
	if rt == nil || rr == nil || ru == nil || rv == nil {
		return underivable
	}
	sin, cos, ok := radSinCosInterval(rt)
	if !ok {
		return underivable
	}
	uIv := intervalAdd(pointInterval(ru), intervalMul(pointInterval(rr), cos))
	vIv := intervalAdd(pointInterval(rv), intervalMul(pointInterval(rr), sin))
	return walkEndBound{u: intervalFloatError(uIv, heldU), v: intervalFloatError(vIv, heldV)}
}

// capBlendCornerLocusGap is how far a MITER corner's built ruling — an Edge
// tagged Line3, straight from the cap-level foot down to the original corner —
// can sit from the conic miter locus it stands for
// (docs/tessellation-reach-design.md §7's locusGap).
//
// The locus is a curve of proven length at most L between two endpoints exactly
// c apart, so it lies inside the ellipse with those two endpoints as foci and
// major axis L, whose semi-minor axis is sqrt(L² − c²)/2. L is the same
// subdivided bound capSlantEdge charges the ruling's own length against
// (capMiterLocusUpper), and c² is taken exactly over the rationals so the
// difference is never inflated by a square root this package cannot bound.
//
// It is zero where both loci are affine in the offset amount — a line-line
// miter, and every reflex corner's own two feet, which ride one carrier each —
// and it is the SAME number at either cap, since the two differ only in the
// sign of an axial span both readings take the magnitude of. A sub-range whose
// speed cannot be enclosed answers +Inf, which refuses.
func capBlendCornerLocusGap(budget *workBudget, cbp capBlendPayload, walks []sideWalk, i int, j cornerJoin) (float64, error) {
	n := len(walks)
	prev, cur := walks[(i+n-1)%n], walks[i]
	if !prev.isCircular() && !cur.isCircular() {
		return 0, nil
	}
	locus, ok, err := capMiterLocusUpper(budget, prev, cur, j.vU, j.vV, cbp.d, cbp.d)
	if err != nil {
		return 0, err
	}
	if !ok || isNonFinite(locus) {
		return 0, fmt.Errorf(`%w: a cap-loop chamfer's miter ruling states no enclosure of the locus it stands for, so this mesh can publish no displacement bound for the patches that share it`, ErrUnsupported)
	}
	chordSq := ratSquaredDistance3(j.m.U, j.m.V, cbp.d, j.vU, j.vV, 0)
	chordSqDown, exact := chordSq.Float64()
	if !exact {
		chordSqDown = math.Nextafter(chordSqDown, math.Inf(-1))
	}
	diff := upRound(productUpper(locus, locus) - chordSqDown)
	if diff <= 0 {
		return 0, nil
	}
	return upRound(math.Sqrt(diff) / 2), nil
}

// emitCapBand writes one band's facets — a quad strip per wall patch and a fan
// per reflex corner's apex patch — and publishes each patch's own displacement
// through docs/tessellation-reach-design.md §7's term table.
func emitCapBand(budget *workBudget, m *Mesh, cbp capBlendPayload, lm *capBlendLoopMesh, start bool, faceOfRole func(string) (*Face, error), geomOfRole map[string]capPatchGeom, bump func(*Face, float64)) error {
	matSign := 1.0
	capZ := cbp.z0
	sideV, capV := lm.sideLo, lm.capLoV
	if !start {
		matSign, capZ = -1, cbp.z1
		sideV, capV = lm.sideHi, lm.capHiV
	}
	sideZ := capZ + matSign*cbp.d
	delta, ok := cbp.bandDelta[capBandKey{loop: lm.li, start: start}]
	if !ok {
		return fmt.Errorf(`%w: the payload states no contour displacement for the chamfer band on loop %d`, ErrDegenerate, lm.li)
	}
	levelDelta := absSumUpper(cbp.dDelta, addRoundError(capZ, matSign*cbp.d, sideZ))
	axial := cbp.capBandLevel(capZ, matSign).bound
	n := len(lm.walks)

	// Patch indices are buildCapBand's own: every reflex corner's apex patch in
	// corner order first, then every wall patch in walk order. Two faces sharing
	// one role string would collapse every last-wins reader onto one of them, so
	// the numbering is reproduced here rather than guessed.
	apexIdx := map[int]int{}
	next := 0
	for i := range n {
		if lm.joins != nil && lm.joins[i].arc {
			apexIdx[i] = next
			next++
		}
	}
	capName := capNameOf(matSign)
	patchFace := func(p int) (*Face, capPatchGeom, error) {
		role := fmt.Sprintf("chamferCap(%s,%d,%d)", capName, lm.li, p)
		f, err := faceOfRole(role)
		if err != nil {
			return nil, capPatchGeom{}, err
		}
		g, ok := geomOfRole[role]
		if !ok {
			return nil, capPatchGeom{}, fmt.Errorf(`%w: the payload states no geometry for patch role %q`, ErrDegenerate, role)
		}
		return f, g, nil
	}

	// Apex patches: a fan from the ORIGINAL corner vertex at the side level out
	// to the connector arc's own stations. Its side directrix is a point, so the
	// held triangle IS the ruled patch between the point and the chord and no
	// twist term arises.
	for i := range n {
		p, isApex := apexIdx[i]
		if !isApex {
			continue
		}
		if err := budget.step(); err != nil {
			return err
		}
		face, g, err := patchFace(p)
		if err != nil {
			return err
		}
		if !g.circular || g.sideRadius != 0 {
			return fmt.Errorf(`%w: patch %d of the chamfer band on loop %d is not the apex patch its corner states`, ErrDegenerate, p, lm.li)
		}
		apex := sideV[lm.sideStart[i]]
		base := lm.capArcStart[i]
		count := lm.arcCount[i]
		for k := range count {
			a := capV[base+k]
			b := capV[base+k+1]
			if k == count-1 {
				// A reflex corner's connector runs pA -> pB, and pB is the
				// FOLLOWING wall's own first cap station, so the fan's last
				// triangle closes on that vertex rather than one of its own.
				b = capV[lm.capWallStart[i]]
			}
			if start {
				m.addTriangle([3]int{a, b, apex}, face)
			} else {
				m.addTriangle([3]int{apex, b, a}, face)
			}
		}
		bump(face, absSumUpper(lm.arcSag[i], delta, levelDelta, axial))
		m.areaSlack = absSumUpper(m.areaSlack, patchDisplacementAreaAllow(g),
			capBlendPatchFacetAllow(m, count, lm.arcSag[i]))
	}

	// Wall patches: one quad per shared station, split on the fixed diagonal.
	for i, w := range lm.walks {
		if err := budget.step(); err != nil {
			return err
		}
		face, g, err := patchFace(next + i)
		if err != nil {
			return err
		}
		if g.circular != w.isCircular() {
			return fmt.Errorf(`%w: patch %d of the chamfer band on loop %d does not state the geometry of the wall it descends from`, ErrDegenerate, next+i, lm.li)
		}
		count := lm.count[i]
		twist := 0.0
		first := len(m.triangles)
		for k := range count {
			s0 := lm.sideStart[i] + k
			s1 := lm.sideStart[i] + k + 1
			c0 := lm.capWallStart[i] + k
			c1 := lm.capWallStart[i] + k + 1
			if k == count-1 {
				s1 = lm.sideStart[(i+1)%n]
				c1 = capBlendNextCapSample(lm, (i+1)%n)
			}
			lo0, lo1, hi0, hi1 := capV[c0], capV[c1], sideV[s0], sideV[s1]
			if !start {
				lo0, lo1, hi0, hi1 = sideV[s0], sideV[s1], capV[c0], capV[c1]
			}
			m.addTriangle([3]int{lo0, lo1, hi1}, face)
			m.addTriangle([3]int{lo0, hi1, hi0}, face)
			// The held pair departs from the bilinear patch through its four
			// corners by cellTwistOffsetUpper. It is zero where the denoted cell
			// is PLANAR — a straight wall's trapezoid, and a circular wall whose
			// two directrices sweep the same window — because the two triangles
			// then tile that quad exactly and the corners' own displacement is
			// already charged as deltaStore.
			if g.circular && capPatchWindowSkew(g) > 0 {
				twist = math.Max(twist, cellTwistOffsetUpper(
					m.vertices[sideV[s0]], m.vertices[sideV[s1]],
					m.vertices[capV[c0]], m.vertices[capV[c1]]))
			}
		}
		skew := capPatchWindowSkew(g)
		if isNonFinite(skew) {
			return fmt.Errorf(`%w: a chamfer band patch states no bound on how far its two directrix windows differ`, ErrUnsupported)
		}
		locus := 0.0
		radiusRound := 0.0
		sagitta := 0.0
		if lm.joins != nil {
			locus = math.Max(lm.locusGap[i], lm.locusGap[(i+1)%n])
		}
		if w.isCircular() {
			sagitta = math.Max(lm.sideSag[i], lm.capSag[i])
			inside := 1.0
			if w.th1 < w.th0 {
				inside = -1
			}
			radiusRound = addRoundError(w.radius, -inside*cbp.d, lm.capRadius[i])
		}
		patchDelta := absSumUpper(twist, sagitta, productUpper(g.capRadius, skew), locus, radiusRound)
		bump(face, absSumUpper(patchDelta, delta, levelDelta, axial))
		m.areaSlack = absSumUpper(m.areaSlack, patchDisplacementAreaAllow(g),
			capBlendFacetAllow(m, first, patchDelta))
	}
	return nil
}

// capBlendNextCapSample is the cap ring index the piece starting at corner i
// opens on: the connector arc's own first station at a reflex corner, and the
// following wall's own first station elsewhere. It is what closes a wall patch's
// last quad and an apex patch's last fan triangle onto the vertex the next piece
// starts from, so no strip ends on a vertex of its own.
func capBlendNextCapSample(lm *capBlendLoopMesh, i int) int {
	if lm.capArcStart != nil && lm.capArcStart[i] >= 0 {
		return lm.capArcStart[i]
	}
	return lm.capWallStart[i]
}

// capBlendFacetAllow sums docs/tessellation-design.md §5's per-triangle area
// allowance over the facets a patch emitted from index first onward, each
// charged the patch's own displacement from the surface it stands for.
func capBlendFacetAllow(m *Mesh, first int, delta float64) float64 {
	if delta <= 0 {
		return 0
	}
	total := 0.0
	for _, tri := range m.triangles[first:] {
		a, b, c := m.vertices[tri[0]], m.vertices[tri[1]], m.vertices[tri[2]]
		total = absSumUpper(total, perturbedTriangleAreaAllow(a, b, c, delta))
	}
	return total
}

// capBlendPatchFacetAllow is capBlendFacetAllow for a patch whose facets were
// just appended, counted from the end.
func capBlendPatchFacetAllow(m *Mesh, count int, delta float64) float64 {
	if count <= 0 || count > len(m.triangles) {
		return 0
	}
	return capBlendFacetAllow(m, len(m.triangles)-count, delta)
}

// emitCapBlendCap triangulates one cap face over the ring bounding it per loop:
// the offset contour ring where that loop is chamfered on this cap, and the
// original ring where it is not. The rings must clear one another by more than
// their own sagitta tubes, or a cap at this tolerance cannot prove it represents
// the region's topology and refuses rather than return a pinched mesh.
func emitCapBlendCap(ctx context.Context, m *Mesh, cbp capBlendPayload, lms []capBlendLoopMesh, start bool, face *Face, bump func(*Face, float64)) error {
	var pts []Point2
	var vtx []int
	var loopIdx [][]int
	var sag []float64
	trim, delta := 0.0, 0.0
	for li := range lms {
		lm := &lms[li]
		chamfered := lm.onStart
		if !start {
			chamfered = lm.onEnd
		}
		ringPts, ringV := lm.sidePts, lm.sideLo
		ringSag := capBlendRingSagitta(lm, false)
		if !start {
			ringV = lm.sideHi
		}
		if chamfered {
			ringPts, ringV = lm.capPts, lm.capLoV
			if !start {
				ringV = lm.capHiV
			}
			ringSag = capBlendRingSagitta(lm, true)
			d, ok := cbp.bandDelta[capBandKey{loop: lm.li, start: start}]
			if !ok {
				return fmt.Errorf(`%w: the payload states no contour displacement for the chamfer band on loop %d`, ErrDegenerate, lm.li)
			}
			delta = math.Max(delta, d)
		}
		base := len(pts)
		pts = append(pts, ringPts...)
		vtx = append(vtx, ringV...)
		idx := make([]int, len(ringPts))
		for k := range ringPts {
			idx[k] = base + k
		}
		loopIdx = append(loopIdx, idx)
		sag = append(sag, ringSag)
		trim = math.Max(trim, ringSag)
	}
	if err := requireLoopClearance(ctx, pts, loopIdx, sag); err != nil {
		return err
	}
	// This cap's own chord-versus-curve deficit: the planar area between each
	// bounding ring and the curve it chords, one term per loop, the same charge
	// walkAreaSlack makes per cap for a prism.
	for li := range lms {
		lm := &lms[li]
		chamfered := lm.onStart
		if !start {
			chamfered = lm.onEnd
		}
		m.areaSlack = absSumUpper(m.areaSlack, capBlendRingSegmentArea(lm, chamfered, cbp.d))
	}
	tris, err := triangulate2DContext(ctx, pts, loopIdx)
	if err != nil {
		return err
	}
	for _, tri := range tris {
		if start {
			m.addTriangle([3]int{vtx[tri[0]], vtx[tri[2]], vtx[tri[1]]}, face)
			continue
		}
		m.addTriangle([3]int{vtx[tri[0]], vtx[tri[1]], vtx[tri[2]]}, face)
	}
	axial := cbp.z0Delta
	if !start {
		axial = cbp.z1Delta
	}
	bump(face, absSumUpper(trim, delta, axial))
	return nil
}

// capBlendRingSagitta is the largest sagitta a loop's ring at one level took:
// the side directrix's over the original walks, or the cap contour's over the
// offset arcs and every reflex corner's connector.
func capBlendRingSagitta(lm *capBlendLoopMesh, contour bool) float64 {
	worst := 0.0
	for i := range lm.walks {
		s := lm.sideSag[i]
		if contour {
			s = lm.capSag[i]
		}
		worst = math.Max(worst, s)
	}
	if !contour {
		return worst
	}
	for i := range lm.arcSag {
		worst = math.Max(worst, lm.arcSag[i])
	}
	return worst
}

// capBlendRingSegmentArea is the planar area one loop's chorded ring at a cap
// loses or gains against the curve it stands for — the closed-form circular
// segment sum walkSegmentArea states, taken over the ORIGINAL walks for an
// unchamfered cap and over the offset arcs plus every reflex connector for a
// chamfered one, since those are the curves that cap's boundary actually
// follows.
func capBlendRingSegmentArea(lm *capBlendLoopMesh, contour bool, d float64) float64 {
	total := 0.0
	for i, w := range lm.walks {
		if !w.isCircular() {
			continue
		}
		if !contour {
			total = absSumUpper(total, walkSegmentArea(w.segmentWalk, lm.count[i]))
			continue
		}
		total = absSumUpper(total, walkSegmentArea(segmentWalk{
			kind: walkCircular, radius: lm.capRadius[i],
			th0: lm.capTh0[i], th1: lm.capTh1[i], closed: w.closed,
		}, lm.count[i]))
	}
	if !contour {
		return total
	}
	for i, count := range lm.arcCount {
		if count == 0 {
			continue
		}
		total = absSumUpper(total, walkSegmentArea(segmentWalk{
			kind: walkCircular, radius: d, th0: lm.arcTh0[i], th1: lm.arcTh1[i],
		}, count))
	}
	return total
}

// composeCapBlendBounds publishes docs/tessellation-design.md §2's sourceBound
// for every face the mesh names: the face's own accumulated trim, band and axial
// terms plus the largest store displacement its own vertices carry. Every source
// face is present by construction — the walk is over mesh.source itself — and a
// face whose composed displacement is not finite refuses rather than publishing
// an infinite bound.
func composeCapBlendBounds(m *Mesh, extra map[*Face]float64, store []float64) error {
	faceStore := map[*Face]float64{}
	for i, f := range m.source {
		for _, v := range m.triangles[i] {
			faceStore[f] = math.Max(faceStore[f], store[v])
		}
	}
	for f, s := range faceStore {
		bound := upRound(extra[f] + s)
		if isNonFinite(bound) {
			return fmt.Errorf(`%w: a cap-loop chamfer face's composed displacement is not finite, so this mesh can state no bound for it`, ErrUnsupported)
		}
		m.setFaceBound(f, bound)
	}
	return nil
}

// requireCapBlendFacetAreas refuses a mesh carrying a zero-area triangle
// (docs/tessellation-design.md §12). The test is the exact rational squared
// cross product, so a sliver is judged by what its own coordinates say rather
// than by a tolerance.
func requireCapBlendFacetAreas(m *Mesh) error {
	for i, tri := range m.triangles {
		a, b, c := m.vertices[tri[0]], m.vertices[tri[1]], m.vertices[tri[2]]
		if capBlendTwiceAreaSq(a, b, c).Sign() <= 0 {
			return fmt.Errorf(`%w: facet %d of this cap-loop chamfer mesh has zero area`, ErrUnsupported, i)
		}
	}
	return nil
}

// capBlendTwiceAreaSq is the exact squared length of (b−a)×(c−a), zero exactly
// for three collinear or coincident held corners.
func capBlendTwiceAreaSq(a, b, c r3.Vec) *big.Rat {
	sub := func(p, q r3.Vec) [3]*big.Rat {
		return [3]*big.Rat{
			new(big.Rat).Sub(floatRat(p.X), floatRat(q.X)),
			new(big.Rat).Sub(floatRat(p.Y), floatRat(q.Y)),
			new(big.Rat).Sub(floatRat(p.Z), floatRat(q.Z)),
		}
	}
	u, v := sub(b, a), sub(c, a)
	cross := [3]*big.Rat{
		new(big.Rat).Sub(new(big.Rat).Mul(u[1], v[2]), new(big.Rat).Mul(u[2], v[1])),
		new(big.Rat).Sub(new(big.Rat).Mul(u[2], v[0]), new(big.Rat).Mul(u[0], v[2])),
		new(big.Rat).Sub(new(big.Rat).Mul(u[0], v[1]), new(big.Rat).Mul(u[1], v[0])),
	}
	out := new(big.Rat)
	for _, k := range cross {
		out.Add(out, new(big.Rat).Mul(k, k))
	}
	return out
}
