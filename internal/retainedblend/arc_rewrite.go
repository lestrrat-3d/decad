package retainedblend

import (
	"context"
	"fmt"
	"math"
	"slices"
	"sort"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/measurement"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
)

// arcRow is one row of an open-arc blend: its tube position, the circle it
// lies on, and the angles at which that circle meets the two end faces.
type arcRow struct {
	rho, z float64
	theta  [2]float64
}

// colPt is a vertex of a row with its position along the row, in [0, 1].
type colPt struct {
	idx int
	t   float64
}

// arcProfile gives the circle (radius, height) of the blend at parameter s in
// [0, 1], from the plane's setback circle to the wall's.
type arcProfile func(s float64) (rho, z float64)

func (a arcRim) profile(spec blendSpec) (arcProfile, bool) {
	kappa, side := a.kappa, a.side
	if spec.fillet {
		r := spec.r
		rhoC := a.radius + kappa*r
		return func(s float64) (float64, float64) {
			alpha := s * math.Pi / 2
			return rhoC - kappa*r*math.Sin(alpha), side * r * (1 - math.Cos(alpha))
		}, true
	}
	rho1 := a.radius + kappa*spec.dA
	return func(s float64) (float64, float64) {
		return rho1 + s*(a.radius-rho1), s * side * spec.dB
	}, false
}

// rowAngles are the angles at which the circle at parameter s meets each end
// face, each within π of ref.
func (a arcRim) rowAngles(p arcProfile, s float64, ref [2]float64) ([2]float64, error) {
	rho, _ := p(s)
	var out [2]float64
	for i := range 2 {
		t, ok := a.endAzimuths(i, rho, ref[i])
		if !ok {
			return out, fmt.Errorf("%w: a blend circle does not reach an end face", decaderr.ErrUnsupported)
		}
		out[i] = t
	}
	return out, nil
}

// rewriteArc blends one open arc and builds the body over the rewritten mesh.
func rewriteArc(ctx context.Context, env topo.Env, fp facetedrecord.Payload, a arcRim, spec blendSpec,
	role string, ord int, faceIndex map[*topo.Face]int) (*topo.Body, error) {
	cyl := a.cyl[0] != nil
	if cyl && !spec.fillet {
		return nil, fmt.Errorf("%w: a chamfer of an arc that ends on a cylinder is not built", decaderr.ErrUnsupported)
	}
	if err := a.checkSize(spec); err != nil {
		return nil, err
	}
	w := newWork(fp, a.rim, spec)
	w.faceIx = faceIndex
	slack := a.slack()
	margin := w.tau + slack
	n := a.n

	var faces [4]faceLoops
	for k, f := range []*topo.Face{a.planeFace, a.wallFace, a.ends[0].face, a.ends[1].face} {
		var err error
		var end *plane
		if k >= 2 && !cyl {
			end = &a.ends[k-2].g
		}
		if k >= 2 && cyl {
			// A cylinder end face keeps its facets and its interior vertices.
			faces[k], err = w.cylFaceLoops(faceIndex[f])
		} else {
			faces[k], err = w.faceLoops(faceIndex[f], end)
		}
		if err != nil {
			return nil, err
		}
	}
	fA, fB, fG := faces[0], faces[1], [2]faceLoops{faces[2], faces[3]}

	// Locate the arc in the plane's loops and in the wall's.
	onArc := func(v int) bool { return a.onRim(w.verts[v], w.beta[v]) }
	var onLineA, onGen [2]func(int) bool
	var tA [2]r3.Vec
	for i := range 2 {
		end := a.ends[i]
		onGen[i] = func(v int) bool { return distToLine(w.verts[v], end.exact, n) <= w.beta[v]+slack }
		if cyl {
			onLineA[i] = a.chainWalker(w, i, fG[i])
			continue
		}
		t, ok := n.Cross(end.g.n).Normalize()
		if !ok {
			return nil, fmt.Errorf("%w: an end face is parallel to the plane", decaderr.ErrUnsupported)
		}
		tA[i] = t
		onLineA[i] = func(v int) bool { return distToLine(w.verts[v], end.exact, t) <= w.beta[v]+slack }
	}
	runA, err := w.locateBy(fA, a.ends[0].v, a.ends[1].v, onArc, onLineA)
	if err != nil {
		return nil, err
	}
	runB, err := w.locateBy(fB, a.ends[0].v, a.ends[1].v, onArc, onGen)
	if err != nil {
		return nil, err
	}
	// At each end the wall's chain along the end plane either climbs the wall,
	// away from the plane (the wall trims there), or runs back past the plane
	// (the wall extends past the end plane, which only a concave arc admits,
	// agent-docs/retained-edge-blend-design.md Table E).
	var kindB [2]sideKind
	for i := range 2 {
		if cyl {
			if err := a.readCylChain(w, i, runA.chain[i], margin); err != nil {
				return nil, err
			}
		} else {
			dir := tA[i]
			if w.verts[runA.chain[i][len(runA.chain[i])-1]].Sub(a.ends[i].exact).Dot(dir) < 0 {
				dir = dir.Scale(-1)
			}
			runA.dir[i] = dir
		}
		last := runB.chain[i][len(runB.chain[i])-1]
		toward := w.verts[last].Sub(a.ends[i].exact).Dot(n) * a.side
		switch {
		case toward > margin:
			kindB[i] = trims
		case !a.convex && -toward > margin+w.beta[last]:
			kindB[i] = extends
		default:
			return nil, fmt.Errorf("%w: the wall's generator at an arc end leaves the wrong way", decaderr.ErrUnsupported)
		}
		if cyl && kindB[i] != extends {
			return nil, fmt.Errorf("%w: a root arc ends on a cylinder its wall does not run on past", decaderr.ErrUnsupported)
		}
	}
	bandMode := a.twoSided || kindB[0] == extends || kindB[1] == extends

	// The profile and its rows.
	profile, _ := a.profile(spec)
	ta, tb := spec.planeSetback(), spec.wallSetback()
	rhoA := a.radius + a.kappa*ta
	ref := [2]float64{a.theta0, a.theta0 + a.span}
	var ends []endSection
	for i := range 2 {
		sec, err := a.endSectionAt(spec, i)
		if err != nil {
			return nil, err
		}
		ends = append(ends, sec)
	}
	// The rows end on both end planes; their chords there follow the end
	// curves to within tau (§6.3).
	ss, err := arcRows(spec, w.tau, arcProfileShare(w.tau), ends)
	if err != nil {
		return nil, err
	}
	m := len(ss) - 1
	rows := []arcRow{}
	prev := ref
	for _, s := range ss {
		th, err := a.rowAngles(profile, s, prev)
		if err != nil {
			return nil, err
		}
		rho, z := profile(s)
		rows = append(rows, arcRow{rho: rho, z: z, theta: th})
		prev = th
	}
	if !(rows[0].theta[1]-rows[0].theta[0] > minSine) {
		return nil, fmt.Errorf("%w: the plane's setback arc has no length", decaderr.ErrDegenerate)
	}

	// The plane: the setback points, and the clearance of its chains.
	var remA [2][]int
	for i := range 2 {
		end := a.ends[i]
		p := a.atTheta(rows[0].rho, rows[0].theta[i], 0)
		if cyl {
			if remA[i], err = a.cylSetback(w, i, p, runA.chain[i], margin); err != nil {
				return nil, err
			}
			continue
		}
		tp := tau(p, end.exact, runA.dir[i])
		chain := runA.chain[i]
		last := tau(w.verts[chain[len(chain)-1]], end.exact, runA.dir[i])
		if tp <= margin {
			return nil, fmt.Errorf("%w: a setback point lies behind an arc end", decaderr.ErrUnsupported)
		}
		if err := clearance(last-tp, w.beta[chain[len(chain)-1]]+margin, "the plane beside the arc"); err != nil {
			return nil, err
		}
		for _, v := range chain {
			tc := tau(w.verts[v], end.exact, runA.dir[i])
			if math.Abs(tc-tp) <= 4*margin {
				return nil, fmt.Errorf("%w: a setback point falls on a vertex of its face's boundary", decaderr.ErrUnsupported)
			}
			if tc < tp {
				remA[i] = append(remA[i], v)
			}
		}
	}

	// The wall is cut at the setback height.
	generator := map[int]bool{}
	for i := range 2 {
		for _, v := range runB.chain[i] {
			generator[v] = true
		}
	}
	var cutIDs []int
	var remB [2][]int
	var pB [2]int
	var cuts bandCuts
	if bandMode {
		cutIDs, remB, pB, cuts, err = w.clipArcWallBand(fB, a, tb, runB, kindB, margin)
	} else {
		cutIDs, remB, pB, err = w.clipArcWall(fB, a, tb, generator, runB, margin)
	}
	if err != nil {
		return nil, err
	}
	// Where the wall extends past an end plane, a fillet's end cap closes in a
	// cusp at the wall's tangent point. If the cap the profile's own rows make is
	// not simple, the rows are spaced again so the last row before the wall's lies
	// farther from the wall's cut than that cut can stray (clearCusp in
	// cyl_rewrite.go). The widened last gap's chord on each end plane is charged
	// below.
	widened := 0.0
	if spec.fillet && (kindB[0] == extends || kindB[1] == extends) {
		reach, simple := 0.0, true
		for i := range 2 {
			if kindB[i] != extends {
				continue
			}
			run, err := cuts.endRun(i, a.ends[i].v)
			if err != nil {
				return nil, err
			}
			reach = max(reach, w.beta[cuts.corner[i]])
			pts := make([]r3.Vec, 0, m+len(run)+2)
			for _, row := range rows[:m] {
				pts = append(pts, a.atTheta(row.rho, row.theta[i], row.z))
			}
			pts = append(pts, w.verts[cuts.corner[i]])
			for _, v := range run {
				reach = max(reach, w.beta[v])
			}
			closeAt := a.ends[i].v
			if cyl {
				// The cap reads its loop without the vertices beside V read as on its
				// radial plane, and closes at the last of them (capFacets).
				if run, closeAt, err = a.closeCap(i, run); err != nil {
					return nil, err
				}
			}
			for _, v := range run {
				pts = append(pts, w.verts[v])
			}
			pts = append(pts, w.verts[closeAt])
			if cyl {
				simple = simple && capSimple2(a.capPoly(i, pts), m) == nil
			} else {
				simple = simple && capSimplePts(a.ends[i].g.n, pts, m) == nil
			}
		}
		if !simple {
			need := proofbound.AbsSumUpper(reach, margin)
			sStar := 1.0
			for i := range 2 {
				if kindB[i] != extends {
					continue
				}
				s, err := a.cuspParam(spec, ends[i], need)
				if err != nil {
					return nil, err
				}
				sStar = min(sStar, s)
			}
			if ss[m-1] > sStar {
				if ss, err = widenRows(spec, w.tau, ends, ss, sStar); err != nil {
					return nil, err
				}
				m = len(ss) - 1
				rows = rows[:0]
				prev = ref
				for _, s := range ss {
					th, err := a.rowAngles(profile, s, prev)
					if err != nil {
						return nil, err
					}
					rho, z := profile(s)
					rows = append(rows, arcRow{rho: rho, z: z, theta: th})
					prev = th
				}
				widened = sStar
			}
		}
	}

	// The plane's row, the wall's row, and the rows between them. Each row's
	// columns step by what its two gaps leave once their sagitta and the
	// shift of the rows' ends on the end planes are spent, so every strip
	// triangle stays within tau (rowSteps, §6.3).
	steps := w.arcRowSteps(rows, ss, rhoA)
	colsA, err := w.arcRow(a, rows[0], 0, rhoA, steps[0])
	if err != nil {
		return nil, err
	}
	bound := w.tau + slack
	rowCols := make([][]colPt, m+1)
	rowCols[0] = colsA
	for j := 1; j < m; j++ {
		if rowCols[j], err = w.interiorRow(a, rows[j], bound, steps[j]); err != nil {
			return nil, err
		}
	}
	rowCols[m] = make([]colPt, len(cutIDs))
	span := rows[m].theta
	for k, id := range cutIDs {
		th := a.theta(w.verts[id], (span[0]+span[1])/2)
		t := (th - span[0]) / (span[1] - span[0])
		rowCols[m][k] = colPt{id, t}
	}
	rowCols[m][0].t, rowCols[m][len(cutIDs)-1].t = 0, 1
	_ = pB
	if widened > 0 {
		// The last gap's chord on each end plane no longer lies within tau of its
		// end curve. Its two ends state how far it does, and how far the wall's cut
		// vertex at the other end of the chord may lie from the exact point
		// (cuspCharge's argument: a chord end that moves by eps moves the chord's
		// points by at most eps).
		for i, e := range ends {
			chord, err := endChord(e, widened, 1, w.tau)
			if err != nil {
				return nil, err
			}
			row, cut := rowCols[m-1][0].idx, rowCols[m][0].idx
			if i == 1 {
				row, cut = rowCols[m-1][len(rowCols[m-1])-1].idx, rowCols[m][len(rowCols[m])-1].idx
			}
			w.beta[row] = max(w.beta[row], proofbound.AbsSumUpper(chord, w.beta[cut], slack))
		}
	}

	// The plane's loops: the arc gives way to the setback arc.
	seqA := make([]int, len(colsA))
	for k, c := range colsA {
		seqA[k] = c.idx
	}
	aLoops, err := replaceRun(fA, runA, [2]int{a.ends[0].v, a.ends[1].v}, remA, [2]sideKind{trims, trims}, seqA)
	if err != nil {
		return nil, err
	}
	// The plane keeps clear of the strip. On a cylinder end the plane also loses the
	// region between its rim arc's ends and the setback arc's, along G, so the zone
	// is the union of the two angular spans.
	zone := rows[0].theta
	if cyl {
		zone = [2]float64{
			min(zone[0], a.theta(a.ends[0].exact, zone[0])), max(zone[1], a.theta(a.ends[1].exact, zone[1])),
		}
	}
	{
		exempt := map[int]bool{a.ends[0].v: true, a.ends[1].v: true}
		for _, v := range runA.inner {
			exempt[v] = true
		}
		for i := range 2 {
			for _, v := range runA.chain[i] {
				exempt[v] = true
			}
		}
		for _, loop := range fA.loops {
			for k, v := range loop {
				q := loop[(k+1)%len(loop)]
				if exempt[v] && exempt[q] {
					continue
				}
				lo, hi := w.segmentRadialRange(w.verts[v], w.verts[q])
				if hi > math.Min(a.radius, rhoA)-margin-w.beta[v] && lo < math.Max(a.radius, rhoA)+margin+w.beta[v] &&
					w.nearArcZone(a, w.verts[v], w.verts[q], rhoA, zone, margin+w.beta[v]) {
					return nil, fmt.Errorf("%w: a face boundary comes within the blend's reach of the arc", decaderr.ErrUnsupported)
				}
			}
		}
	}

	// The end faces.
	var gLoops [2][][]int
	var gainsG [2]bool
	var capEnds, wallRuns [2][]int
	for i := range 2 {
		endIDs := make([]int, m+1)
		reach := make([]r3.Vec, m+1)
		for j := range endIDs {
			c := rowCols[j][0]
			if i == 1 {
				c = rowCols[j][len(rowCols[j])-1]
			}
			endIDs[j] = c.idx
			reach[j] = w.verts[c.idx]
		}
		// Where the wall extends past the end plane, the end face keeps the corner
		// and takes the blend's end cap: the rows, the wall's cut along the end
		// plane back to the corner, and the plane's line to the first row. The cap
		// must be a simple polygon.
		var wallRun []int
		if kindB[i] == extends {
			if wallRun, err = cuts.endRun(i, a.ends[i].v); err != nil {
				return nil, err
			}
			closeAt := a.ends[i].v
			if cyl {
				if wallRun, closeAt, err = a.closeCap(i, wallRun); err != nil {
					return nil, err
				}
			}
			capLoop := append(append(append([]int(nil), endIDs...), wallRun...), closeAt)
			if cyl {
				pts := make([]r3.Vec, len(capLoop))
				for k, id := range capLoop {
					pts[k] = w.verts[id]
				}
				if err := capSimple2(a.capPoly(i, pts), m); err != nil {
					return nil, err
				}
			} else if err := w.capSimple(a.ends[i].g.n, capLoop, m); err != nil {
				return nil, err
			}
		}
		if cyl {
			capEnds[i], wallRuns[i] = endIDs, wallRun
			reachIDs := append(slices.Clone(endIDs), wallRun...)
			if err := a.cylEndChecks(w, i, fG[i], runA.chain[i], runB.chain[i], reachIDs, margin); err != nil {
				return nil, err
			}
			continue
		}
		loops, gains, err := w.endFaceRun(a.ends[i], fG[i], runA.chain[i], runB.chain[i], [2]sideKind{trims, kindB[i]},
			remA[i], remB[i], endIDs, wallRun, reach, margin, a.convex, nil)
		if err != nil {
			return nil, err
		}
		gLoops[i] = loops
		gainsG[i] = gains
	}
	// An end face that takes the cap is new surface where the cap lies: its
	// rebuilt facets are cleared against the rest of the body as the strip is.
	var taken [][2]int
	faceList := []struct {
		loops [][]int
		n     r3.Vec
		fl    faceLoops
		gains bool
	}{{aLoops, n, fA, false}}
	if !cyl {
		faceList = append(faceList,
			struct {
				loops [][]int
				n     r3.Vec
				fl    faceLoops
				gains bool
			}{gLoops[0], a.ends[0].g.n, fG[0], gainsG[0] || kindB[0] == extends},
			struct {
				loops [][]int
				n     r3.Vec
				fl    faceLoops
				gains bool
			}{gLoops[1], a.ends[1].g.n, fG[1], gainsG[1] || kindB[1] == extends})
	}
	for _, f := range faceList {
		from := len(w.out)
		if err := w.retriangulate(f.n, f.loops, f.fl.group); err != nil {
			return nil, err
		}
		if f.gains {
			taken = append(taken, [2]int{from, len(w.out)})
		}
		w.removed = append(w.removed, f.fl.facets...)
	}
	// A cylinder end face is not retriangulated: its held facets stay but for the ones a
	// cap column splits, and the cap is new surface, cleared as a taken cap is.
	if cyl {
		for i := range 2 {
			r, err := a.capFacets(w, i, fG[i], runA.chain[i], remA[i], capEnds[i], wallRuns[i])
			if err != nil {
				return nil, err
			}
			taken = append(taken, r)
		}
	}

	// The strip, and its tag.
	tag := a.arcTag(spec)
	blendGroup := len(fp.Groups)
	stripFrom := len(w.out)
	for j := range m {
		for _, t := range zipOpen(rowCols[j], rowCols[j+1]) {
			va, vb, vc := w.verts[t[0]], w.verts[t[1]], w.verts[t[2]]
			centroid := va.Add(vb).Add(vc).Scale(1.0 / 3)
			if vb.Sub(va).Cross(vc.Sub(va)).Dot(tag.outward(centroid)) < 0 {
				t[1], t[2] = t[2], t[1]
			}
			w.emit(t, blendGroup)
		}
	}
	// The band beside the wall's cut takes its cut vertices' offsets (§6.3),
	// its azimuth term read at the larger radius of its two rows: the last
	// row's and the wall's setback circle's, a.radius.
	w.chargeBand(stripFrom, bandAxis{o: a.center, n: n, ok: true}, max(a.radius, rows[m-1].rho),
		filletSag(spec, (1-ss[m-1])*math.Pi/2), slack)
	w.removedTrue, tag.trueArea = a.arcAreas(profile, rows, tb)
	if cyl {
		for i := range 2 {
			w.capTrue = proofbound.AbsSumUpper(w.capTrue, a.capArea(profile, i))
		}
	}

	groups := append([]facetedrecord.Group(nil), fp.Groups...)
	groups = append(groups, w.tagGroup(facetedrecord.Group{
		Origins: []provenance.Ref{{Producer: env.Ref, Role: role}},
		Source:  facetedrecord.BlendSource(env.Ref, ord),
		Planar:  !spec.fillet,
	}, tag))
	for _, f := range faces {
		groups[f.group].HasCentroid = false
		groups[f.group].Centroid = measurement.VecMeasurement{}
	}
	if err := w.checkStripClear(ctx, stripFrom, taken); err != nil {
		return nil, err
	}
	return w.build(ctx, env, groups, stripFrom, tag, faceIndex)
}

// arcRowSteps is rowSteps for the rows of an arc route, the plane's row
// first, at radius rhoA, at which arcRow places it, and the wall's cut row
// last. A row's azimuth runs linearly from its end on G₀ to its end on G₁, so
// two rows' azimuths at one fraction of their rows differ by at most the
// larger move of their ends.
func (w *work) arcRowSteps(rows []arcRow, ss []float64, rhoA float64) []float64 {
	m := len(rows) - 1
	rho := make([]float64, m+1)
	shift := make([]float64, m-1)
	for j, row := range rows {
		rho[j] = row.rho
		if j > 0 && j < m {
			prev := rows[j-1].theta
			shift[j-1] = max(math.Abs(row.theta[0]-prev[0]), math.Abs(row.theta[1]-prev[1]))
		}
	}
	rho[0] = rhoA
	return rowSteps(w.spec, w.tau, ss, rho, shift)
}

// arcRow builds the plane's row: the setback points and the points between
// them along the circle of radius rho, step apart at most.
func (w *work) arcRow(a arcRim, row arcRow, z, rho, step float64) ([]colPt, error) {
	bound := w.tau + a.slack()
	t0, t1 := row.theta[0], row.theta[1]
	k, err := columnCount(t1-t0, step, rho, w.spec.size(), 1)
	if err != nil {
		return nil, err
	}
	out := make([]colPt, 0, k+1)
	for i := 0; i <= k; i++ {
		f := float64(i) / float64(k)
		out = append(out, colPt{w.vertex(a.atTheta(rho, t0+(t1-t0)*f, z), bound), f})
	}
	return out, nil
}

// interiorRow builds a row between the plane's and the wall's: its end points
// on the end faces and the points between them, step apart at most.
func (w *work) interiorRow(a arcRim, row arcRow, bound, step float64) ([]colPt, error) {
	t0, t1 := row.theta[0], row.theta[1]
	k, err := columnCount(t1-t0, step, row.rho, w.spec.size(), 1)
	if err != nil {
		return nil, err
	}
	out := make([]colPt, 0, k+1)
	for i := 0; i <= k; i++ {
		f := float64(i) / float64(k)
		out = append(out, colPt{w.vertex(a.atTheta(row.rho, t0+(t1-t0)*f, row.z), bound), f})
	}
	return out, nil
}

// zipOpen joins two open rows into a band of triangles by advancing whichever
// row's next vertex lies earlier along its row.
func zipOpen(a, b []colPt) [][3]int {
	var out [][3]int
	i, j := 0, 0
	for i+1 < len(a) || j+1 < len(b) {
		if i+1 < len(a) && (j+1 >= len(b) || a[i+1].t <= b[j+1].t) {
			out = append(out, [3]int{a[i].idx, a[i+1].idx, b[j].idx})
			i++
			continue
		}
		out = append(out, [3]int{a[i].idx, b[j+1].idx, b[j].idx})
		j++
	}
	return out
}

// nearArcZone reports whether segment pq comes within reach of the annular
// sector the plane loses: between the rim and the setback circle, over the
// angles the setback arc spans.
func (w *work) nearArcZone(a arcRim, p, q r3.Vec, rhoA float64, span [2]float64, reach float64) bool {
	lo, hi := math.Min(a.radius, rhoA)-reach, math.Max(a.radius, rhoA)+reach
	const samples = 64
	for i := 0; i <= samples; i++ {
		x := p.Add(q.Sub(p).Scale(float64(i) / samples))
		rho := a.radial(x)
		if rho < lo || rho > hi {
			continue
		}
		th := a.theta(x, (span[0]+span[1])/2)
		if th >= span[0]-reach/math.Max(rho, reach) && th <= span[1]+reach/math.Max(rho, reach) {
			return true
		}
	}
	return false
}

// clipArcWall cuts the wall at the setback height tb below the rim, keeping the
// part beyond it. generator holds the vertices of the wall's lines along the
// end faces, which sit inside the removed zone legitimately. It returns the
// cut vertices in order of angle, the removed part of each end's generator
// chain and the cut vertex on the chain edge that crosses the setback height.
func (w *work) clipArcWall(f faceLoops, a arcRim, tb float64, generator map[int]bool, run edgeRun,
	margin float64) ([]int, [2][]int, [2]int, error) {
	slack := a.slack()
	g := map[int]float64{}
	for _, facet := range f.facets {
		for _, v := range w.fp.Tris[facet] {
			if _, seen := g[v]; seen {
				continue
			}
			p := w.verts[v]
			if a.onRim(p, w.beta[v]) {
				g[v] = -tb
				continue
			}
			gv := a.side*a.height(p) - tb
			if generator[v] && gv < -margin-w.beta[v] {
				g[v] = gv
				continue
			}
			if err := clearance(gv, w.beta[v]+slack+w.tau, "the wall"); err != nil {
				return nil, [2][]int{}, [2]int{}, err
			}
			g[v] = gv
		}
	}
	facetBound := 0.0
	cut := func(x, y int) int { // x removed, y kept
		key := [2]int{min(x, y), max(x, y)}
		if id, ok := w.cuts[key]; ok {
			return id
		}
		t := g[x] / (g[x] - g[y])
		p := w.verts[x].Add(w.verts[y].Sub(w.verts[x]).Scale(t))
		id := w.vertex(p, facetBound+slack+w.tau)
		w.noteCut(id, facetBound)
		w.cuts[key] = id
		return id
	}
	for _, facet := range f.facets {
		t := w.fp.Tris[facet]
		facetBound = w.facetBound(t)
		var above []int
		for k, v := range t {
			if g[v] > 0 {
				above = append(above, k)
			}
		}
		switch len(above) {
		case 0:
		case 3:
			w.emit(t, f.group)
		case 1:
			x := above[0]
			p, q, r := t[x], t[(x+1)%3], t[(x+2)%3]
			w.emit([3]int{p, cut(q, p), cut(r, p)}, f.group)
		case 2:
			z := 0
			for k := range 3 {
				if g[t[k]] <= 0 {
					z = k
				}
			}
			x, y, below := t[(z+1)%3], t[(z+2)%3], t[z]
			cyz, czx := cut(below, y), cut(below, x)
			w.emit([3]int{x, y, cyz}, f.group)
			w.emit([3]int{x, cyz, czx}, f.group)
		}
	}
	w.removed = append(w.removed, f.facets...)

	var rem [2][]int
	var pb [2]int
	for i := range 2 {
		seq := append([]int{a.ends[i].v}, run.chain[i]...)
		crossed := false
		for k := 1; k < len(seq); k++ {
			if g[seq[k-1]] <= 0 && g[seq[k]] > 0 {
				id, ok := w.cuts[[2]int{min(seq[k-1], seq[k]), max(seq[k-1], seq[k])}]
				if !ok {
					return nil, rem, pb, fmt.Errorf("%w: the wall's generator edge at the setback height was not cut", decaderr.ErrUnsupported)
				}
				pb[i], crossed = id, true
				break
			}
			if g[seq[k]] > 0 {
				break
			}
			rem[i] = append(rem[i], seq[k])
		}
		if !crossed {
			return nil, rem, pb, fmt.Errorf("%w: the wall's generator ends before the blend's setback height", decaderr.ErrDegenerate)
		}
	}
	ids := make([]int, 0, len(w.cuts))
	for _, id := range w.cuts {
		ids = append(ids, id)
	}
	mid := a.theta0 + a.span/2
	sort.Slice(ids, func(x, y int) bool {
		return a.theta(w.verts[ids[x]], mid) < a.theta(w.verts[ids[y]], mid)
	})
	if len(ids) < 2 || ids[0] != pb[0] || ids[len(ids)-1] != pb[1] {
		return nil, rem, pb, fmt.Errorf("%w: the wall's cut does not run from one end face to the other", decaderr.ErrUnsupported)
	}
	return ids, rem, pb, nil
}

// arcTag is the exact surface of the blend face.
func (a arcRim) arcTag(spec blendSpec) blendTag {
	size := spec.r
	if !spec.fillet {
		size = max(spec.dA, spec.dB)
	}
	w := &work{r: a.rim, spec: spec, tau: math.Ldexp(size, -10)}
	if spec.fillet {
		return w.torusTag()
	}
	return w.coneTag()
}

// arcAreas returns the exact area the blend removes from the four faces
// (signed: a concave blend adds to the end faces) and the exact area of the
// blend face.
func (a arcRim) arcAreas(p arcProfile, rows []arcRow, tb float64) (removed, blend float64) {
	const samples = 4000
	ref0 := [2]float64{a.theta0, a.theta0 + a.span}
	for i := range 2 {
		pts := []r3.Vec{a.ends[i].exact}
		ref := ref0[i]
		for s := 0; s <= samples; s++ {
			rho, z := p(float64(s) / samples)
			t, ok := a.endAzimuths(i, rho, ref)
			if !ok {
				continue
			}
			ref = t
			pts = append(pts, a.atTheta(rho, t, z))
		}
		corner := polyArea(pts...)
		if a.cyl[i] != nil {
			corner = a.capArea(p, i)
		}
		if !a.convex {
			corner = -corner
		}
		removed += corner
	}
	// The plane loses the region between the rim and the setback arc, bounded
	// by the two lines along the end faces: Green's theorem over the two arcs
	// and the two lines.
	rhoA := rows[0].rho
	t0, t1 := rows[0].theta[0], rows[0].theta[1]
	xy := func(q r3.Vec) (float64, float64) {
		d := q.Sub(a.center)
		return d.Dot(a.u), d.Dot(a.w)
	}
	line := func(q0, q1 r3.Vec) float64 {
		x0, y0 := xy(q0)
		x1, y1 := xy(q1)
		return (x0*y1 - x1*y0) / 2
	}
	// A cylinder end's side is the arc of G's circle from the corner to the setback
	// point, not the straight line.
	side := func(i int, p0, p1 r3.Vec) float64 {
		if a.cyl[i] != nil {
			return a.gArc(i, p0, p1)
		}
		return line(p0, p1)
	}
	region := a.radius*a.radius*a.sign*a.span/2 +
		side(1, a.ends[1].exact, a.atTheta(rhoA, t1, 0)) -
		rhoA*rhoA*a.sign*(t1-t0)/2 +
		side(0, a.atTheta(rhoA, t0, 0), a.ends[0].exact)
	removed += math.Abs(region)
	// The wall loses a band whose sides are generators.
	removed += a.radius * a.span * tb
	prev := 0.0
	for k := 1; k <= samples; k++ {
		s := float64(k) / samples
		sm := (prev + s) / 2
		rho0, z0 := p(prev)
		rho1, z1 := p(s)
		rhoM, _ := p(sm)
		ang, err := a.rowAngles(p, sm, ref0)
		prev = s
		if err != nil {
			continue
		}
		blend += rhoM * (ang[1] - ang[0]) * math.Hypot(rho1-rho0, z1-z0)
	}
	return removed, blend
}
