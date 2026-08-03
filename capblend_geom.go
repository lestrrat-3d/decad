package decad

import (
	"context"
	"fmt"
	"math"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// This file builds the capBlendPayload topology (docs/modify-reach-design.md
// §8.3, Table BX row BX3): the trimmed prism side walls, the cap faces (a
// mix of unchanged loops and offset "cap contour" loops), and the chamfer
// band patches between them — a Plane for a line wall, a Cone for a circular
// wall, and a Cone whose apex is the original corner for a reflex corner's
// extra offset arc.

// capOffsetJoins computes one loop's per-corner offset joins exactly as
// shell_offset.go's offsetLoopBudget does — reusing offsetCarrier,
// intersectOffsets and offsetRadius unchanged — but returns the joins
// themselves rather than the flattened segment list, so the caller can pair
// each patch with the ORIGINAL wall or corner it descends from.
func capOffsetJoins(budget *workBudget, cl cornerLoop, d float64) ([]cornerJoin, error) {
	walks := cl.walks
	n := len(walks)
	if n == 0 {
		return nil, fmt.Errorf(`%w: a cap loop holds no walks`, ErrDegenerate)
	}
	for _, w := range walks {
		if err := wallBudgetStep(budget); err != nil {
			return nil, err
		}
		if w.isCircular() {
			if _, ok := offsetRadius(w, 1, d); !ok {
				return nil, errOffsetDrop
			}
		}
	}
	joins := make([]cornerJoin, n)
	for i := range n {
		if err := wallBudgetStep(budget); err != nil {
			return nil, err
		}
		prev := walks[(i+n-1)%n]
		cur := walks[i]
		vU, vV := cur.startU, cur.startV
		aox, aoy, la := normalize2(prev.tanOutU, prev.tanOutV)
		bix, biy, lb := normalize2(cur.tanInU, cur.tanInV)
		if la == 0 || lb == 0 {
			return nil, fmt.Errorf(`%w: a corner walk has no direction`, ErrDegenerate)
		}
		cross := aox*biy - aoy*bix
		pA := Point2{U: vU + d*(-aoy), V: vV + d*aox}
		pB := Point2{U: vU + d*(-biy), V: vV + d*bix}
		if math.Abs(cross) > shellTol && cross < 0 {
			// Reflex corner (sign(cross) == -1 == -s, s = +1 inward): an arc
			// of radius d about the corner — the extra offset connector this
			// evaluator patches with a Cone apex (§8.3).
			joins[i] = cornerJoin{arc: true, vU: vU, vV: vV, pA: pA, pB: pB}
			continue
		}
		offA := offsetCarrier(prev, 1, d)
		offB := offsetCarrier(cur, 1, d)
		mx, my, err := intersectOffsets(offA, offB, vU, vV)
		if err != nil {
			return nil, errOffsetTopology
		}
		joins[i] = cornerJoin{vU: vU, vV: vV, m: Point2{U: mx, V: my}}
	}
	return joins, nil
}

// capWallFoot returns the offset segment's own (start, end) feet for wall i,
// exactly as offsetLoopBudget's per-wall trim does.
func capWallFoot(joins []cornerJoin, i, n int) (Point2, Point2) {
	j0 := joins[i]
	start := j0.m
	if j0.arc {
		start = j0.pB
	}
	j1 := joins[(i+1)%n]
	end := j1.m
	if j1.arc {
		end = j1.pA
	}
	return start, end
}

// oneLoopCornerLoop decomposes a single recorded loop into its coalesced
// corner walk, the same decomposition prismCornerLoopsBudget applies to
// every loop of a section.
func oneLoopCornerLoop(budget *workBudget, loop LoopRecord, work *freeformWork) (cornerLoop, error) {
	raw := make([]sideWalk, len(loop.Segments))
	for i, seg := range loop.Segments {
		if err := wallBudgetStep(budget); err != nil {
			return cornerLoop{}, err
		}
		w, err := walkOf(seg, work)
		if err != nil {
			return cornerLoop{}, err
		}
		if err := requireAnalyticWalk(w, "a cap-loop chamfer"); err != nil {
			return cornerLoop{}, err
		}
		raw[i] = sideWalk{segmentWalk: w, segs: []int{i}}
	}
	walks, err := coalesceWalksBudget(raw, budget)
	if err != nil {
		return cornerLoop{}, err
	}
	return cornerLoop{walks: walks}, nil
}

// capBandResult is what buildCapBand contributes to the enclosing build: the
// new patch faces, the cap-level coedges bounding the loop's rewritten cap
// boundary (in walk order), and the exact-rational patch geometry the
// moments pass integrates.
type capBandResult struct {
	patches []*Face
	capCo   []coedge
	geom    []capPatchGeom
}

// capPatchGeom is one patch's exact analytic description, plane-local (u, v)
// plus the two axial levels, kept alongside the topology for the moments and
// tessellation passes (docs/modify-reach-design.md §8.4).
type capPatchGeom struct {
	circular bool
	// Plane patch (circular == false): the two side-level (original) points
	// and the two cap-level (offset) points, in walk order.
	sideA, sideB, capA, capB Point2
	// Cone patch (circular == true): concentric center, the two radii
	// (side = original wall radius, cap = offset radius) and the angular
	// range in the walk's own sense (th1 < th0 is a clockwise walk).
	cU, cV                float64
	sideRadius, capRadius float64
	th0, th1              float64
	// apex marks a reflex-corner patch: side is the single original corner
	// point (sideA), not a wall — capA/capB are the offset arc's feet.
	apex bool

	sideZ, capZ float64
}

// buildCapBand builds the chamfer band for one loop selected on one cap: the
// patch faces between the cap contour (offset d into material, at capZ) and
// the side contour (the original loop, at capZ + matSign*d), and the
// cap-level coedges that replace the loop's boundary in the cap face. The
// side-level boundary reuses the trimmed side wall's own near-cap coedges
// (sideCo, from buildLoopSidesAs) — shared, never re-derived.
func buildCapBand(ctx context.Context, body *Body, ref StepRef, cbp capBlendPayload, li int, loop LoopRecord, capZ float64, matSign float64, sideCo []coedge, work *freeformWork) (capBandResult, error) {
	if err := ctx.Err(); err != nil {
		return capBandResult{}, err
	}
	budget := newWorkBudget(ctx)
	cl, err := oneLoopCornerLoop(budget, loop, work)
	if err != nil {
		return capBandResult{}, err
	}
	walks := cl.walks
	n := len(walks)
	d := cbp.d
	sideZ := capZ + matSign*d
	pl := cbp.prismLike(0, 0)

	capName := capNameOf(matSign)

	liftCap := func(p Point2) r3.Vec { return pl.point(p.U, p.V, capZ) }
	liftSide := func(p Point2) r3.Vec { return pl.point(p.U, p.V, sideZ) }

	// A single closed circle has no corner: one Cone patch, full turn.
	if n == 1 && walks[0].closed {
		w := walks[0]
		capRadius, ok := offsetRadius(w, 1, d)
		if !ok {
			return capBandResult{}, errOffsetDrop
		}
		seam0 := sideCo[0].edge // the side wall's own whole-circle bottom/top edge
		capEdge := wholeCircleEdge(pl, w.cU, w.cV, capRadius, capZ, w.th1 > w.th0)
		patch := buildConePatch(pl, body, ref, li, 0, w.cU, w.cV, w.radius, capRadius, sideZ, capZ, matSign, false, seam0, capEdge)
		sign := 1.0
		if w.th1 < w.th0 {
			sign = -1
		}
		samplePoint := pl.point(w.cU+w.radius, w.cV, sideZ)
		fixPatchOrientation(patch, pl, samplePoint, sign, 0, -matSign)
		// th0, th1 record the patch's ANGULAR EXTENT, not the wall's own
		// walked sense — the material-side semantics are already carried by
		// the -matSign correction in capBandVolume/patchRawFlux, so a
		// clockwise (hole) wall's reversed (th1 < th0) recording is
		// normalized to an increasing pair here, or patchRawFlux's trig terms
		// would silently take the wrong sign for the removed/added volume
		// (found via TestCapBlendHoleLoopChamferVolume).
		gth0, gth1 := w.th0, w.th1
		if gth1 < gth0 {
			gth0, gth1 = gth1, gth0
		}
		geom := capPatchGeom{circular: true, cU: w.cU, cV: w.cV, sideRadius: w.radius, capRadius: capRadius, th0: gth0, th1: gth1, sideZ: sideZ, capZ: capZ}
		capLoop := []coedge{{edge: capEdge, forward: true}}
		return capBandResult{patches: []*Face{patch}, capCo: capLoop, geom: []capPatchGeom{geom}}, nil
	}

	joins, err := capOffsetJoins(budget, cl, d)
	if err != nil {
		return capBandResult{}, err
	}

	// sideVertexAt(i) is the ORIGINAL corner point before wall i, at sideZ —
	// buildLoopSidesAs's own shared vertex (sideCo[i].edge.Start() ==
	// sideCo[i-1].edge.End()), reused rather than re-created, so a reflex
	// corner's apex patch and its two neighboring wall patches close on the
	// SAME vertex.
	sideVertexAt := func(i int) *Vertex { return sideCo[i].edge.Start() }

	// Pass 1: every corner's connector edge(s), independent of wall build
	// order (so a reflex corner at index 0 wraps around correctly). A miter
	// corner (joins[i].m) has ONE edge shared by the wall before and the
	// wall after it: slantOut[i] == slantIn[i]. A reflex corner has TWO —
	// slantIn[i] (from the offset foot pA, used by the PRECEDING wall's
	// trailing edge) and slantOut[i] (from pB, used by the FOLLOWING wall's
	// leading edge) — different cap-level points, but both ending at the
	// SAME side-level apex vertex, sideVertexAt(i).
	slantIn := make([]*Edge, n)
	slantOut := make([]*Edge, n)
	arcByCorner := make([]*Edge, n) // non-nil for a reflex corner's cap-level arc
	arcTh0 := make([]float64, n)
	arcTh1 := make([]float64, n)
	for i := range n {
		j := joins[i]
		apex := sideVertexAt(i)
		if !j.arc {
			capV := &Vertex{position: liftCap(j.m), bound: units.Millimeters(0)}
			e := &Edge{curve: Line3{}, start: capV, end: apex, convex: true, length: math.Inf(1), lengthBound: math.Inf(1)}
			slantIn[i], slantOut[i] = e, e
			continue
		}
		pAV := &Vertex{position: liftCap(j.pA), bound: units.Millimeters(0)}
		pBV := &Vertex{position: liftCap(j.pB), bound: units.Millimeters(0)}
		slantIn[i] = &Edge{curve: Line3{}, start: pAV, end: apex, convex: true, length: math.Inf(1), lengthBound: math.Inf(1)}
		slantOut[i] = &Edge{curve: Line3{}, start: pBV, end: apex, convex: true, length: math.Inf(1), lengthBound: math.Inf(1)}
		th0 := math.Atan2(j.pA.V-j.vV, j.pA.U-j.vU)
		th1 := math.Atan2(j.pB.V-j.vV, j.pB.U-j.vU)
		for th1 < th0 {
			th1 += 2 * math.Pi
		}
		arcTh0[i], arcTh1[i] = th0, th1
		arcByCorner[i] = &Edge{
			curve: Arc3{Center: liftCap(Point2{U: j.vU, V: j.vV}), Axis: pl.dir(0, 0, 1), Radius: units.Millimeters(d)},
			start: pAV, end: pBV,
			convex: false,
			length: d * (th1 - th0), lengthBound: math.Inf(1),
		}
	}

	var patches []*Face
	var geoms []capPatchGeom
	capCo := make([]coedge, 0, 2*n)

	// Pass 2: the apex patches (Cone-with-corner-apex, one per reflex
	// corner), independent of wall order.
	for i := range n {
		if !joins[i].arc {
			continue
		}
		j := joins[i]
		arc := arcByCorner[i]
		role := fmt.Sprintf("chamferCap(%s,%d,%d)", capName, li, i)
		surf := coneSurface(pl, j.vU, j.vV, 0, d, sideZ, capZ)
		// Walk order: arc (pAV -> pBV, cap level), slantOut forward
		// (pBV -> apex), slantIn reversed (apex -> pAV) — a closed
		// triangle-like boundary each coedge's end matching the next's start.
		face := &Face{
			surface: surf,
			origins: []FeatureRef{{Step: ref, Role: role}},
			body:    body,
			loops: []*Loop{{coedges: []coedge{
				{edge: arc, forward: true},
				{edge: slantOut[i], forward: true},
				{edge: slantIn[i], forward: false},
			}, outer: true}},
		}
		// Orientation: a corner's own offset carrier is an EROSION (the
		// per-feature offset construction, shell_offset.go, is a Minkowski
		// erosion: the offset boundary is always a SUBSET of the original
		// material, at a reflex corner as much as a convex one). So a fixed
		// radius r < d, near the corner, sits within the ORIGINAL material at
		// sideZ (the unchanged corner) and within the ERODED-AWAY void at
		// capZ — material retreats toward the cap exactly as a regular wall
		// patch's does, and outward tilts the SAME way: toward the cap, plus
		// away from the corner point radially (the direction the offset arc
		// bulges). Verified empirically (never hand-trusted):
		// fixPatchOrientation checks the actual built surface's own NormalAt
		// against this reference and reverses only if they disagree.
		fixPatchOrientation(face, pl, pl.point(j.vU+d*math.Cos(arcTh0[i]), j.vV+d*math.Sin(arcTh0[i]), capZ), math.Cos(arcTh0[i]), math.Sin(arcTh0[i]), -matSign)
		slantIn[i].faces = append(slantIn[i].faces, face)
		arc.faces = append(arc.faces, face)
		slantOut[i].faces = append(slantOut[i].faces, face)
		patches = append(patches, face)
		geoms = append(geoms, capPatchGeom{
			circular: true, apex: true,
			cU: j.vU, cV: j.vV, sideRadius: 0, capRadius: d,
			th0: arcTh0[i], th1: arcTh1[i], sideZ: sideZ, capZ: capZ,
		})
	}

	// Pass 3: the wall patches (Plane or Cone) AND the cap-level boundary
	// coedges, in walk order — a wall's own capEdge, then the reflex arc
	// (if any) at the corner it leads into, exactly the order offsetLoopBudget
	// emits the same offset loop's segments in.
	for i := range n {
		w := walks[i]
		start, end := capWallFoot(joins, i, n)
		nextI := (i + 1) % n
		capA, capB := slantOut[i].start, slantIn[nextI].start
		side := sideCo[i].edge
		leadSlant, trailSlant := slantOut[i], slantIn[nextI]

		var capEdge *Edge
		if !w.isCircular() {
			capEdge = &Edge{curve: Line3{}, start: capA, end: capB, convex: true, length: math.Hypot(end.U-start.U, end.V-start.V), lengthBound: math.Inf(1)}
		} else {
			capRadius, ok := offsetRadius(w, 1, d)
			if !ok {
				return capBandResult{}, errOffsetDrop
			}
			capEdge = arcEdge(pl, w.cU, w.cV, capRadius, capZ, capA, capB, w.th0, w.th1)
		}

		var surf Surface
		if !w.isCircular() {
			f, err := planeFromThree(liftSide(Point2{U: w.startU, V: w.startV}), liftSide(Point2{U: w.endU, V: w.endV}), capB.position)
			if err != nil {
				return capBandResult{}, err
			}
			surf = f
		} else {
			capRadius, _ := offsetRadius(w, 1, d)
			surf = coneSurface(pl, w.cU, w.cV, w.radius, capRadius, sideZ, capZ)
		}

		// Walk order mirrors the ordinary prism side face's own convention
		// (extrude.go: bottom forward, right vertical forward, top reversed,
		// left vertical reversed): side forward (sideVertexAt(i) ->
		// sideVertexAt(nextI)), trailSlant reversed (its natural direction is
		// capB -> sideVertexAt(nextI), so reversed walks UP to capB),
		// capEdge reversed (capB -> capA), leadSlant forward (its natural
		// direction is capA -> sideVertexAt(i), walking back DOWN to close).
		role := fmt.Sprintf("chamferCap(%s,%d,%d)", capName, li, i)
		face := &Face{
			surface: surf,
			origins: []FeatureRef{{Step: ref, Role: role}},
			body:    body,
			loops: []*Loop{{coedges: []coedge{
				{edge: side, forward: true},
				{edge: trailSlant, forward: false},
				{edge: capEdge, forward: false},
				{edge: leadSlant, forward: true},
			}, outer: true}},
		}
		// Orientation: the reference is the ORIGINAL wall's own outward
		// convention (the same "tangent rotated a quarter turn" rule
		// extrude.go's prism side walls use — it already covers hole walls
		// through the wall's OWN walked sense, no separate case needed) plus
		// the toward-the-cap Z sense every patch in this band shares. Checked
		// empirically against the patch's own built NormalAt, never hand
		// trusted: fixPatchOrientation reverses only if they disagree.
		var refU, refV float64
		var samplePoint r3.Vec
		if !w.isCircular() {
			refU, refV = w.tanInV, -w.tanInU
			samplePoint = liftSide(Point2{U: w.startU, V: w.startV})
		} else {
			sign := 1.0
			if w.th1 < w.th0 {
				sign = -1
			}
			refU, refV = sign*math.Cos(w.th0), sign*math.Sin(w.th0)
			samplePoint = pl.point(w.cU+w.radius*math.Cos(w.th0), w.cV+w.radius*math.Sin(w.th0), sideZ)
		}
		fixPatchOrientation(face, pl, samplePoint, refU, refV, -matSign)
		side.faces = append(side.faces, face)
		trailSlant.faces = append(trailSlant.faces, face)
		capEdge.faces = append(capEdge.faces, face)
		leadSlant.faces = append(leadSlant.faces, face)
		patches = append(patches, face)

		g := capPatchGeom{sideZ: sideZ, capZ: capZ}
		if w.isCircular() {
			capRadius, _ := offsetRadius(w, 1, d)
			g.circular = true
			g.cU, g.cV = w.cU, w.cV
			g.sideRadius, g.capRadius = w.radius, capRadius
			// th0, th1 record the ANGULAR EXTENT, not the wall's own walked
			// sense — see the single-closed-circle branch's comment above;
			// the same normalization applies to a partial arc wall.
			g.th0, g.th1 = w.th0, w.th1
			if g.th1 < g.th0 {
				g.th0, g.th1 = g.th1, g.th0
			}
		} else {
			g.sideA = Point2{U: w.startU, V: w.startV}
			g.sideB = Point2{U: w.endU, V: w.endV}
			g.capA, g.capB = start, end
		}
		geoms = append(geoms, g)
		capCo = append(capCo, coedge{edge: capEdge, forward: true})
		if arc := arcByCorner[nextI]; arc != nil {
			// shell_offset.go's reflex-corner arc walks pA -> pB in the
			// SAME sense as the boundary's own travel direction (inward,
			// s=+1: a clockwise arc whose tangent continues the walk).
			capCo = append(capCo, coedge{edge: arc, forward: true})
		}
	}
	return capBandResult{patches: patches, capCo: capCo, geom: geoms}, nil
}

// wholeCircleEdge builds a full-circle Edge (Circle3) in the cap plane at z,
// the same seam-vertex convention extrude.go's singleClosed branch uses.
func wholeCircleEdge(pl prismPayload, cu, cv, r, z float64, ccw bool) *Edge {
	seam := &Vertex{position: pl.point(cu+r, cv, z), bound: units.Millimeters(0)}
	axis := pl.dir(0, 0, 1)
	if !ccw {
		axis = axis.Scale(-1)
	}
	return &Edge{
		curve: Circle3{Center: pl.point(cu, cv, z), Axis: axis, Radius: units.Millimeters(r)},
		start: seam, end: seam,
		convex: ccw,
		length: 2 * math.Pi * r, lengthBound: math.Inf(1),
	}
}

// arcEdge builds an Arc3 Edge in the cap plane at z between the given
// vertices, walking the same sense (th0, th1) the original wall does.
func arcEdge(pl prismPayload, cu, cv, r, z float64, start, end *Vertex, th0, th1 float64) *Edge {
	axis := pl.dir(0, 0, 1)
	if th1 < th0 {
		axis = axis.Scale(-1)
	}
	return &Edge{
		curve: Arc3{Center: pl.point(cu, cv, z), Axis: axis, Radius: units.Millimeters(r)},
		start: start, end: end,
		convex: th1 > th0,
		length: r * math.Abs(th1-th0), lengthBound: math.Inf(1),
	}
}

// planeFromThree builds a Plane surface through three world points, its
// normal fixed by the winding p0->p1->p2 (right-hand rule); the caller flips
// the face's reversed bit if that does not turn out to be the outward sense.
func planeFromThree(p0, p1, p2 r3.Vec) (Plane, error) {
	u := p1.Sub(p0)
	v := p2.Sub(p0)
	f, err := r3.NewFrame(p0, u, v)
	if err != nil {
		return Plane{}, fmt.Errorf(`%w: a chamfer patch's three corners are degenerate: %s`, ErrDegenerate, err)
	}
	return Plane{Frame: f}, nil
}

// coneSurface builds the Cone surface a circular chamfer patch's wall
// occupies: Origin is the apex where the ruling reaches radius 0 (which may
// lie outside [sideZ, capZ] for a regular frustum), Axis the growth
// direction, Radius/HalfAngle read off the two known (z, r) pairs.
func coneSurface(pl prismPayload, cu, cv, r0, r1, z0, z1 float64) Surface {
	dz, dr := z1-z0, r1-r0
	if math.Abs(dr) <= 1e-12*math.Max(1, math.Max(r0, r1)) {
		return Cylinder{Origin: pl.point(cu, cv, z0), Axis: pl.dir(0, 0, 1), Radius: units.Millimeters((r0 + r1) / 2)}
	}
	apexZ := z0 - r0*dz/dr
	growth := 1.0
	if dz*dr < 0 {
		growth = -1
	}
	return Cone{
		Origin:    pl.point(cu, cv, apexZ),
		Axis:      pl.dir(0, 0, growth),
		Radius:    units.Millimeters(0),
		HalfAngle: units.Radians(math.Atan2(math.Abs(dr), math.Abs(dz))),
	}
}

// buildConePatch builds a full-turn Cone chamfer patch (a whole circular
// loop's own chamfer) between two full-circle edges. reversed follows the
// original wall's own sense (a hole/clockwise wall's material lies outside
// its cylinder, so its chamfer cone's geometric normal needs reversing too —
// extrude.go's same rule for a clockwise circular wall).
func buildConePatch(pl prismPayload, body *Body, ref StepRef, li, patchIdx int, cu, cv, sideRadius, capRadius, sideZ, capZ float64, matSign float64, reversed bool, sideEdge, capEdge *Edge) *Face {
	role := fmt.Sprintf("chamferCap(%s,%d,%d)", capNameOf(matSign), li, patchIdx)
	surf := coneSurface(pl, cu, cv, sideRadius, capRadius, sideZ, capZ)
	loops := []*Loop{
		{coedges: []coedge{{edge: sideEdge, forward: true}}, outer: true},
		{coedges: []coedge{{edge: capEdge, forward: false}}, outer: true},
	}
	face := &Face{surface: surf, origins: []FeatureRef{{Step: ref, Role: role}}, body: body, reversed: reversed, loops: loops}
	sideEdge.faces = append(sideEdge.faces, face)
	capEdge.faces = append(capEdge.faces, face)
	return face
}

func capNameOf(matSign float64) string {
	if matSign < 0 {
		return "end"
	}
	return "start"
}

// fixPatchOrientation empirically verifies a patch's outward normal sign
// against a plane-local reference direction (refU, refV, refZ), reversing
// face.reversed if the two disagree. samplePoint must lie on the patch's own
// built surface. It reads the surface's OWN Face.NormalAt — the exact
// formula every downstream reader (DX7's undercut survey included) uses — so
// no consumer can see a different answer than what was checked here, and the
// check never trusts a hand-derived sign convention: a Plane's u x v and a
// Cone's radially-outward formula both flip with which cap a band is on
// (docs/modify-reach-design.md §8.3), so this call is the ONE place that
// decides the sign, from the analytic geometry itself.
func fixPatchOrientation(f *Face, pl prismPayload, samplePoint r3.Vec, refU, refV, refZ float64) {
	n, err := f.NormalAt(samplePoint)
	if err != nil {
		return
	}
	ref := pl.dir(refU, refV, refZ)
	if n.Value.Dot(ref) < 0 {
		f.reversed = !f.reversed
	}
}
