package decad

import (
	"fmt"
	"math"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// This file is the cup payload of docs/modify-design.md §9 (Table B, B5/B6):
// the one new payload this increment introduces. A cup is two co-directional
// prisms over the same plane — the outer region O on its interval and the
// cavity region C on its own, sharing a rim at the open end and a floor at the
// closed one. It re-evaluates under Body.Placed (evaluator §8) and holds every
// measurement §10 specifies, all Exact with a zero bound.
//
// A holed section (k ≥ 1 posts in the pocket) builds too: the outer region and
// the cavity region each carry k holes, so the cup wraps a wall around each
// post. The cavity is a VOID the solid encloses, so its loops invert — the
// void's outer boundary is a hole in the solid, each of the void's own holes a
// solid post — and every band still hangs off the one floor slab, so the result
// is one lump (Table B, B5/B6). A both-caps holed shell keeps no floor and is
// 1 + k disjoint lumps (B4), which is why THAT case stays refused (S12), not
// this one.

// cupPayload is the evaluator's own record of a cup: the outer region O and the
// cavity region C (each walked in its natural sense — the outer loop
// counter-clockwise), the plane frame, the three sweep planes and the
// accumulated rigid placement. The open end (the removed cap, where the rim is)
// is zOpen; the outer prism's floor is at zOuter, the cavity's floor (shellCap)
// at zCav, which lies between the two so the floor slab is [zOuter, zCav]
// (docs/modify-design.md §9).
type cupPayload struct {
	outer  ProfileRecord
	cavity ProfileRecord
	frame  r3.Frame
	zOpen  float64
	zOuter float64
	zCav   float64
	xform  r3.Transform
}

// transform is the accumulated rigid placement.
func (cp cupPayload) transform() r3.Transform { return cp.xform }

// placed re-evaluates the same cup under the composed motion (evaluator §8).
func (cp cupPayload) placed(d *Document, ref StepRef, composed r3.Transform) (*Body, error) {
	cp.xform = composed
	return evalCup(d, ref, cp)
}

// prismLike is a prismPayload sharing the cup's frame and placement, so the
// point/dir/reflected/capFrame machinery serves the cup too.
func (cp cupPayload) prismLike(z0, z1 float64) prismPayload {
	return prismPayload{frame: cp.frame, z0: z0, z1: z1, xform: cp.xform}
}

// extentAlong is the cup's exact extent interval along an arbitrary world
// direction g — the OUTER prism's extent (docs/modify-design.md §10, Table D,
// D5). The cavity is interior and reaches no farther than the outer region, so
// the outward extent is the solid outer prism's, read by the same
// prismPayload.extentAlong the outer prism's bounds already use. This is what a
// through-all stop consults when a cup is a live body in the sweep's path.
func (cp cupPayload) extentAlong(g r3.Vec) (float64, float64, error) {
	oLo, oHi := math.Min(cp.zOuter, cp.zOpen), math.Max(cp.zOuter, cp.zOpen)
	outer := prismPayload{profile: cp.outer, frame: cp.frame, z0: oLo, z1: oHi, xform: cp.xform}
	return outer.extentAlong(g)
}

// cupPayloadFor assembles the cup record from the receiver prism, its offset
// section and the shell sense/opening (docs/modify-design.md §9). removedEnd
// opens the cup at the top (z1); a removed start opens it at the bottom (z0),
// its mirror. Inward, O is the original section P and C the erosion Q; outward,
// O is the dilation Q and C the original P.
func cupPayloadFor(pp prismPayload, offset ProfileRecord, s, t float64, removedEnd bool) cupPayload {
	z0, z1 := pp.z0, pp.z1
	o, c := pp.profile, offset
	if s < 0 {
		o, c = offset, pp.profile
	}
	cp := cupPayload{outer: o, cavity: c, frame: pp.frame, xform: pp.xform}
	if removedEnd { // open at the top
		cp.zOpen = z1
		if s > 0 {
			cp.zOuter, cp.zCav = z0, z0+t
		} else {
			cp.zOuter, cp.zCav = z0-t, z0
		}
	} else { // open at the bottom — the mirror
		cp.zOpen = z0
		if s > 0 {
			cp.zOuter, cp.zCav = z1, z1-t
		} else {
			cp.zOuter, cp.zCav = z1+t, z1
		}
	}
	return cp
}

// evalCup builds the analytic cup body (Table B, B5/B6): outer walls over every
// loop of O, cavity walls over every loop of the reversed C, the kept cap
// capStart, the pocket floor shellCap, and one rim per loop (1 + k of them) —
// one manifold, watertight shell (every edge bounds two faces), with Exact
// measurements per §10. The cavity region is a VOID: its loops invert, so the
// void's outer boundary is walked as a hole in the solid (its wall's material
// lies outside it) and each of the void's own holes as a solid post (material
// inside) — the pairing buildLoopSidesAs's explicit holeLoop expresses.
func evalCup(d *Document, ref StepRef, cp cupPayload) (*Body, error) {
	igO, err := cp.outer.integrals()
	if err != nil {
		return nil, err
	}
	igC, err := cp.cavity.integrals()
	if err != nil {
		return nil, err
	}
	if igO.area <= 0 || igC.area <= 0 {
		return nil, fmt.Errorf(`%w: a cup region encloses no area`, ErrDegenerate)
	}
	oLoops := append([]LoopRecord{cp.outer.Outer}, cp.outer.Holes...)
	cLoops := append([]LoopRecord{cp.cavity.Outer}, cp.cavity.Holes...)
	if len(oLoops) != len(cLoops) {
		// The offset preserves the loop count (a dropped loop is S11a, caught
		// before here); a mismatch would leave a rim with no partner loop.
		return nil, fmt.Errorf(`%w: the cup's outer and cavity regions have different loop counts`, ErrDegenerate)
	}
	hO := math.Abs(cp.zOpen - cp.zOuter)
	hC := math.Abs(cp.zOpen - cp.zCav)
	if hO <= 0 || hC <= 0 {
		return nil, fmt.Errorf(`%w: a cup interval is empty`, ErrDegenerate)
	}
	openIsMax := cp.zOpen > cp.zOuter

	body := &Body{doc: d, origin: FeatureRef{Step: ref, Role: "body"}, solid: true}

	// floorOpen splits a wall's (bottom, top) cap coedges into the floor-side
	// set (kept cap / pocket floor) and the open-side set (the rim).
	floorOpen := func(bottom, top []coedge) (floor, open []coedge) {
		if openIsMax {
			return bottom, top
		}
		return top, bottom
	}

	// Outer walls: every loop of O in its natural sense (outer counter-clockwise,
	// holes clockwise), role side(i,j). A hole of O is a tunnel through the whole
	// body — its wall runs the full outer interval.
	oLo, oHi := math.Min(cp.zOuter, cp.zOpen), math.Max(cp.zOuter, cp.zOpen)
	ppO := cp.prismLike(oLo, oHi)
	var faces []*Face
	perimO := 0.0
	oFloor := make([][]coedge, len(oLoops))
	oOpen := make([][]coedge, len(oLoops))
	for i, loop := range oLoops {
		sf, bottom, top, ll, err := buildLoopSides(body, ref, ppO, i, loop)
		if err != nil {
			return nil, err
		}
		faces = append(faces, sf...)
		perimO += ll
		oFloor[i], oOpen[i] = floorOpen(bottom, top)
	}

	// Cavity walls: every loop of C reversed and built over the cavity interval.
	// The reversed outer walks clockwise (a hole in the solid), each reversed
	// hole counter-clockwise (a post) — holeLoop is (i == 0). Roles are
	// shellSide(i,j) via renameCavityRoles.
	cLo, cHi := math.Min(cp.zCav, cp.zOpen), math.Max(cp.zCav, cp.zOpen)
	ppC := cp.prismLike(cLo, cHi)
	perimC := 0.0
	cFloor := make([][]coedge, len(cLoops))
	cOpen := make([][]coedge, len(cLoops))
	var cavFaces []*Face
	for i, loop := range cLoops {
		rev, err := reverseLoopRecord(loop)
		if err != nil {
			return nil, err
		}
		sf, bottom, top, ll, err := buildLoopSidesAs(body, ref, ppC, i, i == 0, rev)
		if err != nil {
			return nil, err
		}
		cavFaces = append(cavFaces, sf...)
		perimC += ll
		cFloor[i], cOpen[i] = floorOpen(bottom, top)
	}
	renameCavityRoles(cavFaces, ref)
	faces = append(faces, cavFaces...)

	// The planar faces. capFrame orients each normal outward via its flip;
	// capStart faces away from the material, shellCap into the pocket, the rim
	// away from the material at the open end.
	base := cp.prismLike(0, 0)
	capStartFrame, err := capFrame(base, cp.zOuter, openIsMax)
	if err != nil {
		return nil, err
	}
	shellCapFrame, err := capFrame(base, cp.zCav, !openIsMax)
	if err != nil {
		return nil, err
	}
	rimFrame, err := capFrame(base, cp.zOpen, !openIsMax)
	if err != nil {
		return nil, err
	}

	// The kept cap and the pocket floor each carry one loop per region loop: the
	// outer boundary (outer true) and each hole (a tunnel through the kept cap, a
	// post through the pocket floor).
	capStart := &Face{
		surface: Plane{Frame: capStartFrame},
		origins: []FeatureRef{{Step: ref, Role: roleCapStart}},
		body:    body,
		area:    igO.area,
	}
	shellCap := &Face{
		surface: Plane{Frame: shellCapFrame},
		origins: []FeatureRef{{Step: ref, Role: "shellCap"}},
		body:    body,
		area:    igC.area,
	}
	for i := range oLoops {
		capStart.loops = append(capStart.loops, &Loop{coedges: oFloor[i], outer: i == 0})
		shellCap.loops = append(shellCap.loops, &Loop{coedges: cFloor[i], outer: i == 0})
	}

	// Rims: one per loop, the removed cap's plane trimmed to the band between
	// loop i of O and loop i of C. The bigger boundary is O for the outer region
	// (the cavity sits inside it) and C for a hole loop (a post rim is wider than
	// the tunnel it wraps), so the outer/hole roles of the two loops swap at i≥1.
	// Loops() lists the outer loop FIRST (topology.go), so the outer boundary
	// heads the slice: O for the outer region, C for a post rim.
	rims := make([]*Face, len(oLoops))
	for i := range oLoops {
		oIsOuter := i == 0
		aO, err := loopEnclosedArea(oLoops[i])
		if err != nil {
			return nil, err
		}
		aC, err := loopEnclosedArea(cLoops[i])
		if err != nil {
			return nil, err
		}
		oLoop := &Loop{coedges: oOpen[i], outer: oIsOuter}
		cLoop := &Loop{coedges: cOpen[i], outer: !oIsOuter}
		outerLoop, holeLoop := oLoop, cLoop
		if !oIsOuter {
			outerLoop, holeLoop = cLoop, oLoop
		}
		rims[i] = &Face{
			surface: Plane{Frame: rimFrame},
			origins: []FeatureRef{{Step: ref, Role: fmt.Sprintf("rim(%d)", i)}},
			body:    body,
			area:    math.Abs(aO - aC),
			loops:   []*Loop{outerLoop, holeLoop},
		}
	}

	// Attach each planar face to its boundary edges (two faces per edge).
	for i := range oLoops {
		for _, ce := range oFloor[i] {
			ce.edge.faces = append(ce.edge.faces, capStart)
		}
		for _, ce := range oOpen[i] {
			ce.edge.faces = append(ce.edge.faces, rims[i])
		}
	}
	for i := range cLoops {
		for _, ce := range cFloor[i] {
			ce.edge.faces = append(ce.edge.faces, shellCap)
		}
		for _, ce := range cOpen[i] {
			ce.edge.faces = append(ce.edge.faces, rims[i])
		}
	}

	faces = append(faces, capStart, shellCap)
	faces = append(faces, rims...)
	body.lumps = []*Lump{{shells: []*Shell{{faces: faces}}}}

	// Measurements — all Exact (docs/modify-design.md §10).
	body.volume = Measurement{Value: units.CubicMillimeters(igO.area*hO - igC.area*hC), Exactness: Exact, Bound: units.CubicMillimeters(0)}
	area := 2*igO.area + perimO*hO + perimC*hC
	body.area = Measurement{Value: units.SquareMillimeters(area), Exactness: Exact, Bound: units.SquareMillimeters(0)}

	// Centroid: each region's centroid lifted to its own interval midpoint, the
	// two combined with the cavity's mass subtracted (§10).
	zMidO := (cp.zOuter + cp.zOpen) / 2
	zMidC := (cp.zCav + cp.zOpen) / 2
	cO := cp.prismLike(0, 0).point(igO.mu/igO.area, igO.mv/igO.area, zMidO)
	cC := cp.prismLike(0, 0).point(igC.mu/igC.area, igC.mv/igC.area, zMidC)
	massO := igO.area * hO
	massC := igC.area * hC
	denom := massO - massC
	if denom <= 0 {
		return nil, fmt.Errorf(`%w: the cup cavity is not smaller than its outer solid`, ErrDegenerate)
	}
	body.centroid = VecMeasurement{
		Value:     cO.Scale(massO / denom).Sub(cC.Scale(massC / denom)),
		Exactness: Exact,
		Bound:     units.Millimeters(0),
	}

	// Bounds: the outer prism's — the cavity lies within it in both senses (§10).
	bounds, err := prismBounds(prismPayload{profile: cp.outer, frame: cp.frame, z0: oLo, z1: oHi, xform: cp.xform})
	if err != nil {
		return nil, err
	}
	body.bounds = bounds
	body.payload = cp
	return body, nil
}

// renameCavityRoles rewrites the cavity walls' provenance from the
// buildLoopSidesAs default side(i,j) to shellSide(i,j) — the role Table B
// (B5/B6) gives a cavity wall, indexing loop i of the cavity region Q in the
// result's own record (§11). The cavity walls are built with roleLoop equal to
// their Q loop index, so the rename keeps the index and only swaps the tag.
func renameCavityRoles(faces []*Face, ref StepRef) {
	for _, f := range faces {
		for i, o := range f.origins {
			if o.Step != ref {
				continue
			}
			var li, j int
			if n, _ := fmt.Sscanf(o.Role, "side(%d,%d)", &li, &j); n == 2 {
				f.origins[i] = FeatureRef{Step: ref, Role: fmt.Sprintf("shellSide(%d,%d)", li, j)}
			}
		}
	}
}

// loopEnclosedArea is the absolute area a single loop encloses — the magnitude
// of its Green's-theorem signed area, so a hole (clockwise) and an outer loop
// (counter-clockwise) both report a positive area. A rim band's area is the
// difference of the two loops it spans, and both are strictly nested (the audit
// proved the cavity simple and inside the outer region), so the absolute
// difference IS the band's exact area.
func loopEnclosedArea(l LoopRecord) (float64, error) {
	var ig regionIntegrals
	for _, seg := range l.Segments {
		if err := ig.add(seg); err != nil {
			return 0, err
		}
	}
	return math.Abs(ig.area), nil
}
