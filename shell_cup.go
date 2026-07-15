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
// This evaluator's cup builder is hole-free; a one-cap shell of a holed section
// (which the design builds) is staged at the call (shell.go), never a wrong
// body.

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

// evalCup builds the analytic cup body (Table B, B5/B6): outer walls over O,
// the kept cap capStart, the rim, cavity walls over the reversed C, and the
// cavity cap shellCap — one manifold, watertight shell (every edge bounds two
// faces), with Exact measurements per §10. The cavity is walked as a hole (its
// wall's material lies outside it), which is what a reversed loop and the
// hole-side build give it.
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
	hO := math.Abs(cp.zOpen - cp.zOuter)
	hC := math.Abs(cp.zOpen - cp.zCav)
	if hO <= 0 || hC <= 0 {
		return nil, fmt.Errorf(`%w: a cup interval is empty`, ErrDegenerate)
	}
	openIsMax := cp.zOpen > cp.zOuter

	body := &Body{doc: d, origin: FeatureRef{Step: ref, Role: "body"}, solid: true}

	// Outer walls over O, swept between the outer floor and the open end.
	oLo, oHi := math.Min(cp.zOuter, cp.zOpen), math.Max(cp.zOuter, cp.zOpen)
	ppO := cp.prismLike(oLo, oHi)
	outerFaces, oBottom, oTop, perimO, err := buildLoopSides(body, ref, ppO, 0, cp.outer.Outer)
	if err != nil {
		return nil, err
	}
	outerRim, outerCap := oTop, oBottom
	if !openIsMax {
		outerRim, outerCap = oBottom, oTop
	}

	// Cavity walls over the reversed C, swept between the cavity floor and the
	// open end, built as a hole (li = 1) so its wall's material lies outside it;
	// its role is shellSide, not side.
	crev, err := reverseLoopRecord(cp.cavity.Outer)
	if err != nil {
		return nil, err
	}
	cLo, cHi := math.Min(cp.zCav, cp.zOpen), math.Max(cp.zCav, cp.zOpen)
	ppC := cp.prismLike(cLo, cHi)
	cavFaces, cBottom, cTop, perimC, err := buildLoopSides(body, ref, ppC, 1, crev)
	if err != nil {
		return nil, err
	}
	renameCavityRoles(cavFaces, ref)
	cavRim, cavCap := cTop, cBottom
	if !openIsMax {
		cavRim, cavCap = cBottom, cTop
	}

	// The three planar faces. capFrame orients each normal outward via its flip;
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
	capStart := &Face{
		surface: Plane{Frame: capStartFrame},
		origins: []FeatureRef{{Step: ref, Role: roleCapStart}},
		body:    body,
		area:    igO.area,
		loops:   []*Loop{{coedges: outerCap, outer: true}},
	}
	shellCap := &Face{
		surface: Plane{Frame: shellCapFrame},
		origins: []FeatureRef{{Step: ref, Role: "shellCap"}},
		body:    body,
		area:    igC.area,
		loops:   []*Loop{{coedges: cavCap, outer: true}},
	}
	rim := &Face{
		surface: Plane{Frame: rimFrame},
		origins: []FeatureRef{{Step: ref, Role: "rim(0)"}},
		body:    body,
		area:    igO.area - igC.area,
		loops: []*Loop{
			{coedges: outerRim, outer: true},
			{coedges: cavRim, outer: false},
		},
	}
	for _, ce := range outerCap {
		ce.edge.faces = append(ce.edge.faces, capStart)
	}
	for _, ce := range cavCap {
		ce.edge.faces = append(ce.edge.faces, shellCap)
	}
	for _, ce := range append(append([]coedge{}, outerRim...), cavRim...) {
		ce.edge.faces = append(ce.edge.faces, rim)
	}

	faces := append([]*Face{}, outerFaces...)
	faces = append(faces, capStart)
	faces = append(faces, cavFaces...)
	faces = append(faces, shellCap, rim)
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

// renameCavityRoles rewrites the cavity walls' provenance from the buildLoopSides
// default side(1,j) to shellSide(0,j) — the role Table B (B5/B6) gives a cavity
// wall, indexing the result's own record (§11).
func renameCavityRoles(faces []*Face, ref StepRef) {
	for _, f := range faces {
		for i, o := range f.origins {
			if o.Step != ref {
				continue
			}
			var li, j int
			if n, _ := fmt.Sscanf(o.Role, "side(%d,%d)", &li, &j); n == 2 {
				f.origins[i] = FeatureRef{Step: ref, Role: fmt.Sprintf("shellSide(%d,%d)", li-1, j)}
			}
		}
	}
}
