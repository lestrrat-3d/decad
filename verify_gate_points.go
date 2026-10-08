package decad

import (
	"context"
	"math"

	"github.com/lestrrat-3d/decad/internal/diameter"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/tolerance"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// This file lists the points a gate diameter reads (docs/verification-design.md
// §3): station witnesses along a prism's walls and a revolve's meridian, a
// brep's vertices and wall stations, and the pair diameter read over two
// bodies' points together. Every point is held within a gap a proof states
// from a point of the body it belongs to.

// gatePoints is a set of held points, each within allow of a point of the
// body (or bodies) it was read from.
type gatePoints struct {
	pts   []r3.Vec
	allow float64
}

// join adds o's points; the joined set's allow is the larger of the two.
func (g *gatePoints) join(o gatePoints) {
	g.pts = append(g.pts, o.pts...)
	g.allow = math.Max(g.allow, o.allow)
}

// diameter is the lower bound g proves on the diameter of the geometry its
// points belong to: the largest pair distance among the held points, rounded
// toward zero (pointSetDiameterWithBudget), shrunk by twice allow, since each
// end of that pair sits within allow of a point of the geometry.
func (g gatePoints) diameter(budget *proofbound.WorkBudget) (float64, bool, error) {
	d, ok, err := pointSetDiameterWithBudget(budget, g.pts)
	if err != nil || !ok {
		return 0, false, err
	}
	d, ok = lowerDiameterForDisplacement(d, g.allow)
	return d, ok, nil
}

// stationGateDiameter reads a diameter from the station witnesses of prisms
// alone (prismGatePoints). ok is false, with no error, when any prism's
// stations cannot be read.
func stationGateDiameter(budget *proofbound.WorkBudget, prisms []prismPayload, displacement float64) (float64, bool, error) {
	g, ok, err := prismGatePoints(budget, prisms, displacement)
	if err != nil || !ok {
		return 0, false, err
	}
	return g.diameter(budget)
}

// prismGatePoints lists the station witnesses of prisms
// (prismStationWitnesses). Each station is held within a proven gap of a
// point of its prism's recorded walls, and the recorded walls lie within
// displacement of the walls the payload denotes, so allow is displacement
// plus the widest gap. ok is false, with no error, when any prism's stations
// cannot be read.
//
// The carrier witnesses addPrismFaces places (CFace.Wit) are float samples
// with no proven gap to the body: a placed prism's held carrier maximum reads
// above the body's own diameter. This reading never reads them. Each one sits between
// two stations at its own angle, at a station, or inside the section.
//
// Every prism handed in must have walls that run the full height from z0 to
// z1 on the body (or z0 equal to z1, with its section's boundary on the
// body at that level), so that each station is a point of the body (within
// its gap and displacement).
func prismGatePoints(budget *proofbound.WorkBudget, prisms []prismPayload, displacement float64) (gatePoints, bool, error) {
	work := freeform.NewFreeformWork()
	var g gatePoints
	for _, pp := range prisms {
		stations, stationAllow, read, err := prismStationWitnesses(budget, pp, work)
		if err != nil || !read {
			return gatePoints{}, false, err
		}
		g.join(gatePoints{pts: stations, allow: stationAllow})
	}
	g.allow = proofbound.AbsSumUpper(displacement, g.allow)
	return g, true, nil
}

// prismStationWitnesses lifts the stations diameter.SectionStations lists on pp's
// walls to both z0 and z1. allow is
// the widest proven distance from a held point to the point of pp's recorded
// walls it stands for: the station's own gap carried through the frame and
// placement by prismPointBound, together with the lift's own rounding.
//
// read is false, with no error, wherever diameter.SectionStations' is, and for a lift
// whose gap the proof cannot state. Cancellation through budget returns the
// error.
func prismStationWitnesses(budget *proofbound.WorkBudget, pp prismPayload, work *freeform.FreeformWork) ([]r3.Vec, float64, bool, error) {
	stations, ok, err := diameter.SectionStations(budget, append([]LoopRecord{pp.profile.Outer}, pp.profile.Holes...), work)
	if err != nil || !ok {
		return nil, 0, false, err
	}
	pts := make([]r3.Vec, 0, 2*len(stations))
	allow := 0.0
	for _, s := range stations {
		for _, z := range [2]float64{pp.z0, pp.z1} {
			a := prismPointBound(pp, proofbound.MeasuredScalar(s.U, s.Bound.U), proofbound.MeasuredScalar(s.V, s.Bound.V),
				proofbound.MeasuredScalar(z, 0))
			if !tolerance.UsableMagnitude(a) {
				return nil, 0, false, nil
			}
			allow = math.Max(allow, a)
			pts = append(pts, pp.point(s.U, s.V, z))
		}
	}
	return pts, allow, true, nil
}

// revolveGateDiameter reads a revolvePayload's diameter from the points
// revolveGatePoints proves lie on the body.
func revolveGateDiameter(budget *proofbound.WorkBudget, rp revolvePayload) (float64, bool, error) {
	g, ok, err := revolveGatePoints(budget, rp)
	if err != nil || !ok {
		return 0, false, err
	}
	return g.diameter(budget)
}

// revolveGatePoints lists points a revolvePayload proves lie on its body, for
// a gate diameter alone (docs/verification-design.md §3).
//
// Two points on circles of radii r1 and r2 about the axis, at axial
// separation dz, sit sqrt(dz^2 + r1^2 + r2^2 - 2*r1*r2*cos(dphi)) apart, which
// grows with the angle dphi between them up to half a turn. The body's
// farthest pair therefore sits at the widest angle apart the sweep allows, up
// to half a turn: the two ends of a sweep of at most half a turn, or angles
// half a turn apart otherwise. diameter.RevolveGateAngles lists those angles, each
// beside the sine and cosine of the angle it denotes. Every meridian station
// (diameter.SectionStations) is swept to each of them.
//
// Each held point is compared exactly against the point it denotes:
// revolvemesh.RevolveLift.SweptPointGap rotates the recorded station, widened
// by its own gap and the payload's sectionDelta, about the recorded axis to
// the denoted angle and lifts it through the frame and placement. allow is
// the widest gap. ok is false, with no error, where the stations cannot be read, an end states
// no angle (a ToFaceAngular stop), or a gap cannot be stated.
func revolveGatePoints(budget *proofbound.WorkBudget, rp revolvePayload) (gatePoints, bool, error) {
	angles, ok := diameter.RevolveGateAngles(rp.phi0, rp.phi1, rp.den.Phi0, rp.den.Phi1)
	if !ok {
		return gatePoints{}, false, nil
	}
	stations, ok, err := diameter.SectionStations(budget, append([]LoopRecord{rp.profile.Outer}, rp.profile.Holes...),
		freeform.NewFreeformWork())
	if err != nil || !ok {
		return gatePoints{}, false, err
	}
	b, lift, ab, ax := rp.basis(), rp.lift(), rp.axisBound(), rp.ax.numeric()
	pts := make([]r3.Vec, 0, len(stations)*len(angles))
	allow := 0.0
	for _, s := range stations {
		uv := proofbound.WalkEndBound{
			U: proofbound.AbsSumUpper(s.Bound.U, rp.sectionDelta),
			V: proofbound.AbsSumUpper(s.Bound.V, rp.sectionDelta),
		}
		z, rho := ax.ToAxis(s.U, s.V)
		for _, a := range angles {
			if err := budget.Step(); err != nil {
				return gatePoints{}, false, err
			}
			held := rp.point(b, z, rho, a.Phi)
			gap := lift.SweptPointGap(ab, rp.xform, s.U, s.V, uv, a.Sin, a.Cos, held)
			if !proofbound.FiniteVec(held) || !tolerance.UsableMagnitude(gap) {
				return gatePoints{}, false, nil
			}
			allow = math.Max(allow, gap)
			pts = append(pts, held)
		}
	}
	return gatePoints{pts: pts, allow: allow}, true, nil
}

// vertexGatePoints lists the body's vertices, each held within its published
// Position().Bound of the point it denotes (topology.go's Vertex.Position
// contract), with extraAllow added for any displacement those bounds leave
// out. ok is false, with no error, when a vertex has no finite position or
// usable bound.
func vertexGatePoints(budget *proofbound.WorkBudget, body *Body, extraAllow float64) (gatePoints, bool, error) {
	vertices := body.Vertices()
	g := gatePoints{pts: make([]r3.Vec, 0, len(vertices))}
	for _, vertex := range vertices {
		if err := budget.Step(); err != nil {
			return gatePoints{}, false, err
		}
		position := vertex.Position()
		bound := position.Bound.Base()
		if !proofbound.FiniteVec(position.Value) || !tolerance.UsableMagnitude(bound) {
			return gatePoints{}, false, nil
		}
		g.pts = append(g.pts, position.Value)
		g.allow = math.Max(g.allow, bound)
	}
	g.allow = proofbound.AbsSumUpper(g.allow, extraAllow)
	return g, true, nil
}

// chainGatePoints lists an ExtrudeChain body's vertices beside the stations
// along its chains at both levels. The vertices carry their published bounds
// plus the section displacement and the walk-end rounding the topology does
// not charge (chainWalkEndpointAllow). Vertices alone read only an arc's two
// ends, so an arc closed by its chord would read the chord. The stations are
// read off the payload's prism view, whose walls are the chains swept over
// the whole interval, so every station is a point of the sheet; they carry
// their own gaps plus the section and level displacements. A chain the
// stations cannot read (a free-form segment) keeps its vertices alone. ok is
// false, with no error, when the walk ends or vertices cannot be bounded.
func chainGatePoints(ctx context.Context, budget *proofbound.WorkBudget, body *Body, pp chainPayload) (gatePoints, bool, error) {
	endpointAllow, ok, err := chainWalkEndpointAllow(ctx, pp.chains)
	if err != nil || !ok {
		return gatePoints{}, false, err
	}
	g, ok, err := vertexGatePoints(budget, body, proofbound.AbsSumUpper(pp.sectionDelta, endpointAllow))
	if err != nil || !ok {
		return gatePoints{}, false, err
	}
	view := pp.prism()
	stations, read, err := prismGatePoints(budget, []prismPayload{view}, proofbound.AbsSumUpper(pp.sectionDelta, view.axialDelta()))
	if err != nil {
		return gatePoints{}, false, err
	}
	if read {
		g.join(stations)
	}
	return g, true, nil
}

// capBlendGateDiameter is bodyGateDiameter's arm for a cap-loop chamfer,
// read over capBlendGatePoints.
func capBlendGateDiameter(ctx context.Context, budget *proofbound.WorkBudget, body *Body, cbp capBlendPayload) (float64, bool, error) {
	g, ok, err := capBlendGatePoints(ctx, budget, body, cbp)
	if err != nil || !ok {
		return 0, false, err
	}
	return g.diameter(budget)
}

// capBlendGatePoints lists points a cap-loop chamfer proves lie on its body.
// It joins four sets:
//
//   - the stations of capBlendWitnessPrisms: every loop's wall between the
//     side levels its bands leave, charged their gaps and the levels'
//     displacement (capBlendPayload.axialDelta);
//   - the stations on every whole cap circle, the cap contour a cornerless
//     closed circle offsets into, at its cap level. The band holds that
//     circle at its own centre and offset radius, and each station carries
//     its gap from the held circle, the widest band's contour displacement
//     (bandDelta, capband.WholeCircleDisplacement for such a band) and the
//     axial term;
//   - the stations on every cap contour arc a corner trims, the offset of a
//     circular wall cut back at its two corner feet, at its cap level
//     (capArcRim);
//   - the body's vertices with their published bounds, which hold every cap
//     contour corner.
//
// A reflex corner's connector arc is read at its ends alone, through the
// vertices. ok is false, with no error, when the side-level stations or the
// vertices cannot be read.
func capBlendGatePoints(ctx context.Context, budget *proofbound.WorkBudget, body *Body, cbp capBlendPayload) (gatePoints, bool, error) {
	if err := ctx.Err(); err != nil {
		return gatePoints{}, false, err
	}
	g, ok, err := prismGatePoints(budget, capBlendWitnessPrisms(cbp), cbp.axialDelta())
	if err != nil || !ok {
		return gatePoints{}, false, err
	}
	contour := 0.0
	for _, d := range cbp.bandDelta {
		contour = math.Max(contour, d)
	}
	var rims []prismPayload
	for _, p := range cbp.patches {
		if !p.geom.Circular || !p.geom.WholeTurn {
			continue
		}
		rim := cbp.prismLike(p.geom.CapZ, p.geom.CapZ)
		rim.profile = ProfileRecord{Outer: LoopRecord{Segments: []CurveSegment{CircleSeg{
			Center: Point2{U: p.geom.CU, V: p.geom.CV}, Radius: units.Millimeters(p.geom.CapRadius),
			CCW: true, TStart: 0, TEnd: 1,
		}}}}
		rims = append(rims, rim)
	}
	if len(rims) > 0 {
		caps, ok, err := prismGatePoints(budget, rims, proofbound.AbsSumUpper(cbp.axialDelta(), contour))
		if err != nil {
			return gatePoints{}, false, err
		}
		if ok {
			g.join(caps)
		}
	}
	for _, p := range cbp.patches {
		rim, arcAllow, ok := capArcRim(cbp, p.geom, contour)
		if !ok {
			continue
		}
		arc, ok, err := prismGatePoints(budget, []prismPayload{rim}, proofbound.AbsSumUpper(cbp.axialDelta(), arcAllow))
		if err != nil {
			return gatePoints{}, false, err
		}
		if ok {
			g.join(arc)
		}
	}
	vertices, ok, err := vertexGatePoints(budget, body, 0)
	if err != nil || !ok {
		return gatePoints{}, false, err
	}
	g.join(vertices)
	return g, true, nil
}

// capArcRim returns a station prism over the cap contour arc a circular wall
// patch g holds trimmed by its two corners, and how far any point of that arc
// sits from the arc the band denotes. contour is a proven upper bound on the
// band's contour displacement (bandDelta). ok is false for any other patch,
// and wherever the bound cannot be stated.
//
// The prism's one segment is the ArcSeg the cap face records: counter-clockwise
// about the wall's centre (g.CU, g.CV) from the held cap vertex g.CapA, at
// g.CapTh0, to g.CapB, at g.CapTh1, at the cap level g.CapZ. Its stations
// (diameter.SectionStations) each carry their gap from that recorded arc. The
// band denotes the offset of the wall's circle trimmed at the two corner feet
// F0 and F1. A point of the recorded arc at angle θ lies at radius r, the
// distance from the centre to g.CapA:
//
//   - r is within g.Held.CapRadius of the held offset radius g.CapRadius
//     (capband.WallHeldAllow), and g.CapRadius within contour of every radius
//     the band denotes, since capcontour.Displacement's circular term is that
//     distance. So r is within the sum of the two of the denoted radius, and a
//     point at angle θ inside the denoted window is within that sum of the
//     denoted arc.
//   - Each held cap vertex lies within contour of its corner foot, and both
//     sit at least rMin = g.CapRadius − max(g.Held.CapRadius, contour) from
//     the centre. Two points at least rMin out and c apart subtend at most
//     2·asin(c/(2·rMin)) ≤ (π/2)·c/rMin. So each end of the recorded window
//     sits within beta = (π/2)·contour/rMin of the denoted window's end, and
//     a point of the recorded arc outside the denoted window lies within
//     r·beta of the point at the denoted end's angle and radius r, and so
//     within r·beta plus the radius term of the denoted end F0 or F1.
//
// The allowance is the radius term plus (g.CapRadius + g.Held.CapRadius)·beta.
// The recorded arc's own sweep must agree with the held window to 1e-9 rad,
// a check that can only refuse: a mismatch would name the complementary arc.
func capArcRim(cbp capBlendPayload, g capPatchGeom, contour float64) (prismPayload, float64, bool) {
	if !g.Circular || g.WholeTurn || g.SideRadius <= 0 || !(g.CapTh1 > g.CapTh0) {
		return prismPayload{}, 0, false
	}
	center := Point2{U: g.CU, V: g.CV}
	if g.CapA == center || g.CapB == center {
		return prismPayload{}, 0, false
	}
	a0 := math.Atan2(g.CapA.V-g.CV, g.CapA.U-g.CU)
	sweep := math.Mod(math.Atan2(g.CapB.V-g.CV, g.CapB.U-g.CU)-a0, 2*math.Pi)
	if sweep <= 0 {
		sweep += 2 * math.Pi
	}
	if math.Abs(sweep-(g.CapTh1-g.CapTh0)) > 1e-9 {
		return prismPayload{}, 0, false
	}
	rMin := math.Nextafter(g.CapRadius-math.Max(g.Held.CapRadius, contour), 0)
	if !(rMin > contour) || proofbound.IsNonFinite(rMin) {
		return prismPayload{}, 0, false
	}
	beta := proofbound.DivUpper(proofbound.ProductUpper(math.Nextafter(math.Pi/2, math.Inf(1)), contour), rMin)
	radial := proofbound.AbsSumUpper(g.Held.CapRadius, contour)
	allow := proofbound.AbsSumUpper(radial, proofbound.ProductUpper(proofbound.AbsSumUpper(g.CapRadius, g.Held.CapRadius), beta))
	if !tolerance.UsableMagnitude(allow) {
		return prismPayload{}, 0, false
	}
	rim := cbp.prismLike(g.CapZ, g.CapZ)
	rim.profile = ProfileRecord{Outer: LoopRecord{Segments: []CurveSegment{ArcSeg{
		Center: center, Start: g.CapA, End: g.CapB, TStart: 0, TEnd: 1,
	}}}}
	return rim, allow, true
}

// brepGateDiameter is bodyGateDiameter's arm for a brepPayload
// (docs/general-boolean-design.md §4.5), read over brepGatePoints.
func brepGateDiameter(ctx context.Context, body *Body, bp brepPayload) (float64, bool, error) {
	budget := proofbound.NewWorkBudget(ctx)
	g, ok, err := brepGatePoints(budget, body, bp)
	if err != nil || !ok {
		return 0, false, err
	}
	return g.diameter(budget)
}

// brepGatePoints lists every body vertex, held within its published
// Position().Bound of the point it denotes, and on every face the stations
// prismStationWitnesses places on the face's prism view. A swept face is its
// wall swept over the whole of [z0, z1]: its side splits add vertices on the
// side lines and remove nothing, and its two rims are the whole wall at z0
// and z1, so every station on it is a point of the face. A planar face's
// view holds its region at its one level, so every station on the region's
// boundary is a point of the face. A face whose stations cannot be read (a
// free-form wall) adds none, and the vertices still hold its corners. The
// recorded body lies within the largest section displacement plus the
// largest level displacement of the body the record denotes, so allow is
// that sum plus the widest vertex bound or station gap.
func brepGatePoints(budget *proofbound.WorkBudget, body *Body, bp brepPayload) (gatePoints, bool, error) {
	g, ok, err := vertexGatePoints(budget, body, 0)
	if err != nil || !ok {
		return gatePoints{}, false, err
	}
	work := freeform.NewFreeformWork()
	for _, f := range bp.faces {
		stations, stationAllow, read, err := prismStationWitnesses(budget, f.view(bp.xform), work)
		if err != nil {
			return gatePoints{}, false, err
		}
		if read {
			g.join(gatePoints{pts: stations, allow: stationAllow})
		}
	}
	g.allow = proofbound.AbsSumUpper(bp.sectionDelta(), bp.axialDelta(), g.allow)
	return g, true, nil
}

// bodyGatePoints lists points the body proves lie on it, for the pair
// diameter (pairGateDiameter). Each payload reads the points its own gate
// diameter arm reads where that arm reads points: a faceted, loft, mitred
// sweep or stitched body's held vertex table with its published delta, a
// coil's held station table with its largest station rounding, an
// ExtrudeChain body's vertices and stations, a brep's vertices and face
// stations, a draft body's stations on both caps, a cap-loop chamfer's side
// levels, whole cap circles and vertices, an analytic prism's or a revolve's stations, and the stations of
// the witness prisms fallbackGateDiameter reads. Every other body, and any of
// those whose stations cannot be read, reads its vertices with their
// published bounds. ok is false, with no error, when none of those can be
// bounded.
func bodyGatePoints(ctx context.Context, budget *proofbound.WorkBudget, body *Body) (gatePoints, bool, error) {
	switch pl := body.payload.(type) {
	case facetedPayload:
		return gatePoints{pts: pl.verts}, true, nil
	case loftPayload:
		return gatePoints{pts: pl.verts, allow: pl.delta}, true, nil
	case mitredSweepPayload:
		return gatePoints{pts: pl.verts, allow: pl.delta}, true, nil
	case coilPayload:
		// Every held station vertex lies within its own station rounding of
		// the point X(v, j) it denotes, a point of the coil's true helix edge
		// (docs/helix-design.md §5.3), so the largest rounding is the gap.
		return gatePoints{pts: pl.verts, allow: pl.maxRound}, true, nil
	case stitchPayload:
		return gatePoints{pts: pl.verts, allow: pl.delta}, true, nil
	case chainPayload:
		return chainGatePoints(ctx, budget, body, pl)
	case brepPayload:
		return brepGatePoints(budget, body, pl)
	case draftPayload:
		prisms, displacement := draftCapPrisms(pl)
		if g, ok, err := prismGatePoints(budget, prisms, displacement); err != nil || ok {
			return g, ok, err
		}
		return vertexGatePoints(budget, body, 0)
	case revolvePayload:
		if g, ok, err := revolveGatePoints(budget, pl); err != nil || ok {
			return g, ok, err
		}
		return vertexGatePoints(budget, body, 0)
	case capBlendPayload:
		return capBlendGatePoints(ctx, budget, body, pl)
	case prismPayload:
		if pl.sectionDelta == 0 {
			if g, ok, err := prismGatePoints(budget, []prismPayload{pl}, pl.axialDelta()); err != nil || ok {
				return g, ok, err
			}
			return vertexGatePoints(budget, body, 0)
		}
	}
	if prisms, displacement, ok := gateWitnessPrisms(body.payload); ok {
		if g, ok, err := prismGatePoints(budget, prisms, displacement); err != nil || ok {
			return g, ok, err
		}
	}
	return vertexGatePoints(budget, body, 0)
}

// pairGateDiameter reads the diameter D of the pair a and b (verification
// design §3): the greatest distance between two points drawn from either
// body. It reads bodyGatePoints for both bodies together, so the reading
// ranges over pairs across the two bodies as well as within each, and
// shrinks the maximum by twice the larger allow. Every point is held within
// that allow of a point of one of the two bodies, so the reading is a lower
// bound on the pair's own diameter. A body with no point set leaves the pair
// with the other body's points alone, and a pair with neither reads zero: an
// understated D only tightens the gate.
func pairGateDiameter(ctx context.Context, a, b *Body) (float64, error) {
	budget := proofbound.NewWorkBudget(ctx)
	var g gatePoints
	for _, body := range [2]*Body{a, b} {
		points, ok, err := bodyGatePoints(ctx, budget, body)
		if err != nil {
			return 0, err
		}
		if ok {
			g.join(points)
		}
	}
	d, ok, err := g.diameter(budget)
	if err != nil || !ok {
		return 0, err
	}
	return d, nil
}
