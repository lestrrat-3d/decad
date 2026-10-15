package decad

import (
	"context"
	"math"

	"github.com/lestrrat-3d/decad/internal/diameter"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// This file lists the points a gate diameter reads (docs/verification-design.md
// §3): station witnesses along a prism's walls and a revolve's meridian, a
// brep's vertices and wall stations, and the pair diameter read over two
// bodies' points together. Every point is held within a gap a proof states
// from a point of the body it belongs to.

type gatePoints = diameter.GatePoints

// stationGateDiameter reads a diameter from the station witnesses of prisms
// alone (prismGatePoints). ok is false, with no error, when any prism's
// stations cannot be read.
func stationGateDiameter(budget *proofbound.WorkBudget, prisms []prismPayload, displacement float64) (float64, bool, error) {
	g, ok, err := prismGatePoints(budget, prisms, displacement)
	if err != nil || !ok {
		return 0, false, err
	}
	return g.Diameter(budget)
}

// prismGatePoints lists the station witnesses of prisms
// (diameter.PrismStationWitnesses). Each station is held within a proven gap of a
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
	return prismGatePointsMemo(budget, prisms, displacement, nil)
}

// prismGatePointsMemo is prismGatePoints reading section stations through
// stations (nil reads them fresh).
func prismGatePointsMemo(budget *proofbound.WorkBudget, prisms []prismPayload, displacement float64,
	stations *diameter.StationMemo) (gatePoints, bool, error) {
	work := freeform.NewFreeformWork()
	var g gatePoints
	for _, pp := range prisms {
		points, stationAllow, read, err := diameter.PrismStationWitnessesMemo(budget, pp, work, stations)
		if err != nil || !read {
			return gatePoints{}, false, err
		}
		g.Join(gatePoints{Points: points, Allow: stationAllow})
	}
	g.Allow = proofbound.AbsSumUpper(displacement, g.Allow)
	return g, true, nil
}

// revolveGateDiameter reads a revolvePayload's diameter from the points
// revolveGatePoints proves lie on the body.
func revolveGateDiameter(budget *proofbound.WorkBudget, rp revolvePayload) (float64, bool, error) {
	g, ok, err := revolveGatePoints(budget, rp)
	if err != nil || !ok {
		return 0, false, err
	}
	return g.Diameter(budget)
}

// revolveGatePoints passes the payload's recorded meridian, axis, and pose
// to the witness calculation used by its diameter gate.
func revolveGatePoints(budget *proofbound.WorkBudget, rp revolvePayload) (gatePoints, bool, error) {
	return diameter.RevolveWitnesses(budget, diameter.RevolveWitnessInput{
		Phi0: rp.Phi0, Phi1: rp.Phi1, Den0: rp.Den.Phi0, Den1: rp.Den.Phi1,
		Loops: append([]loopRecord{rp.Profile.Outer}, rp.Profile.Holes...),
		Lift:  rp.Lift(), AxisBound: rp.AxisBound(), Axis: rp.Ax,
		Transform: rp.Xform, SectionDelta: rp.SectionDelta,
	})
}

// vertexGatePoints lists the body's vertices, each held within its published
// Position().Bound of the point it denotes (topology.go's Vertex.Position
// contract), with extraAllow added for any displacement those bounds leave
// out. ok is false, with no error, when a vertex has no finite position or
// usable bound.
func vertexGatePoints(budget *proofbound.WorkBudget, body *Body, extraAllow float64) (gatePoints, bool, error) {
	vertices := body.Vertices()
	points := make([]r3.Vec, len(vertices))
	bounds := make([]float64, len(vertices))
	for i, vertex := range vertices {
		position := vertex.Position()
		points[i], bounds[i] = position.Value, position.Bound.Base()
	}
	return diameter.VertexWitnesses(budget, points, bounds, extraAllow)
}

// chainGatePoints lists a chain Extrude body's vertices beside the stations
// along its chains at both levels. The vertices carry their published bounds
// plus the section displacement and the walk-end rounding the topology does
// not charge (diameter.ChainWalkEndpointAllow). Vertices alone read only an arc's two
// ends, so an arc closed by its chord would read the chord. The stations are
// read off the payload's prism view, whose walls are the chains swept over
// the whole interval, so every station is a point of the sheet; they carry
// their own gaps plus the section and level displacements. A chain the
// stations cannot read (a free-form segment) keeps its vertices alone. ok is
// false, with no error, when the walk ends or vertices cannot be bounded.
func chainGatePoints(ctx context.Context, budget *proofbound.WorkBudget, body *Body, pp chainPayload) (gatePoints, bool, error) {
	endpointAllow, ok, err := diameter.ChainWalkEndpointAllow(ctx, pp.Chains)
	if err != nil || !ok {
		return gatePoints{}, false, err
	}
	g, ok, err := vertexGatePoints(budget, body, proofbound.AbsSumUpper(pp.SectionDelta, endpointAllow))
	if err != nil || !ok {
		return gatePoints{}, false, err
	}
	view := pp.Prism()
	stations, read, err := prismGatePoints(budget, []prismPayload{view}, proofbound.AbsSumUpper(pp.SectionDelta, view.AxialDelta()))
	if err != nil {
		return gatePoints{}, false, err
	}
	if read {
		g.Join(stations)
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
	return g.Diameter(budget)
}

// capBlendGatePoints lists points a cap-loop chamfer proves lie on its body.
// It joins four sets:
//
//   - the stations of capBlendWitnessPrisms: every loop's wall between the
//     side levels its bands leave, charged their gaps and the levels'
//     displacement (capBlendPayload.AxialDelta);
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
	g, ok, err := prismGatePoints(budget, capBlendWitnessPrisms(cbp), cbp.AxialDelta())
	if err != nil || !ok {
		return gatePoints{}, false, err
	}
	contour := 0.0
	for _, d := range cbp.BandDelta {
		contour = math.Max(contour, d)
	}
	var rims []prismPayload
	for _, p := range cbp.Patches {
		if !p.Geom.Circular || !p.Geom.WholeTurn {
			continue
		}
		rim := cbp.PrismLike(p.Geom.CapZ, p.Geom.CapZ)
		rim.Profile = profileRecord{Outer: loopRecord{Segments: []curveSegment{circleSeg{
			Center: planePoint{U: p.Geom.CU, V: p.Geom.CV}, Radius: units.Millimeters(p.Geom.CapRadius),
			CCW: true, TStart: 0, TEnd: 1,
		}}}}
		rims = append(rims, rim)
	}
	if len(rims) > 0 {
		caps, ok, err := prismGatePoints(budget, rims, proofbound.AbsSumUpper(cbp.AxialDelta(), contour))
		if err != nil {
			return gatePoints{}, false, err
		}
		if ok {
			g.Join(caps)
		}
	}
	for _, p := range cbp.Patches {
		rim, arcAllow, ok := capArcRim(cbp, p.Geom, contour)
		if !ok {
			continue
		}
		arc, ok, err := prismGatePoints(budget, []prismPayload{rim}, proofbound.AbsSumUpper(cbp.AxialDelta(), arcAllow))
		if err != nil {
			return gatePoints{}, false, err
		}
		if ok {
			g.Join(arc)
		}
	}
	vertices, ok, err := vertexGatePoints(budget, body, 0)
	if err != nil || !ok {
		return gatePoints{}, false, err
	}
	g.Join(vertices)
	return g, true, nil
}

// capArcRim builds the cap contour arc that diameter.CapArcWitness bounds as
// a station prism for the shared prism witness reader.
func capArcRim(cbp capBlendPayload, g capPatchGeom, contour float64) (prismPayload, float64, bool) {
	arc, ok := diameter.CapArcWitness(g, contour)
	if !ok {
		return prismPayload{}, 0, false
	}
	rim := cbp.PrismLike(arc.Level, arc.Level)
	rim.Profile = profileRecord{Outer: loopRecord{Segments: []curveSegment{arcSeg{
		Center: arc.Center, Start: arc.Start, End: arc.End, TStart: 0, TEnd: 1,
	}}}}
	return rim, arc.Allow, true
}

// brepGateDiameter is bodyGateDiameter's arm for a brepPayload
// (docs/general-boolean-design.md §4.5), read over brepGatePoints.
func brepGateDiameter(ctx context.Context, body *Body, bp brepPayload) (float64, bool, error) {
	budget := proofbound.NewWorkBudget(ctx)
	g, ok, err := brepGatePoints(budget, body, bp)
	if err != nil || !ok {
		return 0, false, err
	}
	return g.Diameter(budget)
}

// brepGatePoints lists every body vertex, held within its published
// Position().Bound of the point it denotes, and on every face the stations
// diameter.PrismStationWitnesses places on the face's prism view. A swept face is its
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
		stations, stationAllow, read, err := diameter.PrismStationWitnesses(budget, f.view(bp.xform), work)
		if err != nil {
			return gatePoints{}, false, err
		}
		if read {
			g.Join(gatePoints{Points: stations, Allow: stationAllow})
		}
	}
	g.Allow = proofbound.AbsSumUpper(bp.sectionDelta(), bp.AxialDelta(), g.Allow)
	return g, true, nil
}

// bodyGatePoints lists points the body proves lie on it, for the pair
// diameter (pairGateDiameter). Each payload reads the points its own gate
// diameter arm reads where that arm reads points: a faceted, loft, mitred
// sweep or stitched body's held vertex table with its published delta, a
// coil's held station table with its largest station rounding, an
// chain Extrude body's vertices and stations, a brep's vertices and face
// stations, a draft body's stations on both caps, a cap-loop chamfer's side
// levels, whole cap circles and vertices, an analytic prism's or a revolve's stations, and the stations of
// the witness prisms fallbackGateDiameter reads. Every other body, and any of
// those whose stations cannot be read, reads its vertices with their
// published bounds. ok is false, with no error, when none of those can be
// bounded.
func bodyGatePoints(ctx context.Context, budget *proofbound.WorkBudget, body *Body,
	stations *diameter.StationMemo) (gatePoints, bool, error) {
	switch pl := body.n.Payload.(type) {
	case facetedPayload:
		return gatePoints{Points: pl.verts}, true, nil
	case loftPayload:
		return gatePoints{Points: pl.verts, Allow: pl.delta}, true, nil
	case twistedSweepPayload:
		return gatePoints{Points: pl.held.verts, Allow: pl.delta}, true, nil
	case compositeTwistedSweepPayload:
		return gatePoints{Points: pl.mesh.n.Vertices, Allow: pl.delta}, true, nil
	case mitredSweepPayload:
		return gatePoints{Points: pl.Verts, Allow: pl.Delta}, true, nil
	case coilPayload:
		// Every held station vertex lies within its own station rounding of
		// the point X(v, j) it denotes, a point of the coil's true helix edge
		// (docs/helix-design.md §5.3), so the largest rounding is the gap.
		return gatePoints{Points: pl.Verts, Allow: pl.MaxRound}, true, nil
	case stitchPayload:
		return gatePoints{Points: pl.verts, Allow: pl.delta}, true, nil
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
	case twoSidedDraftPayload:
		negative, negDelta := draftCapPrisms(pl.negative)
		positive, posDelta := draftCapPrisms(pl.positive)
		prisms := append(negative, positive...)
		if g, ok, err := prismGatePoints(budget, prisms, math.Max(negDelta, posDelta)); err != nil || ok {
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
		if pl.SectionDelta == 0 {
			if g, ok, err := prismGatePointsMemo(budget, []prismPayload{pl}, pl.AxialDelta(), stations); err != nil || ok {
				return g, ok, err
			}
			return vertexGatePoints(budget, body, 0)
		}
	}
	if prisms, displacement, ok := gateWitnessPrisms(body.n.Payload); ok {
		if g, ok, err := prismGatePoints(budget, prisms, displacement); err != nil || ok {
			return g, ok, err
		}
	}
	return vertexGatePoints(budget, body, 0)
}

// pairGateDiameter reads the pair's diameter without a call cache
// (pairGateDiameterCached with a nil cache).
func pairGateDiameter(ctx context.Context, a, b *Body) (float64, error) {
	return pairGateDiameterCached(ctx, a, b, nil)
}

// pairGateDiameterCached reads the diameter D of the pair a and b (verification
// design §3): the greatest distance between two points drawn from either
// body. It reads bodyGatePoints for both bodies together, so the reading
// ranges over pairs across the two bodies as well as within each, and
// shrinks the maximum by twice the larger allow. Every point is held within
// that allow of a point of one of the two bodies, so the reading is a lower
// bound on the pair's own diameter. A body with no point set leaves the pair
// with the other body's points alone, and a pair with neither reads zero: an
// understated D only tightens the gate.
//
// Each body's points come through cache.gatePoints, so a body read in an
// earlier pair of the same call is not read again. The joined set g starts
// empty and Join only appends into it, so a stored reading's slice is never
// appended to; keep it that way. A nil cache reads every body fresh.
func pairGateDiameterCached(ctx context.Context, a, b *Body, cache *bodyGeomCache) (float64, error) {
	budget := proofbound.NewWorkBudget(ctx)
	var g gatePoints
	for _, body := range [2]*Body{a, b} {
		points, ok, err := cache.gatePoints(ctx, budget, body)
		if err != nil {
			return 0, err
		}
		if ok {
			g.Join(points)
		}
	}
	d, ok, err := g.Diameter(budget)
	if err != nil || !ok {
		return 0, err
	}
	return d, nil
}
