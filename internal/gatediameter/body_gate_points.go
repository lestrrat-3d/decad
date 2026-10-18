package gatediameter

import (
	"context"
	"math"

	"github.com/lestrrat-3d/decad/internal/breprecord"
	"github.com/lestrrat-3d/decad/internal/capband"
	"github.com/lestrrat-3d/decad/internal/capblend"
	"github.com/lestrrat-3d/decad/internal/chainrecord"
	"github.com/lestrrat-3d/decad/internal/coilshell"
	"github.com/lestrrat-3d/decad/internal/diameter"
	"github.com/lestrrat-3d/decad/internal/draftrecord"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/gatewitness"
	"github.com/lestrrat-3d/decad/internal/loftrecord"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/revolveplan"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/stitchrecord"
	"github.com/lestrrat-3d/decad/internal/sweepmitre"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
	"github.com/lestrrat-3d/decad/internal/twistsweep"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// This file lists the points a gate diameter reads (docs/verification-design.md
// §3): station witnesses along a prism's walls and a revolve's meridian, a
// brep's vertices and wall stations. Root joins the point sets to read the
// pair diameter. Every point is held within a gap a proof states
// from a point of the body it belongs to.

// revolveGateDiameter reads a revolveplan.Payload's diameter from the points
// revolveGatePoints proves lie on the body.
func revolveGateDiameter(budget *proofbound.WorkBudget, rp revolveplan.Payload) (float64, bool, error) {
	g, ok, err := revolveGatePoints(budget, rp)
	if err != nil || !ok {
		return 0, false, err
	}
	return g.Diameter(budget)
}

// revolveGatePoints passes the payload's recorded meridian, axis, and pose
// to the witness calculation used by its diameter gate.
func revolveGatePoints(budget *proofbound.WorkBudget, rp revolveplan.Payload) (diameter.GatePoints, bool, error) {
	return diameter.RevolveWitnesses(budget, diameter.RevolveWitnessInput{
		Phi0: rp.Phi0, Phi1: rp.Phi1, Den0: rp.Den.Phi0, Den1: rp.Den.Phi1,
		Loops: append([]sectionrecord.LoopRecord{rp.Profile.Outer}, rp.Profile.Holes...),
		Lift:  rp.Lift(), AxisBound: rp.AxisBound(), Axis: rp.Ax,
		Transform: rp.Xform, SectionDelta: rp.SectionDelta,
	})
}

// vertexGatePoints lists the body's vertices, each held within its published
// topo.Vertex.Bound of the point it denotes, with extraAllow added for any displacement those bounds leave
// out. ok is false, with no error, when a vertex has no finite position or
// usable bound.
func vertexGatePoints(budget *proofbound.WorkBudget, body *topo.Body, extraAllow float64) (diameter.GatePoints, bool, error) {
	vertices := topobuild.Vertices(body)
	points := make([]r3.Vec, len(vertices))
	bounds := make([]float64, len(vertices))
	for i, vertex := range vertices {
		points[i], bounds[i] = vertex.Position, vertex.Bound.Base()
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
func chainGatePoints(ctx context.Context, budget *proofbound.WorkBudget, body *topo.Body, pp chainrecord.Payload) (diameter.GatePoints, bool, error) {
	endpointAllow, ok, err := diameter.ChainWalkEndpointAllow(ctx, pp.Chains)
	if err != nil || !ok {
		return diameter.GatePoints{}, false, err
	}
	g, ok, err := vertexGatePoints(budget, body, proofbound.AbsSumUpper(pp.SectionDelta, endpointAllow))
	if err != nil || !ok {
		return diameter.GatePoints{}, false, err
	}
	view := pp.Prism()
	stations, read, err := diameter.PrismGatePoints(budget, []prismrecord.Payload{view}, proofbound.AbsSumUpper(pp.SectionDelta, view.AxialDelta()))
	if err != nil {
		return diameter.GatePoints{}, false, err
	}
	if read {
		g.Join(stations)
	}
	return g, true, nil
}

// capBlendGateDiameter is bodyGateDiameter's arm for a cap-loop chamfer,
// read over capBlendGatePoints.
func capBlendGateDiameter(ctx context.Context, budget *proofbound.WorkBudget, body *topo.Body, cbp capblend.Payload) (float64, bool, error) {
	g, ok, err := capBlendGatePoints(ctx, budget, body, cbp)
	if err != nil || !ok {
		return 0, false, err
	}
	return g.Diameter(budget)
}

// capBlendGatePoints lists points a cap-loop chamfer proves lie on its body.
// It joins four sets:
//
//   - the stations of capblend.WitnessPrisms: every loop's wall between the
//     side levels its bands leave, charged their gaps and the levels'
//     displacement (capblend.Payload.AxialDelta);
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
func capBlendGatePoints(ctx context.Context, budget *proofbound.WorkBudget, body *topo.Body, cbp capblend.Payload) (diameter.GatePoints, bool, error) {
	if err := ctx.Err(); err != nil {
		return diameter.GatePoints{}, false, err
	}
	g, ok, err := diameter.PrismGatePoints(budget, capblend.WitnessPrisms(cbp), cbp.AxialDelta())
	if err != nil || !ok {
		return diameter.GatePoints{}, false, err
	}
	contour := 0.0
	for _, d := range cbp.BandDelta {
		contour = math.Max(contour, d)
	}
	var rims []prismrecord.Payload
	for _, p := range cbp.Patches {
		if !p.Geom.Circular || !p.Geom.WholeTurn {
			continue
		}
		rim := cbp.PrismLike(p.Geom.CapZ, p.Geom.CapZ)
		rim.Profile = momentinput.Profile{Outer: sectionrecord.LoopRecord{Segments: []sectionrecord.CurveSegment{sectionrecord.CircleSeg{
			Center: sectionrecord.Point2{U: p.Geom.CU, V: p.Geom.CV}, Radius: units.Millimeters(p.Geom.CapRadius),
			CCW: true, TStart: 0, TEnd: 1,
		}}}}
		rims = append(rims, rim)
	}
	if len(rims) > 0 {
		caps, ok, err := diameter.PrismGatePoints(budget, rims, proofbound.AbsSumUpper(cbp.AxialDelta(), contour))
		if err != nil {
			return diameter.GatePoints{}, false, err
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
		arc, ok, err := diameter.PrismGatePoints(budget, []prismrecord.Payload{rim}, proofbound.AbsSumUpper(cbp.AxialDelta(), arcAllow))
		if err != nil {
			return diameter.GatePoints{}, false, err
		}
		if ok {
			g.Join(arc)
		}
	}
	vertices, ok, err := vertexGatePoints(budget, body, 0)
	if err != nil || !ok {
		return diameter.GatePoints{}, false, err
	}
	g.Join(vertices)
	return g, true, nil
}

// capArcRim builds the cap contour arc that CapArcWitness bounds as
// a station prism for the shared prism witness reader.
func capArcRim(cbp capblend.Payload, g capband.Patch, contour float64) (prismrecord.Payload, float64, bool) {
	arc, ok := diameter.CapArcWitness(g, contour)
	if !ok {
		return prismrecord.Payload{}, 0, false
	}
	rim := cbp.PrismLike(arc.Level, arc.Level)
	rim.Profile = momentinput.Profile{Outer: sectionrecord.LoopRecord{Segments: []sectionrecord.CurveSegment{sectionrecord.ArcSeg{
		Center: arc.Center, Start: arc.Start, End: arc.End, TStart: 0, TEnd: 1,
	}}}}
	return rim, arc.Allow, true
}

// brepGateDiameter is bodyGateDiameter's arm for a breprecord.Payload
// (docs/general-boolean-design.md §4.5), read over brepGatePoints.
func brepGateDiameter(ctx context.Context, body *topo.Body, bp breprecord.Payload) (float64, bool, error) {
	budget := proofbound.NewWorkBudget(ctx)
	g, ok, err := brepGatePoints(budget, body, bp)
	if err != nil || !ok {
		return 0, false, err
	}
	return g.Diameter(budget)
}

// brepGatePoints lists every body vertex, held within its published
// topo.Vertex.Bound of the point it denotes, and on every face the stations
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
func brepGatePoints(budget *proofbound.WorkBudget, body *topo.Body, bp breprecord.Payload) (diameter.GatePoints, bool, error) {
	g, ok, err := vertexGatePoints(budget, body, 0)
	if err != nil || !ok {
		return diameter.GatePoints{}, false, err
	}
	work := freeform.NewFreeformWork()
	for _, f := range bp.Faces {
		stations, stationAllow, read, err := diameter.PrismStationWitnesses(budget, f.View(bp.Xform), work)
		if err != nil {
			return diameter.GatePoints{}, false, err
		}
		if read {
			g.Join(diameter.GatePoints{Points: stations, Allow: stationAllow})
		}
	}
	g.Allow = proofbound.AbsSumUpper(bp.SectionDelta(), bp.AxialDelta(), g.Allow)
	return g, true, nil
}

// BodyGatePoints lists non-faceted points the body proves lie on it for the pair
// diameter. Root handles the faceted vertex table first. Each payload reads
// the points its own gate diameter arm reads: a loft, mitred
// sweep or stitched body's held vertex table with its published delta, a
// coil's held station table with its largest station rounding, an
// chain Extrude body's vertices and stations, a brep's vertices and face
// stations, a draft body's stations on both caps, a cap-loop chamfer's side
// levels, whole cap circles and vertices, an analytic prism's or a revolve's
// stations, and the stations of the witness prisms FallbackGateDiameter reads. Every other body, and any of
// those whose stations cannot be read, reads its vertices with their
// published bounds. ok is false, with no error, when none of those can be
// bounded.
func BodyGatePoints(ctx context.Context, budget *proofbound.WorkBudget, body *topo.Body,
	stations *diameter.StationMemo) (diameter.GatePoints, bool, error) {
	switch pl := body.Payload.(type) {
	case loftrecord.Payload:
		return diameter.GatePoints{Points: pl.Verts, Allow: pl.Delta}, true, nil
	case twistsweep.Payload:
		return diameter.GatePoints{Points: pl.Held.Verts, Allow: pl.Delta}, true, nil
	case twistsweep.CompositePayload:
		return diameter.GatePoints{Points: pl.Mesh.Vertices, Allow: pl.Delta}, true, nil
	case sweepmitre.Payload:
		return diameter.GatePoints{Points: pl.Verts, Allow: pl.Delta}, true, nil
	case coilshell.Payload:
		// Every held station vertex lies within its own station rounding of
		// the point X(v, j) it denotes, a point of the coil's true helix edge
		// (docs/helix-design.md §5.3), so the largest rounding is the gap.
		return diameter.GatePoints{Points: pl.Verts, Allow: pl.MaxRound}, true, nil
	case stitchrecord.Payload:
		return diameter.GatePoints{Points: pl.Verts, Allow: pl.Delta}, true, nil
	case chainrecord.Payload:
		return chainGatePoints(ctx, budget, body, pl)
	case breprecord.Payload:
		return brepGatePoints(budget, body, pl)
	case draftrecord.Payload:
		prisms, displacement := pl.CapPrisms()
		if g, ok, err := diameter.PrismGatePoints(budget, prisms, displacement); err != nil || ok {
			return g, ok, err
		}
		return vertexGatePoints(budget, body, 0)
	case draftrecord.TwoSidedPayload:
		negative, negDelta := pl.Negative.CapPrisms()
		positive, posDelta := pl.Positive.CapPrisms()
		prisms := append(negative, positive...)
		if g, ok, err := diameter.PrismGatePoints(budget, prisms, math.Max(negDelta, posDelta)); err != nil || ok {
			return g, ok, err
		}
		return vertexGatePoints(budget, body, 0)
	case revolveplan.Payload:
		if g, ok, err := revolveGatePoints(budget, pl); err != nil || ok {
			return g, ok, err
		}
		return vertexGatePoints(budget, body, 0)
	case capblend.Payload:
		return capBlendGatePoints(ctx, budget, body, pl)
	case prismrecord.Payload:
		if pl.SectionDelta == 0 {
			if g, ok, err := diameter.PrismGatePointsMemo(budget, []prismrecord.Payload{pl}, pl.AxialDelta(), stations); err != nil || ok {
				return g, ok, err
			}
			return vertexGatePoints(budget, body, 0)
		}
	}
	if prisms, displacement, ok := gatewitness.Prisms(body.Payload); ok {
		if g, ok, err := diameter.PrismGatePoints(budget, prisms, displacement); err != nil || ok {
			return g, ok, err
		}
	}
	return vertexGatePoints(budget, body, 0)
}
