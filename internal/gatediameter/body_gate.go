package gatediameter

import (
	"context"
	"math"

	"github.com/lestrrat-3d/decad/internal/breprecord"
	"github.com/lestrrat-3d/decad/internal/capblend"
	"github.com/lestrrat-3d/decad/internal/chainrecord"
	"github.com/lestrrat-3d/decad/internal/coilshell"
	"github.com/lestrrat-3d/decad/internal/diameter"
	"github.com/lestrrat-3d/decad/internal/draftrecord"
	"github.com/lestrrat-3d/decad/internal/gatewitness"
	"github.com/lestrrat-3d/decad/internal/loftrecord"
	"github.com/lestrrat-3d/decad/internal/patchchain"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/revolveplan"
	"github.com/lestrrat-3d/decad/internal/stitchrecord"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/sweepmitre"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
	"github.com/lestrrat-3d/decad/internal/twistsweep"
)

// This file proves the reference diameter Verify's tolerance gate is
// anchored on, per body and per payload.
//
// pairkernel.BodyGateDiameter is the entry point and states the order it tries: a
// payload that can state its own diameter does, a free-form prism section
// resolves one through diameter.PrismFreeformSection, and fallbackGateDiameter
// is the last resort over the body's own vertices. A payload with no
// provable diameter returns false rather than a guess, which withholds the
// gate instead of anchoring it on a number nothing proved. See
// docs/verification-design.md §3.

// pairkernel.BodyGateDiameter returns the body's own diameter, never a document scale or
// a bounds-box diagonal — and returns it as a value proven to be at or below
// that diameter. Point witnesses publish through diameter.PointsWithBudget,
// and every arm charges each point it reads the gap a proof states between it
// and a point of the body; a chain revolve's circular edge uses its certified
// length interval.
// A Faceted body's cached value covers every held
// payload vertex, including vertices absent from the B-rep boundary loops. The
// analytic carrier model is built through the shared work budget (§7.2), so a
// cancelled Verify observes cancellation during the build instead of waiting for
// the whole model to finish. The carrier build's payload switch is the
// clearance kernel's own — it only covers the payloads whose carrier model is
// an exact restatement of the shipped boundary, because internal/clearance/pairkernel and
// internal/verifyrun/interference.go trust that model for containment and contact proofs, not
// only for a diameter. A miss there is not necessarily a body with no usable
// diameter: a free-form-walled prismrecord.Payload — the one shipped payload whose
// side face the clearance kernel's exact model has no arm for at all — can read
// its diameter through diameter.PrismFreeformSection's own witness set of
// analytic vertices and free-form span endpoints instead, tried first because
// its zero sectionDelta would otherwise read as "no arm" below. That arm
// WITHHOLDS its answer on each of the paths its own doc comment lists, and a
// body it withholds from falls through exactly like any other miss. Every miss
// reaches fallbackGateDiameter, which covers the payloads the exact model does
// not (cup, and a prismrecord.Payload whose own sectionDelta is nonzero) with a bound
// that is sound for THIS gate without being eligible for that stronger trust,
// and answers for nothing else — so a free-form-walled prismrecord.Payload whose own
// arm declined ends with no gate diameter at all.
//
// The exact model decides only which arm reads the body. Its carrier
// witnesses (CFace.Wit) are float samples whose gaps (clearance.Witness) are
// proven to the carriers, not to the body, and a placed body's carrier
// maximum reads above its own diameter, so no reading here takes them. A prism reads station witnesses along its walls
// (diameter.PrismGateDiameter), each held within a proven gap of a point of the
// body, and shrinks their maximum by twice the widest gap plus its
// AxialDelta. fallbackGateDiameter applies the same reading to the prisms it
// reads, over each one's own displacement. Every revolve, solid or sheet and
// whatever its section displacement, sweeps its meridian stations to the
// angles its farthest pair sits at (revolveGateDiameter) and compares each
// held point exactly against the point it denotes, the sweep angle's own
// displacement included. A cap-loop chamfer reads capBlendGateDiameter.
//
// A loftrecord.Payload reads its OWN held vertex-set diameter (diameter.PointsContext),
// never an envelope: the boundary is a polyhedron, and a convex-hull diameter
// is realized at vertices, so the vertex set's own maximum IS AT OR BELOW the
// body's true diameter — the strongest arm in this function, ahead of the
// exact carrier model that does not yet cover this payload class. For an
// unplaced LineSeg-only loft the claim is the stronger one, IS the true
// diameter, because every held vertex is then exact (docs/loft-design.md
// §5) — which is a claim about that payload's own published delta, never
// about a station's KIND, since a station can be a recorded coordinate and
// still sit off the point the record denotes (docs/loft-design.md §5.2's
// arc-end radial residual); a same-kind circular pairing's own interior stations are held on the
// true recorded curve but are themselves COMPUTED (a10-plan.md Part 3 PR 6),
// so the held maximum is still a chorded (equal-or-fewer, never additional)
// vertex set's own diameter over a set that sits ON the true boundary — at
// or below it, whether or not the body is placed. That weaker claim is what
// this arm always publishes, and it still fails safe: understating a
// diameter can only turn a passing reading into a false Suspect, never a
// false Sound.
//
// This arm's zero-subtraction fast path reads payload.Delta == 0 exactly
// (internal/loftrecord's exact identity-transform-and-no-computed-station
// comparison, never a tolerance) and then reports the shared reader's
// answer UNCHANGED: no subtraction and no rounding of its own. What that
// answer is, is the reader's to state — the largest float64 at or below the
// held diameter, since the reader publishes every witness maximum rounded
// toward zero (diameter.PointsWithBudget) — so this arm publishes the
// tightest lower bound a float64 can carry on a diameter that is itself
// already at or below the true one. Subtracting a zero allowance on top of
// it would move that reading in exchange for nothing, which is why
// capblend.Payload.extentBoundedAlong's own `outward` helper (capblend.go)
// keeps a zero-displacement candidate untouched too. delta == 0 no longer
// implies the body is unplaced (a10-plan.md Part 3 PR 6): a curved pair
// chorded at ONE station (m = 1, docs/loft-design.md §12's m = 1 case) has
// no interior computed station either, so an UNPLACED body with such a pair
// can reach this same fast path with delta == 0 and an unshrunk reference.
// What makes that sound is the published ZERO and nothing else: at delta == 0
// every held vertex sits exactly at the point the record denotes for it, so
// the vertex set lies ON the true boundary and its maximum cannot exceed the
// true diameter. Being a RECORDED coordinate is not that premise and never
// stands in for it — an untrimmed ArcSeg's t == 1 end is recorded verbatim
// and still sits its own arc-end radial residual off the denoted curve,
// outward as easily as inward (docs/loft-design.md §5.2). Such a build
// publishes a positive delta and takes the shrink below, which is exactly how
// this arm sees the difference.
//
// A PLACED loft's held vertices are no
// longer provably exact (§12 PR 2a): the true diameter can differ from the
// held one by up to 2*delta (each of the two farthest points can sit up to
// delta from its true position), so this arm shrinks the held reading by
// 2*delta before reporting it, understating rather than overstating —
// tightening the gate can only turn a passing reading into a false Suspect,
// never a false Sound, the identical reasoning fallbackGateDiameter already
// carries. That direction is the SUBTRACTION's to lose: 2*delta is exact (a
// power-of-two scaling), so the difference is the one rounding here, and
// round-to-nearest can land it ABOVE the exact d - 2*delta — a reference
// larger than the one proven, which loosens the very gate this arm exists to
// tighten. freeform.DownRound (internal/freeform/spline_length.go, proofbound.UpRound's mirror) steps it back
// toward zero, so the published reference is at or below the exact shrunken
// value for every input rather than only for the ones whose subtraction
// happens to round down. A shrink that collapses to non-positive leaves the
// body with no usable diameter, exactly like any other unusable magnitude
// here.
// DirectBodyGateDiameter selects non-faceted payload arms before the carrier build.
func DirectBodyGateDiameter(ctx context.Context, body *topo.Body) (float64, bool, bool, error) {
	if payload, ok := body.Payload.(loftrecord.Payload); ok {
		d, ok, err := diameter.PointsContext(ctx, payload.Verts)
		if err != nil || !ok {
			return handledGate(d, ok, err)
		}
		if payload.Delta == 0 {
			return handledGate(d, true, nil)
		}
		d, ok = diameter.LowerForDisplacement(d, payload.Delta)
		return handledGate(d, ok, nil)
	}
	if payload, ok := body.Payload.(twistsweep.Payload); ok {
		points := payload.Held.Verts
		if payload.Mesh != nil {
			// The stationed build sews several Lofts into this authenticated
			// mesh. Its held Loft field is empty; delta bounds these vertices.
			points = payload.Mesh.Vertices
		}
		d, ok, err := diameter.PointsContext(ctx, points)
		if err != nil || !ok {
			return handledGate(d, ok, err)
		}
		d, ok = diameter.LowerForDisplacement(d, payload.Delta)
		return handledGate(d, ok, nil)
	}
	if payload, ok := body.Payload.(twistsweep.CompositePayload); ok {
		d, ok, err := diameter.PointsContext(ctx, payload.Mesh.Vertices)
		if err != nil || !ok {
			return handledGate(d, ok, err)
		}
		d, ok = diameter.LowerForDisplacement(d, payload.Delta)
		return handledGate(d, ok, nil)
	}
	if payload, ok := body.Payload.(sweepmitre.Payload); ok {
		// A mitred sweep's boundary is a polyhedron over its held vertex
		// table, each vertex within delta of the exact one, so the loft arm's
		// reading and shrink apply unchanged (docs/sweep-design.md §16.6).
		d, ok, err := diameter.PointsContext(ctx, payload.Verts)
		if err != nil || !ok {
			return handledGate(d, ok, err)
		}
		d, ok = diameter.LowerForDisplacement(d, payload.Delta)
		return handledGate(d, ok, nil)
	}
	if payload, ok := body.Payload.(coilshell.Payload); ok {
		// A coil's held shell is a polyhedron over its vertex table, each
		// vertex within delta of a true point, so the mitred arm's reading
		// and shrink apply unchanged (docs/helix-design.md §7).
		d, ok, err := diameter.PointsContext(ctx, payload.Verts)
		if err != nil || !ok {
			return handledGate(d, ok, err)
		}
		d, ok = diameter.LowerForDisplacement(d, payload.Delta)
		return handledGate(d, ok, nil)
	}
	if payload, ok := body.Payload.(stitchrecord.Payload); ok {
		// Modelled on the loft arm immediately above: a stitched body's
		// boundary is a polyhedron over its own shared vertex table exactly
		// as a loft's is, so the same held-vertex-set diameter is a
		// certified LOWER bound on the body's true diameter, tightened by
		// the placement's own proven displacement.
		d, ok, err := diameter.PointsContext(ctx, payload.Verts)
		if err != nil || !ok {
			return handledGate(d, ok, err)
		}
		if payload.Delta == 0 {
			return handledGate(d, true, nil)
		}
		d, ok = diameter.LowerForDisplacement(d, payload.Delta)
		return handledGate(d, ok, nil)
	}
	if payload, ok := body.Payload.(chainrecord.Payload); ok {
		budget := proofbound.NewWorkBudget(ctx)
		g, ok, err := chainGatePoints(ctx, budget, body, payload)
		if err != nil || !ok {
			return handledGate(0, false, err)
		}
		d, ok, err := g.Diameter(budget)
		return handledGate(d, ok && d > 0, err)
	}
	if payload, ok := body.Payload.(draftrecord.Payload); ok {
		return handledGate(draftGateDiameter(ctx, body, payload))
	}
	if payload, ok := body.Payload.(draftrecord.TwoSidedPayload); ok {
		negative, okNeg, err := draftGateDiameter(ctx, body, payload.Negative)
		if err != nil {
			return handledGate(0, false, err)
		}
		positive, okPos, err := draftGateDiameter(ctx, body, payload.Positive)
		if err != nil {
			return handledGate(0, false, err)
		}
		return handledGate(math.Max(negative, positive), okNeg || okPos, nil)
	}
	if _, ok := body.Payload.(loftrecord.ChainPayload); ok {
		return handledGate(chainVertexGateDiameter(ctx, body, 0))
	}
	if payload, ok := body.Payload.(revolveplan.ChainPayload); ok {
		return handledGate(chainRevolveEdgeGateDiameter(ctx, body, payload.SectionDelta))
	}
	if payload, ok := body.Payload.(breprecord.Payload); ok {
		return handledGate(brepGateDiameter(ctx, body, payload))
	}
	if _, ok := body.Payload.(patchchain.Payload); ok {
		// A patch has no exact carrier model of its own (it is a single flat
		// face, not a prism or a revolve), and no witness set gatewitness.Prism
		// reads either. Its own box is a proven LOWER bound on its diameter
		// instead: the largest single-axis extent of the body's own box is at
		// or below the true diameter (a diameter realizes as SOME pair's
		// distance, whose spread along at least one axis cannot exceed that
		// axis's own box extent), and LowerForDisplacement shrinks it
		// by the box's own proven Bound so the box's uncertainty can only
		// tighten the gate, never loosen it (docs/surface-design.md §5.1,
		// verification §3).
		box := body.Bounds
		d := math.Max(box.Max.X-box.Min.X, math.Max(box.Max.Y-box.Min.Y, box.Max.Z-box.Min.Z))
		d, ok := diameter.LowerForDisplacement(d, box.Bound.Base())
		return handledGate(d, ok, nil)
	}
	if _, ok := body.Payload.(patchchain.BodyPayload); ok {
		// PatchEdges's own payload reuses the patchchain.Payload arm immediately
		// above verbatim: its Bounds is the rebuilt receiver's own box
		// (docs/surface-design.md §5.2), so the same box-based lower bound
		// applies for the same reason.
		box := body.Bounds
		d := math.Max(box.Max.X-box.Min.X, math.Max(box.Max.Y-box.Min.Y, box.Max.Z-box.Min.Z))
		d, ok := diameter.LowerForDisplacement(d, box.Bound.Base())
		return handledGate(d, ok, nil)
	}
	budget := proofbound.NewWorkBudget(ctx)
	switch payload := body.Payload.(type) {
	case revolveplan.Payload:
		// Every revolve reads its meridian stations, solid or sheet, with or
		// without a section displacement: each point carries its own gap,
		// sectionDelta included, so the exact model's coverage decides nothing.
		return handledGate(revolveGateDiameter(budget, payload))
	case capblend.Payload:
		return handledGate(capBlendGateDiameter(ctx, budget, body, payload))
	}
	return 0, false, false, nil
}

// handledGate separates a supported payload's unusable answer from a payload miss.
func handledGate(d float64, ok bool, err error) (float64, bool, bool, error) {
	return d, ok, true, err
}

// CarrierOrFallbackGateDiameter reads a prism through its proven carrier when
// available, then tries the free-form section and recorded fallback witnesses.
func CarrierOrFallbackGateDiameter(ctx context.Context, budget *proofbound.WorkBudget,
	body *topo.Body, carrierOK bool) (float64, bool, error) {
	if carrierOK {
		if payload, isPrism := body.Payload.(prismrecord.Payload); isPrism {
			return diameter.PrismGateDiameter(budget, []prismrecord.Payload{payload}, payload.AxialDelta())
		}
		return 0, false, nil
	}
	if payload, isPrism := body.Payload.(prismrecord.Payload); isPrism {
		if d, ok, err := diameter.PrismFreeformSection(ctx, payload, payload.AxialDelta); ok || err != nil {
			return d, ok, err
		}
	}
	return FallbackGateDiameter(budget, body)
}

// chainVertexGateDiameter reads only actual boundary vertices. Each builder
// publishes the displacement of its held vertex in topo.Vertex.Bound; the
// supplied extra allowance covers section displacement and any endpoint
// rounding that the topology does not charge. The exact-down point-set reader
// and two-sided shrink keep the result below the denoted body's diameter.
func chainVertexGateDiameter(ctx context.Context, body *topo.Body, extraAllow float64) (float64, bool, error) {
	return diameter.HeldPoints(ctx, func(yield func(diameter.HeldPoint) bool) {
		for _, vertex := range topobuild.Vertices(body) {
			if !yield(diameter.HeldPoint{Value: vertex.Position, Bound: vertex.Bound.Base()}) {
				return
			}
		}
	}, extraAllow)
}

// chainRevolveEdgeGateDiameter reads the lengths of the body's circle and arc
// edges. A full-turn chain can expose only one seam vertex even though its
// circular rim has positive diameter (diameter.CircularArcs).
func chainRevolveEdgeGateDiameter(ctx context.Context, body *topo.Body, sectionDelta float64) (float64, bool, error) {
	return diameter.CircularArcs(ctx, func(yield func(diameter.ArcLength) bool) {
		for _, edge := range topobuild.Edges(body) {
			switch edge.Curve.(type) {
			case surfacegeom.Circle3, surfacegeom.Arc3:
			default:
				continue
			}
			arc := diameter.ArcLength{Unbounded: edge.LengthUnbounded}
			if !edge.LengthUnbounded {
				arc.Value, arc.Bound = edge.Length, edge.LengthBound
			}
			if !yield(arc) {
				return
			}
		}
	}, sectionDelta)
}

// fallbackGateDiameter is pairkernel.BodyGateDiameter's fallback for a payload whose true
// boundary the clearance kernel's exact carrier model does not cover
// (cupPayload, capblend.Payload, and a prismrecord.Payload carrying a section
// displacement — verification design §3's "usable finite, non-negative body
// diameter", never a box diagonal, document scale or zero). A free-form-walled
// prismrecord.Payload misses that same exact carrier too, and pairkernel.BodyGateDiameter tries
// PrismFreeformSection for it ahead of this function — but when that arm
// withholds its diameter the body DOES reach here, and this function has no arm
// for it either: its own sectionDelta is zero, the one value gatewitness.Prism
// reads as "no arm at all" for a prismrecord.Payload. Such a body ends with no
// gate diameter and its bounded readings read Suspect.
//
// Each payload earns a witness prism for its own reason, and gatewitness.Prism's
// doc comment states each arm separately rather than pooling them behind one
// justification: a cap-loop chamfer never cuts past the receiver's own recorded
// walls, a cup's shell can ADD material (an outward shell), and a displaced
// section is no containing shape at all — it is the body's own recorded
// boundary, read within a proven displacement of the one it denotes. So what
// each arm proves has to be checked against the geometry it actually returns,
// not against "the receiver" as a stand-in for all three. For the two envelope
// arms the returned prism is a SHAPE that provably contains the true body, so
// the reduction itself can only overstate the true diameter, never understate
// it; the displacement subtracted below is what turns any of the three into the
// lower bound §3 requires.
//
// What this function reports is a reading of that geometry through
// PrismGateDiameter: the station witnesses PrismStationWitnesses places
// along every wall of every witness prism at both of its levels, each one
// charged with its own proven gap. A station is a point of the body only
// where the witness prism's walls run the full height between its two levels
// on the body. That holds for a cup's outer region, a stacked prism's outer
// runs, a displaced section and a sweep's witness prism (the straight prism
// itself, or the start section at one level). A cap-loop chamfer cuts the
// receiver's walls back at the cap levels, so its witness prisms
// (capblend.WitnessPrisms) stop each chamfered loop at its band's side level;
// pairkernel.BodyGateDiameter reads such a body through capBlendGateDiameter, which
// adds its cap circles and vertices to those stations.
//
// For an all-line section the reading reaches the farthest pair, which is
// realized at vertices. A circular wall that sweeps past 180 degrees holds a
// station opposite its start, so its own diameter 2R is read up to rounding
// and the stations' allowance. Between two different walls the farthest pair
// can fall between stations, but every point of a wall of radius R lies
// within 2R*sin(3.75 degrees) of one. An understated D tightens Ref and can
// turn a passing reading into a false Suspect, never a false Sound
// (verification design §3). The reading is built from the payload's own
// frame/xform, the same map a shipped prism's diameter is read through, so it
// carries none of the pose-dependence verification design §4 excludes an
// axis-aligned box for.
//
// A witness prism whose payload carries a displacement has the same
// held-witness issue as the exact prism path: each witness sits within that
// displacement of the point the payload denotes, so the held maximum can
// overstate the denoted body's diameter by twice it. The fallback shrinks the
// held witness maximum by that amount before it becomes a lower-bound
// reference, which is why gatewitness.Prisms hands back a displacement beside
// the prisms to read.
func FallbackGateDiameter(budget *proofbound.WorkBudget, body *topo.Body) (float64, bool, error) {
	witnesses, displacement, ok := gatewitness.Prisms(body.Payload)
	if !ok {
		return 0, false, nil
	}
	return diameter.PrismGateDiameter(budget, witnesses, displacement)
}

// draftGateDiameter is pairkernel.BodyGateDiameter's arm for a draft body
// (docs/draft-design.md Table DD row DD6). The body has no exact carrier
// model here and no single witness prism: its far section is not the near
// one. It reads the stations SectionStations places on the near section P at
// the near level and on the far section Q, read from the payload's own
// far-section record, at the far level. Both are the boundaries of the
// body's two caps, so every station is a point of the body. A near station
// carries its own gap and the near level's displacement; a far station also
// carries farDelta, the displacement of the recorded far contour from the one
// the sweep and taper denote. Where the stations cannot be read, the arm
// reads the body's vertices with their published bounds instead.
func draftGateDiameter(ctx context.Context, body *topo.Body, dp draftrecord.Payload) (float64, bool, error) {
	prisms, displacement := dp.CapPrisms()
	d, ok, err := diameter.PrismGateDiameter(proofbound.NewWorkBudget(ctx), prisms, displacement)
	if err != nil || ok {
		return d, ok, err
	}
	return chainVertexGateDiameter(ctx, body, 0)
}
