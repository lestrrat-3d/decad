package decad

import (
	"context"
	"errors"
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/loftmesh"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sweepmitre"
	"github.com/lestrrat-3d/decad/internal/triangulation"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// This file builds docs/sweep-design.md §16's mitred polyline sweep: the
// §16.3 construction over exact rationals, Table SM rows SM5 through SM8, the
// single rounding into the held vertex table, Table BM's topology, and
// §16.6's four readings, all taken over the rational vertices.
//
// Every section is the image of the one before it under its span's wall
// lines, and every wall line of one span passes through that span's apex (or
// is parallel to the span when its ratio is one). So every wall quad is
// exactly planar, and the map from one section to the next is a projective
// map on a convex region holding the whole section (SM6 states each of its
// conditions as a linear inequality over the vertex, so it holds on the hull
// once it holds at every vertex). That map carries a simple polygon with
// holes to a simple polygon with holes and carries a triangulation of one to
// a triangulation of the other, which is why capEnd reuses capStart's
// triangulation index for index rather than triangulating a second time.

// constructMitredSweep triangulates the recorded profile once, then adapts
// its spans to §16.3's exact section construction.
func constructMitredSweep(ctx context.Context, mp mitredSweepPayload) (sweepmitre.Construction, error) {
	pts2, loopIdx, err := mitredSweepLoops(mp.profile)
	if err != nil {
		return sweepmitre.Construction{}, err
	}
	capTris, err := triangulation.Triangulate(ctx, point2ToRecordSlice(pts2), loopIdx)
	if err != nil {
		return sweepmitre.Construction{}, triangulation.WrapLoftError(err)
	}
	spans := make([]sweepmitre.Span, len(mp.path.records))
	for k, record := range mp.path.records {
		spans[k] = sweepmitre.Span{Start: record.Start, End: record.End}
	}
	built, err := sweepmitre.Construct(ctx, mp.plane, point2ToRecordSlice(pts2), loopIdx, spans, mp.factors)
	if err != nil {
		return sweepmitre.Construction{}, err
	}
	built.LoopIdx, built.CapTris = loopIdx, capTris
	return built, nil
}

// evalMitredSweep builds the body for one payload record: §16.3's exact
// construction and placement, the single rounding, the triangle set and
// orientation, SM8's crossing audit, Table BM's topology and §16.6's
// readings. The first build and every placement run it.
func evalMitredSweep(ctx context.Context, d *Document, ref producerID, mp mitredSweepPayload) (*Body, error) {
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	c, err := constructMitredSweep(ctx, mp)
	if err != nil {
		return nil, err
	}

	placed, err := sweepmitre.PlaceAndRound(c, mp.xform, mp.localVol6 == nil && mp.xform != r3.Identity())
	if err != nil {
		return nil, err
	}
	exact, localExact, anchor := placed.Exact, placed.LocalExact, placed.Anchor
	verts, vertexBound, delta := placed.Rounded, placed.VertexBound, placed.Delta

	a := sweepmitre.Assemble(c, roleCapStart, roleCapEnd)
	localVol6, localMoments := mp.localVol6, mp.localMoments
	if localVol6 == nil {
		if localExact == nil {
			localExact = exact
		}
		localVol6, localMoments = mitredVolumeMoments(localExact, a.Tris, c.Anchor)
	}
	vol6, moments := sweepmitre.PlacedVolumeMoments(localVol6, localMoments, mp.xform)
	switch vol6.Sign() {
	case 0:
		return nil, fmt.Errorf(`%w: the mitred sweep encloses no volume`, ErrDegenerate)
	case -1:
		// §16.3's orientation rule: one exact sign for the whole shell.
		a.Reversed = true
		for t, tri := range a.Tris {
			a.Tris[t] = [3]int{tri[0], tri[2], tri[1]}
		}
		vol6.Neg(vol6)
		for axis := range moments {
			moments[axis].Neg(moments[axis])
		}
	}
	for t, tri := range a.Tris {
		if loftmesh.TriangleCollapsed(verts, tri) {
			return nil, fmt.Errorf(`%w: rounding the mitred sweep's vertices collapsed its triangle %d (%s)`, ErrUnsupported, t, a.Roles[a.TriFace[t]])
		}
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	if err := loftmesh.LoftCrossingAudit(proofbound.NewWorkBudget(ctx), verts, a.Tris); err != nil {
		var contact *loftmesh.LoftContactError
		if errors.As(err, &contact) {
			return nil, fmt.Errorf(`%w: mitred sweep faces %s and %s %s`, ErrDegenerate,
				a.Roles[a.TriFace[contact.I]], a.Roles[a.TriFace[contact.J]], contact.Reason)
		}
		return nil, err
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}

	areas, err := mitredTriangleAreas(ctx, exact, a.Tris)
	if err != nil {
		return nil, err
	}
	body := &Body{doc: d, origin: FeatureRef{producer: ref, Role: roleBody}, solid: true, kind: BodySolid}
	faces, err := buildMitredTopology(ctx, body, ref, a, exact, verts, areas, delta)
	if err != nil {
		return nil, err
	}
	if err := attachFaceLoopsContext(ctx, faces); err != nil {
		return nil, err
	}
	body.lumps = []*Lump{{shells: []*Shell{{faces: faces}}}}

	body.volume = mitredVolume(vol6)
	if body.centroid, err = mitredCentroid(anchor, vol6, moments); err != nil {
		return nil, err
	}
	body.bounds = mitredBounds(exact)
	areaLo, areaHi := new(big.Rat), new(big.Rat)
	for _, enclosure := range areas {
		areaLo.Add(areaLo, enclosure[0])
		areaHi.Add(areaHi, enclosure[1])
	}
	value, bound := mitredEnclosure(areaLo, areaHi)
	body.area = Measurement{
		Value:     units.SquareMillimeters(value),
		Exactness: exactnessOf(bound),
		Bound:     units.SquareMillimeters(bound),
	}
	if err := validateLoftBodyMeasurements(body); err != nil {
		return nil, err
	}
	if !finiteMeasurementValues(bound) {
		return nil, fmt.Errorf(`%w: the mitred sweep's area has no finite bound`, ErrUnsupported)
	}

	mp.exact, mp.verts, mp.tris, mp.triFace, mp.faceRoles, mp.delta = exact, verts, a.Tris, a.TriFace, a.Roles, delta
	mp.vertexBound = vertexBound
	mp.localVol6, mp.localMoments = localVol6, localMoments
	body.payload = mp
	return body, nil
}
