package decad

import (
	"context"
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/loftcylinder"
	"github.com/lestrrat-3d/decad/internal/loftmesh"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
)

// The mirrored bore retains a private lower-half bore. Chamfer only its
// exterior start cap, retain its middle annulus exactly, and reflect that
// audited half. Both exterior bands then have the same held construction.
func tryLoftMirrorCylinderCapChamfer(ctx context.Context, b *Body, fp facetedPayload,
	edges []*Edge, setback, delta float64, asym *asymmetricChamfer) (*Body, bool, error) {
	record := fp.LoftMirrorCylinderBore
	if record == nil || !fp.ContactAudited || fp.Xform != r3.Identity() {
		return nil, false, nil
	}
	if asym != nil || delta != 0 || setback <= 0 || proofbound.IsNonFinite(setback) {
		return nil, true, fmt.Errorf("%w: mirrored cap chamfer needs one exact equal setback", ErrUnsupported)
	}
	if record.Join == nil || record.Join.Blank == nil || record.LowerBore == nil ||
		record.LowerBore.Doc != b.n.Doc || record.Join.Blank.Doc != b.n.Doc {
		return nil, true, fmt.Errorf("%w: mirrored cap chamfer lost its authenticated source", ErrUnsupported)
	}
	lower, ok := record.LowerBore.Payload.(facetedPayload)
	if !ok || lower.LoftCylinderBore == nil || !lower.ContactAudited || lower.Xform != r3.Identity() {
		return nil, true, fmt.Errorf("%w: mirrored cap chamfer lost its lower bore", ErrUnsupported)
	}
	source, ok := loftcylinder.ChamferSourceOf(lower)
	if !ok {
		return nil, true, fmt.Errorf("%w: mirrored cap chamfer lost its lower source", ErrUnsupported)
	}
	if err := source.BindBridge(); err != nil {
		return nil, true, err
	}
	start := new(big.Rat).Add(proofarith.FloatRat(source.Z0), proofarith.FloatRat(setback))
	end := new(big.Rat).Sub(proofarith.FloatRat(source.Z1), proofarith.FloatRat(setback))
	if start.Cmp(end) >= 0 || source.Z0+setback == source.Z0 || source.Z1-setback == source.Z1 {
		return nil, true, fmt.Errorf("%w: mirrored cap setback reaches the middle side section", ErrUnsupported)
	}
	if err := loftMirrorCylinderChamferSelect(ctx, b, fp, record, edges); err != nil {
		return nil, true, err
	}
	shifts, err := loftcylinder.MirrorChamferUpperShifts(record, source)
	if err != nil {
		return nil, true, err
	}
	var half loftcylinder.ChamferHalf
	ref := b.n.Doc.NextProducer
	reserve, err := loftcylinder.MirrorChamferReflectionReserve(lower, record, setback)
	if err != nil {
		return nil, true, err
	}
	bridgeReserve, err := loftmesh.MirrorBridgeReserve(lower.Verts, lower.MeshBound,
		record.Join.BridgeBound, setback)
	if err != nil {
		return nil, true, err
	}
	totalReserve := proofbound.AbsSumUpper(reserve, bridgeReserve)
	allowance := math.Nextafter(0.1-totalReserve, math.Inf(-1))
	for i := 0; i < 4 && proofbound.AbsSumUpper(allowance, totalReserve) > 0.1; i++ {
		allowance = math.Nextafter(allowance, math.Inf(-1))
	}
	if allowance <= 0 || proofbound.AbsSumUpper(allowance, totalReserve) > 0.1 {
		return nil, true, fmt.Errorf("%w: mirrored cap chamfer has no reflection allowance", ErrUnsupported)
	}
	lowerBody, err := bodyResult(loftcylinder.ChamferBottomCap(ctx, envOf(b.n.Doc, ref), lower, source,
		setback, allowance, shifts, &half))
	if err != nil {
		return nil, true, err
	}
	result, err := buildLoftMirrorCylinderCapChamfer(ctx, documentHandle(b.n.Doc), ref, fp, record,
		lowerBody, half, reserve, bridgeReserve, setback)
	return result, true, err
}

func loftMirrorCylinderChamferSelect(ctx context.Context, b *Body, fp facetedPayload,
	record *loftMirrorCylinderBoreRecord, edges []*Edge) error {
	var caps [2]*Face
	for _, face := range b.Faces() {
		for _, origin := range face.n.Origins {
			if origin == refOf(CapStart(b)) {
				if caps[0] != nil {
					return fmt.Errorf("%w: mirrored start cap is split", ErrUnsupported)
				}
				caps[0] = face
			}
			if origin == refOf(CapEnd(b)) {
				if caps[1] != nil {
					return fmt.Errorf("%w: mirrored end cap is split", ErrUnsupported)
				}
				caps[1] = face
			}
		}
	}
	selected := make(map[*topo.Edge]struct{}, len(edges))
	for _, edge := range edges {
		selected[edgeNode(edge)] = struct{}{}
	}
	if len(selected) != len(edges) {
		return fmt.Errorf("%w: mirrored cap chamfer repeats an edge", ErrUnsupported)
	}
	blank, ok := record.Join.Blank.Payload.(prismPayload)
	if !ok || !loftcylinder.MirrorBlank(blank) || caps[0] == nil || caps[1] == nil {
		return fmt.Errorf("%w: mirrored bore lost its two exterior annuli", ErrUnsupported)
	}
	for cap, face := range caps {
		z := blank.Z0
		if cap == 1 {
			z = blank.Z1
		}
		if record.CapGroups[cap] < 0 || record.WallGroups[cap] < 0 ||
			!loftcylinder.AuditAnnulus(ctx, fp.Verts, fp.Tris, fp.Src, record.Outer[cap], record.Inner[cap],
				record.CapGroups[cap], z, cap == 0) || len(face.n.Loops) != 2 ||
			!loftcylinder.ChamferCapWalk(face.n.Loops[0], fp.Verts, record.Outer[cap], cap == 0) {
			return fmt.Errorf("%w: mirrored bore cap %d has no audited outer loop", ErrUnsupported, cap)
		}
		for _, ce := range face.n.Loops[0].CoEdges {
			edge := ce.Edge
			if _, yes := selected[edge]; !yes {
				return fmt.Errorf("%w: mirrored chamfer needs both complete outer cap loops", ErrUnsupported)
			}
			delete(selected, edge)
		}
	}
	if len(selected) != 0 {
		return fmt.Errorf("%w: mirrored chamfer selection includes a bore or side edge", ErrUnsupported)
	}
	return nil
}

// buildLoftMirrorCylinderCapChamfer reflects the chamfered lower half through
// placePayload, has internal/loftcylinder assemble and prove the mirrored
// shell, then gives its exterior caps the bored Join's cap origins and the
// blank's planar cap tags, which read the blank's root Face handles.
func buildLoftMirrorCylinderCapChamfer(ctx context.Context, d *Document, ref producerID,
	old facetedPayload, record *loftMirrorCylinderBoreRecord, lowerBody *Body,
	half loftcylinder.ChamferHalf, reflectionReserve, bridgeReserve, setback float64) (*Body, error) {
	lower, ok := lowerBody.n.Payload.(facetedPayload)
	if !ok {
		return nil, fmt.Errorf("%w: mirrored cap chamfer lost its audited lower half", ErrUnsupported)
	}
	reflection, _, err := loftcylinder.MirrorChamferFrame(record)
	if err != nil {
		return nil, err
	}
	upperBody, err := placePayload(ctx, lower, d, ref, reflection)
	if err != nil {
		return nil, err
	}
	upper, ok := upperBody.n.Payload.(facetedPayload)
	if !ok {
		return nil, fmt.Errorf("%w: mirrored cap chamfer halves have no matching held shell", ErrUnsupported)
	}
	assembled, err := loftcylinder.AssembleMirrorChamfer(ctx, old, lower, upper, record, half,
		reflectionReserve, bridgeReserve, setback)
	if err != nil {
		return nil, err
	}
	result := assembled.Payload
	for cap, group := range assembled.CapGroups {
		result.Groups[group].Origins = append([]provenance.Ref(nil), old.Groups[record.CapGroups[cap]].Origins...)
	}
	if err := loftCylinderTagCaps(&result, bodyHandle(record.Join.Blank), assembled.CapGroups,
		assembled.CapLevels, ref); err != nil {
		return nil, err
	}
	return bodyResult(facetedrecord.Build(ctx, envOf(documentNode(d), ref), result))
}
