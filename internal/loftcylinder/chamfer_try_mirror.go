package loftcylinder

import (
	"context"
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/loftmesh"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
	"github.com/lestrrat-3d/r3"
)

// MirrorCapChamfer chamfers a mirrored bore. The mirrored bore retains a private lower-half bore. Chamfer only its
// exterior start cap, retain its middle annulus exactly, and reflect that
// audited half. Both exterior bands then have the same held construction.
func MirrorCapChamfer(ctx context.Context, env topo.Env, b *topo.Body, fp facetedrecord.Payload,
	edges []*topo.Edge, setback, delta float64, asymmetric bool) (*topo.Body, bool, error) {
	record := fp.LoftMirrorCylinderBore
	if record == nil || !fp.ContactAudited || fp.Xform != r3.Identity() {
		return nil, false, nil
	}
	if asymmetric || delta != 0 || setback <= 0 || proofbound.IsNonFinite(setback) {
		return nil, true, fmt.Errorf("%w: mirrored cap chamfer needs one exact equal setback", decaderr.ErrUnsupported)
	}
	if record.Join == nil || record.Join.Blank == nil || record.LowerBore == nil ||
		record.LowerBore.Doc != b.Doc || record.Join.Blank.Doc != b.Doc {
		return nil, true, fmt.Errorf("%w: mirrored cap chamfer lost its authenticated source", decaderr.ErrUnsupported)
	}
	lower, ok := record.LowerBore.Payload.(facetedrecord.Payload)
	if !ok || lower.LoftCylinderBore == nil || !lower.ContactAudited || lower.Xform != r3.Identity() {
		return nil, true, fmt.Errorf("%w: mirrored cap chamfer lost its lower bore", decaderr.ErrUnsupported)
	}
	source, ok := ChamferSourceOf(lower)
	if !ok {
		return nil, true, fmt.Errorf("%w: mirrored cap chamfer lost its lower source", decaderr.ErrUnsupported)
	}
	if err := source.BindBridge(); err != nil {
		return nil, true, err
	}
	start := new(big.Rat).Add(proofarith.FloatRat(source.Z0), proofarith.FloatRat(setback))
	end := new(big.Rat).Sub(proofarith.FloatRat(source.Z1), proofarith.FloatRat(setback))
	if start.Cmp(end) >= 0 || source.Z0+setback == source.Z0 || source.Z1-setback == source.Z1 {
		return nil, true, fmt.Errorf("%w: mirrored cap setback reaches the middle side section", decaderr.ErrUnsupported)
	}
	if err := mirrorChamferSelect(ctx, b, fp, record, edges); err != nil {
		return nil, true, err
	}
	shifts, err := MirrorChamferUpperShifts(record, source)
	if err != nil {
		return nil, true, err
	}
	var half ChamferHalf
	reserve, err := MirrorChamferReflectionReserve(lower, record, setback)
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
		return nil, true, fmt.Errorf("%w: mirrored cap chamfer has no reflection allowance", decaderr.ErrUnsupported)
	}
	lowerBody, err := ChamferBottomCap(ctx, env, lower, source,
		setback, allowance, shifts, &half)
	if err != nil {
		return nil, true, err
	}
	result, err := BuildMirrorCapChamfer(ctx, env, fp, record,
		lowerBody, half, reserve, bridgeReserve, setback)
	return result, true, err
}

// mirrorChamferSelect checks that the selection is exactly the two complete
// outer cap loops of the mirrored bore.
func mirrorChamferSelect(ctx context.Context, b *topo.Body, fp facetedrecord.Payload,
	record *facetedrecord.LoftMirrorCylinderBoreRecord, edges []*topo.Edge) error {
	var caps [2]*topo.Face
	capStart := provenance.Ref{Producer: b.Origin.Producer, Role: provenance.RoleCapStart}
	capEnd := provenance.Ref{Producer: b.Origin.Producer, Role: provenance.RoleCapEnd}
	for _, face := range topobuild.Faces(b) {
		for _, origin := range face.Origins {
			if origin == capStart {
				if caps[0] != nil {
					return fmt.Errorf("%w: mirrored start cap is split", decaderr.ErrUnsupported)
				}
				caps[0] = face
			}
			if origin == capEnd {
				if caps[1] != nil {
					return fmt.Errorf("%w: mirrored end cap is split", decaderr.ErrUnsupported)
				}
				caps[1] = face
			}
		}
	}
	selected := make(map[*topo.Edge]struct{}, len(edges))
	for _, edge := range edges {
		selected[edge] = struct{}{}
	}
	if len(selected) != len(edges) {
		return fmt.Errorf("%w: mirrored cap chamfer repeats an edge", decaderr.ErrUnsupported)
	}
	blank, ok := record.Join.Blank.Payload.(prismrecord.Payload)
	if !ok || !MirrorBlank(blank) || caps[0] == nil || caps[1] == nil {
		return fmt.Errorf("%w: mirrored bore lost its two exterior annuli", decaderr.ErrUnsupported)
	}
	for cap, face := range caps {
		z := blank.Z0
		if cap == 1 {
			z = blank.Z1
		}
		if record.CapGroups[cap] < 0 || record.WallGroups[cap] < 0 ||
			!AuditAnnulus(ctx, fp.Verts, fp.Tris, fp.Src, record.Outer[cap], record.Inner[cap],
				record.CapGroups[cap], z, cap == 0) || len(face.Loops) != 2 ||
			!ChamferCapWalk(face.Loops[0], fp.Verts, record.Outer[cap], cap == 0) {
			return fmt.Errorf("%w: mirrored bore cap %d has no audited outer loop", decaderr.ErrUnsupported, cap)
		}
		for _, ce := range face.Loops[0].CoEdges {
			edge := ce.Edge
			if _, yes := selected[edge]; !yes {
				return fmt.Errorf("%w: mirrored chamfer needs both complete outer cap loops", decaderr.ErrUnsupported)
			}
			delete(selected, edge)
		}
	}
	if len(selected) != 0 {
		return fmt.Errorf("%w: mirrored chamfer selection includes a bore or side edge", decaderr.ErrUnsupported)
	}
	return nil
}
