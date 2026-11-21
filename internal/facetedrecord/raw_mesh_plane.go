package facetedrecord

import (
	"context"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
	"github.com/lestrrat-3d/r3"
)

// BindMeshPlaneSlabs uses the exact coplanarity already established by
// meshplane.Groups. The source and the tag share the first corner. Therefore
// every point of this group is within diameter times the sine of the angle
// between planes of the tag. Trimming that face can only remove points.
func BindMeshPlaneSlabs(ctx context.Context, body *topo.Body) error {
	payload, ok := body.Payload.(Payload)
	if !ok || payload.MeshBound != 0 {
		return nil
	}
	faces := topobuild.Faces(body)
	slabs := make([]*topo.MeshPlaneSlab, len(payload.Groups))
	for i, tri := range payload.Tris {
		if err := ctx.Err(); err != nil {
			return err
		}
		id := payload.Src[i]
		if payload.Groups[id].Retained || !payload.Groups[id].Planar {
			continue
		}
		if slabs[id] == nil {
			slabs[id] = rawTriangleSlab(payload.Verts[tri[0]], payload.Verts[tri[1]], payload.Verts[tri[2]], payload.DPair)
		}
		faces[payload.FaceOf[i]].SetMeshPlaneSlab(slabs[id])
	}
	return nil
}

func rawTriangleSlab(a, b, c r3.Vec, diameter float64) *topo.MeshPlaneSlab {
	frame, err := r3.NewFrame(a, b.Sub(a), c.Sub(a))
	if err != nil {
		return nil
	}
	origin := proof.DyVec(a)
	source := proof.DvCross(proof.DvSub(proof.DyVec(b), origin), proof.DvSub(proof.DyVec(c), origin))
	tag := proof.DvCross(proof.DyVec(frame.U()), proof.DyVec(frame.V()))
	den := proof.DyMul(proof.DvDot(source, source), proof.DvDot(tag, tag))
	if den.IsZero() {
		return nil
	}
	cross := proof.DvCross(source, tag)
	sine := proofbound.RatSqrtUp(new(big.Rat).Quo(proof.DvDot(cross, cross).Rat(), den.Rat()))
	distance := proofbound.ProductUpper(diameter, sine)
	if !(distance >= 0) || proofbound.IsNonFinite(distance) {
		return nil
	}
	return &topo.MeshPlaneSlab{Frame: frame, Distance: distance}
}
