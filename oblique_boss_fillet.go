package decad

import (
	"context"
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/facetproof"
	"github.com/lestrrat-3d/decad/internal/loftmesh"
	"github.com/lestrrat-3d/decad/internal/obliqueboss"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/r3"
)

// filletObliqueBoss uses the joined body's edge only to authenticate the
// selector. Its source record supplies every geometric surface.
func (b *Body) filletObliqueBoss(ctx context.Context, edges []*Edge,
	rec *obliqueBossRecord, radius, radiusDelta float64) (*Body, error) {
	if len(edges) != 1 || edges[0].n.Start != edges[0].n.End {
		return nil, fmt.Errorf("%w: an oblique boss fillet needs its one complete junction edge", ErrUnsupported)
	}
	if _, ok := edges[0].n.Curve.(surfacegeom.FacetedCurve); !ok {
		return nil, fmt.Errorf("%w: the selected oblique junction has no faceted edge witness", ErrUnsupported)
	}
	plateTop, topOK := obliqueboss.FaceGroup(rec.Plate, provenance.RoleCapEnd)
	bossSide, sideOK := obliqueboss.FaceGroup(rec.Boss, prismrecord.OuterFirstSideRole)
	if !topOK || !sideOK || len(edges[0].n.Faces) != 2 {
		return nil, fmt.Errorf("%w: the selected edge is not the complete plate-to-cylinder junction", ErrUnsupported)
	}
	matched := (obliqueboss.HasOrigin(edges[0].n.Faces[0], plateTop) &&
		obliqueboss.HasOrigin(edges[0].n.Faces[1], bossSide)) ||
		(obliqueboss.HasOrigin(edges[0].n.Faces[1], plateTop) &&
			obliqueboss.HasOrigin(edges[0].n.Faces[0], bossSide))
	if !matched {
		return nil, fmt.Errorf("%w: the selected edge is not the complete plate-to-cylinder junction", ErrUnsupported)
	}
	if radiusDelta != 0 || !(radius > 0) ||
		proofbound.IsNonFinite(radius) || radius >= rec.Radius/4 {
		return nil, fmt.Errorf("%w: the oblique junction fillet radius exceeds its proven clearance", ErrUnsupported)
	}
	margin := proofbound.DivUpper(proofbound.AbsSumUpper(rec.Radius, radius), 0.93)
	center := rec.Base.Add(rec.Axis.Scale((rec.Top + radius - rec.Base.Z) / rec.Axis.Z))
	if !(center.X-margin-rec.SourceBound > rec.Rect.U0 &&
		center.X+margin+rec.SourceBound < rec.Rect.U1 &&
		center.Y-margin-rec.SourceBound > rec.Rect.V0 &&
		center.Y+margin+rec.SourceBound < rec.Rect.V1 &&
		rec.Base.Z+rec.Length*rec.Axis.Z-rec.Radius*math.Hypot(rec.Axis.X, rec.Axis.Y)-rec.SourceBound >
			rec.Top+radius*(1+math.Hypot(rec.Axis.X, rec.Axis.Y))) {
		return nil, fmt.Errorf("%w: the oblique junction fillet reaches a plate side or the boss far cap", ErrUnsupported)
	}
	mesh, err := obliqueboss.BuildFilletMesh(ctx, rec, radius)
	if err != nil {
		return nil, err
	}
	if err := loftmesh.LoftCrossingAuditBounded(proofbound.NewWorkBudget(ctx),
		mesh.Verts, mesh.Tris, loftmesh.MaxLoftAuditTriangles, obliqueboss.AuditScans); err != nil {
		return nil, err
	}
	bound, volumeGap, areaSlack, err := obliqueboss.ProveFillet(ctx, rec, radius, mesh)
	if err != nil {
		return nil, err
	}
	audit, err := facetproof.AuditFacetedMesh(ctx, mesh.Verts, mesh.Tris)
	if err != nil {
		return nil, err
	}
	volume, heldVolume, err := meshVolumeMeasurement(ctx, audit, volumeGap)
	if err != nil {
		return nil, err
	}
	if heldVolume.Cmp(proofarith.FloatRat(volumeGap)) <= 0 {
		return nil, fmt.Errorf("%w: the oblique fillet has no proven positive occupied volume", ErrUnsupported)
	}
	dPair := obliqueboss.DiameterBound(rec, bound)
	if proofbound.IsNonFinite(dPair) {
		return nil, fmt.Errorf("%w: the oblique boss has no finite diameter proof", ErrUnsupported)
	}
	vertexBound := make([]float64, len(mesh.Verts))
	for i := range vertexBound {
		vertexBound[i] = bound
	}
	ref := b.n.Doc.NextProducer
	mesh.Groups[8].Origins[0].Producer = ref
	result, err := bodyResult(facetedrecord.BuildWithProof(ctx, envOf(documentNode(documentHandle(b.n.Doc)), ref), facetedPayload{
		Verts: mesh.Verts, Tris: mesh.Tris, Src: mesh.Src, Groups: mesh.Groups,
		VertexBound: vertexBound, MeshBound: bound, VolSymDiff: volumeGap,
		AreaSlack: areaSlack, DPair: dPair, ContactAudited: true, Xform: r3.Identity(),
	}, audit, measurementToInternal(volume), heldVolume))
	if err != nil {
		return nil, err
	}
	return commitModifyResult(ctx, b, result)
}
