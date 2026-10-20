package obliqueboss

import (
	"context"
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/booleaneval"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/facetproof"
	"github.com/lestrrat-3d/decad/internal/loftmesh"
	"github.com/lestrrat-3d/decad/internal/measurement"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// FilletBoss uses the joined body's edge only to authenticate the
// selector. Its source record supplies every geometric surface.
func FilletBoss(ctx context.Context, env topo.Env, edges []*topo.Edge,
	rec *facetedrecord.ObliqueBossRecord, radius, radiusDelta float64) (*topo.Body, error) {
	if len(edges) != 1 || edges[0].Start != edges[0].End {
		return nil, fmt.Errorf("%w: an oblique boss fillet needs its one complete junction edge", decaderr.ErrUnsupported)
	}
	if _, ok := edges[0].Curve.(surfacegeom.FacetedCurve); !ok {
		return nil, fmt.Errorf("%w: the selected oblique junction has no faceted edge witness", decaderr.ErrUnsupported)
	}
	plateTop, topOK := FaceGroup(rec.Plate, provenance.RoleCapEnd)
	bossSide, sideOK := FaceGroup(rec.Boss, prismrecord.OuterFirstSideRole)
	if !topOK || !sideOK || len(edges[0].Faces) != 2 {
		return nil, fmt.Errorf("%w: the selected edge is not the complete plate-to-cylinder junction", decaderr.ErrUnsupported)
	}
	matched := (HasOrigin(edges[0].Faces[0], plateTop) &&
		HasOrigin(edges[0].Faces[1], bossSide)) ||
		(HasOrigin(edges[0].Faces[1], plateTop) &&
			HasOrigin(edges[0].Faces[0], bossSide))
	if !matched {
		return nil, fmt.Errorf("%w: the selected edge is not the complete plate-to-cylinder junction", decaderr.ErrUnsupported)
	}
	if radiusDelta != 0 || !(radius > 0) ||
		proofbound.IsNonFinite(radius) || radius >= rec.Radius/4 {
		return nil, fmt.Errorf("%w: the oblique junction fillet radius exceeds its proven clearance", decaderr.ErrUnsupported)
	}
	margin := proofbound.DivUpper(proofbound.AbsSumUpper(rec.Radius, radius), 0.93)
	center := rec.Base.Add(rec.Axis.Scale((rec.Top + radius - rec.Base.Z) / rec.Axis.Z))
	if !(center.X-margin-rec.SourceBound > rec.Rect.U0 &&
		center.X+margin+rec.SourceBound < rec.Rect.U1 &&
		center.Y-margin-rec.SourceBound > rec.Rect.V0 &&
		center.Y+margin+rec.SourceBound < rec.Rect.V1 &&
		rec.Base.Z+rec.Length*rec.Axis.Z-rec.Radius*math.Hypot(rec.Axis.X, rec.Axis.Y)-rec.SourceBound >
			rec.Top+radius*(1+math.Hypot(rec.Axis.X, rec.Axis.Y))) {
		return nil, fmt.Errorf("%w: the oblique junction fillet reaches a plate side or the boss far cap", decaderr.ErrUnsupported)
	}
	mesh, err := BuildFilletMesh(ctx, rec, radius)
	if err != nil {
		return nil, err
	}
	if err := loftmesh.LoftCrossingAuditBounded(proofbound.NewWorkBudget(ctx),
		mesh.Verts, mesh.Tris, loftmesh.MaxLoftAuditTriangles, AuditScans); err != nil {
		return nil, err
	}
	bound, volumeGap, areaSlack, err := ProveFillet(ctx, rec, radius, mesh)
	if err != nil {
		return nil, err
	}
	audit, err := facetproof.AuditFacetedMesh(ctx, mesh.Verts, mesh.Tris)
	if err != nil {
		return nil, err
	}
	volume, volumeBound, heldVolume, err := booleaneval.VolumeReading(ctx, audit, volumeGap)
	if err != nil {
		return nil, err
	}
	if heldVolume.Cmp(proofarith.FloatRat(volumeGap)) <= 0 {
		return nil, fmt.Errorf("%w: the oblique fillet has no proven positive occupied volume", decaderr.ErrUnsupported)
	}
	dPair := DiameterBound(rec, bound)
	if proofbound.IsNonFinite(dPair) {
		return nil, fmt.Errorf("%w: the oblique boss has no finite diameter proof", decaderr.ErrUnsupported)
	}
	vertexBound := make([]float64, len(mesh.Verts))
	for i := range vertexBound {
		vertexBound[i] = bound
	}
	mesh.Groups[8].Origins[0].Producer = env.Ref
	return facetedrecord.BuildWithProof(ctx, env, facetedrecord.Payload{
		Verts: mesh.Verts, Tris: mesh.Tris, Src: mesh.Src, Groups: mesh.Groups,
		VertexBound: vertexBound, MeshBound: bound, VolSymDiff: volumeGap,
		AreaSlack: areaSlack, DPair: dPair, ContactAudited: true, Xform: r3.Identity(),
	}, audit, measurement.Measurement{Value: units.CubicMillimeters(volume),
		Exactness: measurement.ExactnessOf(volumeBound), Bound: units.CubicMillimeters(volumeBound)}, heldVolume)
}
