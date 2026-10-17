package decad

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/crossbore"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/provenance"
)

func crossBoreFaceGroup(body *Body, role string) (facetGroup, bool) {
	for _, face := range body.Faces() {
		for _, origin := range face.originRefs() {
			if origin.Producer == body.n.Origin.Producer && origin.Role == role {
				return facetGroup{Origins: face.originRefs(), Planar: face.isPlanar()}, true
			}
		}
	}
	return facetGroup{}, false
}

func crossBoreMeshGroups(rec *crossCylinderBoreRecord) ([]facetGroup, error) {
	groups := make([]facetGroup, 0, 5)
	for _, source := range []struct {
		body *Body
		role string
	}{
		{bodyHandle(rec.Target), provenance.RoleCapStart},
		{bodyHandle(rec.Target), provenance.RoleCapEnd},
		{bodyHandle(rec.Target), prismrecord.OuterFirstSideRole},
		{bodyHandle(rec.Tool), prismrecord.OuterFirstSideRole},
	} {
		group, ok := crossBoreFaceGroup(source.body, source.role)
		if !ok {
			return nil, fmt.Errorf("%w: a cross-bore source lost face %q", ErrUnsupported, source.role)
		}
		groups = append(groups, group)
	}
	groups = append(groups, facetGroup{Origins: []provenance.Ref{{Role: "boreMouthChamfer"}}})
	return groups, nil
}

func buildCrossBoreChamferMesh(ctx context.Context,
	rec *crossCylinderBoreRecord, setback float64) (*crossbore.Mesh, error) {
	return buildCrossBoreChamferMeshWorkers(ctx, rec, setback, -1)
}

// buildCrossBoreChamferMeshWorkers is buildCrossBoreChamferMesh with the
// station-error fan-out width given; a negative workers picks
// orderedwork.EachWorkers for the station count. The mesh does not depend on
// workers.
func buildCrossBoreChamferMeshWorkers(ctx context.Context,
	rec *crossCylinderBoreRecord, setback float64, workers int) (*crossbore.Mesh, error) {
	groups, err := crossBoreMeshGroups(rec)
	if err != nil {
		return nil, err
	}
	return crossbore.BuildMesh(ctx, rec, setback, groups, workers)
}
