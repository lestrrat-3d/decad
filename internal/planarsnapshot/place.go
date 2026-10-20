package planarsnapshot

import (
	"slices"

	"github.com/lestrrat-3d/decad/internal/pair/planar"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// Place maps a cached snapshot's vertices through pose into a
// fresh vertex slice and audits the result: the snapshot's topology already
// passed the combinatorial half of planar.CheckPlanarSolid, and
// planar.CheckPlanarPose runs the coordinate half and attaches the pose's
// derived data. The triangle and face slices are shared with the cache,
// clipped so no append can reach its storage.
func Place(budget *proofbound.WorkBudget, snapshot *SnapshotEntry,
	pose r3.Transform) (planar.PlanarSolid, bool, error) {
	solid := planar.PlanarSolid{Verts: make([]proofarith.DyV3, len(snapshot.Solid.Verts)),
		Tris: slices.Clip(snapshot.Solid.Tris), Faces: slices.Clip(snapshot.Solid.Faces)}
	place := proofarith.NewDyadicMap(pose)
	for i, v := range snapshot.Solid.Verts {
		if err := budget.Step(); err != nil {
			return planar.PlanarSolid{}, false, err
		}
		solid.Verts[i] = place.Apply(v)
	}
	audited, err := planar.CheckPlanarPose(&solid, snapshot.Topology, budget.Step)
	if err != nil || !audited {
		return planar.PlanarSolid{}, false, err
	}
	return solid, true, nil
}
