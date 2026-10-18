package patchchain

import (
	"fmt"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/tessellation"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
)

// OrientBodyPatchFilledFaces makes each new fill's mesh boundary run opposite
// its source face's mesh boundary. The source face already carries its
// feature's outward sense, including any reflection before PatchEdges. The
// patch's new plane frame can have the opposite local handedness, so its
// triangle winding is settled by the shared mesh edge, not that frame.
func OrientBodyPatchFilledFaces(m *topo.Mesh, b *topo.Body, classOf map[*topo.Vertex]int,
	budget *proofbound.WorkBudget) error {
	faces := topobuild.Faces(b)
	byFace := make(map[*topo.Face][]int, len(faces))
	for i, f := range m.Source {
		byFace[f] = append(byFace[f], i)
	}
	for _, f := range faces {
		if err := budget.Step(); err != nil {
			return err
		}
		if len(f.Origins) != 1 || f.Origins[0].Producer != b.Origin.Producer || f.Origins[0].Role != RolePatch {
			continue
		}
		if len(f.Loops) == 0 || len(f.Loops[0].CoEdges) == 0 {
			return fmt.Errorf(`%w: a PatchEdges fill has no boundary walk to orient`, decaderr.ErrDegenerate)
		}
		ce := f.Loops[0].CoEdges[0]
		start, end := classOf[topobuild.CoEdgeStart(ce)], classOf[topobuild.CoEdgeEnd(ce)]
		var source *topo.Face
		for _, adjacent := range ce.Edge.Faces {
			if adjacent != f {
				source = adjacent
				break
			}
		}
		if source == nil {
			return fmt.Errorf(`%w: a PatchEdges fill has no source face on its rim`, decaderr.ErrDegenerate)
		}
		patchSense := tessellation.PatchEdgeSense(m.Triangles, byFace[f], start, end)
		sourceSense := tessellation.PatchEdgeSense(m.Triangles, byFace[source], start, end)
		if patchSense == 0 || sourceSense == 0 {
			return fmt.Errorf(`%w: a PatchEdges rim has no matching source and fill mesh edges`, decaderr.ErrUnsupported)
		}
		if patchSense != sourceSense {
			continue
		}
		for _, ti := range byFace[f] {
			m.Triangles[ti][1], m.Triangles[ti][2] = m.Triangles[ti][2], m.Triangles[ti][1]
		}
	}
	return nil
}
