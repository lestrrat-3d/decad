package export

import (
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
)

// meshSolid names, by connected facet set, the shells of one lump: the outer
// shell and its cavities (agent-docs/step-export-design.md §9).
type meshSolid struct {
	outer int
	voids []int
}

// meshSolids groups the connected facet sets of a verified mesh by the body's
// lumps. sources is Mesh.SourceFaces, parallel to the triangles, and
// components is checkMesh's facet set per triangle. The body decides which
// shell is outer (FaceShell.IsVoid) and which lump a shell belongs to; the mesh
// only locates each shell's facets. The result is reject-only: a facet set that
// mixes shells, a shell without facets, a lump without exactly one outer
// shell, an outer facet set whose signed volume is not positive, or a void set
// whose signed volume is not negative is ErrUnsupported. The signed volume
// never selects a shell, it only refuses a mesh wound against its shell's kind.
func meshSolids(
	body *decad.Body, sources []*decad.Face, vertices []r3.Vec, triangles [][3]int, components []int, count int,
) ([]meshSolid, error) {
	if len(sources) != len(components) || len(triangles) != len(components) {
		return nil, fmt.Errorf("export: STEP: %w: the mesh does not map its facets to faces", decad.ErrUnsupported)
	}
	type position struct{ lump, shell int }
	where := map[*decad.Face]position{}
	lumps := body.Lumps()
	for i, lump := range lumps {
		for j, shell := range lump.Shells() {
			for _, face := range shell.Faces() {
				where[face] = position{i, j}
			}
		}
	}
	componentAt := make([]map[int]int, len(lumps))
	for i, lump := range lumps {
		componentAt[i] = make(map[int]int, len(lump.Shells()))
	}
	owner := make([]position, count)
	seen := make([]bool, count)
	for t, face := range sources {
		at, ok := where[face]
		if !ok {
			return nil, fmt.Errorf("export: STEP: %w: facet %d has no face in the body", decad.ErrUnsupported, t)
		}
		c := components[t]
		if seen[c] && owner[c] != at {
			return nil, fmt.Errorf("export: STEP: %w: one connected facet set spans two shells", decad.ErrUnsupported)
		}
		owner[c], seen[c] = at, true
		if prev, ok := componentAt[at.lump][at.shell]; ok && prev != c {
			return nil, fmt.Errorf("export: STEP: %w: one shell spans two connected facet sets", decad.ErrUnsupported)
		}
		componentAt[at.lump][at.shell] = c
	}
	volume := make([]float64, count)
	for i, tri := range triangles {
		a, b, c := vertices[tri[0]], vertices[tri[1]], vertices[tri[2]]
		volume[components[i]] += a.Dot(b.Cross(c)) / 6
	}
	solids := make([]meshSolid, len(lumps))
	for i, lump := range lumps {
		solids[i].outer = -1
		for j, shell := range lump.Shells() {
			c, ok := componentAt[i][j]
			if !ok {
				return nil, fmt.Errorf("export: STEP: %w: a shell has no facets", decad.ErrUnsupported)
			}
			if shell.IsVoid() {
				if !(volume[c] < 0) {
					return nil, fmt.Errorf("export: STEP: %w: a cavity's facets do not wind into it", decad.ErrUnsupported)
				}
				solids[i].voids = append(solids[i].voids, c)
				continue
			}
			if solids[i].outer >= 0 {
				return nil, fmt.Errorf("export: STEP: %w: a lump has more than one outer shell", decad.ErrUnsupported)
			}
			if !(volume[c] > 0) {
				return nil, fmt.Errorf("export: STEP: %w: an outer shell's facets do not wind outward", decad.ErrUnsupported)
			}
			solids[i].outer = c
		}
		if solids[i].outer < 0 {
			return nil, fmt.Errorf("export: STEP: %w: a lump has no outer shell", decad.ErrUnsupported)
		}
	}
	return solids, nil
}
