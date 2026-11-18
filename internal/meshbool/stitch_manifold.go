package meshbool

import (
	"fmt"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proof"
)

// auditContactManifold is the manifold audit of
// agent-docs/coplanar-contact-design.md §5. Classifying edge-only and
// point-only coplanar contacts can leave a result that meets itself along an
// edge or at a vertex, which the closure audit reports as an invariant failure
// or misses (a pinched vertex). The audit finds both: every undirected edge
// must be used by two facets in opposite directions, and the facets around
// every vertex must form one cycle through shared edges. A failure all of whose
// vertices are points the coplanar pass accepted is the contact itself and
// returns keys.Legacy; any other failure is ErrBooleanFailed.
func auditContactManifold(tris [][3]int, xverts []proof.Xpt, keys *ContactKeys) error {
	bad := manifoldDefects(tris)
	if len(bad) == 0 {
		return nil
	}
	for v := range bad {
		if _, ok := keys.Owned[xverts[v].Key()]; !ok {
			return fmt.Errorf(`%w: the stitched boundary does not close`, decaderr.ErrBooleanFailed)
		}
	}
	if keys.Legacy == nil {
		return fmt.Errorf(`%w: the stitched boundary meets itself at an accepted contact`, decaderr.ErrBooleanFailed)
	}
	return keys.Legacy
}

// manifoldDefects returns the vertices on a failing edge or a failing vertex
// fan, empty for a closed manifold with consistent orientation.
func manifoldDefects(tris [][3]int) map[int]struct{} {
	bad := map[int]struct{}{}
	type uses struct{ fwd, rev uint8 }
	edges := make(map[[2]int]uses, 3*len(tris)/2)
	for _, tri := range tris {
		for k, a := range tri {
			b := tri[(k+1)%3]
			if a == b {
				bad[a] = struct{}{}
				continue
			}
			key := [2]int{min(a, b), max(a, b)}
			u := edges[key]
			if a < b {
				u.fwd = min(u.fwd+1, 2)
			} else {
				u.rev = min(u.rev+1, 2)
			}
			edges[key] = u
		}
	}
	for key, u := range edges {
		if u.fwd != 1 || u.rev != 1 {
			bad[key[0]] = struct{}{}
			bad[key[1]] = struct{}{}
		}
	}
	if len(bad) > 0 {
		return bad
	}
	// Vertex fans. A facet (v, a, b) leaves v along v->a and enters along
	// b->v; the next facet around v is the one entering along a->v.
	incident := map[int][]int{}
	for f, tri := range tris {
		for _, v := range tri {
			incident[v] = append(incident[v], f)
		}
	}
	for v, fs := range incident {
		enters := make(map[int]int, len(fs)) // neighbour a -> facet entering v along a->v
		leaves := make(map[int]int, len(fs)) // facet -> neighbour it leaves toward
		for _, f := range fs {
			tri := tris[f]
			for k := range 3 {
				if tri[k] != v {
					continue
				}
				leaves[f] = tri[(k+1)%3]
				enters[tri[(k+2)%3]] = f
			}
		}
		seen := 0
		f := fs[0]
		for {
			seen++
			next, ok := enters[leaves[f]]
			if !ok || seen > len(fs) {
				break
			}
			f = next
			if f == fs[0] {
				break
			}
		}
		if f != fs[0] || seen != len(fs) {
			bad[v] = struct{}{}
		}
	}
	return bad
}
