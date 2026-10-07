package planar

import "github.com/lestrrat-3d/decad/internal/proof"

// This file splits a planar snapshot's audit and derived data into the half
// that reads only its triangle indices and the half that reads its vertex
// coordinates. A body placed at many poses keeps one triangle set, so the
// combinatorial half (PlanarTopology) is built once per snapshot and every
// pose pays only for its normals, boxes and volume sign (CheckPlanarPose).
// Together the two halves are CheckPlanarSolid and preparePlanar: the same
// conditions, the same edge order, the same shell labels.

// PlanarTopology is the pose-invariant combinatorics of an audited triangle
// set: the unique undirected edges in first-seen order with the two triangles
// holding each, and the shell of every vertex. It is read only, so every pose
// of the snapshot it was built from shares it.
type PlanarTopology struct {
	verts      int
	tris       [][3]int
	edges      [][2]int
	edgeFacets [][2]int
	shellOf    []int
	shells     int
}

// NewPlanarTopology runs the combinatorial half of CheckPlanarSolid on tris
// over verts vertices: at least four of each, indices in range, three distinct
// corners per triangle, and every directed edge matched by exactly one
// reverse. It reports false when that audit refuses. The topology keeps tris,
// which must not change afterwards. poll is charged as CheckPlanarSolid
// charges it.
func NewPlanarTopology(verts int, tris [][3]int, poll func() error) (*PlanarTopology, bool, error) {
	if verts < 4 || len(tris) < 4 {
		return nil, false, nil
	}
	directed := make(map[[2]int]int, 3*len(tris))
	for _, tri := range tris {
		if err := poll(); err != nil {
			return nil, false, err
		}
		for _, v := range tri {
			if v < 0 || v >= verts {
				return nil, false, nil
			}
		}
		if tri[0] == tri[1] || tri[1] == tri[2] || tri[2] == tri[0] {
			return nil, false, nil
		}
		for i := range 3 {
			directed[[2]int{tri[i], tri[(i+1)%3]}]++
		}
	}
	for edge, count := range directed {
		if err := poll(); err != nil {
			return nil, false, err
		}
		if count != 1 || directed[[2]int{edge[1], edge[0]}] != 1 {
			return nil, false, nil
		}
	}
	return planarTopologyOf(verts, tris), true, nil
}

// planarTopologyOf builds the edges, edge facets and shells of tris over verts
// vertices, the combinatorial half of preparePlanar.
func planarTopologyOf(verts int, tris [][3]int) *PlanarTopology {
	topo := &PlanarTopology{verts: verts, tris: tris}
	parent := make([]int, verts)
	for i := range parent {
		parent[i] = i
	}
	find := func(v int) int {
		for parent[v] != v {
			parent[v] = parent[parent[v]]
			v = parent[v]
		}
		return v
	}
	seen := make(map[[2]int]int, 3*len(tris)/2)
	for t, tri := range tris {
		for i := range 3 {
			u, w := tri[i], tri[(i+1)%3]
			parent[find(u)] = find(w)
			key := [2]int{min(u, w), max(u, w)}
			if e, ok := seen[key]; ok {
				// An audited solid holds each edge in exactly two triangles.
				topo.edgeFacets[e][1] = t
				continue
			}
			seen[key] = len(topo.edges)
			topo.edges = append(topo.edges, key)
			topo.edgeFacets = append(topo.edgeFacets, [2]int{t, t})
		}
	}
	topo.shellOf = make([]int, verts)
	ids := make(map[int]int)
	for v := range verts {
		root := find(v)
		id, ok := ids[root]
		if !ok {
			id = len(ids)
			ids[root] = id
		}
		topo.shellOf[v] = id
	}
	topo.shells = len(ids)
	return topo
}

// planarPose is the coordinate half of a solid's derived data at one pose,
// attached by CheckPlanarPose. It serves only the solid whose Verts and Tris
// are the slices it was read off.
type planarPose struct {
	topo           *PlanarTopology
	verts          []proof.DyV3
	normal         []proof.DyV3
	triLo, triHi   [][3]proof.Dyadic
	edgeLo, edgeHi [][3]proof.Dyadic
}

func sameSlice[T any](a, b []T) bool {
	return len(a) == len(b) && (len(a) == 0 || &a[0] == &b[0])
}

func (d *planarPose) serves(s *PlanarSolid) bool {
	return sameSlice(d.verts, s.Verts) && sameSlice(d.topo.tris, s.Tris)
}

// CheckPlanarPose finishes CheckPlanarSolid's audit of s, whose triangles
// passed topo's combinatorial half: a nonzero exact normal on every triangle
// and a positive exact signed volume. The two halves together accept exactly
// the solids CheckPlanarSolid accepts. On success it attaches the solid's
// derived data, which ClassifyPlanar and every other reader of s then reuse
// instead of rebuilding it; s's slices must not change afterwards. When s.Tris
// is not the slice topo was built from, or s.Verts does not hold topo's
// vertex count, it runs CheckPlanarSolid in full and attaches nothing. poll
// is charged once per triangle.
func CheckPlanarPose(s *PlanarSolid, topo *PlanarTopology, poll func() error) (bool, error) {
	s.pose = nil
	if topo == nil || len(s.Verts) != topo.verts || !sameSlice(topo.tris, s.Tris) {
		return CheckPlanarSolid(s, poll)
	}
	normal := planarNormals(s)
	volume := proof.DyZero()
	origin := s.Verts[0]
	for t, tri := range s.Tris {
		if err := poll(); err != nil {
			return false, err
		}
		if proof.DvIsZero(normal[t]) {
			return false, nil
		}
		volume = proof.DyAdd(volume, proof.DvDot(normal[t], proof.DvSub(s.Verts[tri[0]], origin)))
	}
	if volume.Sign() <= 0 {
		return false, nil
	}
	s.pose = newPlanarPose(s, topo, normal)
	return true, nil
}

// planarNormals is every triangle's (b−a)×(c−a).
func planarNormals(s *PlanarSolid) []proof.DyV3 {
	normal := make([]proof.DyV3, len(s.Tris))
	for t, tri := range s.Tris {
		a := s.Verts[tri[0]]
		normal[t] = proof.DvCross(proof.DvSub(s.Verts[tri[1]], a), proof.DvSub(s.Verts[tri[2]], a))
	}
	return normal
}

// newPlanarPose reads the triangle and edge boxes of s under topo.
func newPlanarPose(s *PlanarSolid, topo *PlanarTopology, normal []proof.DyV3) *planarPose {
	d := &planarPose{topo: topo, verts: s.Verts, normal: normal,
		triLo: make([][3]proof.Dyadic, len(s.Tris)), triHi: make([][3]proof.Dyadic, len(s.Tris)),
		edgeLo: make([][3]proof.Dyadic, len(topo.edges)), edgeHi: make([][3]proof.Dyadic, len(topo.edges))}
	for t, tri := range s.Tris {
		d.triLo[t], d.triHi[t] = pointBox(s.Verts, tri[:])
	}
	for e, edge := range topo.edges {
		d.edgeLo[e], d.edgeHi[e] = pointBox(s.Verts, edge[:])
	}
	return d
}

// prep is the planarPrep of s over this pose's data.
func (d *planarPose) prep(s *PlanarSolid) *planarPrep {
	return &planarPrep{s: s, normal: d.normal, triLo: d.triLo, triHi: d.triHi,
		edges: d.topo.edges, edgeFacets: d.topo.edgeFacets, edgeLo: d.edgeLo, edgeHi: d.edgeHi,
		shellOf: d.topo.shellOf, shells: d.topo.shells}
}
