package booleaneval

import (
	"math/big"

	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/loftrecord"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/occupancy"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/revolveplan"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
	"github.com/lestrrat-3d/r3"
)

// capSides returns the cap sides of owner (agent-docs/coplanar-contact-design.md §19.2): for each planar face, the
// closed half-space of its plane the true solid lies in, when the payload proves the solid meets that plane only in
// the face. It reads four payloads and returns nil for every other:
//
//   - a straight prism, whose walls run along its frame's normal between its two caps;
//   - a plain loft, whose walls are ruled between its two sections, with neither a fillet, a bore, a join gap nor a
//     construction displacement on its record, which meshbool.SlabCapSides then requires to lie on parallel planes;
//   - a revolve whose recorded meridian is the one it denotes, which meshbool.RevolveCapSides then requires to be
//     straight;
//   - a mesh Union result that carried an operand's cap side (§20.1, facetedrecord.Payload.CapSideRecords).
//
// A sheet result has no solid to bound.
func capSides(owner *topo.Body) []meshbool.CapSide {
	switch p := owner.Payload.(type) {
	case prismrecord.Payload:
		if p.SurfaceResult {
			return nil
		}
		return meshbool.SlabCapSides(recordCaps(owner))
	case loftrecord.Payload:
		if p.SurfaceResult || len(p.BlendSegs) > 0 || p.ThroughBoreCut || p.AnalyticBore || p.ConstructionDelta != 0 ||
			p.JoinDelta != 0 || p.JoinPlaneGap != 0 {
			return nil
		}
		return meshbool.SlabCapSides(recordCaps(owner))
	case revolveplan.Payload:
		if p.SurfaceResult || p.SectionDelta != 0 {
			return nil
		}
		return meshbool.RevolveCapSides(topobuild.Faces(owner))
	case facetedrecord.Payload:
		return facetedCapSides(owner, p)
	}
	return nil
}

// facetedCapSides reads the cap sides a mesh Union recorded on its result (§20.1). Each names a source group, and it
// is read only when every facet of that group belongs to one face of owner, which is then the side's face.
func facetedCapSides(owner *topo.Body, p facetedrecord.Payload) []meshbool.CapSide {
	records := p.CapSideRecords()
	if len(records) == 0 || len(p.FaceOf) != len(p.Tris) || len(p.Src) != len(p.Tris) {
		return nil
	}
	faces := topobuild.Faces(owner)
	var out []meshbool.CapSide
	for _, r := range records {
		face := -1
		for t, g := range p.Src {
			if g != r.Group {
				continue
			}
			if face >= 0 && p.FaceOf[t] != face {
				face = -2
				break
			}
			face = p.FaceOf[t]
		}
		if face < 0 || face >= len(faces) {
			continue
		}
		out = append(out, meshbool.CapSide{Face: faces[face], O: r.O, N: r.N, KeptOutline: r.KeptOutline})
	}
	return out
}

// unionCapSides returns the cap sides a mesh Union of a and b carries to its result (§20.1), as records against the
// result's source groups: facesA and facesB are topobuild.Faces of each operand, in the order the result's groups
// copy them, ma and mb the operands' tessellations, bmA and bmB their prepared meshes and st the stitched result. A
// side of one operand carries when a side of the other operand faces the same way from a plane strictly inside the
// first side's half-space, so the other operand's true solid lies in that half-space and misses its plane. The result
// then lies in the half-space and meets the plane only where the first operand does, in the side's face.
func unionCapSides(a, b *topo.Body, facesA, facesB []*topo.Face, ma, mb *topo.Mesh, bmA, bmB *meshbool.BoolMesh,
	st *meshbool.StitchedMesh) []facetedrecord.CapSideRecord {
	sa, sb := capSides(a), capSides(b)
	if len(sa) == 0 || len(sb) == 0 {
		return nil
	}
	var out []facetedrecord.CapSideRecord
	carry := func(own, other []meshbool.CapSide, faces []*topo.Face, base int, m *topo.Mesh, bm *meshbool.BoolMesh) {
		for _, s := range own {
			group := -1
			for i, f := range faces {
				if f == s.Face {
					group = base + i
					break
				}
			}
			if group < 0 || !anyStrictlyInside(other, s) {
				continue
			}
			kept := (s.KeptOutline || m.VertexBound == nil) && keptFacets(m, bm, s.Face, st, group)
			out = append(out, facetedrecord.CapSideRecord{Group: group, O: s.O, N: s.N, KeptOutline: kept})
		}
	}
	carry(sa, sb, facesA, 0, ma, bmA)
	carry(sb, sa, facesB, len(facesA), mb, bmB)
	return out
}

// keptFacets reports whether the stitched result's group holds exactly the facets of operand face f, compared by
// their exact corners, and every corner's bound in the result is no smaller than its bound in the operand: the
// result face is the operand face's held mesh, and its vertices stand for the same true points (§20.1).
func keptFacets(m *topo.Mesh, bm *meshbool.BoolMesh, f *topo.Face, st *meshbool.StitchedMesh, group int) bool {
	if st == nil || len(st.VertexBound) != len(st.Verts) || len(bm.VertexBound) != len(bm.Verts) {
		return false
	}
	type corners [3]r3.Vec
	key := func(c corners) corners {
		less := func(p, q r3.Vec) bool {
			if p.X != q.X {
				return p.X < q.X
			}
			if p.Y != q.Y {
				return p.Y < q.Y
			}
			return p.Z < q.Z
		}
		for i := range 2 {
			for j := range 2 - i {
				if less(c[j+1], c[j]) {
					c[j], c[j+1] = c[j+1], c[j]
				}
			}
		}
		return c
	}
	own := map[corners]int{}
	bound := map[r3.Vec]float64{}
	for t, tri := range bm.Tris {
		if t >= len(m.Source) || m.Source[t] != f {
			continue
		}
		var c corners
		for k, v := range tri {
			c[k] = bm.Verts[v]
			bound[c[k]] = max(bound[c[k]], bm.VertexBound[v])
		}
		own[key(c)]++
	}
	if len(own) == 0 {
		return false
	}
	n := 0
	for t, tri := range st.Tris {
		if t >= len(st.Src) || st.Src[t] != group {
			continue
		}
		var c corners
		for k, v := range tri {
			c[k] = st.Verts[v]
			b, ok := bound[c[k]]
			if !ok || st.VertexBound[v] < b {
				return false
			}
		}
		k := key(c)
		if own[k] == 0 {
			return false
		}
		own[k]--
		n++
	}
	for _, left := range own {
		if left != 0 {
			return false
		}
	}
	return n > 0
}

// anyStrictlyInside reports whether some side t of others bounds its operand within the open half-space of s: t's
// normal is a positive multiple of s's, and t's plane lies strictly on the inner side of s's plane.
func anyStrictlyInside(others []meshbool.CapSide, s meshbool.CapSide) bool {
	for _, t := range others {
		if !ratParallelSameWay(t.N, s.N) {
			continue
		}
		if ratDot3(s.N, ratSub3(t.O, s.O)).Sign() < 0 {
			return true
		}
	}
	return false
}

// bindCapSides attaches records to payload, bound to its held mesh (facetedrecord.Payload.CapSideRecords).
func bindCapSides(payload *facetedrecord.Payload, records []facetedrecord.CapSideRecord) {
	if len(records) == 0 {
		return
	}
	payload.CapSides = records
	payload.CapSideMesh = occupancy.Bind(payload.Verts, payload.Tris)
}

func ratDot3(a, b [3]*big.Rat) *big.Rat {
	s := new(big.Rat)
	for k := range 3 {
		s.Add(s, new(big.Rat).Mul(a[k], b[k]))
	}
	return s
}

func ratSub3(a, b [3]*big.Rat) [3]*big.Rat {
	return [3]*big.Rat{new(big.Rat).Sub(a[0], b[0]), new(big.Rat).Sub(a[1], b[1]), new(big.Rat).Sub(a[2], b[2])}
}

// ratParallelSameWay reports whether a is a positive multiple of b, both nonzero.
func ratParallelSameWay(a, b [3]*big.Rat) bool {
	zero := true
	for k := range 3 {
		i, j := (k+1)%3, (k+2)%3
		c := new(big.Rat).Sub(new(big.Rat).Mul(a[i], b[j]), new(big.Rat).Mul(a[j], b[i]))
		if c.Sign() != 0 {
			return false
		}
		if a[k].Sign() != 0 {
			zero = false
		}
	}
	return !zero && ratDot3(a, b).Sign() > 0
}

// recordCaps returns owner's faces that carry a CapRecord.
func recordCaps(owner *topo.Body) []*topo.Face {
	var out []*topo.Face
	for _, f := range topobuild.Faces(owner) {
		if f.CapRecord != nil {
			out = append(out, f)
		}
	}
	return out
}
